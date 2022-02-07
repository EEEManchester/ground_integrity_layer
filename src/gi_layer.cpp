#include <ground_integrity_layer/gi_layer.h>
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/footprint.h> // Added for footprint functions - inflation of sensor
#include <costmap_2d/costmap_math.h>  // Added for updateWithMax
#include <cmath> // Required for basic maths operators
#include <ros/console.h> // ROS debugging

PLUGINLIB_EXPORT_CLASS(ground_integrity::GILayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;  // 255
using costmap_2d::LETHAL_OBSTACLE;  // 254
using costmap_2d::INSCRIBED_INFLATED_OBSTACLE;  // 253
using costmap_2d::FREE_SPACE;  // 0

namespace ground_integrity
{

GILayer::GILayer() {}

// How to Initialize the layer
void GILayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_), g_nh;
  current_ = true;
  default_value_ = NO_INFORMATION;  // What should the layer be filled with by default
  rolling_window_ = layered_costmap_->isRolling();

  // Setup up clean averages storage
  averages_ = NULL;
  weight_obs_= NULL;
  averages_size_ = 0;

  update_full_layer_ = false;


  matchSize();  // Resize layer to match the layered costmap (usually is same as Static layer) OBSTACLE LAYER USES OBSTACLELAYER::MATCHSIZE???
  global_frame_ = layered_costmap_->getGlobalFrameID();

  // Get amplitude topic name
  std::string amplitude_topic;
  nh.param("amplitude_topic", amplitude_topic, std::string("fmcw/amplitude"));

  inflate_observation_ = false;
  sensor_footprint_ = makeSensorFootprintFromParams(nh);
  nh.param("averaging_scale_length", averaging_scale_length_, 0.0);
  nh.param("combination_method", combination_method_, 0);
  nh.param("minimum_weight", minimum_weight_, 0.0);
  

  // Setup subscriber to topic, callback function = observationCB
  observation_sub_ = g_nh.subscribe(amplitude_topic, 1, &GILayer::observationCB, this);
  layer_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("ground_integrity_map",100);

  // Initialise threshold values - WILL REPLACE WITH PARAMETERS LATER
  threshold_centre_ = 10;  // Nominal radar amplitude which equates to no additional cost
  threshold_centre_scale_ = 3;
  tc_ = threshold_centre_ * pow(10.0, threshold_variance_scale_);
  threshold_variance_ = 5; // Defines the shape of the super gaussian - +-0.3 variance = no additional cost region
  threshold_variance_scale_ = 3;
  tv_ = threshold_variance_ * pow(10.0, threshold_variance_scale_);

  use_lethal_ = false;
  max_cost_ = INSCRIBED_INFLATED_OBSTACLE - 1;


  // Add dynamic reconfigure to layer
  dsrv_ = new dynamic_reconfigure::Server<ground_integrity_layer::GroundIntegrityLayerConfig>(nh);
  dynamic_reconfigure::Server<ground_integrity_layer::GroundIntegrityLayerConfig>::CallbackType cb = boost::bind(
      &GILayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}


void GILayer::matchSize()
{
  // Cache current observation data
  // first pair = x & y coords, second pair = weight_obs and average
  std::list<std::pair<std::pair<double,double>, std::pair<float,float> > > observation_cache;
  getCache(observation_cache);

  // Ensure local costmap layer size and resolution matches master
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());

  // Resize arrays for data averages and weight_obs per cell
  unsigned int size_x = master->getSizeInCellsX();
  unsigned int size_y = master->getSizeInCellsY();
  // Set up correct size for averages_
  if (averages_ == NULL) {
    //ROS_WARN("GILayer::updateCosts(): averages_ array is NULL");
    averages_size_ = size_x * size_y;
    averages_ = new float[averages_size_];
    weight_obs_ = new float[averages_size_];
  }
  else if (averages_size_ != size_x * size_y)
  {
    //ROS_WARN("GILayer::updateCosts(): averages_ array size is wrong");
    delete[] averages_;
    averages_size_ = size_x * size_y;
    averages_ = new float[averages_size_];
    weight_obs_ = new float[averages_size_];
  }
  std::fill_n(weight_obs_, averages_size_, 0.0);
  std::fill_n(averages_, averages_size_, std::numeric_limits<float>::quiet_NaN()); // Fill averages_ with nan

  // Repopulate averages_ and weight_obs with any cached data

  for (std::list<std::pair<std::pair<double,double>, std::pair<float,float> > >::iterator cache_item = observation_cache.begin(); cache_item != observation_cache.end(); cache_item++) {
    unsigned int idx_x, idx_y;  // Take x and y values and check they exist inside the map bounds, if so return x & y indices
    if (!worldToMap(cache_item -> first.first, cache_item -> first.second, idx_x, idx_y)) {
      continue;
    }

    unsigned int linear_idx = getIndex(idx_x, idx_y);  // Convert x and y cell index to linear index to reference average value storage later

    weight_obs_[linear_idx] = cache_item -> second.first;
    averages_[linear_idx] = cache_item -> second.second;
  } //end for
  update_full_layer_ = true;
}


void GILayer::reconfigureCB(ground_integrity_layer::GroundIntegrityLayerConfig &config, uint32_t level)
{
  // If threshold values change, recompute entire costmap
  if (config.threshold_centre != threshold_centre_ || config.threshold_centre_scale != threshold_centre_scale_ || config.threshold_variance != threshold_variance_ || config.threshold_variance_scale != threshold_variance_scale_ || config.use_lethal != use_lethal_) {
    update_full_layer_ = true;  // Flag that all cell costs should be recalculated
  }
    enabled_ = config.enabled;

    threshold_centre_ = config.threshold_centre;
    threshold_centre_scale_ = config.threshold_centre_scale;
    tc_ = threshold_centre_ * pow(10.0, threshold_centre_scale_);

    threshold_variance_ = config.threshold_variance;
    threshold_variance_scale_ = config.threshold_variance_scale;
    tv_ = threshold_variance_ * pow(10.0, threshold_variance_scale_);

    use_lethal_ = config.use_lethal;
    max_cost_ = use_lethal_ ? LETHAL_OBSTACLE : INSCRIBED_INFLATED_OBSTACLE - 1;  // If true, set max_cost_ to lethal_obstacle, else use 252 (use of ? is a stand in for if-else statement)

    ROS_INFO("Updated threshold params: Centre= %.2f, Variance= %.2f", tc_, tv_);
}

void GILayer::observationCB(const fmcw::ChannelAmplitudeStamped& obs_msg)
{
  boost::recursive_mutex::scoped_lock lock(lock_);  // Ensure only this functon can interact with observation_msg_buffer_
  observation_msg_buffer_.push_back(obs_msg);  // Place new message onto buffer
  boost::recursive_mutex::scoped_lock unlock(lock_);  // Release control of the buffer
}

void GILayer::updateObservations(std::list<std::pair<unsigned int, float> > &updates)
{
  std::list<fmcw::ChannelAmplitudeStamped> observation_msg_buffer_copy_;  // Create blank buffer for observations

  boost::recursive_mutex::scoped_lock lock(lock_);  // Restrict access to buffer to this function only (i.e. can't be updated via observationCB())
  observation_msg_buffer_copy_ = std::list<fmcw::ChannelAmplitudeStamped>(observation_msg_buffer_);  // Make copy of current observation buffer
  observation_msg_buffer_.clear();  // Clear current observation buffer for new messages
  boost::recursive_mutex::scoped_lock unlock(lock_);  // Release access to observation buffer


  // Perform required processing on last set of observations //
  if (observation_msg_buffer_copy_.size() == 0) {   //  If no observations are available for processing then return
    return;
  }

  // Loop over all observations
  for (std::list<fmcw::ChannelAmplitudeStamped>::iterator obs = observation_msg_buffer_copy_.begin(); obs != observation_msg_buffer_copy_.end(); obs++) {

    // Convert observation frame to costmap frame at message timestamp
    geometry_msgs::TransformStamped transformStamped;  // Store transform between frames

    try
    {
      // Get transform between observation frame and costmap frame.  tf_ is somehow passed to the layer (hence it is not declared in this layer, via inheritance from the costmap_layer class
      
      transformStamped = tf_ -> lookupTransform(global_frame_, obs -> header.frame_id, obs -> header.stamp,ros::Duration(0.1));
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      return;  // If cannot get transform return
    }

    // Extract amplitude data from message
    float value = obs -> amplitude;

       
    // Vector of x and y cell indices
    std::vector<costmap_2d::MapLocation> cell_locations;

    if (inflate_observation_) {
      // Vector of x and y cell indices
      std::vector<costmap_2d::MapLocation> footprint_locations;
      // transformFootprint(x,y,yaw,footprint,footprint_out)
      // As the footprint is circular, the yaw can be ignored (removes need for more TF and conversion)
      //  Would try: double yaw = tf2::getYaw(pose.pose.orientation); - need to get the right thing out of transformStamped
      std::vector<geometry_msgs::Point> transformed_footprint;
      double yaw_sub = 0.0;
      // Transform footprint to be centred about the sensor
      costmap_2d::transformFootprint(transformStamped.transform.translation.x, transformStamped.transform.translation.y, yaw_sub, sensor_footprint_, transformed_footprint);
      
      // ROS_INFO("%d", int(transformed_footprint.size()));

      for (unsigned int i = 0; i < transformed_footprint.size(); ++i)
      {
        costmap_2d::MapLocation loc;
        if (worldToMap(transformed_footprint[i].x, transformed_footprint[i].y, loc.x, loc.y))
        {
          footprint_locations.push_back(loc);  // If footprint inbounds, push back
        }
      }
      convexFillCells(footprint_locations, cell_locations);
    } else {  // end if inflate_observation_
      costmap_2d::MapLocation loc;
      if (!worldToMap(transformStamped.transform.translation.x, transformStamped.transform.translation.y, loc.x, loc.y)) {
      ROS_WARN("Observation out of bounds");
      continue;
      }
      cell_locations.push_back(loc);
    }  // end if inflate_ function
    
    for (size_t i = 0; i < cell_locations.size(); i++)
    {
      unsigned int idx_x = cell_locations[i].x, idx_y = cell_locations[i].y;  // Take x and y values and check they exist inside the map bounds, if so return x & y indices

      // GET POLYGON AREA BASED ON MEASUREMENT LOCATION
      // PRODUCE LIST OF CELL INDICES TO BE UPDATED INSIDE THE POLYGON
      // FOR ELEMENTS IN LIST{UPDATE weight_obs_, calculate averages_}
      unsigned int linear_idx = getIndex(idx_x, idx_y);  // Convert x and y cell index to linear index to reference average value storage later


      

      double cell_pos_x, cell_pos_y;
      mapToWorld(idx_x, idx_y, cell_pos_x, cell_pos_y);
      double dist = distance(transformStamped.transform.translation.x, transformStamped.transform.translation.y, cell_pos_x, cell_pos_y);
      double dist_weighting = 1.0;
      if (averaging_scale_length_ > 0.0)
      {
        dist_weighting = exp(-0.5 * pow((dist/(averaging_scale_length_)), 2.0));
      }
     

      // Catch first observation, where value will be nan and can't be averaged
      if (weight_obs_[linear_idx] == 0.0) {
        averages_[linear_idx] = value;
      }
      else
      {
        // Perform weighted average for cell
        // BASED ON CENTRE OF MASS EQUATION
        averages_[linear_idx] = ( (dist_weighting * value)+ (weight_obs_[linear_idx]*averages_[linear_idx]) ) / (dist_weighting + weight_obs_[linear_idx]);
        
      }
      weight_obs_[linear_idx] = weight_obs_[linear_idx] + dist_weighting;  // Update cell weight with total weight including last measurement
      // Update list of updated cells for costmap
      // NEED TO CHECK IF CELL INDEX IN PAIR LIST, THEREFORE UPDATE THAT INDEX, DON'T PUT THE INDEX IN TWICE, SAVE ON COMPUTATION
      if (weight_obs_[linear_idx] >= minimum_weight_)  // Only update costmap cells if cell weight is above minimum
      {
        std::pair <unsigned int, float> data_pair (linear_idx, averages_[linear_idx]);
        updates.push_back(data_pair);
      }
      
      
    }// end for loop over footprint cells

  } // end for loop over observations


}

void GILayer::getCache(std::list<std::pair<std::pair<double,double>, std::pair<float,float> > > &observation_cache)
{
  float* averages_copy;
  float* weight_obs_copy;
  unsigned int averages_size_copy;

  // Ensure observation data is copied before being erased or modified by another function
  boost::recursive_mutex::scoped_lock lock(cache_);
  averages_copy = averages_;
  weight_obs_copy = weight_obs_;
  averages_size_copy = averages_size_;
  boost::recursive_mutex::scoped_lock unlock(cache_);

  for (size_t i = 0; i < averages_size_copy; i++) {
    if (weight_obs_copy[i] > 0.0) {
      unsigned int mx, my;
      double wx, wy;
      indexToCells(i, mx, my);  // Convert cell index to map cell (x and y index)
      mapToWorld(mx, my, wx, wy);  // Convert cell x and y to world coords

      // Pair up
      std::pair<double, double> cell_location (wx, wy);
      std::pair<float, float> observation_data (weight_obs_copy[i], averages_copy[i]);

      observation_cache.push_back(std::make_pair(cell_location, observation_data));  //Push a pair of pairs to list
    }
  } // end for
  return;
}

unsigned int GILayer::scaledValue(float value)
{
  // Pass value through super-gaussian function and invert (1->0 i.e. middle of the distribution is zero, wings tend to 1)
  float super_gaussian = 1 - exp(-0.5 * pow(4.0, ((value - tc_) / tv_)));  // Power of 4 rather than 2 is what makes it a super-gaussian
  int scaled_value = int((max_cost_* super_gaussian) - 0.5);  // The 0.5 effectively rounds down central values to ensure a good 0 cost coverage
  if (scaled_value > max_cost_) { // If cost value > LETHAL_OBSTACLE, then limit to LETHAL_OBSTACLE
    scaled_value = max_cost_;
  }
  if (scaled_value < FREE_SPACE) {
    scaled_value = FREE_SPACE;  // If cost value < 0, then limit to FREE_SPACE
  }

  return (unsigned int)scaled_value;
}

void GILayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  // If layer not enabled, do not record data to local costmap - not sure why you would _want_ to do thi
  // if (!enabled_)
  //   return;

  if (rolling_window_){
    matchSize(); // Will cache all current observation data, plus set the origin etc of the map
  }
  std::list<std::pair<unsigned int, float> > updates;  // Store all updates for processing

  if (update_full_layer_) {
    for (size_t i = 0; i < averages_size_; i++) {
      if (weight_obs_[i] > 0.0) {  // If observations have been entered for that cell
        std::pair <unsigned int, float> data_pair (i, averages_[i]);  // Create index and average value pair
        updates.push_back(data_pair);  // Push back pair to updates
      }
    } // end for
    update_full_layer_ = false;
  }
    
  updateObservations(updates);// Include new observations
  

  // Take all observation updates and mark costmap accordingly
  for (std::list<std::pair<unsigned int, float> >::iterator updt = updates.begin(); updt != updates.end(); updt++) {
    unsigned int temp_idx = updt -> first;  // first of pair = index
    float temp_value = updt -> second;  // Second of pair = averaged float value
    costmap_[temp_idx] = scaledValue(temp_value);  // Set scaled (0-254) value based on thresholds in costmap
  }

  // THIS WORKED - TAKEN FROM INFLATION LAYER
  // Allows for total enable/disable of layer, removing all cost when !enabled
  // I don't understand why this works rather than updating the min to max of only all observed data
  *min_x = -std::numeric_limits<float>::max();
  *min_y = -std::numeric_limits<float>::max();
  *max_x = std::numeric_limits<float>::max();
  *max_y = std::numeric_limits<float>::max();
}

void GILayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_){
    return;
  }

switch (combination_method_)
  {
    case 0:  // Maximum
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
      break;
    case 1:  // Overwrite
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
      break;
    case 2:  // Addition
      updateWithAddition(master_grid, min_i, min_j, max_i, max_j);
      break;
    case 3:  // Maximum, preserve No Info
      updatePreserveNoInfo(master_grid, min_i, min_j, max_i, max_j);
      break;  
    default:  // Maximum
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
      break;
  }
  // UPDATE WITH MAX //
  // Nice behaviour because it maintains lethal obstacles from other layers
  // if (master_cost ==  NO_INFORMATION || master_cost < local_cost): update
  // updateWithMax(master_grid, min_i, min_j, max_i, max_j);
  // updatePreserveNoInfo(master_grid, min_i, min_j, max_i, max_j);

  // UPDATE WITH OVERWRITE //
  // Regardless of other layers, overwrite
  // if (local != NO_INFORMATION): update
  // updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);

  // UPDATE WITH ADDITION //
  // update with master_cost + local_cost, up to max_val = 253 (i.e. LETHAL_OBSTACLE - 1)
  // updateWithAddition(master_grid, min_i, min_j, max_i, max_j);

  publishLayer("ground_integrity_layer");
  
}

void GILayer::updatePreserveNoInfo(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;

  unsigned char* master_array = master_grid.getCharMap();
  unsigned int span = master_grid.getSizeInCellsX();

  for (int j = min_j; j < max_j; j++)
  {
    unsigned int it = j * span + min_i;
    for (int i = min_i; i < max_i; i++)
    {
      if (costmap_[it] == NO_INFORMATION){
        it++;
        continue;
      }

      unsigned char old_cost = master_array[it];
      if (old_cost < costmap_[it])
        master_array[it] = costmap_[it];
      it++;
    }
  }
}

std::vector<geometry_msgs::Point> GILayer::makeSensorFootprintFromParams(ros::NodeHandle& nh)
{
  std::string full_param_name;
  std::string full_radius_param_name;
  std::vector<geometry_msgs::Point> points;

  if (nh.searchParam("detection_footprint", full_param_name))
  {
    XmlRpc::XmlRpcValue footprint_xmlrpc;
    nh.getParam(full_param_name, footprint_xmlrpc);

    if (footprint_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeString &&
        footprint_xmlrpc != "" && footprint_xmlrpc != "[]")
    {
      if (costmap_2d::makeFootprintFromString(std::string(footprint_xmlrpc), points))
      {
        costmap_2d::writeFootprintToParam(nh, points);
        inflate_observation_ = true;
        return points;
      }
    }
    else if (footprint_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeArray  && footprint_xmlrpc.size() > 2)
    {
      points = costmap_2d::makeFootprintFromXMLRPC(footprint_xmlrpc, full_param_name);
      costmap_2d::writeFootprintToParam(nh, points);
      inflate_observation_ = true;
      return points;
    }
  }

  if (nh.searchParam("detection_radius", full_radius_param_name))
  {
    double detection_radius;
    nh.param(full_radius_param_name, detection_radius, 0.0);
    points = costmap_2d::makeFootprintFromRadius(detection_radius);
    nh.setParam("detection_radius", detection_radius);
    inflate_observation_ = true;
  }
  return points;
}

void GILayer::publishLayer(std::string layer_name){

    std::vector<boost::shared_ptr<costmap_2d::Layer> >* plugins = layered_costmap_->getPlugins();
    for (std::vector<boost::shared_ptr<costmap_2d::Layer> >::iterator pluginp = plugins->begin(); pluginp != plugins->end(); ++pluginp) {

      boost::shared_ptr<costmap_2d::Layer> plugin = *pluginp;

      
      if(plugin->getName().find(layer_name)!=std::string::npos) {
          boost::shared_ptr<ground_integrity::GILayer> costmap;
          costmap = boost::static_pointer_cast<ground_integrity::GILayer>(plugin);
          unsigned char* grid = costmap->getCharMap();
          int x_size = costmap->getSizeInCellsX();
          int y_size = costmap->getSizeInCellsY();
          float res = costmap->getResolution();
          int size = x_size*y_size;

          nav_msgs::OccupancyGrid grid_msg;
          grid_msg.header.stamp = ros::Time::now();
          grid_msg.header.frame_id = "map";
          grid_msg.info.width = x_size;
          grid_msg.info.height = y_size;
          grid_msg.info.resolution = res;
          grid_msg.info.origin.position.x = costmap->getOriginX();
          grid_msg.info.origin.position.y = costmap->getOriginY();

          for(int i = 0; i < size ;i++){
            grid_msg.data.push_back(*(grid+i));
          }
          this->layer_pub_.publish(grid_msg);
      }
  }
}

} // end namespace
