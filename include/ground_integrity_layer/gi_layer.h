#ifndef GI_LAYER_H_
#define GI_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/costmap_layer.h>
#include <nav_msgs/OccupancyGrid.h>
#include <fmcw/ChannelAmplitudeStamped.h>
#include <ground_integrity_layer/GroundIntegrityLayerConfig.h>
#include <dynamic_reconfigure/server.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>


namespace ground_integrity
{

class GILayer: public costmap_2d::CostmapLayer //costmap_2d::Layer, public costmap_2d::Costmap2D
{
public:
  GILayer();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  bool isDiscretized()
  {
    return true;
  }

  virtual void matchSize();


private:
  //void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
  // dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
  void reconfigureCB(ground_integrity_layer::GroundIntegrityLayerConfig &config, uint32_t level);
  dynamic_reconfigure::Server<ground_integrity_layer::GroundIntegrityLayerConfig> *dsrv_;

  void observationCB(const fmcw::ChannelAmplitudeStamped& obs_msg); // Callback for incoming radar observation messages
  void updateObservations(std::list<std::pair<unsigned int, float> > &updates); // Process recent observations and return value to be added to costmap
  void publishLayer(std::string);
  
  float* averages_;
  unsigned int* n_obs_;
  float* weight_obs_;
  unsigned int averages_size_;

  bool update_full_layer_;
  bool rolling_window_;
  bool use_lethal_;

  void getCache(std::list<std::pair<std::pair<double,double>, std::pair<float,float> > > &observation_cache);

  double threshold_centre_, threshold_variance_, tc_, tv_;
  int threshold_centre_scale_, threshold_variance_scale_, max_cost_;
  unsigned int scaledValue(float value);

  double averaging_scale_length_, minimum_weight_;
  bool inflate_observation_;
  int combination_method_;
  std::vector<geometry_msgs::Point> sensor_footprint_;

  ros::Subscriber observation_sub_;
  ros::Publisher layer_pub_;
  std::list<fmcw::ChannelAmplitudeStamped> observation_msg_buffer_;
  std::string global_frame_; // Global frame of costmap

  boost::recursive_mutex lock_; // Stop observation buffer being modified by multiple functions at once
  boost::recursive_mutex cache_;  // Stop cache from being modified when performing resizing

  void updatePreserveNoInfo(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  std::vector<geometry_msgs::Point> makeSensorFootprintFromParams(ros::NodeHandle& nh);

};
}
#endif
