# Ground Integrity Costmap Plugin
## Overview

For robots traversing hazardous environments, the ground beneath them may pose a risk to the robot, causing it to get bogged down, stuck, beached, with no possibility of recovery.  By using ground penetrating radar, more specifically FMCW (frequency modulated continous wave) units developed at Heriot Watt University, it is possible to sense the quality of the ground up to a distance.

The FMCW radar can infer features such as water (water logged ground or sand could be a hazard) and voids (pockets of air underneath a surface), by interpreting the returned chirp signal amplitude, phase and frequency response.  By defining chirp return characteristics which indicate nominal ground integrity, it is possible to identify when this is not the case.  It would then be useful for the robot to avoid these areas, converting point observations into a spatially resolved representation of risk.

The existing ROS [navigation stack](http://wiki.ros.org/navigation) allows for occupancy grid representations of hazards (such as [obstacles](http://wiki.ros.org/costmap_2d/hydro/obstacles)) or objects of interest (such as [people](http://wiki.ros.org/social_navigation_layers)).  Path planners can use this occupancy grid for optimal planning whilst avoiding obstacles, based on total cost.

This plugin allows for additional layers of the stock ROS [costmap_2d](http://wiki.ros.org/costmap_2d) implementation to be added based on data from FMCW radar instrumentation.  Cost and nominal return characteristics associated with poor ground integrity can be adjusted on an *ad hoc* basis, allowing for great flexibilty.

# Installation

To build from this repo, clone the latest version from this repository into your catkin workspace and compile using

```
cd catkin_ws/src
git clone https://github.com/EEEManchester/ground_integrity_layer.git
cd ..
catkin_make
```

This package currently has dependancies on a package for the FMCW radar instrumentation, providing specialised message structures.  This is currently private - please contact for access, note you will also need access to the hardware.  There are also dependancies on the stock ROS costmap_2d package from the navigation stack.

# How it works

The plugin layer subscribes to a stamped topic representing FFT amplitude from a particular FFT bin, and parses the location (using TF) and amplitude value to a cost on the costmap.  The ground integrity layer operates at the same resolution and size as the main costmap (e.g. if using normal static and obstacle layers), or defined in a yaml file.

The average FFT amplitude value at each costmap cell is maintained, even when the costmap is resized.  Ampltiude value can be any units or scale, as the actual cost is calculated based on user defined thresholds.  Using thresholds allows for the robot to assign no additional cost to regions of nominal FFT amplitude, and avoid areas where ampltiudes fall outside a user defined bounds.

The user defined bounds is given by a super gaussian function:

```
hazard_value = 1 - exp(-0.5 * ( (mu - mu0) / sigma)^4 )
```

where mu0 is the central nominal amplitude, mu is the measured ampltiude value, and sigma is a variance value.  It is similar to a traditional gaussian function, bar the exponent is ^4 rather than ^2, this generates a broader central range and sharper drop-off in the tails - something between a gaussian and a top-hat function.  As a rule of thumb, +- 0.3 * sigma around the nominal amplitude value mu0 will yield no additional cost in the costmap, this can be used to tailor the variance sigma.  The proceeding ```1 - exp()``` is for computing convenience, values near mu0 (considered safe) are ~=0 where as values far away (more risky) are ~=1, this can then be scaled into the 0-252 integer values of the costmap layer.

Cost in each cell ranges from 0 (free space - robot can travel with no penalty) to 252 (with 253 and 254 reserved for "lethal" obstacles, and 255 reserved for unknown cells), to convert amplitude to a cost, the 0->1 values from the super gaussian approach are quantised:

```
cost = 252 * hazard_value
```
The nominal amplitude and variance are managed by the user, and can be changed at any time through use of [dynamically reconfigurable](http://wiki.ros.org/dynamic_reconfigure) parameters.

When the costmap is resized, all the averaged amplitude values on a per grid cell basis are temporarily stored, then the costmap is repopulated once the costmap has been expanded.  In the event that the threshold values are altered (and therefore the cost 0-254 has changed), the entire plugin layer cost is recalculated and sent out to the rest of the costmap.

## Inflating Cost

Updating individual cells is useful, however, it is better for path planners to inflate the cost to a reasonable scale length of the robot. Users can define this scale length using ```observation_radius``` or ```observation_footprint``` options in the yaml file (which is covered later).  These act identically to the usual robot footprint parameters, but can not be reconfigured once the costmap layer is running.  The ```observation_footprint``` must have three or more vertices.  If no valid radius or footprint is provided, the layer will default to averaging individual based on observation location.  The method of averaging is explained in the next section.

## Inverse Distance Weighting

To gain the estimated average of a costmap cell based on surrounding observations, the principle of distance weighting is used, i.e. observations which are closer to the cell are more likely to be closer to the real value in the cell compared to those further away.  Things that are close are more likely to be similar.

This is achieved using a Gaussian kernel, values close to the observation (distance = 0.0) have weightings close to 1.0, which tails off as a function of distance.  The scale length of this Gaussian kernel is given by ```averaging_scale_length``` in the appropriate yaml file.  If 0.0 or undefined, the averaging will happen with equal weighting (1.0) across the footprint regardless of distance from the observation.  Equal weighting is generally not recommended.

The weight, <img src="https://render.githubusercontent.com/render/math?math=w">, is given by:
<img src="https://render.githubusercontent.com/render/math?math=w%20%3D%20e%5E%7B-%5Cfrac%7B1%7D%7B2%7D%20%5Cleft(%20%5Cfrac%7Bd%7D%7Bs%7D%20%5Cright)%5E2%7D">
where <img src="https://render.githubusercontent.com/render/math?math=d"> is the euclidean distance between the observation and the cell centre, and <img src="https://render.githubusercontent.com/render/math?math=s"> is the averaging scale length.  The larger the scale length factor, the greater the cells weighting as a function of distance.  This means the weighting can be tuned to an appropriate length for the system.


The averaged value of a cell in the costmap, <img src="https://render.githubusercontent.com/render/math?math=v_%7Bi%7D">, given a new observation in time, <img src="https://render.githubusercontent.com/render/math?math=O_%7Bt%7D"> is geometrically averaged with the existing total and weight and averaged value previous to the new observation:
<img src="https://render.githubusercontent.com/render/math?math=v_%7Bi%7D%20%3D%20%20%5Cfrac%7BO_%7Bt%7Dw_%7Bi%7D%20%2B%20%5Csum_%7Bj%3D1%7D%5E%7Bi-1%7D%20v_%7Bj%7Dw_%7Bj%7D%7D%7Bw_%7Bi%7D%20%2B%20%5Csum_%7Bj%3D1%7D%5E%7Bi-1%7D%20w_%7Bj%7D%7D">


This is strictly the average for raw observation values, i.e. they are stored as floating point values, not integer 0-255 values.  The scaling factor mentioned previously converts the floating averages into integer cost values, allowing for cost to be scaled dynamically.

## Running an example
Once the package is installed, the example launch file can be run using:
```
roslaunch ground_integrity_layer ground_integrity_demo.launch
```

Note this will only work using real hardware!  Change the conditions in the view of the FMCW radar unit, and the cost will be reflected in the occupancy grid.  A node is used to spoof a robot driving up and down in a defined space, and a single costmap_2d instance is used with only the ground integrity plugin layer.  RVIZ can be used for visualisation.

In a separate terminal, type:
```
rosrun rqt_reconfigure rqt_reconfigure
```
The dynamically reconfigurable options can be changed by the user in real-time.  These parameters can be modified by other ROS nodes (allowing for autonomous re-weighting of cost).  Check out how to setup a dynamic reconfigure client [here](http://wiki.ros.org/dynamic_reconfigure/Tutorials).

In this example, the amplitude centre (of the super gaussian) is defined, along with the variance - however these may be wildly different based on the materials, distances, and general configuration used with the hardware.  These can be configured in various yaml files, launch file variables, or dynamically.

### Dynamic Reconfigure Parameters

The plugin can be enabled and disabled using the ```enable``` functionality, the plugin layer will continue to process radar observation data but no cost will be added to the costmap.  Behaviour of a robot with respect to ground integrity can be easily switched on and off using this enable functionality.

To accommodate the wide range of value when of measuring FFT amplitude threshold centre and variance consist of a double value (between -999 and 999) and a power multiplier.  For example to set a threshold centre of 100,000 *units*, the value would be set to ```100``` and the mulitplier to ```kilo (3)```, i.e. one with three zeros.  Likewise it could also be equally defined as ```0.1``` and ```mega (6)```, however, it is usually easier to manipulate pre decimal significant figures.

The power modifier for thresholds ranges from:

- nano (E-9)
- micro (E-6)
- milli (E-3)
- unit (E0)
- kilo (E3)
- mega (E6)
- giga (E9)


# Loading the plugin for **costmap_2d** or **move_base**

Using the plugin for both costmap_2d or directly for path planning into [move_base](http://wiki.ros.org/move_base) is achieved through the use of .yaml files.  An example yaml file can be found in the package ```configs``` folder.

Costmap layers are accessed as per the order specified in the yaml file, for example,
```
plugins: 
    - {name: static_map,       type: "costmap_2d::StaticLayer"}
    - {name: obstacles,        type: "costmap_2d::VoxelLayer"}
    - {name: inflation,        type: "costmap_2d::InflationLayer"}
    - {name: ground_integrity, type: "ground_integrity::GILayer"}
```
results in the ```inflation``` plugin layer only operating on the levels above itself (i.e. static_map and obstacles), not the ```ground_integrity``` layer.  Order of plugins may affect the desired output and performance of the costmap.  To specify the topic name of the FMCW radar amplitude, use the snippet
```
ground_integrity:
    observation_topic: fmcw/amplitude
```
with ```ground_integrity:``` being the same name as in the previous plugins lists.  The default topic name is ```fmcw/amplitude```.

There are more options which can passed to the costmap layer including:

```
    detection_footprint: # List of vertices in format [[x0, y0], [x1, y1], [x2, y2], ..., [xN, yN]]
    detection_radius: # radius in metres e.g. 0.5
    averaging_scale_length:  # distance in metres e.g. 0.2
    combination_method: # How are costmap costs integrated with other layers above, using integer values e.g. 1
    minimum_weight: # Total weight of cell before added to costmap (used with IDW)
```

Footprint and radius perform identically to those used of the robot footprint, however, this is a footprint around the frame of the sensor, not the robot.  The footprint argument will supersede any radius arguments, ensure the footprint argument is missing or commented out if wishing to use radius value.  The average scale length has been covered earlier, and ```combination_method``` allows the user to define how cost is incorporated in the layered costmap.  This includes taking the maximum value of all layers (0), overwriting all other layers (1), addition with cost of other layers (2), and maximum value whilst preserving unknown space (3).  Default is 0 (keep maximum value of all layers).

The option ```minimum_weight``` can be useful when the edges of an inflated space are greatly different from the local average, and is causing issues with path planning.  By requiring a minimum total cell weight, only cells which have seen multiple observations, or very nearby observations are updated in the costmap.  It is recommended this is kept to the default value of ```0.0```.


# Bugs & Feature Requests
Please report bugs and request features using the [Issue Tracker](https://github.com/EEEManchester/ground_integrity_layer/issues).
