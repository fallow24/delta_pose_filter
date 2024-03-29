# 6-DoF "Delta" pose filter for sensor fusion

This is a ROS program that implements an unconventional pose filter that listens to two unreliable pose streams (describing the same motion) and fuse them together into one, more reliable pose stream. 
My implementation assumes that the sensors are located inside a rolling spherical robot. 
However, the filter is not limited to only spherical robots, but can be applied to any scenario where two pose streams appear.
Note that the filter works well even if the frequency of both pose streams is not the same. 
The filter will publish in the speed of the slower stream, and use interpolated information of the faster stream for pose fusion. 

If you plan using this filter with your own systems, some modifications need to be made in order to adjust to your motion model accordingly. Feel free to create an issue if you need help.

This interpolated information from one stream is combined with the so called pose "measurement" on the other stream, as well as an underlying motion model.
The name "Delta filter" stems from the fact that the filter consideres the transformation changes (= deltas) of both pose streams.
The current filtered pose is constantly updated using the deltas.
Both deltas are compared to an estimated model delta from a motion model.
The underlying motion model is that of a rolling ball, and the model delta estimation uses information from both the interpolated and measured deltas.
To construct the filtered pose, the interpolation-, measurement-, and model-deltas are fused together using an outlier robust weighted geometric mean, which is often used in statistics.

![Example](https://github.com/fallow24/delta_pose_filter/blob/master/img/delta_example1_pub.jpg?raw=true)

## ROS Dependencies
Tested on Ubuntu 20.04 on ROS Noetic.
The program has a minimal dependency list:
- geometry_msgs
- roscpp
- tf

## How to install 

```bash
cd ~/catkin_ws/src
git clone https://github.com/fallow24/delta_pose_filter.git
cd ..
catkin_make
source ~/catkin_ws/devel/setup.bash
```

## How to run 
```
roslaunch delta_pose_filter config_params.launch
```

## How to setup

There are the following parameters for you to set:
- topic_publish: Sets topic name on which to advertise the filtered pose
- topic_pose_imu: Sets topic name for subscribing to the first stream
- topic_pose_cam: Sets topic name for subscribing to the second stream
- frame_id_imu: Coordinate frame id of the first pose stream
- frame_id_cam: Coordinate frame id of the second pose stream
- imu_rate: Rate in Hz for the first pose stream
- cam_rate: Rate in Hz for the second pose stream
- sphere_radius: Radius of the spherical robot 
- debug_topics: Wether to publish the debug topic

Furthermore, the filter needs information about the coordinate frame definition of both pose streams.
Thus, the "config_params.launch" file includes two [static_transform_publishers](http://wiki.ros.org/tf#static_transform_publisher) which describe a [change of basis](https://en.wikipedia.org/wiki/Change_of_basis#Linear_maps) according to $\bf M_2 = P^{-1} M_1 Q$, where $\bf Q$ defines the transformation between the principal axes definitions of both sensor frames $\bf M_1$ and $\bf M_2$ (e.g. one sensor frame left handed, other one right handed), and $\bf P$ defines the transformation between them in the fixed global coordinate system.
Use the config file to define
- $\bf Q$ using the [static_transform_publisher](http://wiki.ros.org/tf#static_transform_publisher) "axis_definition_tf_imu_cam"
- $\bf P$ using the [static_transform_publisher](http://wiki.ros.org/tf#static_transform_publisher) "global_static_tf_imu_cam"

### Using the debug topic to check P and Q 
1. Set the parameter "debug_topics" found in the config file to "true"
2. Use rviz to vizualize "/delta/debug". Set global frame to "map". 
3. Use rviz to also vizualize the SLOWER pose stream
3. If those two poses dont have the same axis definitions, adjust Q
4. If those two poses dont move in the same principal direction, adjust P 

Note that the origins DO NOT HAVE TO BE ALIGNED.
As this filter only looks at the "deltas", i.e. pose changes, it will still work.
![Debug_example](https://github.com/fallow24/delta_pose_filter/blob/master/img/debug_topic.jpg?raw=true)

## Cite us 
A corresponding publication will soon be added.
This work is to be presented at the European Conference on Mobile Robotics (ECMR), held in Coimbra, Portugal on 4-7 Sept.
```
@InProceedings{ecmr2023delta,
  author       = {Fabian Arzberger and Fabian Wiecha and Jasper Zevering and Julian Rothe and Dorit Borrmann and Sergio Montenegro and Andreas N{u\"}chter},
  title        = {Delta filter -- robust visual-inertial pose estimation in real-time: A multi-trajectory filter on a spherical mobile mapping system},
  year         = {2023},
  month        = {September},
  address      = {Coimbra, Portugal},
  publisher    = {Accepted for publication on the European Conference on Mobile Robots (ECMR) 2023}
}
```
