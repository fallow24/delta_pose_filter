# delta_pose_filter

This is an unconventional pose filter that listens to two unreliable pose streams (describing the same motion) and fuse them together into one, more reliable pose stream. My implementation assumes that the sensors are located inside a rolling spherical robot. However, the filter is not limited to only spherical robots, but can be applied to any scenario where two pose streams appear.

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

The filter needs information about the coordinate frame definition of both pose streams.
Thus, the "config_params.launch" file includes two [static_transform_publishers](http://wiki.ros.org/tf#static_transform_publisher) which describe a [change of basis](https://en.wikipedia.org/wiki/Change_of_basis#Linear_maps) according to $\bf P^{-1} M Q$, where $\bf Q$ defines the transformation between the principal axes definitions of both coordinate systems (e.g. one sensor left handed, other one right handed), and $\bf P$ defines the transformation between the sensors in the fixed global coordinate system.

### How to debug P and Q 
TODO: COMMING SOON


