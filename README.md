# delta_pose_filter

This is an unconventional pose filter that listens to two unreliable pose streams (describing the same motion) and fuse them together into one, more reliable pose stream. My implementation assumes that the sensors are located inside a rolling spherical robot. However, the filter is not limited to only spherical robots, but can be applied to any scenario where two pose streams appear.

## How to use

The filter needs information about the coordinate frame definition of both pose streams.
Thus, the "config_params.launch" file includes two [static_transform_publishers](http://wiki.ros.org/tf#static_transform_publisher) which describe a [change of basis](https://en.wikipedia.org/wiki/Change_of_basis#Linear_maps) according to $P^{-1} \bf M Q$
