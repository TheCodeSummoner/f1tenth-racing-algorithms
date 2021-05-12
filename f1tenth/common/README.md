## Common

### Racer

Racer module provides a base class for interacting with the simulator. It supports reading lidar and odometry data,
and publishing drive commands. In addition, an inheriting class for Follow The Gap is included in the module.

### MPC

MPC module provides a base class for formulating your Model Predictive Control problem. It's essentially an 
F1TENTH-specific wrapper of the `do_mpc` library. Moreover, the module features a simple distance-based point following
MPC.

### Marker

Contains wrappers for visualising the algorithms in the simulator. Note that since the visualisations happen by
publishing messages to ROS channels, the provided selection of channels may be busy by default (e.g. to plot the
starting line).

### Point

Provides simple data classes for dealing with cartesian and lidar points.