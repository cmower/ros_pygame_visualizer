# ros_pygame_visualizer

This package allows you to quickly setup various visualizations. When the main
`ros_pygame_visualizer` node is launched a *screen* appears that can be
configured to track ROS data and visualize it. Any number of config files can be
specified that give the layout and properties of the screen. The package is
powered by [pygame](https://www.pygame.org/news).

A main config file provides parameters that adjust the overall layout. A number
of other configs can be specified that each create a "*window*". Every window
can be setup to visualize certain ROS data in a helpful way. See the *Windows*
section below for details. Examples are provided, see `launch/`.

## Windows

### Joystick

A joystick window can be setup to visualize `sensor_msgs/Joy` messages.

### Planar Workspace

A 2D workspace can be setup to visualize simple representations, e.g. a robot
position in a 2D space. The path taken by a robot, additional lines/points can
be easily specified. The following objects can be visualized:
- *Static point*: creates a single unchanging point in the window.
- *Static line*: creates a single unchanging line in the window.
- *Dynamic point*: tracks `geometry_msgs/Point` messages defined in the local
  coordinate axes.
- *Dynamic line*: tracks `std_msgs/Float64MultiArray` messages that define a
  continuous line.
- *Dynamic point array*: tracks `std_msgs/Float64MultiArray` that define points
  on the surface to visualize.

**Note**, currently coordinates must be specified in the local window frame
(scaling between meters and pixels is handled in the config file). At present,
you can not specify additional coordinate frames - future work.
