# ros_pygame_visualizer

This package allows you to quickly setup various visualizations. When the main
`ros_pygame_visualizer` node is launched a *screen*, a number of config files
can be specified that specify the layout and properties of the screen. The
package is powered by [pygame](https://www.pygame.org/news).


A main config file provides parameters that adjust the overall layout. A number
of configs can be specified that each create a "*window*". Every window can be
setup to visualize some streaming data from ROS. See the *Windows* section below
for details.

## Windows

### Joystick

A joystick window can be setup to visualize `sensor_msgs/Joy` messages.

### Planar Workspace

A 2D workspace can be setup to visualize simple representations, e.g. a robot
position in a 2D space. The path taken by a robot, additional lines/points can
be easily specified.
