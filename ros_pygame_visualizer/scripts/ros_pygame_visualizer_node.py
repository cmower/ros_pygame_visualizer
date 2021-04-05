#!/usr/bin/env python3
import re
import rospy
import rospkg
import yaml
from sensor_msgs.msg import Joy
from std_msgs.msg import Int64
import ros_pygame_visualizer.pygame_interface as interface

RP = rospkg.RosPack()

class Node:

    def __init__(self):

        # Setup variables
        self.msgs = {}

        # Init ros
        rospy.init_node('ros_pygame_visualizer_node')

        # Init main screen
        hz = 40
        self.main_loop_iter = 0
        main_config_filename = rospy.get_param('~main')
        main_config = self.loadConfig(main_config_filename)
        self.main_screen = interface.MainScreen(main_config, hz)

        # Init window
        self.windows = []
        for idx, filename in enumerate(rospy.get_param('~windows')):

            # Window setup
            config = self.loadConfig(filename)
            window = {
                'index': idx,
                'type': config['type'],
                'position': config['position']
            }

            # Joystick
            if config['type'] == 'joystick':
                name = f'window_{idx}'
                window['name'] = name
                window['topic'] = config['topic']
                window['object'] = interface.JoystickWindow(config)
                window['update_handle'] = self.handleJoystick
                window['horizontal_index'] = config['horizontal_index']
                window['vertical_index'] = config['vertical_index']
                rospy.Subscriber(config['topic'], Joy, self.callback, callback_args=name)

            # Append window
            self.windows.append(window)

        # Init status publisher
        self.status_pub = rospy.Publisher('ros_pygame_visualizer/status', Int64, queue_size=10)

    def loadConfig(self, filename):

        # Replace package path in filename
        if '$(find' in filename and ')' in filename:
            # pattern = '$(find \(.*?\))'
            # matches = re.findall(pattern, filename)
            filename = filename.replace('$(find ', '') # assume filename starts with '$(find '
            idx_closing_bracket = filename.find(')')
            package = filename[:idx_closing_bracket]
            root = RP.get_path(package)
            filename = filename.replace(f'{package})', root)

        # Load configuration
        with open(filename, 'r') as configfile:
            config = yaml.load(configfile, Loader=yaml.FullLoader)
        return config

    def callback(self, msg, label):
        self.msgs[label] = msg

    def handleJoystick(self, window):

        # Extract joy data
        try:
            msg = self.msgs[window['name']]
            horizontal = msg.axes[window['horizontal_index']]
            vertical = msg.axes[window['vertical_index']]
        except KeyError:
            topic = window['topic']
            rospy.logwarn(f'Did not receive messages on the topic {topic} yet')
            horizontal = 0.0
            vertical = 0.0

        # Update window
        window['object'].setJoy(horizontal, vertical)
        window['object'].reset()

    def mainLoop(self):

        # Reset main screen
        self.main_screen.reset()

        # Update windows
        for window in self.windows:
            handle = window['update_handle']
            handle(window)
            self.main_screen.blit(window['object'].surface, window['position'])
        self.main_screen.update()

        # Publish status
        did_user_quit = self.main_screen.didUserQuit()
        if did_user_quit:
            rospy.logwarn('User quit visualizer, shutting down.')

        # Report main loop completion
        self.main_loop_iter += 1
        s = 's' if self.main_loop_iter > 1 else ''
        rospy.loginfo(f'Completed {self.main_loop_iter} iteration{s} of main loop.')

        return not did_user_quit

    def spin(self):

        # Start looping
        keep_running = True
        while keep_running:

            # Main update
            keep_running_main_loop = self.mainLoop()
            is_ros_shutdown = rospy.is_shutdown()

            # Report status
            flag = 0
            if not keep_running_main_loop:
                flag = 1
            if is_ros_shutdown:
                rospy.logwarn('ROS shutdown, killing visualizer.')
                flag = 2
            self.status_pub.publish(Int64(data=flag))

            # Update
            keep_running = keep_running_main_loop and (not is_ros_shutdown)

        self.main_screen.shutdown()

if __name__ == '__main__':
    Node().spin()
