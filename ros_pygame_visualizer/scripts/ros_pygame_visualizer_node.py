#!/usr/bin/env python3
import rospy
# import re
import rospkg
import yaml
from sensor_msgs.msg import Joy
from std_msgs.msg import Int64, Float64MultiArray
from geometry_msgs.msg import Point
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
        self.msg_keys = set()

        # Init window
        self.windows = []
        for idx, filename in enumerate(rospy.get_param('~windows')):

            # Window setup
            config = self.loadConfig(filename)
            name = f'window_{idx}'
            window = {
                'index': idx,
                'type': config['type'],
                'position': config['position'],
                'name': name,
            }

            # Joystick
            if config['type'] == 'joystick':
                window['topic'] = config['topic']
                window['object'] = interface.JoystickWindow(config)
                window['update_handle'] = self.handleJoystick
                window['horizontal_index'] = config['horizontal_index']
                window['vertical_index'] = config['vertical_index']
                self.startSubscriber(name, config['topic'], Joy)

            # Planar workspace
            if config['type'] == 'planar_workspace':
                window['object'] = interface.PlanarWorkspaceWindow(config)
                window['update_handle'] = self.handlePlannarWorkspaceWindow
                window['dynamic_objects'] = []

                for object_index, full_object_config in enumerate(config['objects']):
                    object_type = list(full_object_config.keys())[0]
                    object_config = list(full_object_config.values())[0]
                    if 'dynamic' not in object_type: continue  # static objects are handled when window['object'] is created
                    object_name = object_config['name']
                    object_topic = object_config['topic']
                    dynamic_object = {
                        'name': object_name,
                        'topic': object_topic,
                    }

                    if object_type == 'dynamic_point':
                        dynamic_object['handle'] = self.handleDynamicPoint
                        self.startSubscriber(object_name, object_topic, Point)

                    elif object_type == 'dynamic_line':
                        dynamic_object['handle'] = self.handleDynamicLine
                        self.startSubscriber(object_name, object_topic, Float64MultiArray)

                    window['dynamic_objects'].append(dynamic_object)

            # Append window
            self.windows.append(window)

        # Init status publisher
        self.status_pub = rospy.Publisher('ros_pygame_visualizer/status', Int64, queue_size=10)

    def startSubscriber(self, name, topic, msg_type):
        assert name not in self.msg_keys, "name ({name}) must be unique."
        rospy.Subscriber(topic, msg_type, self.callback, callback_args=name)
        self.msg_keys.add(name)

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

    def handlePlannarWorkspaceWindow(self, window):
        for dynamic_object in window['dynamic_objects']:
            handle = dynamic_object['handle']
            handle(window, dynamic_object)
        window['object'].reset()

    def getMsg(self, name):
        try:
            msg = self.msgs[name]
        except KeyError:
            rospy.logwarn(f'Did not receive messages for the object called {name} yet!')
            msg = None
        return msg

    def handleDynamicPoint(self, window, dynamic_object):
        name = dynamic_object['name']
        msg = self.getMsg(name)
        if msg is None: return
        update = {
            'position': [msg.x, msg.y],
        }
        window['object'].updateDynamicObject(name, update)

    def handleDynamicLine(self, window, dynamic_object):
        name = dynamic_object['name']
        msg = self.getMsg(name)
        if msg is None: return
        # Positions are contained in msg.data. Assume, len(msg.data) = 2*N,
        # where msg.data[:N] are x-axis coordinates and msg.data[N:] are y-axis
        # coordinates.
        N = int(len(msg.data)/2)
        update = {
            'positions': [[px, py] for px, py in zip(msg.data[:N], msg.data[N:])]
        }
        window['object'].updateDynamicObject(name, update)

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
        # s = 's' if self.main_loop_iter > 1 else ''
        # rospy.loginfo(f'Completed {self.main_loop_iter} iteration{s} of main loop.')

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
