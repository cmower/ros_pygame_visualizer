#!/usr/bin/env python3
import os
import rospy
from ros_helper.node import RosNode
from sensor_msgs.msg import Joy, Image
from std_msgs.msg import Int64, Float64MultiArray, String
from geometry_msgs.msg import Point
import ros_pygame_visualizer.pygame_interface as interface
from ros_pygame_visualizer.srv import SaveImage, SaveImageResponse
from cv_bridge import CvBridge

class Node(RosNode):

    hz = 40

    def __init__(self):

        # Init ros
        super().__init__(rospy)
        self.initNode('ros_pygame_visualizer_node')
        self.getParams([
            ('main_filename', '~main',),
            ('window_filenames', '~windows',),
        ])

        # Init main screen
        self.main_loop_iter = 0
        main_config = self.loadConfig(self.params['main_filename'])
        self.main_screen = interface.MainScreen(main_config, self.hz)
        if 'pictures_directory' in main_config:
            self.pictures_directory = main_config['pictures_directory']
        else:
            self.pictures_directory = os.path.join(os.environ['HOME'], 'Pictures')
        rospy.loginfo(f'Setup main screen window at index 0.')

        # Setup cv bridge
        self.cv_bridge = CvBridge()

        # Init window
        self.windows = []
        for idx, filename in enumerate(self.params['window_filenames']):

            # Window setup
            config = self.loadConfig(filename)
            window_index = idx+1
            window_type = config['type']
            name = f'window_{idx}'
            window = {
                'index': window_index,
                'type': window_type,
                'position': config['position'],
                'name': name,
            }

            # Joystick
            if window_type == 'joystick':
                window['topic'] = config['topic']
                window['object'] = interface.JoystickWindow(config)
                window['update_handle'] = self.handleJoystick
                window['horizontal_index'] = config['horizontal_index']
                window['vertical_index'] = config['vertical_index']
                self.startSubscriber(name, config['topic'], Joy)

            # Image
            if window_type == 'image':
                window['topic'] = config['topic']
                window['object'] = interface.ImageWindow(config)
                window['update_handle'] = self.handleImage
                window['width'] = config['width']
                window['height'] = config['height']
                self.startSubscriber(name, config['topic'], Image)

            # Text
            if window_type == 'text':
                if 'static' in config.keys():
                    static = config['static']
                else:
                    static = True
                window['static'] = static
                window['object'] = interface.TextWindow(config)

                if static:
                    # Treat as static
                    window['update_handle'] = self.null
                else:
                    # Treat as dynamic
                    window['topic'] = config['topic']
                    window['update_handle'] = self.handleText
                    self.startSubscriber(name, config['topic'], String)

            # Planar workspace
            if window_type == 'planar_workspace':
                window['object'] = interface.PlanarWorkspaceWindow(config)
                window['update_handle'] = self.handlePlanarWorkspaceWindow
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
                        dynamic_object['handle'] = self.handlePositionArrayLike
                        self.startSubscriber(object_name, object_topic, Float64MultiArray)

                    elif object_type == 'dynamic_point_array':
                        dynamic_object['handle'] = self.handlePositionArrayLike
                        self.startSubscriber(object_name, object_topic, Float64MultiArray)

                    window['dynamic_objects'].append(dynamic_object)

            # Append window
            self.windows.append(window)
            rospy.loginfo(f'Setup {window_type} window at index {window_index}.')

        # Setup publishers and start services
        self.setupInt64Publisher('status', 'ros_pygame_visualizer/status', Int64)
        self.startService('save_image', SaveImage, self.saveImageService)

    def saveImageService(self, req):
        if req.window == 0:
            name = 'main_screen'
            obj = self.main_screen
        elif req.window > 0:
            idx = req.window - 1
            name = self.windows[idx]['name']
            obj = self.windows[idx]['object']
        else:
            rospy.logwarn('Window request for service save_image needs to be >=0.')
            return SaveImageResponse(1)
        stamp = rospy.Time.now().to_sec()
        filename = os.path.join(self.pictures_directory, f'{name}_{stamp}.png')
        obj.save(filename)
        rospy.loginfo(f'Saved {filename}.')
        return SaveImageResponse(0)

    def handleImage(self, window):
        name = window['name']
        msg = self.getMsg(name)
        if msg is None: return
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        window['object'].setImage(cv_image)
        window['object'].reset()

    def handleJoystick(self, window):

        # Extract joy data
        try:
            msg = self.msgs[window['name']]
            horizontal = msg.axes[window['horizontal_index']]
            vertical = msg.axes[window['vertical_index']]
        except KeyError:
            topic = window['topic']
            rospy.logdebug(f'Did not receive messages on the topic {topic} yet')
            horizontal = 0.0
            vertical = 0.0

        # Update window
        window['object'].setJoy(horizontal, vertical)
        window['object'].reset()

    def handlePlanarWorkspaceWindow(self, window):
        for dynamic_object in window['dynamic_objects']:
            handle = dynamic_object['handle']
            handle(window, dynamic_object)
        window['object'].reset()

    def handleText(self, window):
        name = window['name']
        msg = self.getMsg(name)
        if msg is None: return
        window['object'].setText(msg.data)
        window['object'].reset()

    def handleDynamicPoint(self, window, dynamic_object):
        name = dynamic_object['name']
        msg = self.getMsg(name)
        if msg is None: return
        update = {
            'position': [msg.x, msg.y],
        }
        window['object'].updateDynamicObject(name, update)

    def extractPositionsFromFloat64MultiArrayMsg(self, msg):
        # Positions are contained in msg.data. Assume, len(msg.data) = 2*N,
        # where msg.data[:N] are x-axis coordinates and msg.data[N:] are y-axis
        # coordinates.
        N = int(len(msg.data)/2)
        return [[px, py] for px, py in zip(msg.data[:N], msg.data[N:])]

    def handlePositionArrayLike(self, window, dynamic_object):
        name = dynamic_object['name']
        msg = self.getMsg(name)
        if msg is None: return
        update = {
            'positions': self.extractPositionsFromFloat64MultiArrayMsg(msg)
        }
        window['object'].updateDynamicObject(name, update)

    def updateMainScreen(self):

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

        return not did_user_quit

    def spin(self):

        # Start looping
        keep_running = True
        while keep_running:

            # Main update
            keep_running_main_loop = self.updateMainScreen()
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
            self.main_loop_iter += 1

        self.main_screen.shutdown()

if __name__ == '__main__':
    Node().spin()
