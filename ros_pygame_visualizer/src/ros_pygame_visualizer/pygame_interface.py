import pygame
pygame.init()

def scaleInt(a, x):
    return int(round(a * float(x)))

class BaseObject:

    def __init__(self, config):

        # Save config
        self.config = config

        # Setup surface
        width = self.config['width']
        if 'height' in self.config.keys():
            height = self.config['height']
        else:
            height = width
        dims = (width, height)
        self.surface = pygame.Surface(dims)

        # Save width/height
        self.width = width
        self.height = height

        # Setup static surface
        self.static_surface = pygame.Surface(dims)

        # Extract background color
        background_color = self.inConfig('background_color')
        if background_color is not None:
            background_color = pygame.Color(background_color)
        else:
            background_color = pygame.Color('white')
        self.static_surface.fill(background_color)

    def inConfig(self, key):
        if key in self.config.keys():
            return self.config[key]
        else:
            return None

    def saveScreen(self, filename):
        pygame.image.save(self.surface, filename)

    def blit(self, surface, position):
        self.surface.blit(surface, position)

    def resetStatic(self):
        # Q: Which line is quicker from below?
        # self.surface.blit(self.static_surface, (0, 0))
        # self.surface = self.static_surface.copy()
        #
        # Results:
        #
        # n [1]: import pygame
        # pygame 2.0.0.dev10 (SDL 2.0.12, python 3.8.5)
        # Hello from the pygame community. https://www.pygame.org/contribute.html
        #
        # In [2]: pygame.init()
        # Out[2]: (6, 0)
        #
        # In [4]: static_surface = pygame.Surface((100, 100))
        #
        # In [5]: %timeit surface = static_surface.copy()
        # 12.8 µs ± 165 ns per loop (mean ± std. dev. of 7 runs, 100000 loops each)
        #
        # In [7]: surface = static_surface.copy()
        #
        # In [9]: %timeit surface.blit(static_surface, (0, 0))
        # 11.8 µs ± 729 ns per loop (mean ± std. dev. of 7 runs, 100000 loops each)
        #
        # In [10]: surface = static_surface.copy()
        #
        # In [11]: %timeit surface = static_surface.copy()
        # 12.5 µs ± 77.5 ns per loop (mean ± std. dev. of 7 runs, 100000 loops each)
        #
        # In [12]: %timeit surface.blit(static_surface, (0, 0))
        # 11.3 µs ± 50.1 ns per loop (mean ± std. dev. of 7 runs, 100000 loops each)
        #
        # In [13]: static_surface = pygame.Surface((1000, 1000))
        #
        # In [14]: %timeit surface = static_surface.copy()
        # 344 µs ± 1.46 µs per loop (mean ± std. dev. of 7 runs, 1000 loops each)
        #
        # In [15]: surface = static_surface.copy()
        #
        # In [16]: %timeit surface.blit(static_surface, (0, 0))
        # 215 µs ± 1.35 µs per loop (mean ± std. dev. of 7 runs, 1000 loops each)
        self.blit(self.static_surface, (0, 0))

    def resetBaseObject(self):
        self.resetStatic()

    def drawRectangle(self, surface, color, width, height, position, border_width=0, border_radius=0):

        # Setup rect object
        left = position[0]
        top = position[1]
        rect = pygame.Rect(left, top, width, height)

        # Draw rectangle
        pygame.draw.rect(surface, pygame.Color(color), rect, width=border_width, border_radius=border_radius)

    def drawCircle(self, surface, color, position, radius, border_width=0):
        pygame.draw.circle(surface, pygame.Color(color), tuple(position), radius, width=border_width)

    def drawLine(self, surface, color, start, end, width=1):
        pygame.draw.line(surface, pygame.Color(color), start, end, width=width)

    def drawTrajectory(self, surface, color, positions, width=1):
        pygame.draw.lines(surface, pygame.Color(color), False, positions, width)

class PlanarWorkspaceWindow(BaseObject):

    def __init__(self, config):

        # Define height (note, user only must define real width/height and the
        # width in pixels)
        self.real_width = float(config['real_width'])
        self.real_height = float(config['real_height'])
        self.px_per_m = float(config['width']) / self.real_width
        self.m_per_px = 1.0/self.px_per_m
        config['height'] = scaleInt(self.real_height, self.px_per_m)
        print("Creating planar workspace object with dimensions (%d, %d)" % (config['width'], config['height']))

        # Main setup
        super().__init__(config)

        # Init objects
        self.dynamic_objects = {}
        for full_object_config in config['objects']:

            object_type = list(full_object_config.keys())[0]
            object_config = list(full_object_config.values())[0]

            if object_type == 'dynamic_point':

                # Create object specification from config
                spec = {
                    'type': object_type,
                    # 'position': [0, 0],
                    'trail': [],
                    'color': object_config['color'],
                    'radius': scaleInt(self.px_per_m, object_config['real_radius']),
                    'reset_handle': self.resetDynamicPoint,
                    'update_handle': self.updateDynamicPoint,
                }
                if 'show_trail' in object_config.keys():
                    spec['show_trail'] = object_config['show_trail']
                    spec['width'] = scaleInt(0.02, self.width)
                else:
                    spec['show_trail'] = False
                if 'paint' in object_config.keys():
                    spec['paint'] = object_config['paint']
                else:
                    spec['paint'] = False

                # Save object
                name = object_config['name']
                self.dynamic_objects[name] = spec

            elif object_type == 'static_line':
                real_start = object_config['real_start']
                real_end = object_config['real_end']
                color = object_config['color']
                width = object_config['width']
                start = self.toPygameCoordinates(real_start)
                end = self.toPygameCoordinates(real_end)
                self.drawLine(self.static_surface, color, start, end, width=width)

            elif object_type == 'static_point':
                real_position = object_config['real_position']
                color = object_config['color']
                real_radius = object_config['real_radius']
                position = self.toPygameCoordinates(real_position)
                radius = scaleInt(self.px_per_m, real_radius)
                self.drawCircle(self.static_surface, color, position, radius)

    def toPygameCoordinates(self, position):
        return [scaleInt(self.px_per_m, position[0]), scaleInt(self.px_per_m, position[1])]

    def resetDynamicPoint(self, spec):
        if spec['show_trail'] and len(spec['trail']) > 2:
            self.drawTrajectory(self.surface, spec['color'], spec['trail'], width=spec['width'])
        if spec['paint'] and 'position' in spec:
            # Note, this won't be seen till next iteration so the current
            # position will always have to be drawn to self.surface too
            self.drawCircle(self.static_surface, spec['color'], spec['position'], spec['radius'])
        if 'position' in spec:
            self.drawCircle(self.surface, spec['color'], spec['position'], spec['radius'])

    def updateDynamicPoint(self, name, update):
        position = self.toPygameCoordinates(update['position'])
        self.dynamic_objects[name]['position'] = position
        if self.dynamic_objects[name]['show_trail']:
            self.dynamic_objects[name]['trail'].append(position)

    def updateDynamicObject(self, name, update):
        handle = self.dynamic_objects[name]['update_handle']
        handle(name, update)

    def reset(self):
        self.resetBaseObject()
        for spec in self.dynamic_objects.values():
            handle = spec['reset_handle']
            handle(spec)


class JoystickWindow(BaseObject):

    base_color = 'grey'
    tip_color = 'black'

    def __init__(self, config):

        # Main setup
        super().__init__(config)

        # Draw static features
        middle = scaleInt(0.5, self.width)
        self.centre_position = (middle, middle)
        radius = scaleInt(0.09, self.width)
        self.drawCircle(self.static_surface, self.base_color, self.centre_position, radius)

        # Compute joy visual parameters
        self.line_width = scaleInt(0.075, self.width)
        self.tip_radius = scaleInt(0.2*0.5, self.width)
        self.virtual_width = self.width - 2*self.tip_radius

        # Setup joy axes
        def handleDirection(label):
            direction = self.inConfig(label)
            if direction is None:
                direction = 1
            setattr(self, label, float(direction))
        handleDirection('horizontal_direction')
        handleDirection('vertical_direction')

    def toPygameCoordinate(self, j):
        return scaleInt(0.5 * (j+1.0), self.virtual_width) + self.tip_radius

    def setJoy(self, horizontal, vertical):
        xj = self.horizontal_direction * float(horizontal)  # j - joystick
        yj = self.vertical_direction * float(vertical)
        xs = self.toPygameCoordinate(xj)  # s - surface
        ys = self.toPygameCoordinate(yj)
        self.position = (xs, ys)

    def reset(self):
        self.resetBaseObject()
        self.drawLine(self.surface, self.base_color, self.centre_position, self.position, width=self.line_width)
        self.drawCircle(self.surface, self.tip_color, self.position, self.tip_radius)

class MainScreen(BaseObject):

    def __init__(self, config, frequency):
        super().__init__(config)

        # Setup main screen
        self.screen = pygame.display.set_mode(self.surface.get_size())

        if self.inConfig('caption'):
            caption = config['caption']
        else:
            caption = 'ros_pygame_visualizer'
        pygame.display.set_caption(caption)

        # Setup main clock
        self.frequency = frequency
        self.clock = pygame.time.Clock()

    def reset(self):
        self.events = pygame.event.get()
        self.resetBaseObject()

    def didUserQuit(self):
        return any(e.type==pygame.QUIT for e in self.events)

    def update(self):
        self.screen.blit(self.surface, (0, 0))
        pygame.display.flip()
        self.sleep()

    def sleep(self):
        self.clock.tick(self.frequency)

    def shutdown(self):
        pygame.quit()
