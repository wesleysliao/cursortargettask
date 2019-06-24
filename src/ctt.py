#!/usr/bin/env python

from kivy.app import App
from kivy.uix.widget import Widget
from kivy.event import EventDispatcher
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.relativelayout import RelativeLayout
from kivy.uix.screenmanager import ScreenManager, Screen, NoTransition, SlideTransition
from kivy.properties import BooleanProperty, NumericProperty, ReferenceListProperty, ObjectProperty, ListProperty, StringProperty
from kivy.vector import Vector
from kivy.clock import Clock
from kivy.graphics import Color, Line
import random

from kivy.core.window import Window
Window.size = (1080, 1080)

from kivy.config import Config
Config.set('kivy', 'log_level', 'trace')

import numpy as np

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import WrenchStamped

class InputSource(EventDispatcher):
    value = NumericProperty(0)

class ROSInputSource(InputSource):
    def __init__(self, topic, msg_type, msg_decoder, **kwargs):
        super(ROSInputSource, self).__init__(**kwargs)
        self.ros_sub = rospy.Subscriber(topic, msg_type, self.ros_msg_cb)
        self.msg_decoder = msg_decoder

    def ros_msg_cb(self, msg):
        self.value = self.msg_decoder(msg)

class Cursor(Widget):
    ontarget = BooleanProperty(False)
    ontarget_time = NumericProperty(0)
    lockin_time = NumericProperty(1)


    #  initial conditions    x     y     dx    dy    ddx   ddy
    initial = ListProperty([ 0.0,  0.0,  0.0,  0.0,  0.0,  0.0])

    #   dynamic coeff.              c   x   y  dx  dy ddx ddy
    dynamics_td = ListProperty([[   0,  0,  0,  1,  0,  0,  0], # x
                               [    0,  0,  0,  0,  1,  0,  0], # y
                               [    0,  0,  0,  0,  0,  1,  0], # dx
                               [    0,  0,  0,  0,  0,  0,  0], # dy
                               [    0,  0,  0,  0,  0,  0,  0], # ddx
                               [    0,  0,  0,  0,  0,  0,  0]])# ddy
    #   dynamic coeff.              c   x   y  dx  dy ddx ddy
    dynamics_ct = ListProperty([[   0,  1,  0,  0,  0,  0,  0], # x
                               [    0,  0,  1,  0,  0,  0,  0], # y
                               [    0,  0,  0,0.95, 0,  0,  0], # dx
                               [    0,  0,  0,  0,  0,  0,  0], # dy
                               [    0,  0,  0,  0,  0,  0,  0], # ddx
                               [    0,  0,  0,  0,  0,  0,  0]])# ddy

    def __init__(self, **kwargs):
        super(Cursor, self).__init__(**kwargs)
        self.inputdict = dict()

    def set_dynamics(self):
        self.dynamics_timedep = np.array( self.dynamics_td, dtype=float)
        self.dynamics_const  =  np.array( self.dynamics_ct, dtype=float)

    def add_input(self, inputsource, dynamics_timedep, dynamics_const):
        #record the input nubmer
        self.inputdict[inputsource] = len(self.dynamics_const[0])

        #update the dynamics matrices
        self.dynamics_const = np.hstack((self.dynamics_const, dynamics_const))
        self.dynamics_timedep = np.hstack((self.dynamics_timedep, dynamics_timedep))

        #bind
        inputsource.bind(value=self.update_input)

        self.init_state()

    def init_state(self):
        self.state = np.zeros(len(self.dynamics_const[0,:]), dtype=float)
        self.state[0] = 1
        self.state[1:7] = self.initial

    def update_input(self, obj, value):
        self.state[self.inputdict[obj]] = value

    def update(self, dt):
        for order in range(len(self.dynamics_timedep), 0, -1):
            self.state[order] = ( np.dot(self.state, self.dynamics_const[order-1,:]) 
                                  + np.dot(dt*self.state, self.dynamics_timedep[order-1,:]) )

        self.x = float(self.state[1])
        self.y = float(self.state[2])

        if self.ontarget:
            self.ontarget_time += dt
        else:
            self.ontarget_time = 0

class TargetList(RelativeLayout):
    current = NumericProperty(0)

    def current_target(self):
        return self.children[self.current]

    def next(self):
        self.children[self.current].active = False
        if self.current < len(self.children):
            self.current += 1
            self.children[self.current].active = True
        else:
            self.parent.complete()

class EndlessTargetList(TargetList):
    def add_target(self):
        self.add_widget(Target(), len(self.children))
        print(self.children)
        self.children[-1].y = self.children[-2].y + self.children[0].y
        self.children[-1].x = random.randint(100, 880)

    def next(self):
        self.add_target()
        super(EndlessTargetList, self).next()


class Target(Widget):
    active = BooleanProperty(False)
    hit = BooleanProperty(False)

class BackgroundGrid(Widget):
    def __init__(self, **kwargs):
        super(BackgroundGrid, self).__init__(**kwargs)
        self.bind(pos=self.update_canvas)
        self.update_canvas()

    def update_canvas(self, *args):
        self.canvas.clear()
        with self.canvas:
            Color(rgba=(1,1,1,0.5))
            self.draw_grid(100, 4, 100, 1, self.x-200, self.y)

    def draw_grid(self, spacing, hsize, vsize, linewidth, origin_x, origin_y):
        for hline in range(vsize+1):
            Line(points=(origin_x, origin_y+(spacing*hline), origin_x+(spacing*hsize), origin_y+(spacing*hline)), width=linewidth)
        for vline in range(hsize+1):
            Line(points=(origin_x+(spacing*vline), origin_y, origin_x+(spacing*vline), origin_y+(spacing*vsize)), width=linewidth)

    def draw_grid_centered(self, spacing, hsize, vsize, linewidth):
        offset_x = self.x-((hsize*spacing)/2)
        offset_y = self.y-((vsize*spacing)/2)
        self.draw_grid(spacing, hsize, vsize, linewidth, offset_x, offset_y)

class TimedScreen(Screen):
    timeout = NumericProperty(0)

    def __init__(self, **kwargs):
        super(TimedScreen, self).__init__(**kwargs)
        self.initialized = False

    def setup(self):
        if(self.timeout != 0):
            Clock.schedule_once(lambda dt: self.complete(), self.timeout)
        self.initialized = True

    def complete(self):
        print('complete')
        self.manager.current = self.manager.next()

    def update(self, dt):
        if not self.initialized:
            self.setup()

    def on_enter(self):
        print(self.size)


class TextScreen(TimedScreen):
    text = StringProperty()

class Task(TimedScreen):
    cursor = ObjectProperty()
    targets = ObjectProperty()

    def setup(self):
        self.cursor.set_dynamics()
        self.add_inputs()

        super(Task, self).setup()

    def add_inputs(self):
        self.cursor.init_state()

    def update(self, dt):
        super(Task, self).update(dt)
        self.cursor.update(dt)


class Measure(EventDispatcher):
    value = NumericProperty(0)
    def __init__(self, topic, msg_type, **kwargs):
        super(Measure, self).__init__(**kwargs)
        self.ros_pub = rospy.Publisher(topic, msg_type, queue_size=10)
    
    def update_value(self, obj, value):
        pass
        

class Procedure(ScreenManager):
    def __init__(self, **kwargs):
        super(Procedure, self).__init__(transition = NoTransition(),**kwargs)

    def update(self, dt):
        self.update_state(dt)
        self.update_measures(dt)

    def update_state(self, dt):
        try:
            self.current_screen.update(dt)
        except Exception as error:
            print(self.current, error)

    def update_measures(self, dt):
        pass

class CursorTargetTaskApp(App):
    def build(self):
        rospy.init_node('CursorTargetTask', anonymous=True)
        rospy.on_shutdown(self.stop)

        procedure = Procedure()
        Clock.schedule_interval(procedure.update, 1.0 / 60.0)
        return procedure 

if __name__ == '__main__':
    app = CursorTargetTaskApp()
    app.run()
