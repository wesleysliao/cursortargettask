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

    def update(self, dt, t):
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

    def update(self, dt, t):
        self.current_target().update(dt, t)

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

    def update(self, dt, t):
        pass


class TimeDepTarget(Target):
    def position_fn(self, t):
        x = 0
        y = 0
        return x, y

    def update(self, dt, t):
        x, y = self.position_fn(t)

        self.x = x
        self.y = y

        print t, x, self.x

class TimedScreen(Screen):
    timeout = NumericProperty(0)
    start_t = NumericProperty(0)
    elapsed_t = NumericProperty(0)

    def __init__(self, **kwargs):
        super(TimedScreen, self).__init__(**kwargs)
        self.initialized = False

    def setup(self):
        self.start_t = Clock.get_time()
        self.initialized = True

    def complete(self):
        print('complete')
        self.manager.current = self.manager.next()

    def update(self, dt):
        if not self.initialized:
            self.setup()
        self.elapsed_t = Clock.get_time() - self.start_t

        if self.timeout > 0 and self.elapsed_t >= self.timeout:
            self.complete()

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
        self.cursor.update(dt, self.elapsed_t)
        self.targets.update(dt, self.elapsed_t)


class Procedure(ScreenManager):
    def __init__(self, **kwargs):
        super(Procedure, self).__init__(transition = NoTransition(),**kwargs)

    def update(self, dt):
        self.update_state(dt)
	self.publish_state(dt)

    def update_state(self, dt):
        try:
            self.current_screen.update(dt)
        except Exception as error:
            print(self.current, error)

    def publish_state(self,dt):
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
