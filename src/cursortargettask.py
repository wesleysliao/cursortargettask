#!/usr/bin/env python

from kivy.app import App
from kivy.uix.widget import Widget
from kivy.event import EventDispatcher
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.relativelayout import RelativeLayout
from kivy.properties import BooleanProperty, NumericProperty, ReferenceListProperty, ObjectProperty
from kivy.vector import Vector
from kivy.clock import Clock
from kivy.graphics import Color, Line

import numpy as np

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3

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
    def __init__(self, **kwargs):
        super(Cursor, self).__init__(**kwargs)

        self.inputdict = dict()

        #  initial conditions      x   y  dx  dy ddx ddy
        self.initial = np.array( [ 0,  0,  0,  0,  0,  0], dtype=float)
        self.set_dynamics()
        self.init_state()


    def set_dynamics(self):
        #   dynamic coeff.                       c   x   y  dx  dy ddx ddy
        self.dynamics_timedep = np.array( [[    0,  0,  0,  1,  0,  0,  0], # x
                                           [  100,  0,  0,  0,  1,  0,  0], # y
                                           [    0,  0,  0,  0,  0,  1,  0], # dx
                                           [    0,  0,  0,  0,  0,  0,  0], # dy
                                           [    0,  0,  0,  0,  0,  0,  0], # ddx
                                           [    0,  0,  0,  0,  0,  0,  0]],# ddy
                                          dtype=float)

        #                                    c   x   y  dx     dy ddx ddy
        self.dynamics_const  =  np.array( [[ 0,  1,  0,  0,     0,  0,  0], # x
                                           [ 0,  0,  1,  0,     0,  0,  0], # y
                                           [ 0,  0,  0,  0.95,  0,  0,  0], # dx
                                           [ 0,  0,  0,  0,     0,  0,  0], # dy
                                           [ 0,  0,  0,  0,     0,  0,  0], # ddx
                                           [ 0,  0,  0,  0,     0,  0,  0]],# ddy
                                          dtype=float)

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

class Target(Widget):
    pass

class Path(Target):
    def __init__(self, **kwargs):
        super(Path, self).__init__(**kwargs)
        self.set_path()
        self.bind(pos=self.update_canvas)
        self.update_canvas()

    def update_canvas(self, *args):
        self.canvas.clear()
        with self.canvas:
            Color(rgba=(0,0,1,1))
            Line(points = np.ravel(self.points).tolist(), width = 4)

    def interp_x(self, y):
        return np.interp(y, self.points[:,1], self.points[:,0])

    def interp_y(self, x):
        return np.interp(x, self.points[:,0], self.points[:,1])

    def set_path(self):
        self.points = np.array([[   0,   0],
                                [   0, 100],
                                [ 100, 200],
                                [ 100, 300],
                                [   0, 400],
                                [-100, 500],
                                [-100, 600],
                                [-200, 700],
                                [-200, 800],
                                [   0, 900],
                                [   0,1000],
                                [   0,1100],
                                [   0,1200],
                                [   0,1300],
                                [ 100,1400],
                                [   0,1500],
                                [   0,1600],
                                [   0,1700],
                                [   0,1800],
                                [   0,1900]])

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

class Field(RelativeLayout):
    target = ObjectProperty()
    cursor = ObjectProperty()
    bggrid = ObjectProperty()

    def __init__(self, **kwargs):
        super(Field, self).__init__(**kwargs)

    def update(self, dt):
        self.cursor.update(dt)
        self.y = -self.cursor.y
        # self.x = (self.parent.width/2)-self.target.interp_x(self.cursor.y)

class Measure(EventDispatcher):
    value = NumericProperty(0)
    def __init__(self, topic, msg_type, **kwargs):
        super(Measure, self).__init__(**kwargs)
        self.ros_pub = rospy.Publisher(topic, msg_type, queue_size=10)
    
    def update_value(self, obj, value):
        pass
        



class CursorTargetTaskGame(Widget):
    field = ObjectProperty()

    def __init__(self):
        super(CursorTargetTaskGame, self).__init__()
        self.ros_statpub = rospy.Publisher('cursortargettask', String, queue_size=10)

        self.touchinput = InputSource()
        self.ros_joy_x = ROSInputSource("joy", Joy, lambda a : a.axes[0])
        self.ros_force_x = ROSInputSource("FSE103/force", Vector3, lambda a : a.x)

    def setup_objects(self):
        # self.field.cursor.add_input(self.touchinput, np.array([[0],[0],[0],[0],[0],[0]]),np.array([[0],[0],[0],[0],[1000],[0]]),)
        # self.field.cursor.add_input(self.ros_force_x, np.array([[0],[0],[0],[0],[0],[0]]),np.array([[0],[0],[0],[0],[100],[0]]),)
        self.field.cursor.add_input(self.ros_joy_x, np.array([[0],[0],[0],[0],[0],[0]]),np.array([[0],[0],[0],[0],[-1000],[0]]),)


    def on_touch_down(self, touch):
        x = (touch.x-self.center_x)/(self.width/2)
        self.touchinput.value =x

    def on_touch_up(self, touch):
       self.touchinput.value = 0

    def on_touch_move(self, touch):
        self.on_touch_down(touch)

    def update(self, dt):
        self.update_state(dt)
        self.update_measures(dt)

        # self.publish_state(dt)
        # self.publish_measures(dt)

    def update_state(self, dt):
        self.field.update(dt)

    def update_measures(self, dt):
        pass

class CursorTargetTaskApp(App):
    def build(self):
        rospy.init_node('CursorTargetTask', anonymous=True)
        rospy.on_shutdown(self.stop)

        game = CursorTargetTaskGame()
        game.setup_objects()
        Clock.schedule_interval(game.update, 1.0 / 60.0)
        return game

if __name__ == '__main__':
    app = CursorTargetTaskApp()
    app.run()