#!/usr/bin/python

import numpy as np
from kivy.clock import Clock
from kivy.properties import ObjectProperty, NumericProperty
import rospy
import tf
from geometry_msgs.msg import WrenchStamped

from ctt import InputSource, ROSInputSource, Task, Procedure, CursorTargetTaskApp
from cursortargettask.msg import EggDropState

class EggDrop(Task):
    p2_cursor = ObjectProperty()
    cursor_diff = NumericProperty()
    cursor_center = NumericProperty()

    def setup(self):
        self.p2_cursor.set_dynamics()
        super(EggDrop, self).setup()

    def add_inputs(self):
        self.touchinput_x = InputSource()
        self.touchinput_y = InputSource()
        self.forceinput_p1 = ROSInputSource("load_cell_1", WrenchStamped, lambda msg: msg.wrench.force.x)
        self.forceinput_p2 = ROSInputSource("load_cell_2", WrenchStamped, lambda msg: msg.wrench.force.x)

        self.encoderinput = InputSource()
        self.encoderlistener = tf.TransformListener()

        self.cursor.add_input(self.touchinput_y, np.array([[0],[0],[0],[0],[0],[0]]),np.array([[0],[self.get_root_window().height/2],[0],[0],[0],[0]]))
        self.p2_cursor.add_input(self.touchinput_x, np.array([[0],[0],[0],[0],[0],[0]]),np.array([[0],[self.get_root_window().height/2],[0],[0],[0],[0]]))
        self.cursor.add_input(self.forceinput_p1, np.array([[0],[0],[0],[0],[0],[0]]),np.array([[0],[3],[0],[0],[0],[0]]))
        self.p2_cursor.add_input(self.forceinput_p2, np.array([[0],[0],[0],[0],[0],[0]]),np.array([[0],[-3],[0],[0],[0],[0]]))

        self.cursor.add_input(self.encoderinput, np.array([[0],[0],[0],[0],[0],[0]]),np.array([[0],[2*self.get_root_window().height],[0],[0],[0],[0]]))
        self.p2_cursor.add_input(self.encoderinput, np.array([[0],[0],[0],[0],[0],[0]]),np.array([[0],[2*self.get_root_window().height],[0],[0],[0],[0]]))

    def on_touch_down(self, touch):
        x = (touch.x-self.center_x)/(self.width/2)
        self.touchinput_x.value = x
        y = (touch.y-self.center_y)/(self.height/2)
        self.touchinput_y.value =y

    def on_touch_up(self, touch):
        self.touchinput_x.value = 0
        self.touchinput_y.value = 0

    def on_touch_move(self, touch):
        self.on_touch_down(touch)

    def update(self, dt):
        super(EggDrop, self).update(dt)
        self.p2_cursor.update(dt)

        try:
            (trans,rot) = self.encoderlistener.lookupTransform('/baseboard', '/slider', rospy.Time(0))
            print(trans, rot)
            self.encoderinput.value = trans[1]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as error:
            print(error)

class EggDropProcedure(Procedure):
    def __init__(self, **kwargs):
        super(EggDropProcedure, self).__init__(**kwargs)
	self.pub = rospy.Publisher('eggdroptrial', EggDropState, queue_size=10)

    def publish_state(self,dt):
        timestamp = rospy.get_rostime()
        p1_cursor_y = None
        p2_cursor_y = None
        reference_y = None
        screen_index = self.screen_names.index(self.current)

        if isinstance( self.current_screen, EggDrop):
            p1_cursor_y = self.current_screen.cursor.y
            p2_cursor_y = self.current_screen.p2_cursor.y
            reference_y = 0
        print(timestamp, screen_index, p1_cursor_y, p2_cursor_y, reference_y)
        print(self.current_screen.cursor_diff, self.current_screen.cursor_center)
        self.pub.publish(EggDropState(timestamp, p1_cursor_y, p2_cursor_y, reference_y, screen_index))


class EggDropApp(CursorTargetTaskApp):
    def build(self):
        rospy.init_node('EggDropTask', anonymous=True)
        rospy.on_shutdown(self.stop)

        procedure = EggDropProcedure()
        Clock.schedule_interval(procedure.update, 1.0 / 60.0)
        return procedure 

if __name__ == '__main__':
    app = EggDropApp()
    app.run()
