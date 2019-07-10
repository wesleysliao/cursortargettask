#!/usr/bin/python

import numpy as np
from kivy.clock import Clock
import rospy
from geometry_msgs.msg import WrenchStamped

from ctt import InputSource, ROSInputSource, Task, Procedure, CursorTargetTaskApp
from cursortargettask.msg import EggDropState

class EggDrop(Task):
    def setup(self):
        super(EggDrop, self).setup()

    def add_inputs(self):
        self.touchinput = InputSource()
	    #self.forceinput = ROSInputSource("fse103", WrenchStamped, lambda msg: msg.wrench.force.x)
        self.cursor.add_input(self.touchinput, np.array([[0],[0],[0],[0],[0],[0]]),np.array([[0],[self.get_root_window().height/2],[0],[0],[0],[0]]))
        #self.cursor.add_input(self.forceinput, np.array([[0],[0],[0],[0],[0],[0]]),np.array([[-7],[0],[0],[0],[0],[0]]))

    def on_touch_down(self, touch):
        y = (touch.y-self.center_y)/(self.height/2)
        self.touchinput.value =y
        print("td", self.touchinput.value)

    def on_touch_up(self, touch):
        self.touchinput.value = 0
        print("td", self.touchinput.value)

    def on_touch_move(self, touch):
        self.on_touch_down(touch)
        print("td", self.touchinput.value)

    def update(self, dt):
        super(EggDrop, self).update(dt)

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
            p1_cursor_y = 1
            p2_cursor_y = 2
            reference_y = 0
        print(timestamp, screen_index, p1_cursor_y, p2_cursor_y, reference_y)
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
