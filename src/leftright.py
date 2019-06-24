#!/usr/bin/python
import numpy as np
from kivy.clock import Clock

from ctt import InputSource, Task, Procedure, CursorTargetTaskApp

class LeftRight(Task):
    def setup(self):
        super(LeftRight, self).setup()
        self.tracking_target = False
        self.returning_center = False
        Clock.schedule_once(self.set_target_random, 0.5+(2*np.random.random()))

    def add_inputs(self):
        self.touchinput = InputSource()
        self.cursor.add_input(self.touchinput, np.array([[0],[0],[0],[0],[0],[0]]),np.array([[self.get_root_window().width/2],[0],[0],[0],[0],[0]]))

    def on_touch_down(self, touch):
        x = (touch.x-self.center_x)/(self.width/2)
        self.touchinput.value =x
        print("td", self.touchinput.value)


    def on_touch_up(self, touch):
        self.touchinput.value = 0
        print("td", self.touchinput.value)


    def on_touch_move(self, touch):
        self.on_touch_down(touch)
        print("td", self.touchinput.value)


    def set_target_random(self, dt):
        self.targets.current = 2
        while self.targets.current ==2:
            self.targets.current = np.random.randint(0, len(self.targets.children))
        self.targets.current_target().active = True
        self.tracking_target = True
        self.returning_center = False

    def deactivate_current_target(self, dt):
        self.targets.current_target().active = False

    def set_target_center(self, dt):
        self.targets.current_target().active = False
        self.targets.current = 2
        self.targets.children[2].active = True
        self.tracking_target = False
        self.returning_center = True


    def update(self, dt):
        super(LeftRight, self).update(dt)

        margin = (self.targets.current_target().width-self.cursor.width)/2

        if self.tracking_target:
            if ((self.cursor.x > (self.targets.current_target().center_x - margin))
                and (self.cursor.x < (self.targets.current_target().center_x + margin))):
                self.targets.current_target().hit = True
                Clock.schedule_once(self.deactivate_current_target, 0.25)
                Clock.schedule_once(self.set_target_center, 1)
                self.tracking_target = False

        if self.returning_center:
            if ((self.cursor.x > (self.targets.current_target().center_x - margin))
                and (self.cursor.x < (self.targets.current_target().center_x + margin))):
                self.targets.current_target().hit = True
                Clock.schedule_once(lambda dt: self.complete(), 0.1)
                self.returning_center = False



class LeftRightApp(CursorTargetTaskApp):
    pass

if __name__ == '__main__':
    app = LeftRightApp()
    app.run()
