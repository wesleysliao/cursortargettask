#!/usr/bin/python
import numpy as np

from ctt import InputSource, Task, Procedure, CursorTargetTaskApp

class LeftRight(Task):
    def setup(self):
        super(LeftRight, self).setup()

        self.targets.current = 2
        while self.targets.current ==2:
            self.targets.current = np.random.randint(0, len(self.targets.children))
        self.targets.current_target().active = True

    def add_inputs(self):
        self.touchinput = InputSource()
        self.cursor.add_input(self.touchinput, np.array([[0],[0],[0],[0],[0],[0]]),np.array([[self.width/2],[0],[0],[0],[0],[0]]))

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


    def update(self, dt):
        super(LeftRight, self).update(dt)

        margin = (self.targets.current_target().width-self.cursor.width)/2

        if self.targets.current_target().center_x > self.center_x:
            if self.cursor.x > (self.targets.current_target().center_x - margin):
                self.targets.current_target().hit = True
        else:
            if self.cursor.x < (self.targets.current_target().center_x + margin):
                self.targets.current_target().hit = True


class LeftRightApp(CursorTargetTaskApp):
    pass

if __name__ == '__main__':
    app = LeftRightApp()
    app.run()
