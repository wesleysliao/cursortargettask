#!/usr/bin/python
import numpy as np

from ctt import InputSource, Task, Procedure, CursorTargetTaskApp

class LeftRight(Task):
    def add_inputs(self):
        self.touchinput = InputSource()

        self.cursor.add_input(self.touchinput, np.array([[0],[0],[0],[0],[0],[0]]),np.array([[self.width/3],[0],[0],[0],[0],[0]]))
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


class LeftRightApp(CursorTargetTaskApp):
    pass

if __name__ == '__main__':
    app = LeftRightApp()
    app.run()
