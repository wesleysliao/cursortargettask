#!/usr/bin/python

from ctt import Task, Procedure, CursorTargetTaskApp

class LeftRight(Task):
    pass

class LeftRightApp(CursorTargetTaskApp):
    pass

if __name__ == '__main__':
    app = LeftRightApp()
    app.run()
