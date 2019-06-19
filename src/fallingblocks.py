#!/usr/bin/python
from ctt import Task, Procedure, CursorTargetTaskApp

class CatchFallingBlocks(Task):
    def update(self, dt):
        if self.initialized:
            self.y = 400-self.cursor.y
            if self.cursor.y > self.targets.current_target().top:
                self.targets.next()

            if self.cursor.collide_widget(self.targets.current_target()):
                self.cursor.ontarget = True
                if self.cursor.ontarget_time >= self.cursor.lockin_time:
                    self.targets.current_target().hit = True
            else:
                if not self.targets.current_target().hit:
                    self.cursor.ontarget = False
        super(CatchFallingBlocks, self).update(dt)

class FallingBlocksApp(CursorTargetTaskApp):
    pass

if __name__ == '__main__':
    app = FallingBlocksApp()
    app.run()
