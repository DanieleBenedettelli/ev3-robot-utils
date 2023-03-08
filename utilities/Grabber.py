from pybricks.ev3devices import Motor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch


class Grabber(Motor):

    def __init__(self, port):
        super().__init__(port)

    def calibrate(self):
        self.control.limits(speed=1000, acceleration=400)
        self.run_until_stalled(-200, then=Stop.HOLD, duty_limit=40)
        #wait(100)
        self.stop()
        wait(100)
        self.reset_angle(angle = 0)

    def prepareForGrabbing(self, waitForCompletion=True):
        self.run_target(speed=400,target_angle=280, then=Stop.BRAKE, wait=waitForCompletion)

    def grab(self, waitForCompletion=True):
        self.run_target(speed=400,target_angle=-118, then=Stop.BRAKE, wait=waitForCompletion)

    def unload(self, waitForCompletion=True):
        self.run_target(speed=400,target_angle=-140, then=Stop.BRAKE, wait=waitForCompletion)

    def retract(self, waitForCompletion=True):
        self.run_target(speed=400,target_angle=1, then=Stop.BRAKE, wait=waitForCompletion)        

    def prepareHook(self, waitForCompletion=True):
        self.run_target(speed=400,target_angle=313, then=Stop.BRAKE, wait=waitForCompletion)        

    def unloadBuffer(self, waitForCompletion=True):
        self.run_target(speed=400,target_angle=400, then=Stop.BRAKE, wait=waitForCompletion)  