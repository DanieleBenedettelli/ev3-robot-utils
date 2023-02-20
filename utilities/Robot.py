""" Line Following navigator

Author  : Daniele Benedettelli
Date    : February 2023
Version : 0.1

"""

from pybricks.ev3devices import ColorSensor, Motor
from pybricks.parameters import Port, Color, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait
from math import atan2

class Side:
    LEFT = 0
    RIGHT = 1

class Robot(DriveBase):
    """ This class contains all general functionalities of Pixy.

    Keyword arguments:
    port1        -- Port which the camera is connected to (Port.S1 ...)

    """

    def __init__(self, left_motor, right_motor, wheel_diameter, axle_track, sensor1, sensor2 ):
        self.sensorRight = sensor1 # for line following
        self.sensorLeft = sensor2 # for intersections
        self.sensorLine = self.sensorRight
        self.sensorIntersection = self.sensorLeft
        self.gain = -0.7
        self.blackThreshold = 20
        self.integralError = 0
        self.travelSpeed = 100 # mm/s
        self.turnSpeed = 90
        self.wheelbase = axle_track
        self.wheelDiameter = wheel_diameter
        self.left_motor = left_motor
        self.right_motor = right_motor
        #initialize super class DriveBase
        super().__init__(left_motor, right_motor, wheel_diameter, axle_track)
  
    def settings(self, straight_speed, straight_acceleration, turn_rate, turn_acceleration) :
        self.travelSpeed = straight_speed
        self.turnSpeed = turn_rate
        super().settings(straight_speed, straight_acceleration, turn_rate, turn_acceleration) 

    def __subang(self,a,b):
        d = a-b
        while d>180:
            d -= 360
        while d<-180:
            d += 360
        return d

    def arc(self, radius, angle, speed=100) :
        if (abs(radius)<2) :
            self.turn(-angle)
        else: 
            angular_speed = -57.295*speed/radius #deg/s
            #print("drive speed: %.2f mm/s"%speed)
            #print("angular speed: %.2f deg/s"%angular_speed)
            heading0 = self.angle()
            self.drive(speed, angular_speed)
            #diff = self.__subang(self.angle(), heading0)
            while abs(self.__subang(self.angle(), heading0)) < abs(angle) :
                #print("ang diff: %.2f deg"%diff)
                #diff = self.__subang(self.angle(), heading0)
                wait(1)
            self.straight(0)

    def Scurve(self, dx, dy, speed=100):
        # Wolfram Alpha: solve [ y/2=R*sin(a), x/2 = R*(1-cos(a)) ] for R,a
        angle = 2*atan2(dx, dy)*57.296
        radius = (dx*dx+dy*dy)/(4*dx)
        #print("angle: %.2f deg"%angle)
        #print("radius: %.2f mm"%radius)
        self.arc(radius, angle,speed)
        self.arc(-radius, angle, speed)


    def spin(self, angle, stopType=Stop.BRAKE, waitForCompletion=True) :
        self.turn(-angle)# = -angle, then = stopType, wait= waitForCompletion)

    def __lineFollowCore(self) :
        e = self.target - self.sensorLine.reflection()
        steer = self.gain * e
        self.drive(self.travelSpeed,steer)


    def lineFollowerSettings(self, speed, target, gain, darkThreshold = 10, whichSensor=Side.RIGHT, whichBorder = Side.LEFT) :
        if whichSensor is Side.LEFT:
            sensorLine = self.sensorLeft
            sensorIntersection = self.sensorRight
        else :
            sensorLine = self.sensorRight
            sensorIntersection = self.sensorLeft
        self.gain = gain
        if whichBorder is Side.LEFT:
            self.gain = -gain
        
        self.blackThreshold = darkThreshold     
        self.target = target       

    def seguiLineaFinoAIncrocio(self):
        self.integralError = 0
        while self.sensorIntersection.reflection() > self.blackThreshold:
            self.__lineFollowCore()
        self.straight(0) # stop con frenata
        
    def seguiLineaPerDistanza(self, distanza):
        self.integralError = 0
        self.reset() # reset distance
        while self.distance() < distanza :
            self.__lineFollowCore()
        self.straight(0) # stop con frenata
