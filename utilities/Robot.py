""" Line Following navigator

Author  : Daniele Benedettelli
Date    : February 2023
Version : 0.1

"""
from pybricks.ev3devices import ColorSensor, Motor, GyroSensor
from pybricks.parameters import Port, Color, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from pybricks.iodevices import AnalogSensor

from math import atan2

class Side:
    LEFT = 0
    RIGHT = 1

class Robot(DriveBase):
    """ This class contains all functionality of DriveBase + my stuff

    Keyword arguments:

    """

    def __init__(self, left_motor, right_motor, wheel_diameter, axle_track, colorSensor1, colorSensor2, blockSensor, gyroSensorPort ):
        self.sensorRight = colorSensor1 # for line following
        self.sensorLeft = colorSensor2 # for intersections
        self.sensorLine = self.sensorRight
        self.sensorIntersection = self.sensorLeft
        self.blockSensor = blockSensor
        self.gyro = GyroSensor(gyroSensorPort)
        self.target = 50
        self.gain = -0.7
        self.blackThreshold = 20
        self.integralError = 0
        self.travelSpeed = 100 # mm/s
        self.lineFollowSpeed = self.travelSpeed
        self.turnSpeed = 90
        self.wheelbase = axle_track
        self.wheelDiameter = wheel_diameter
        self.left_motor = left_motor
        self.right_motor = right_motor
        #initialize super class DriveBase
        super().__init__(left_motor, right_motor, wheel_diameter, axle_track)
        self.resetGyro()
  
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

    def saturate(self, speed, max_speed=1000):
        """ Limit speed in range [-1000,1000] """
        if speed > max_speed:
            speed = max_speed
        elif speed < -max_speed:
            speed = -max_speed
        return speed

    def resetGyro(self):
        self.gyro.reset_angle(0)
        wait(500)
        print("gyro reset...",end="")
        while self.gyro.angle()!=0 :
            # hard reset (simulate disconnection and reconnection)
            dummy = AnalogSensor(Port.S4)    
            wait(500)
            self.gyro = GyroSensor(Port.S4)
            self.gyro.reset_angle(0)
        print("done!")

    def readGyro(self):
        a = self.gyro.angle()
        heading = -((a+180)%360-180)
        return heading

    # uses unregulated motors
    def headTo(self, angle=0, useGyro=True):
        integral = 0
        e = angle - self.readGyro()
        if not useGyro:
            self.turn(e)
            return
        #done = False
        timerDone = StopWatch()
        MAX_CMD = 100 # percent power
        preSat = 0
        angSpeed = 0
        self.stop()
        while timerDone.time() < 300:
            e = angle - self.gyro.angle()
            if abs(preSat)>MAX_CMD and angSpeed*integral > 0 : # segni concordi
                integral = 0
            integral += e
            #integral = saturate(integral, 360)
            #print("I:",integral)
            preSat = 3*e + 0.02*integral
            angSpeed = self.saturate(preSat, MAX_CMD)
            #print("U:",angSpeed)
            self.right_motor.dc(-angSpeed)
            self.left_motor.dc(angSpeed)
            if abs(e) > 5:
                timerDone.reset()
            wait(20)
        self.stop()

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

    def __lineFollowCore(self, speed) :
        e = self.target - self.sensorLine.reflection()
        steer = self.gain * e
        self.drive(speed,steer)


    def lineFollowerSettings(self, speed, target, gain, darkThreshold = 10, whichSensor=Side.RIGHT, whichBorder = Side.LEFT) :
        if whichSensor is Side.LEFT:
            sensorLine = self.sensorLeft
            sensorIntersection = self.sensorRight
        else :
            sensorLine = self.sensorRight
            sensorIntersection = self.sensorLeft
        self.lineFollowSpeed = speed            
        self.gain = gain
        if whichBorder is Side.LEFT:
            self.gain = -gain
        
        self.blackThreshold = darkThreshold     
        self.target = target       

    def seguiLineaFinoAIncrocio(self, thr = 24, speed = None):
        self.integralError = 0
        if speed is None:
            spd = self.lineFollowSpeed 
        else:
            spd = speed
        while self.sensorIntersection.reflection() > thr:
            print(self.sensorIntersection.reflection())
            self.__lineFollowCore(spd)
        self.straight(0) # stop con frenata
        
    def seguiLineaPerDistanza(self, distanza, speed = None):
        self.integralError = 0
        if speed is None:
            spd = self.lineFollowSpeed 
        else:
            spd = speed        
        self.reset() # reset distance
        while self.distance() < distanza :
            self.__lineFollowCore(spd)
        self.straight(0) # stop con frenata

    # TODO collaudare
    def seguiContainerPerDistanza(self, sensorBlocks, distance, gain=0.5):
        gain = 0.5
        self.reset() # reset distance
        while self.distance() < distance :
            #(r,g,b) = sensorBlocksRaw.read('RGB-RAW')
            color = sensorBlocks.getColor()
            h,s,v = sensorBlocks.getHSV()
            print("COLOR: ", str(color))
            if color == Color.BLUE or color == Color.GREEN:
                v_error = 40 - v
                variazione = gain * v_error
                self.drive(50,-variazione)
            elif color == Color.WHITE:
                v_error = 100 - v
                gain = 0.3
                variazione = gain * v_error
                self.drive(50,-variazione)
            else:
                self.drive(50,0)