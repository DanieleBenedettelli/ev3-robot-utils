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
from pybricks.hubs import EV3Brick

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
        self.gyroPort = gyroSensorPort
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
        # container logic
        self.slot1 = 0
        self.slot2 = 0
        self.whiteContainerInStock = 0
        self.coloredContainerStock = 0
        self.gotWhiteContainer = 0
        self.grabber = None
        #print(left_motor, right_motor, wheel_diameter, axle_track)
        #initialize super class DriveBase
        super().__init__(left_motor, right_motor, wheel_diameter, axle_track)        
        self.resetGyro()
  
    def setGrabber(self,grb):
        #print(type(grb))
        self.grabber = grb

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
            dummy = AnalogSensor(self.gyroPort )    
            wait(800)
            self.gyro = GyroSensor(self.gyroPort)
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
        e_old = 0
        while timerDone.time() < 300:
            e = angle - self.gyro.angle()
            diff = e - e_old
            e_old = e
            #if abs(preSat)>MAX_CMD and angSpeed*integral > 0 : # segni concordi
            #    integral = 0
            #integral += e
            #integral = saturate(integral, 360)
            #print("I:",integral)
            preSat = 4*e + 0.1*diff #+ 0.02*integral
            angSpeed = self.saturate(preSat, self.turnSpeed)
            #print("U:",angSpeed)
            #self.right_motor.dc(-angSpeed)
            #self.left_motor.dc(angSpeed)
            self.drive(0,angSpeed)
            if abs(e) > 1:
                timerDone.reset()
            wait(20)
        self.stop()

    # radius > 0: punto rotazione a sinistra 
    # radius < 0: punto rotazione a destra
    """ 
    Arguments:
        radius {integer} : radius > 0: punto rotazione a sinistra 
                    radius < 0: punto rotazione a destra
        angle {integer} : degrees (positive counterclockwise)
    """
    def arc(self, radius, angle, speed=100) :
        if (abs(radius)<2) :
            self.turn(-angle)
        else: 
            angular_speed = -57.295*speed/radius #deg/s
            #print("drive speed: %.2f mm/s"%speed)
            #print("angular speed: %.2f deg/s"%angular_speed)
            heading0 = self.gyro.angle()
            
            if angle < 0:
                speed = -speed
                angular_speed =  -angular_speed

            self.drive(speed, angular_speed)
            diff = self.__subang(self.gyro.angle(), heading0)
            print(diff)
            while abs(abs(diff) - abs(angle)) > 1 :
                diff = self.__subang(self.gyro.angle(), heading0)
                #print("ang diff: %.2f deg"%diff)
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

    def followLineUntilIntersection(self, thr = 24, speed = None):
        self.integralError = 0
        if speed is None:
            spd = self.lineFollowSpeed 
        else:
            spd = speed
        while self.sensorIntersection.reflection() > thr:
            print(self.sensorIntersection.reflection())
            self.__lineFollowCore(spd)
        self.straight(0) # stop con frenata
        
    def followLineForDistance(self, distanza, speed = None):
        self.integralError = 0
        if speed is None:
            spd = self.lineFollowSpeed 
        else:
            spd = speed        
        self.reset() # reset distance
        while self.distance() < distanza :
            self.__lineFollowCore(spd)
        self.straight(0) # stop con frenata

    def straightGyroContainerForDistance(self, distance, gain=0.5):
        self.reset()
        while self.distance() < distance:
            color = self.blockSensor.getColor()
            h,s,v = self.blockSensor.getHSV()
            print (h,s,v)
            print("COLOR: ", str(color))
            
            if color == Color.BLUE or color == Color.GREEN:
                gain = 0.5
                v_error = 40 - v
                steer = gain * v_error
                self.drive(50,-steer)
            elif color == Color.WHITE:
                v_error = 100 - v
                gain = 0.3
                steer = gain * v_error
                self.drive(50,-steer)
            else:
                heading = self.readGyro()
                gain = 3
                gyro_error = 0 - heading
                steer = gain * gyro_error
                self.drive(50,-steer)                
        self.straight(0)
        self.stop()

    
    def straightUntilLine(self, white_thr = 70, black_thr=15, maxSpeed=None):
        if maxSpeed is None:
            maxSpeed = self.travelSpeed
        self.drive(maxSpeed , 0 )
        while self.sensorRight.reflection() < white_thr:
            #print(self.sensorRight.reflection())
            wait(10)
        #sled.speaker.beep()    
        while self.sensorRight.reflection() > black_thr:
            #print(self.sensorRight.reflection())
            wait(10)
        self.straight(0)      

    def straightGyroForDistance(self, distance, maxSpeed = None, steerGain=0.6, driveGain = 4, absoluteHeading = True):
        if maxSpeed is None:
            maxSpeed = self.travelSpeed

        self.reset()
        togo = distance - self.distance()

        if absoluteHeading:
            headingNow = 0
        else:
            headingNow = self.readGyro()

        while abs(togo)>2:
            heading = self.readGyro()
            steerGain = 3
            gyro_error = headingNow - heading
            togo = distance - self.distance()
            steer = steerGain * gyro_error
            speed = driveGain * togo
            speed = self.saturate(speed, maxSpeed)
            self.drive(speed,-steer)                
        self.straight(0)
        self.stop()        

    def straightGyroUntilContainer(self,maxSpeed = None, steerGain=0.6, driveGain = 4, colors =[Color.BLUE, Color.GREEN]):
        if maxSpeed is None:
            maxSpeed = self.travelSpeed

        #headingNow = self.readGyro()
        print("searching...")
        while self.blockSensor.getColor(False) not in colors:
            
            heading = self.readGyro()
            steerGain = 3
            gyro_error = 0 - heading # maintain always same heading w.r.t 0
            steer = steerGain * gyro_error
            self.drive(maxSpeed,-steer)                
        self.straight(0)
        self.stop() 
        return self.blockSensor.getColor(False)         
    
    def resetContainerLogic(self):
        self.slot1 = 0
        self.slot2 = 0
        self.whiteContainerInStock = 0
        self.coloredContainerStock = 0
        self.gotWhiteContainer = 0

    
    def grabContainer(self, offset = 0):
        self.grabber.lift()
        #wait(200)
        self.straightGyroForDistance(-offset,maxSpeed = 120, absoluteHeading=True)
        #self.straight(-offset)
        self.grabber.unloadOnRamp()
        self.grabber.prepareForGrabbing()
        wait(100) # wait for container to slide onto the boat
        self.straightGyroForDistance(offset,maxSpeed = 120, absoluteHeading=True)

    def manageContainer(self, orderColor1, orderColor2, containerColorSeen):
        DISTANZA_CARICO = 50
        DISTANZA_CONSERVO = 90
        TRAVEL_SPEED = 60
        
        #global slot1, slot2, coloredContainerStock, daParteBianchi, biancoPreso
        if containerColorSeen in [Color.BLUE, Color.GREEN]:
            if orderColor1 == containerColorSeen and self.slot1 == 0:
                print("Slot_1")
                self.straightGyroForDistance(DISTANZA_CARICO, maxSpeed=TRAVEL_SPEED, absoluteHeading=True)
                self.grabContainer(0)
                self.slot1 = 1
            elif orderColor2 == containerColorSeen and self.slot2 == 0:
                print("Slot_3")
                self.straightGyroForDistance(DISTANZA_CARICO, maxSpeed=TRAVEL_SPEED, absoluteHeading=True)
                self.grabContainer(12*8)
                self.slot2 = 1
            elif self.coloredContainerStock == 0:
                print("Stock")
                self.straightGyroForDistance(DISTANZA_CONSERVO, maxSpeed=TRAVEL_SPEED, absoluteHeading=True)
                self.grabContainer(0)
                self.straightGyroForDistance(-24, maxSpeed=TRAVEL_SPEED, absoluteHeading=True)
                self.coloredContainerStock = 1

        if self.slot1 == 1 and self.slot2 == 1 and self.coloredContainerStock == 1 :
            return True
        else:
            return False
        
    def manageWhiteContainers(self):
        DISTANZA_CARICO = 50
        DISTANZA_CONSERVO = 85
        TRAVEL_SPEED = 60

        if self.gotWhiteContainer == 0:
            self.straightGyroForDistance(DISTANZA_CARICO, maxSpeed=TRAVEL_SPEED, absoluteHeading=True)
            self.grabContainer(18*8+4)
            print("Slot_4")
            self.gotWhiteContainer = 1
        else:
            self.straightGyroForDistance(DISTANZA_CONSERVO,maxSpeed=TRAVEL_SPEED, absoluteHeading=True)
            self.grabContainer(0)
            print("Stock")
                    

