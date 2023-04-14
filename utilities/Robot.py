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
from utilities.Grabber import Grabber

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
        self.grabber = Grabber()
        self.ev3 = EV3Brick()
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
            #dummy = AnalogSensor(self.gyroPort)    
            self.ev3.screen.print("Gyro Reset")
            wait(800)
            self.gyro = GyroSensor(self.gyroPort)
            self.gyro.reset_angle(0)
        print("done!")

    def readGyro(self):
        a = self.gyro.angle()
        heading = -((a+180)%360-180)
        return heading

    # uses unregulated motors
    def headTo(self, angle=0, useGyro=True, absoluteHeading=True):
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
        if absoluteHeading:
            headingNow = 0
        else:
            headingNow = self.gyro.angle()     

        while timerDone.time() < 300:
            #e = angle - self.gyro.angle()
            #diff = e - e_old
            diff = self.__subang(angle, self.gyro.angle()+headingNow) #TODO debug headingNow
            #e_old = e
            #if abs(preSat)>MAX_CMD and angSpeed*integral > 0 : # segni concordi
            #    integral = 0
            #integral += e
            #integral = saturate(integral, 360)
            #print("I:",integral)
            preSat = 4.0*diff #+ 0.02*integral
            angSpeed = self.saturate(preSat, self.turnSpeed)
            #print("U:",angSpeed)
            #self.right_motor.dc(-angSpeed)
            #self.left_motor.dc(angSpeed)
            self.drive(0,angSpeed)
            if abs(diff) > 0:
                timerDone.reset()
            #wait(20)
        self.stop()

    def __sign(self,n):
        if n>0:
            return 1
        elif n<0:
            return -1
        else :
            return 0

    # radius > 0: punto rotazione a sinistra 
    # radius < 0: punto rotazione a destra
    """ 
    Arguments:
        radius {integer} : radius > 0: punto rotazione a sinistra 
                    radius < 0: punto rotazione a destra
        angle {integer} : degrees (positive counterclockwise)
    """
    def arc(self, radius, angle, speed=80) :
        #if speed>110: 
        #    speed = 110
        if (abs(radius)<2) :
            self.turn(-angle)
        else: 
            angular_speed = -57.295*speed/radius #deg/s
            #print("drive speed: %.2f mm/s"%speed)
            #print("angular speed: %.2f deg/s"%angular_speed)
            heading0 = self.readGyro()
            target = heading0+angle#((heading0+angle+180)%360-180)
            #print("initial heading= ",heading0)
            #print("target= ",target)

            if self.__sign(radius*angle) < 0:
                speed = -speed
                angular_speed =  -angular_speed

            self.drive(speed, angular_speed) # this settings create the arc
            
            signAngle = self.__sign(angle)
            print("sign ", signAngle)
            diff = signAngle*10
            #print(diff)
            while abs(diff)>2 and signAngle*diff > 0 :
                heading = self.readGyro()
                diff = self.__subang(target, heading )
                #print("ang diff: %.2f deg"%(signAngle*diff) )
                wait(1)
            self.straight(0)

    def Scurve(self, dx, dy, speed=100):
        # Wolfram Alpha: solve [ y/2=R*sin(a), x/2 = R*(1-cos(a)) ] for R,a
        angle = 2*atan2(dx, dy)*57.296
        radius = (dx*dx+dy*dy)/(4*dx)
        print("angle: %.2f deg"%angle)
        print("radius: %.2f mm"%radius)
        self.arc(radius=radius,angle= angle,speed=speed)
        self.arc(radius=-radius, angle=-angle, speed=speed)


    def spin(self, angle, stopType=Stop.BRAKE, waitForCompletion=True) :
        self.turn(-angle)# = -angle, then = stopType, wait= waitForCompletion)

    def __lineFollowCore(self, speed) :
        e = self.target - self.sensorLine.reflection()
        #print("lnfl e:", e)
        steer = self.gain * e
        self.drive(speed,steer)


    def lineFollowerSettings(self, speed, target, gain, darkThreshold = 10, whichSensor=Side.RIGHT, whichBorder = Side.LEFT) :
        if whichSensor is Side.LEFT:
            self.sensorLine = self.sensorLeft
            self.sensorIntersection = self.sensorRight
        else :
            self.sensorLine = self.sensorRight
            self.sensorIntersection = self.sensorLeft
        self.lineFollowSpeed = speed     
        #print("speed",self.lineFollowSpeed)       
        self.gain = gain
        if whichBorder is Side.LEFT:#whichSensor : #  if same side
            self.gain = -gain
            #print("invert gain", self.gain)
        
        self.blackThreshold = darkThreshold     
        self.target = target       

    def followLineUntilIntersection(self, thr = 24, speed = None, brake=True):
        self.integralError = 0
        if speed is None:
            spd = self.lineFollowSpeed 
        else:
            spd = speed
        while self.sensorIntersection.reflection() > thr:
            #print(self.sensorIntersection.reflection())
            self.__lineFollowCore(spd)
            #wait()
        if brake:    
            self.straight(0) # stop con frenata
        else:
            self.stop()
        
    def followLineForDistance(self, distance, speed = None, brake=True):
        self.integralError = 0
        if speed is None:
            spd = self.lineFollowSpeed 
        else:
            spd = speed        
        self.reset() # reset distance
        while self.distance() < distance :
            self.__lineFollowCore(spd)
        if brake:
            self.straight(0) # stop con frenata
        else :
            self.stop()

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
            wait(1)
        #sled.speaker.beep()    
        while self.sensorRight.reflection() > black_thr:
            #print(self.sensorRight.reflection())
            wait(1)
        self.straight(0)      

    def straightGyroForDistance(self, distance, maxSpeed = None, steerGain=3, driveGain = 3.5, absoluteHeading = True, headingOffset = 0):
        if maxSpeed is None:
            maxSpeed = self.travelSpeed

        self.reset()
        togo = distance - self.distance()

        if absoluteHeading:
            headingNow = headingOffset
        else:
            headingNow = self.readGyro()

        print("heading now: ", headingNow)

        while abs(togo)>1:
            heading = self.readGyro()
            gyro_error = headingNow - heading
            togo = distance - self.distance()
            steer = steerGain * gyro_error
            speed = driveGain * togo
            speed = self.saturate(speed, maxSpeed)
            self.drive(speed,-steer)                
        self.straight(0)
        self.stop()        

    def straightGyroUntilContainer(self,maxSpeed = None, steerGain=0.6, driveGain = 4, colors =[Color.BLUE, Color.GREEN], headingToKeep=-90, maxDistance=400):
        if maxSpeed is None:
            maxSpeed = self.travelSpeed

        pos0 = self.distance()
        while self.blockSensor.getColor(False) not in colors and (self.distance()-pos0) < maxDistance:
            heading = self.readGyro()
            steerGain = 3
            gyro_error = headingToKeep - heading # maintain always same heading w.r.t 0
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

    
    def grabContainer(self, offset = 0, heading=-90):
        self.grabber.lift()
        #wait(200)
        #self.straightGyroForDistance(-offset,maxSpeed = 120, headingOffset=heading)
        self.straight(-offset)
        self.grabber.unloadOnRamp()
        self.grabber.prepareForGrabbing()
        wait(200) # wait for container to slide onto the boat
        #self.straightGyroForDistance(offset,maxSpeed = 120,  headingOffset=heading)
        self.straight(offset)

    def manageContainer(self, orderColor1, orderColor2, containerColorSeen,headingToKeep):
        DISTANZA_CARICO = 45
        DISTANZA_CONSERVO = 94
        TRAVEL_SPEED = 60
                
        #global slot1, slot2, coloredContainerStock, daParteBianchi, biancoPreso
        if containerColorSeen in [Color.BLUE, Color.GREEN]:
            if orderColor1 == containerColorSeen and self.slot1 == 0:
                print("Slot 1")
                self.straightGyroForDistance(DISTANZA_CARICO, maxSpeed=TRAVEL_SPEED, headingOffset=headingToKeep)
                self.grabContainer(offset=0, heading=headingToKeep)
                self.slot1 = 1
            elif orderColor2 == containerColorSeen and self.slot2 == 0:
                print("Slot 3")
                self.straightGyroForDistance(DISTANZA_CARICO, maxSpeed=TRAVEL_SPEED, headingOffset=headingToKeep)
                self.grabContainer(offset=12*8, heading=headingToKeep)
                self.slot2 = 1
            elif self.coloredContainerStock == 0:
                print("Stock")
                self.straightGyroForDistance(DISTANZA_CONSERVO, maxSpeed=TRAVEL_SPEED, headingOffset=headingToKeep)
                self.grabContainer(offset=0, heading=headingToKeep)
                self.straightGyroForDistance(-24, maxSpeed=TRAVEL_SPEED, headingOffset=headingToKeep)
                self.coloredContainerStock = 1
            else : # skip container 
                self.straightGyroForDistance(26, maxSpeed=TRAVEL_SPEED, headingOffset=headingToKeep)

        if self.slot1 == 1 and self.slot2 == 1 and self.coloredContainerStock == 1 :
            return True
        else:
            return False
        
    def manageWhiteContainers(self,headingToKeep):
        DISTANZA_CARICO = 45
        DISTANZA_CONSERVO = 94
        TRAVEL_SPEED = 60

        if self.gotWhiteContainer == 0:
            self.straightGyroForDistance(DISTANZA_CARICO, maxSpeed=TRAVEL_SPEED, headingOffset=headingToKeep)
            self.grabContainer(offset=18*8+4,heading=headingToKeep)
            print("Slot 4")
            self.gotWhiteContainer = 1
        else:
            self.straightGyroForDistance(DISTANZA_CONSERVO,maxSpeed=TRAVEL_SPEED, headingOffset=headingToKeep)
            self.grabContainer(offset=0,heading=headingToKeep)
            print("Stock")
                    

