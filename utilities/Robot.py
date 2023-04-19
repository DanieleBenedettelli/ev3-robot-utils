""" Line Following navigator

Author  : Daniele Benedettelli
Date    : February 2023
Version : 0.1

"""
from pybricks.ev3devices import ColorSensor, Motor, GyroSensor, InfraredSensor
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

    def saturate(self, speed, maxSpeed=1000, minSpeed = 0):
        """ Limit speed in range [-1000,1000] """
        if speed > 0 :
            if speed > maxSpeed:
                speed = maxSpeed
            if speed < minSpeed:
                speed = minSpeed
        elif speed < 0 :
            if speed < -maxSpeed:
                speed = -maxSpeed
            if speed > -minSpeed:
                speed = -minSpeed       
       
        return speed

    def resetGyro(self):
        self.gyro.reset_angle(0)
        wait(500)
        print("gyro reset...",end="")
        while self.gyro.angle()!=0 :
            # hard reset (simulate disconnection and reconnection)
            self.ev3.screen.print("reconnection")
            print("try hard reset")
            try:
                dummy = InfraredSensor(self.gyroPort)   
            except OSError:
                print("dummy sensor")
            self.gyro = GyroSensor(self.gyroPort)
            self.ev3.screen.print("keep me still!")
            print("keep me still!")
            wait(4000)
            self.gyro.reset_angle(0)
            wait(100)
        print("done!")

    def readGyro(self):
        a = self.gyro.angle()
        heading = -((a+180)%360-180)
        return heading

    def headTo2(self, angle=0, speedMax = None, absoluteHeading=True, radius=0):
        if speedMax is None:
            speedMax = self.turnSpeed 
        timerDone = StopWatch()
        preSat = 0
        angSpeed = 0
        self.stop()

        if absoluteHeading:
            headingNow = 0
        else:
            headingNow = self.gyro.angle() # TODO debug absoluteHeading=False

        #print("heading now: ", headingNow)
        while timerDone.time() < 200:
            diff = self.__subang(-angle, self.gyro.angle()+headingNow) 
            #print("diff: ", diff)
            preSat = 5.0*diff 
            angSpeed = self.saturate(preSat, speedMax)
            #print("U:",angSpeed)
            speed = -angSpeed*radius/57.295
            self.drive(speed,angSpeed)
            if abs(diff) > 0:
                timerDone.reset()
        self.stop()

    def headTo(self, angle=0, useGyro=True, absoluteHeading=True):
        e = angle - self.readGyro()
        if not useGyro:
            self.turn(e)
            return
        timerDone = StopWatch()
        preSat = 0
        angSpeed = 0
        self.stop()

        if absoluteHeading:
            headingNow = 0
        else:
            headingNow = self.gyro.angle()     

        while timerDone.time() < 100:
            diff = self.__subang(angle, self.gyro.angle()+headingNow) 
            preSat = 4.0*diff 
            angSpeed = self.saturate(preSat, self.turnSpeed)
            #print("U:",angSpeed)
            self.drive(0,angSpeed)
            #self.left_motor.run(angSpeed)
            #self.right_motor.run(-angSpeed)
            if abs(diff) > 0:
                timerDone.reset()
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
            #print("sign ", signAngle)
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
        #print("angle: %.2f deg"%angle)
        #print("radius: %.2f mm"%radius)
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

    
    def straightUntilLine(self, white_thr = 70, black_thr=15, maxSpeed=None, blackLine = True):
        if maxSpeed is None:
            maxSpeed = self.travelSpeed
        self.drive(maxSpeed , 0 )
        if blackLine:
            while self.sensorRight.reflection() < white_thr:
                #print(self.sensorRight.reflection())
                wait(2)
            #self.speaker.beep()    
            while self.sensorRight.reflection() > black_thr:
                #print(self.sensorRight.reflection())
                wait(2)
        else:
            print("look for white")
            while self.sensorLeft.reflection() < white_thr:
                print(self.sensorRight.reflection())
                wait(1)              
            while self.sensorRight.reflection() < white_thr:
                print(self.sensorRight.reflection())
                wait(1)            
        
        self.straight(0)      

    def straightGyroForDistance(self, distance, maxSpeed = None, steerGain=4.5, driveGain = 3.5, absoluteHeading = True, headingOffset = 0):
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
            #print("gyro ", heading)
            gyro_error = headingNow - heading
            togo = distance - self.distance()
            steer = steerGain * gyro_error
            #speed = driveGain * togo
            #speed = self.saturate(speed, maxSpeed)
            speed = abs(maxSpeed)*self.__sign(distance)
            self.drive(speed,-steer)                
        self.straight(0)
        self.stop()        

    def straightGyroUntilContainer(self,maxSpeed = None, steerGain=4.5, colors =[Color.BLUE, Color.GREEN], longRange = False, headingToKeep=-90, maxDistance=400):
        if maxSpeed is None:
            maxSpeed = self.travelSpeed

        pos0 = self.distance()
        event = 0
        while event is 0 :
            if self.blockSensor.getColor(longRange) in colors:
                event = 1
            if (self.distance()-pos0) > maxDistance:
                event = -1
            heading = self.readGyro()
            
            #print("gyro ", heading)
            #steerGain = 3
            gyro_error = headingToKeep - heading # maintain always same heading w.r.t 0
            steer = steerGain * gyro_error
            self.drive(maxSpeed,-steer)                
        self.straight(0)
        self.stop() 
        return event
    
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
        if offset != 0 :
            self.straight(-40) # evita che il rallentatore si blocchi sul container
            self.straight(offset)

    def manageContainer(self, orderColor1, orderColor2, containerColorSeen,headingToKeep):
        DISTANZA_CARICO = 43
        DISTANZA_CONSERVO = 80
        TRAVEL_SPEED = 70
                
        #global slot1, slot2, coloredContainerStock, daParteBianchi, biancoPreso
        if containerColorSeen in [Color.BLUE, Color.GREEN]:
            if orderColor1 == containerColorSeen and self.slot1 == 0:
                print("Slot 1")
                self.straightGyroForDistance(distance=DISTANZA_CARICO, maxSpeed=TRAVEL_SPEED, headingOffset=headingToKeep)
                self.grabContainer(offset=0, heading=headingToKeep)
                self.slot1 = 1
            elif orderColor2 == containerColorSeen and self.slot2 == 0:
                print("Slot 3")
                self.straightGyroForDistance(distance=DISTANZA_CARICO, maxSpeed=TRAVEL_SPEED, headingOffset=headingToKeep)
                self.grabContainer(offset=12*8, heading=headingToKeep)
                self.slot2 = 1
            elif self.coloredContainerStock == 0:
                print("Stock")
                self.straightGyroForDistance(distance=DISTANZA_CONSERVO, maxSpeed=TRAVEL_SPEED, headingOffset=headingToKeep)
                self.grabContainer(offset=0, heading=headingToKeep)
                self.straightGyroForDistance(-24, maxSpeed=TRAVEL_SPEED, headingOffset=headingToKeep)
                self.coloredContainerStock = 1
            else : # skip container 
                self.straightGyroForDistance(distance=26, maxSpeed=TRAVEL_SPEED, headingOffset=headingToKeep)

        if self.slot1 == 1 and self.slot2 == 1 and self.coloredContainerStock == 1 :
            return True
        else:
            return False
        
    def manageWhiteContainers(self,headingToKeep):
        DISTANZA_CARICO = 53
        DISTANZA_CONSERVO = 93 
        TRAVEL_SPEED = 70

        if self.gotWhiteContainer == 0:
            self.straightGyroForDistance(distance=DISTANZA_CARICO, maxSpeed=TRAVEL_SPEED, headingOffset=headingToKeep)
            self.grabContainer(offset=18*8,heading=headingToKeep)
            print("Slot 4")
            self.gotWhiteContainer = 1
        else:
            self.straightGyroForDistance(distance=DISTANZA_CONSERVO,maxSpeed=TRAVEL_SPEED, headingOffset=headingToKeep)
            self.grabContainer(offset=0,heading=headingToKeep)
            print("Stock")
                    

