#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.iodevices import Ev3devSensor                                 
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch

from pybricks.media.ev3dev import SoundFile, ImageFile
from math import atan2

from utilities.HSVColorSensor import HSVColorSensor
from utilities.Robot import Robot, Side
from utilities.Grabber import Grabber


# https://pybricks.com/ev3-micropython/

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

ev3 = EV3Brick()
sensorBlocks = HSVColorSensor(Port.S4)
sensorLine = ColorSensor(Port.S1)
sensorIntersections = ColorSensor(Port.S3)
GYRO_PORT = Port.S2
GRABBER_PORT = Port.A

"""
while True:
    #(r,g,b) = sensorBlocksRaw.read('RGB-RAW')
    color = sensorBlocks.getColor()
    print("COLOR: ", str(color))
    wait(200)
"""


WHEEL_DIAM = 56 # mm/s
WHEEL_DIST = 96 # mm/s
TRAVEL_SPEED = 200 # mm/s
TRAVEL_ACC = 400 # mm/s/s
SPIN_SPEED = 270 # deg/s
SPIN_ACC = 270 # deg/s/s

timer = StopWatch()

motorRight = Motor(Port.C)
motorLeft = Motor(Port.B)

# costruttore robot
robot = Robot(motorLeft, motorRight, WHEEL_DIAM, WHEEL_DIST, sensorLine, sensorIntersections, sensorBlocks, GYRO_PORT)
robot.settings(TRAVEL_SPEED, TRAVEL_ACC, SPIN_SPEED, SPIN_ACC)
grabber = Grabber(GRABBER_PORT)
robot.setGrabber(grabber)

# settings for line following
robot.lineFollowerSettings(speed=90, target=45, gain=0.55, darkThreshold = 10 )

robot.gyro.reset_angle(0)#resetGyro()

"""
while True:
  if Button.LEFT in ev3.buttons.pressed():
    robot.headTo(0)
  if Button.DOWN in ev3.buttons.pressed():
    robot.headTo(-90)
  if Button.UP in ev3.buttons.pressed():
    robot.headTo(90)
"""


def ASPETTA_VIA():
  ev3.light.on(Color.ORANGE)
  while Button.LEFT not in ev3.buttons.pressed():
    wait(100)
  ev3.light.on(Color.GREEN)  

# esci da base, leggi ordine, prendi barca grande e riforniscila
def FASE1():
  global order1, order2
  timer.reset()
  robot.spin(25) 
  robot.straight(70)
  robot.spin(-25) 
  #robot.headTo(0)

  robot.followLineForDistance(145)

  # leggi blocco ordine 1
  #TODO read until valid
  # eventualmente sterzatina verso sx per avvicinarsi al blocco
  ev3.speaker.beep()
  order1 = sensorBlocks.getRobustColor()
  ev3.screen.print("color1: "+str(order1))
  print("color1: "+str(order1))

  robot.followLineForDistance(48)

  ev3.speaker.beep()
  order2 = sensorBlocks.getRobustColor()
  ev3.screen.clear()
  ev3.screen.print("color1: "+str(order2))
  print("color2: "+str(order2))
  #ev3.screen.print("time: "+str(timer.time()))

  #robot.straightGyroForDistance(-40)
  robot.followLineUntilIntersection(thr=15, speed = 100) #TODO debug, non viene eseguito
  robot.gyro.reset_angle(0)
  robot.straightGyroForDistance(-180)

  # prendi barca
  robot.Scurve(75,280,120)
  robot.headTo(0)
  robot.straightGyroForDistance(distance=185, maxSpeed=150)
  print(timer.time())
  ev3.screen.print(timer.time())


# curva con la barca verso i container
def FASE2():
  grabber.retract()
  robot.gyro.reset_angle(0) # TODO remove this, as it was done before
  timer.reset()
  robot.spin(-35)
  robot.straight(35)
  robot.spin(-35)
  robot.straight(35)  
  robot.spin(-30)
  robot.straight(260) # regola quanto mi avvicino ai container
  robot.spin(-20)
  robot.straight(40)
  robot.headTo(90)
  robot.straightGyroForDistance(distance=65,maxSpeed=80, absoluteHeading=False)
  print(timer.time())
  ev3.screen.print(timer.time())

# TODO da finire
def prendiBarcaPiccola():
  robot.followLineUntilIntersection()
  robot.gyro.reset_angle(180)
  robot.arc(radius=105, angle=187, speed=150) 
  wait(1000)
  robot.turnSpeed = 60
  robot.headTo(angle=0)
  robot.straightGyroForDistance(400)



def bringBigBoatOut():
  # vai fuori dal porto con la barca grande
  robot.gyro.reset_angle(0)
  robot.spin(30)
  robot.straight(30)
  robot.headTo(25)
  robot.straight(30)
  robot.drive(100,0)
  while sensorLine.reflection() <50:
    #print(sensorLine.reflection())
    wait(5)
  ev3.speaker.beep()  
  while sensorLine.reflection() >10:
    #print(sensorLine.reflection())
    wait(5)
  ev3.speaker.beep()  
  robot.straight(30)  
  robot.stop()
  robot.arc(radius=110, angle=45)
  robot.followLineUntilIntersection()
  #robot.headTo(0)

def uscitaPortoContiniFe() :
  robot.arc(radius=-45 , angle=50 , speed=60)
  robot.drive(80 , 0 )
  while sensorIntersections.reflection() < 70:
      wait(10)
  while sensorIntersections.reflection() > 15:
      wait(10)
  robot.straight(85)
  #robot.spin(50)  
  robot.headTo(0)
  robot.followLineForDistance(150)
  robot.seguiLineaFinoAIncrocio()
  robot.Scurve(300,50)

def testGrabber():
  while True:
    if Button.CENTER in ev3.buttons.pressed():
        robot.grabContainer()
        while Button.CENTER in ev3.buttons.pressed():
          wait(5)


def getAllContainers(c1=Color.GREEN, c2=Color.BLUE):

  grabber.prepareForGrabbing()

  done = False

  robot.resetContainerLogic()
  robot.gyro.reset_angle(0)

  # TODO DEBUG THIS
  while not done:
      #avanza finché non vedi un colore
      color = robot.straightGyroUntilContainer(maxSpeed = 40, colors=[Color.BLUE, Color.GREEN])
      ev3.speaker.beep()
      wait(100)
      seenColor = sensorBlocks.getRobustColor(longRange=False)
      done = robot.manageContainer(c1, c2, seenColor)

  # TODO da collaudare
  robot.straightGyroForDistance(260, maxSpeed = 120, absoluteHeading=True) 

  robot.straightGyroUntilContainer(maxSpeed = 40, colors=[Color.WHITE])
  ev3.speaker.beep()
  robot.manageWhiteContainers()
  robot.straightGyroUntilContainer(maxSpeed = 40, colors=[Color.WHITE])
  ev3.speaker.beep()
  robot.manageWhiteContainers()


def portaBarcaGrandeFuori():
  robot.arc(radius=-45 , angle=45 , speed=50)
  robot.straightUntilLine()
  robot.arc(radius=130, angle=48,speed=80)
  robot.lineFollowerSettings(speed=100, target=45, gain=0.6, darkThreshold = 10)
  robot.followLineUntilIntersection(speed=180)
  robot.gyro.reset_angle(0)
  robot.straight(70)# allinea anello container rosso con gancio gru
  robot.straight(-220)
  robot.spin(45)
  robot.straight(220)
  robot.headTo(0)
  robot.straight(150)
  robot.headTo(90)
  robot.straightGyroForDistance(distance=400,maxSpeed=200,absoluteHeading=False)
  robot.straightGyroForDistance(distance=-160, maxSpeed = 200, absoluteHeading=False) # regola posizione sensore su bordo linea
  robot.headTo(180)

def prendiBarcaPiccola():
  robot.lineFollowerSettings(speed=100, target=40, gain=0.5, darkThreshold = 10, whichSensor=Side.RIGHT, whichBorder=Side.LEFT)
  robot.followLineUntilIntersection(speed=150)
  robot.straight(30)
  robot.followLineUntilIntersection(speed=150)
  robot.gyro.reset_angle(180)
  robot.followLineForDistance(430,speed=200)
  robot.arc(radius=95, angle=180, speed=120 ) # arco per prendere barca
  robot.straightGyroForDistance(200, absoluteHeading=True)

  # carica barca piccola
  robot.grabber.unloadBuffer()
  robot.grabber.prepareForGrabbing(False)
  robot.straight(-48)
  robot.grabber.unloadBuffer()
  robot.grabber.retract(False)
  robot.straightGyroForDistance(400,absoluteHeading=False)

  robot.straightUntilLine(maxSpeed=120)
  robot.headTo(-45)
  robot.straight(100)
  robot.straightUntilLine(maxSpeed=200)
  robot.straight(130)
  robot.headTo(0)
  robot.lineFollowerSettings(speed=150,target=45,gain=0.5,darkThreshold=10,whichSensor=Side.LEFT,whichBorder=Side.RIGHT)
  robot.followLineUntilIntersection(speed=200)
  robot.arc(radius=100, angle=90, speed=140)
  robot.straight(180) # end of mission

"""
  __  __    _    ___ _   _ 
 |  \/  |  / \  |_ _| \ | |
 | |\/| | / _ \  | ||  \| |
 | |  | |/ ___ \ | || |\  |
 |_|  |_/_/   \_\___|_| \_|
                           
"""
# DATA: 12/04/2023
# VERSIONE 0.8

# CALIBRAZIONE INIZIALE
ev3.speaker.play_notes(notes=['C5/8_', 'E5/8_', 'G5/4'],tempo=200)
grabber.calibrate()

ASPETTA_VIA()
grabber.unloadBuffer()
grabber.retract(False)
FASE1()
FASE2()
getAllContainers(order1, order2)
portaBarcaGrandeFuori()
prendiBarcaPiccola()





#TODO invert sign of angle parameter in headTo method
#TODO spingi barca grande verso gru
#TODO vai a prendere barca piccola
#TODO carica barca piccola
#TODO porta barca piccola fuori dal porto
#TODO parcheggia robot