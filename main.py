#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.iodevices import Ev3devSensor                                 
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch

from math import atan2

from utilities.HSVColorSensor import HSVColorSensor
from utilities.Robot import Robot, Side
from utilities.Grabber import Grabber


# https://pybricks.com/ev3-micropython/

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
TRAVEL_SPEED = 220 # mm/s
TRAVEL_ACC = 300 # mm/s/s
SPIN_SPEED = 180 # deg/s
SPIN_ACC = 90 # deg/s/s

timer = StopWatch()

motorRight = Motor(Port.C)
motorLeft = Motor(Port.B)

# costruttore robot
robot = Robot(motorLeft, motorRight, WHEEL_DIAM, WHEEL_DIST, sensorLine, sensorIntersections, sensorBlocks, GYRO_PORT)
robot.settings(TRAVEL_SPEED, TRAVEL_ACC, SPIN_SPEED, SPIN_ACC)
grabber = Grabber(GRABBER_PORT)
robot.setGrabber(grabber)

# settings for line following
robot.lineFollowerSettings(speed=90, target=45, gain=0.55, darkThreshold = 10, whichSensor=Side.RIGHT, whichBorder = Side.LEFT )

robot.gyro.reset_angle(0)#resetGyro()

def aspettaIlVia():
  ev3.light.on(Color.ORANGE)
  while not ev3.buttons.pressed():
    wait(100)
  ev3.light.on(Color.GREEN)  

# esci da base, leggi ordine, prendi barca grande e riforniscila
def leggiOrdineRifornisciBarca():
  global order1, order2
  timer.reset()
  robot.straight(DISTANZA_USCITA_START)
  robot.lineFollowerSettings(speed=90, target=30, gain=0.2, darkThreshold = 10, whichSensor=Side.RIGHT, whichBorder = Side.LEFT )
  robot.followLineForDistance(155)

  # leggi blocco ordine 1
  ev3.speaker.beep()
  order1 = sensorBlocks.getRobustColor()
  ev3.screen.print("color1: "+str(order1))
  print("color1: "+str(order1))

  robot.followLineForDistance(48)

  ev3.speaker.beep()
  order2 = sensorBlocks.getRobustColor()
  ev3.screen.print("color2: "+str(order2))
  print("color2: "+str(order2))

  print("follow until X")
  robot.followLineUntilIntersection(thr=20, speed = 50)
  wait(100)
    
  print("reset gyro")
  robot.gyro.reset_angle(0) # UNICO RESET DA FARE

  print("vai indietro")
  #robot.straightGyroForDistance(distance=-180, maxSpeed = 90, absoluteHeading=True)
  robot.straight(distance=-180)

  # prendi barca
  print("curva S")
  robot.Scurve(65,280,120)
  
  #robot.straightGyroForDistance(distance=178, maxSpeed=150)
  robot.straight(DISTANZA_PER_RIFORNIMENTO)
  
  robot.straight(-10) 
  print(timer.time())
  ev3.screen.print(timer.time())


# curva con la barca verso i container
def curvaVersoContainer():
  timer.reset()
  robot.turnSpeed = 90
  #TODO test this new headTo2 API 
  robot.headTo2(angle=-45, radius=-WHEEL_DIST/2, speedMax=100)
  #robot.arc(angle=-45,radius=-WHEEL_DIST/2,speed=90)
  
  robot.straight(DISTANZA_AVVICINAMENTO_CONTAINER) # determina avvicinamento ai container
  
  #robot.headTo(angle=-HEADING_CONTAINER)  
  robot.headTo2(angle=HEADING_CONTAINER, radius=-WHEEL_DIST/2, speedMax=100)

  robot.straightGyroForDistance(distance=50,maxSpeed=80, absoluteHeading=True,headingOffset=HEADING_CONTAINER) 
  robot.turnSpeed = 270
  print(timer.time())
  ev3.screen.print(timer.time())

def testGrabber():
  robot.grabber.prepareForGrabbing()
  while True:
    if Button.CENTER in ev3.buttons.pressed():
        robot.grabContainer()
        while Button.CENTER in ev3.buttons.pressed():
          wait(5)

def testContainerColor() :
  while Button.CENTER not in ev3.buttons.pressed():
    color=sensorBlocks.getColor(longRange=False)
    print("************* ", color)
    wait(500)

def prendiTuttiContainer(c1=Color.GREEN, c2=Color.BLUE):
  # TODO seguire sempre l'angolo dell'ultimo reset (quindi assoluto, ma a -90)
  grabber.prepareForGrabbing()
  done = False

  robot.resetContainerLogic()
  heading0 = robot.readGyro() # should be -90 deg
  #heading0 = HEADING_CONTAINER
  print("heading to keep:", heading0)

  pos0 = robot.distance() # per misurare distanza massima percorsa e fermarsi
  MAX_DISTANCE = 400
  while not done and (robot.distance()-pos0)<MAX_DISTANCE: # 55cm per trovare i container colorati
      #avanza finché non vedi un colore
      print("searching...")
      distanceLeft = MAX_DISTANCE - (robot.distance()-pos0)
      robot.straightGyroUntilContainer(maxSpeed = 40, colors=[Color.BLUE, Color.GREEN], headingToKeep=heading0, maxDistance = distanceLeft)
      ev3.speaker.beep()
      #wait(100)
      seenColor = sensorBlocks.getRobustColor(longRange=False)
      print("container color:", seenColor)
      done = robot.manageContainer(orderColor1=c1, orderColor2=c2, containerColorSeen=seenColor, headingToKeep=heading0)

  print("presi tutti colorati:", done)
  
  robot.straightGyroForDistance(distance=240, maxSpeed = 120, steerGain = 4, absoluteHeading=True, headingOffset=heading0) 

  found = robot.straightGyroUntilContainer(maxSpeed = 40, colors=[Color.WHITE], headingToKeep=heading0, maxDistance=270, longRange=True)
  if found == 1:
    ev3.speaker.beep()
    robot.manageWhiteContainers(heading0)
    found = robot.straightGyroUntilContainer(maxSpeed = 40, colors=[Color.WHITE],headingToKeep=heading0,  maxDistance=80, longRange=True)
  if found == 1:
    ev3.speaker.beep()
    robot.manageWhiteContainers(heading0)


def portaBarcaGrandeFuori():
  robot.grabber.retract()
  robot.arc(radius=-45 , angle=-45 , speed=50)

  # TODO rendi più robusto andare verso la linea, indipendentemente dall'aver preso i container bianchi
  robot.straight(80)
  robot.straightUntilLine()
  robot.straight(-10) 

  # TODO test HeadTo2
  robot.headTo2(radius=140, angle=-HEADING_CONTAINER, speedMax=90)
  #robot.arc(radius=140, angle=48,speed=80)

  robot.lineFollowerSettings(speed=100, target=35, gain=0.6, darkThreshold = 10)
  robot.followLineForDistance(distance=100,speed=150, brake=False)
  robot.followLineUntilIntersection(speed=90)
  robot.gyro.reset_angle(0)
  robot.straight(70)# allinea anello container rosso con gancio gru
  robot.straight(-220)
  robot.spin(45)
  robot.straight(220)
  robot.headTo(0)
  robot.straight(150)
  robot.headTo(90)
  robot.straightGyroForDistance(distance=DISTANZA_GRU,maxSpeed=200,absoluteHeading=False) # avvicinamento alla gru
  robot.straightGyroForDistance(distance=DISTANZA_DOPO_GRU, maxSpeed = 200, absoluteHeading=False) # regola posizione sensore su bordo linea
  robot.headTo(180)

def prendiBarcaPiccola():
  robot.lineFollowerSettings(speed=80, target=30, gain=0.8, darkThreshold = 10, whichSensor=Side.RIGHT, whichBorder=Side.LEFT)
  robot.followLineForDistance(distance=450 , speed=200, brake=False)

  robot.lineFollowerSettings(speed=100, target=30, gain=0.4, darkThreshold = 10, whichSensor=Side.RIGHT, whichBorder=Side.LEFT)
  robot.followLineUntilIntersection(speed=100,brake=False)
  #robot.straight(30)

  robot.followLineForDistance(distance=450 , speed=200, brake=False)  
  robot.followLineUntilIntersection(speed=100)
  wait(100)
  robot.gyro.reset_angle(180)

  robot.followLineForDistance(430,speed=200)
  wait(100)

  # arco per prendere barca
  robot.arc(radius=95, angle=180, speed=110 ) 
  
  robot.straightGyroForDistance(600, absoluteHeading=True)

  robot.straightUntilLine(maxSpeed=150)
  robot.headTo(-45)
  robot.straight(150)
  robot.straightUntilLine(maxSpeed=150)
  robot.straight(140)
  
  robot.arc(angle=-45,radius=-WHEEL_DIST/2,speed=90)

  robot.lineFollowerSettings(speed=150,target=40,gain=0.3,darkThreshold=10,whichSensor=Side.LEFT,whichBorder=Side.RIGHT)
  robot.followLineForDistance(distance=300, speed=150, brake=False) 
  robot.followLineUntilIntersection(speed=150)
  robot.arc(radius=100, angle=90, speed=140)

  # carica barca piccola
  robot.grabber.unloadBuffer()
  robot.grabber.retract(False)
  robot.straight(-48)
  robot.grabber.unloadBuffer()
  robot.grabber.retract(False)

  robot.straight(230) # end of mission

"""
  __  __    _    ___ _   _ 
 |  \/  |  / \  |_ _| \ | |
 | |\/| | / _ \  | ||  \| |
 | |  | |/ ___ \ | || |\  |
 |_|  |_/_/   \_\___|_| \_|
                           
"""
# DATA: 18/04/2023
# VERSIONE 1.1

order1 = Color.BLUE
order2 = Color.GREEN

HEADING_CONTAINER = -90
DISTANZA_USCITA_START = 50 #mm
DISTANZA_PER_RIFORNIMENTO = 197 # mm 
DISTANZA_AVVICINAMENTO_CONTAINER = 315 # mm
DISTANZA_GRU = 390 # mm
DISTANZA_DOPO_GRU = -125 # mm

grabber.calibrate()

# testContainerColor() # se sbaglia a leggere i container
ev3.speaker.play_notes(notes=['C5/8_', 'E5/8_', 'G5/4'],tempo=200)

#testGrabber()

robot.resetGyro()

aspettaIlVia()
timer.reset()
#grabber.unloadBuffer()
#grabber.retract(False)

#leggiOrdineRifornisciBarca() 
#print("BARCA RIFORNITA @ "+str(timer.time()/1000))

#curvaVersoContainer()
#print("ATTRACCO @ "+str(timer.time()/1000))

prendiTuttiContainer(order1, order2)
print("PRESI CONTAINER @ "+str(timer.time()/1000))

portaBarcaGrandeFuori()
print("BARCA GRANDE FUORI @ "+str(timer.time()/1000))

prendiBarcaPiccola()

ev3.screen.print("time: "+str(timer.time()/1000))
print("MISSIONE COMPLETATA @ "+str(timer.time()/1000))
#TODO invert sign of angle parameter in headTo method
