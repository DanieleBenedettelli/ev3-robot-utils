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
sensorIntersections = ColorSensor(Port.S3)
sensorLine = ColorSensor(Port.S2)
GYRO_PORT = Port.S1
GRABBER_PORT = Port.D

WHEEL_DIAM = 56 # mm/s
WHEEL_DIST = 96 # mm/s
TRAVEL_SPEED = 250 # mm/s
TRAVEL_ACC = 350 # mm/s/s
SPIN_SPEED = 500 # deg/s
SPIN_ACC = 300 # deg/s/s

timer = StopWatch()

motorRight = Motor(Port.C)
motorLeft = Motor(Port.B)
grabber = Grabber(GRABBER_PORT)

# costruttore robot
robot = Robot(motorLeft, motorRight, WHEEL_DIAM, WHEEL_DIST, sensorLine, sensorIntersections, sensorBlocks, GYRO_PORT, grabber)
robot.settings(TRAVEL_SPEED, TRAVEL_ACC, SPIN_SPEED, SPIN_ACC)

# settings for line following
#robot.lineFollowerSettings(speed=90, target=45, gain=0.55, darkThreshold = 10, whichSensor=Side.RIGHT, whichBorder = Side.LEFT )
colorTones = {Color.BLUE:2000, Color.GREEN:1000, Color.WHITE:4000, None: 500}

def aspettaIlVia():
  ev3.light.on(Color.ORANGE)
  while not ev3.buttons.pressed():
    wait(100)
  ev3.light.on(Color.GREEN)  

# esci da base, leggi ordine, prendi barca grande e riforniscila
def leggiOrdineRifornisciBarca():
  global order1, order2
  timer.reset()

  robot.straightUntilLine(black_thr=20,maxSpeed=50) 
  robot.straight(20)
  robot.lineFollowerSettings(speed=90, target=30, gain=0.2, darkThreshold = 10, whichSensor=Side.RIGHT, whichBorder = Side.LEFT )
  robot.followLineForDistance(DISTANZA_ORDINE)

  # leggi blocco ordine 1
  order1 = sensorBlocks.getRobustColor()
  ev3.screen.print("color1: "+str(order1))
  print("color1: "+str(order1))
  ev3.speaker.beep(frequency=colorTones[order1])

  robot.followLineForDistance(48)

  order2 = sensorBlocks.getRobustColor()
  ev3.screen.print("color2: "+str(order2))
  print("color2: "+str(order2))
  ev3.speaker.beep(frequency=colorTones[order2])

  #print("follow until X")
  robot.followLineUntilIntersection(thr=20, speed = 50)
  wait(100)
    
  #print("reset gyro")
  robot.gyro.reset_angle(0) # UNICO RESET DA FARE

  #print("vai indietro")
  #robot.straightGyroForDistance(distance=-180, maxSpeed = 90, absoluteHeading=True)
  robot.straight(distance=-180)

  # prendi barca
  #print("curva S")
  robot.Scurve(dx=65, dy=280,angSpeed=120)
  
  #robot.straightGyroForDistance(distance=DISTANZA_PER_RIFORNIMENTO, maxSpeed=150)
  robot.straight(DISTANZA_PER_RIFORNIMENTO)
  robot.straight(10) 
  robot.straight(-10) 


# curva con la barca verso i container
def curvaVersoContainer():
  timer.reset()
  robot.turnSpeed = 90

  robot.arc(angle=-45, radius=-WHEEL_DIST/2, maxAngSpeed=100)
  
  robot.straight(DISTANZA_AVVICINAMENTO_CONTAINER) # determina avvicinamento ai container
    
  robot.arc(angle=HEADING_CONTAINER, radius=-WHEEL_DIST/2, maxAngSpeed=50)

  robot.straightGyroForDistance(distance=50,maxSpeed=80, absoluteHeading=True,headingOffset=HEADING_CONTAINER) 
  #robot.turnSpeed = 270
  print(timer.time())

  ev3.screen.print(timer.time())

def testGrabber():
  ev3.screen.print("TEST GRABBER")
  robot.grabber.prepareForGrabbing()
  while True:
    if Button.LEFT in ev3.buttons.pressed():
        robot.grabContainer()
    if Button.RIGHT in ev3.buttons.pressed():
        robot.grabber.unloadBuffer()
        robot.grabber.retract(False)


def testContainerColor() :
  while Button.CENTER not in ev3.buttons.pressed():
    color=sensorBlocks.getColor(longRange=False, printHSV=True)
    print("************* ", color)
    wait(500)

def prendiTuttiContainer(c1=Color.GREEN, c2=Color.BLUE):

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
  
  grabber.retract() # rilascia eventuali container incastrati sotto
  robot.straightGyroForDistance(distance=DISTANZA_PRIMA_CONTAINER_BIANCHI, maxSpeed = 120, steerGain = 4, absoluteHeading=True, headingOffset=heading0) 
  grabber.prepareForGrabbing(False)

  found = robot.straightGyroUntilContainer(maxSpeed = 40, colors=[Color.WHITE], headingToKeep=heading0, maxDistance=270, longRange=True)
  if found == 1:
    ev3.speaker.beep()
    robot.manageWhiteContainers(heading0)
    found = robot.straightGyroUntilContainer(maxSpeed = 40, colors=[Color.WHITE],headingToKeep=heading0,  maxDistance=80, longRange=True)
  if found == 1:
    ev3.speaker.beep()
    robot.manageWhiteContainers(heading0)


def portaBarcaGrandeFuori():
  robot.gyro.reset_angle(HEADING_CONTAINER) # TODO remove, it's just for debug from here
  robot.grabber.retract()


  """
  robot.arc(angle=50, radius=-WHEEL_DIST/2, maxAngSpeed=100) #angle was 45

  # TODO rendi più robusto andare verso la linea, indipendentemente dall'aver preso i container bianchi o meno
  robot.straight(100)
  robot.straightUntilLine(white_thr=60, black_thr=20)
  robot.straight(35) # was 50

  # sterza imperniato a sx finché non vede la linea
  robot.stop() 
  motorLeft.hold()
  motorRight.run(130)
  print("prima: ",motorRight.angle())
  while sensorLine.reflection()<60:
    wait(10)
  while sensorLine.reflection()>20:
    wait(10)
  while sensorLine.reflection()<60:
    wait(100)    
  motorRight.brake()
  print("dopo: ",motorRight.angle())
  #motorRight.run_angle(speed=100,rotation_angle=60)
  motorLeft.stop()

  robot.lineFollowerSettings(speed=100, target=35, gain=0.7, darkThreshold = 10)
  robot.followLineForDistance(distance=280,speed=120, brake=False)

  robot.lineFollowerSettings(speed=100, target=35, gain=0.6, darkThreshold = 10)
  robot.followLineUntilIntersection(speed=90)
  """
  robot.curveForward(radius=-WHEEL_DIST/2, maxAngSpeed=80)
  while sensorLine.reflection() < 60 :
    #print("W:", sensorLine.reflection())
    wait(10)
  ev3.speaker.beep(1000,10)
  while sensorLine.reflection() > 20 :
    #print("B:", sensorLine.reflection())
    wait(10)
  ev3.speaker.beep(2000,10)  
  robot.straight(0)

  robot.followLineUntilIntersection(speed=100)
  robot.straight(100)

  robot.curveForward(radius=WHEEL_DIST/2, maxAngSpeed=80)
  while sensorLine.reflection() < 60 :
    #print("W:", sensorLine.reflection())
    wait(10)
  ev3.speaker.beep(1000,10)    
  while sensorLine.reflection() > 20 :
    #print("B:", sensorLine.reflection())
    wait(10)
  ev3.speaker.beep(2000,10)      
  while sensorLine.reflection() < 60 :
    #print("B:", sensorLine.reflection())
    wait(10)
  ev3.speaker.beep(1000,10)      
  robot.straight(0)

  robot.followLineForDistance(distance=280,speed=120, brake=False)
  robot.followLineUntilIntersection(speed=90)
  
  robot.gyro.reset_angle(0)
  robot.straight(73)# allinea anello container rosso con gancio gru
  robot.straight(-220)
  robot.arc(45)
  robot.straight(220)
  robot.arc(0)
  robot.straight(150)
  robot.arc(90)
  
  robot.lineFollowerSettings(speed=80, target=30, gain=0.4, darkThreshold = 10, whichSensor=Side.LEFT, whichBorder=Side.RIGHT)
  robot.followLineForDistance(distance = DISTANZA_GRU,speed=180)
  #robot.straightGyroForDistance(distance=DISTANZA_GRU,maxSpeed=200,absoluteHeading=False) # avvicinamento alla gru

  #print("indietro")
  robot.drive(-180,0)
  while (sensorIntersections.reflection()>30 or sensorLine.reflection()>30):
    #print("R: ", sensorIntersections.reflection())
    #print("L: ", sensorLine.reflection())
    wait(5)
  robot.straight(0)
  #print("avanti")  
  robot.straightGyroForDistance(distance=DISTANZA_DOPO_GRU, maxSpeed = 200, absoluteHeading=False) # regola posizione sensore su bordo linea
  robot.arc(180)

def prendiBarcaPiccola():
  robot.lineFollowerSettings(speed=80, target=30, gain=0.4, darkThreshold = 10, whichSensor=Side.RIGHT, whichBorder=Side.LEFT)
  robot.followLineForDistance(distance=480 , speed=200, brake=False)

  robot.lineFollowerSettings(speed=100, target=30, gain=0.4, darkThreshold = 10, whichSensor=Side.RIGHT, whichBorder=Side.LEFT)
  robot.followLineUntilIntersection(speed=100,brake=False)

  robot.followLineForDistance(distance=480 , speed=200, brake=False)  
  robot.followLineUntilIntersection(speed=100)

  wait(100)
  robot.gyro.reset_angle(180)

  robot.followLineForDistance(430,speed=200)
  wait(100)

  # arco per prendere barca
  robot.arc(radius=93, angle=180, maxAngSpeed=100 )  # radius era 95
  
  robot.straightGyroForDistance(600, maxSpeed=200, absoluteHeading=True)

  robot.straightUntilLine(maxSpeed=150)
  
  robot.arc(angle = 45, maxAngSpeed=90)

  robot.straight(150)
  robot.straightUntilLine(maxSpeed=150)
  robot.straight(140)
  
  robot.arc(angle=-45,radius=-WHEEL_DIST/2,maxAngSpeed=90)

  robot.lineFollowerSettings(speed=150,target=40,gain=0.3,darkThreshold=10,whichSensor=Side.LEFT,whichBorder=Side.RIGHT)
  robot.followLineForDistance(distance=300, speed=150, brake=False) 
  robot.followLineUntilIntersection(speed=150)
  robot.arc(radius=100, angle=90, maxAngSpeed=140)

  robot.straight(20)
  # carica barca piccola
  robot.grabber.unloadBuffer()
  robot.grabber.retract(False)
  robot.straight(-48)
  robot.grabber.unloadBuffer()
  robot.grabber.retract(False)

  robot.straight(230) # end of mission

"""
while True:
  robot.arc(angle=90,absoluteHeading=True,radius = -WHEEL_DIST/2)
  print(robot.readGyro())
  robot.arc(angle=0,absoluteHeading=True, radius = -WHEEL_DIST/2)
  print(robot.readGyro())
  robot.arc(angle=-90,absoluteHeading=True, radius = -WHEEL_DIST/2)
  print(robot.readGyro())
  robot.arc(angle=0,absoluteHeading=True, radius = -WHEEL_DIST/2)
  print(robot.readGyro())

while True:
  robot.turnGyro(angle=45,absoluteHeading=True)
  print(robot.readGyro())
  robot.turnGyro(angle=0,absoluteHeading=True)
  print(robot.readGyro())
  robot.turnGyro(angle=-45,absoluteHeading=True)
  print(robot.readGyro())
  robot.turnGyro(angle=0,absoluteHeading=True)
  print(robot.readGyro())
"""

"""
  __  __    _    ___ _   _ 
 |  \/  |  / \  |_ _| \ | |
 | |\/| | / _ \  | ||  \| |
 | |  | |/ ___ \ | || |\  |
 |_|  |_/_/   \_\___|_| \_|
                           
"""
# DATA: 12/05/2023
# VERSIONE 1.3
#TODO cambia prendiTuttiContainer() per caricamento container: caricare o accumulare tutti i colorati, caricare un bianco su nave grande
#TODO irrobustire navigazione per portare nave grande fuori


order1 = Color.BLUE
order2 = Color.GREEN

HEADING_CONTAINER = -90 #gradi

# tutte distanze in millimetri
DISTANZA_ORDINE = 144
DISTANZA_PER_RIFORNIMENTO = 185 
DISTANZA_AVVICINAMENTO_CONTAINER = 275 #275 # senza rilevamento banchina
DISTANZA_PRIMA_CONTAINER_BIANCHI = 290 #250
DISTANZA_GRU = 390 
DISTANZA_DOPO_GRU = 200 

grabber.calibrate()
robot.resetGyro() # non scollegare e ricollegare il giroscopio, fa tutto da sé!

ev3.speaker.set_volume(100)
ev3.speaker.play_notes(notes=['C5/8_', 'E5/8_', 'G5/4'],tempo=200)

#testContainerColor() # se sbaglia a leggere i container
#testGrabber() # per testare la pinza

# porta barca grande fuori

aspettaIlVia()

timer.reset()

grabber.run_until_stalled(500, then=Stop.HOLD, duty_limit=90)
grabber.retract(False)

leggiOrdineRifornisciBarca() 
print("BARCA RIFORNITA @ "+str(timer.time()/1000))

curvaVersoContainer()
print("ATTRACCO @ "+str(timer.time()/1000))

prendiTuttiContainer(order1, order2)
print("PRESI CONTAINER @ "+str(timer.time()/1000))

portaBarcaGrandeFuori()
print("BARCA GRANDE FUORI @ "+str(timer.time()/1000))

prendiBarcaPiccola()

ev3.screen.print("time: "+str(timer.time()/1000))
print("MISSIONE COMPLETATA @ "+str(timer.time()/1000))
#TODO invert sign of angle parameter in headTo method
