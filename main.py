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
from utilities.Robot import Robot
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
robot.lineFollowerSettings(speed=90, target=45, gain=0.3, darkThreshold = 10 )

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

"""
  __  __    _    ___ _   _ 
 |  \/  |  / \  |_ _| \ | |
 | |\/| | / _ \  | ||  \| |
 | |  | |/ ___ \ | || |\  |
 |_|  |_/_/   \_\___|_| \_|
                           
"""

def ASPETTA_VIA():
  ev3.light.on(Color.ORANGE)
  while Button.LEFT not in ev3.buttons.pressed():
    wait(100)
  ev3.light.on(Color.GREEN)  

# esci da base, leggi ordine, prendi barca grande e riforniscila
def FASE1():
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
  robot.seguiLineaFinoAIncrocio(thr=15, speed = 100) #TODO debug, non viene eseguito
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
  robot.seguiLineaFinoAIncrocio()
  robot.gyro.reset_angle(180)
  robot.arc(radius=105, angle=187, speed=150) 
  wait(1000)
  robot.turnSpeed = 60
  robot.headTo(angle=0)
  robot.straightGyroForDistance(400)
"""
while True:
  grabber.prepareForGrabbing()
  ev3.speaker.beep()
  while Button.CENTER not in ev3.buttons.pressed():
    wait(10)
  
  grabber.lift()
  grabber.unloadOnRamp()
"""

# CALIBRAZIONE INIZIALE
ev3.speaker.play_notes(notes=['C5/8_', 'E5/8_', 'G5/4'],tempo=200)
"""
while True:
  sensorBlocks.getColor(False)
  wait(200)
"""

grabber.calibrate()

#ASPETTA_VIA()
#grabber.unloadBuffer()
grabber.retract()
FASE1()
#FASE2()

# PRENDERE CONTAINER
grabber.prepareForGrabbing()

# OVERRIDE ORDER COLOR2
c1 = Color.GREEN
c2 = Color.BLUE

done = False

robot.resetContainerLogic()
robot.gyro.reset_angle(0)

# TODO DEBUG THIS
while not done:
    #avanza finché non vedi un colore
    # TODO  continuare
    color = robot.straightGyroUntilContainer(maxSpeed = 40, colors=[Color.BLUE, Color.GREEN])
    ev3.speaker.beep()
    wait(100)
    seenColor = sensorBlocks.getRobustColor()
    done = robot.manageContainer(c1, c2, seenColor)

# TODO da collaudare
robot.straightGyroForDistance(260, maxSpeed = 120, absoluteHeading=True) 

robot.straightGyroUntilContainer(maxSpeed = 40, colors=[Color.WHITE])
ev3.speaker.beep()
robot.manageWhiteContainers()
robot.straightGyroUntilContainer(maxSpeed = 40, colors=[Color.WHITE])
ev3.speaker.beep()
robot.manageWhiteContainers()

