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
SPIN_SPEED = 360 # deg/s
SPIN_ACC = 400 # deg/s/s

timer = StopWatch()

motorRight = Motor(Port.C)
motorLeft = Motor(Port.B)

# costruttore robot
robot = Robot(motorLeft, motorRight, WHEEL_DIAM, WHEEL_DIST, sensorLine, sensorIntersections, sensorBlocks, GYRO_PORT)
robot.settings(TRAVEL_SPEED, TRAVEL_ACC, SPIN_SPEED, SPIN_ACC)

# settings for line following
robot.lineFollowerSettings(speed=120, target=45, gain=0.4, darkThreshold = 10 )

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

grabber = Grabber(Port.A)
#grabber.calibrate()
#grabber.unloadBuffer()
#grabber.prepareForGrabbing()

ev3.speaker.beep()
wait(400)

ev3.speaker.beep()
ev3.screen.clear()
timer.reset()

#TODO aggiungi dritto con giroscopio

timer.reset()
# gira in senso antiorario di 45 gradi
#robot.Scurve(30,60.80)
robot.spin(25) 
robot.straight(70)
robot.headTo(0)
#robot.spin(-20)

robot.seguiLineaPerDistanza(145)

# leggi blocco ordine 1
#TODO read until valid
# eventualmente sterzatina verso sx per avvicinarsi al blocco
ev3.speaker.beep()
order1 = sensorBlocks.getRobustColor()
ev3.screen.print("color1: "+str(order1))
print("color1: "+str(order1))

robot.seguiLineaPerDistanza(48)

ev3.speaker.beep()
order2 = sensorBlocks.getRobustColor()
ev3.screen.print("color1: "+str(order2))
print("color2: "+str(order2))
#ev3.screen.print("time: "+str(timer.time()))

robot.seguiLineaFinoAIncrocio(100)
robot.gyro.reset_angle(0)
robot.straight(-180)

# prendi barca
robot.Scurve(73,280,120)
robot.straightGyroForDistance(distance=220, maxSpeed=150)
#robot.straight(235)
print(timer.time())
ev3.screen.print(timer.time())

#robot.arc(WHEEL_DIST/2,-45)
#robot.arc(300,45)

#wait(2000)
