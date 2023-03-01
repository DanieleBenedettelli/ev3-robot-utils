#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.iodevices import Ev3devSensor                                 
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog

from pybricks.media.ev3dev import SoundFile, ImageFile
from math import atan2

from utilities.HSVColorSensor import HSVColorSensor
from utilities.Robot import Robot


# https://pybricks.com/ev3-micropython/

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

ev3 = EV3Brick()
sensorBlocks = HSVColorSensor(Port.S4)
sensorLine = ColorSensor(Port.S1)
sensorIntersections = ColorSensor(Port.S3)

"""
while True:
    #(r,g,b) = sensorBlocksRaw.read('RGB-RAW')
    color = sensorBlocks.getColor()
    print("COLOR: ", str(color))
    wait(200)
"""


WHEEL_DIAM = 56 # mm/s
WHEEL_DIST = 96 # mm/s
TRAVEL_SPEED = 100 # mm/s
TRAVEL_ACC = 600 # mm/s/s
SPIN_SPEED = 360 # deg/s
SPIN_ACC = 400 # deg/s/s

timer = StopWatch()

motorRight = Motor(Port.C)
motorLeft = Motor(Port.B)

# costruttore robot
robot = Robot(motorLeft, motorRight, WHEEL_DIAM, WHEEL_DIST, sensorLine, sensorIntersections)
robot.settings(TRAVEL_SPEED, TRAVEL_ACC, SPIN_SPEED, SPIN_ACC)

# settings for line following
robot.lineFollowerSettings(speed=200, target=45, gain=0.3, darkThreshold = 10 )

# PROGRAMMA

ev3.speaker.beep()
ev3.screen.clear()
timer.reset()



# gira in senso antiorario di 45 gradi
robot.spin(25) #45
robot.straight(70)
robot.spin(-20)

robot.seguiLineaPerDistanza(143)

# leggi blocco ordine 1
#TODO read until valid
# eventualmente sterzatina verso sx per avvicinarsi al blocco
ev3.speaker.beep()
order1 = sensorBlocks.getColor()
ev3.screen.print("color1: "+str(order1))

robot.seguiLineaPerDistanza(48)

ev3.speaker.beep()
order2 = sensorBlocks.getColor()
ev3.screen.print("color2: "+str(order2))
ev3.screen.print("time: "+str(timer.time()))
wait(2000)


"""
robot.seguiLineaFinoAIncrocio()
wait(1000)
# prendi barca
robot.spin(62)
robot.straight(60)
robot.spin(-60)

"""