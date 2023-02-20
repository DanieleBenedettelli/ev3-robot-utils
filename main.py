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
sensorBlocksRaw = Ev3devSensor(Port.S4)

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
robot.lineFollowerSettings(speed=200, target=50, gain=0.7, darkThreshold = 10 )

ev3.speaker.beep()
ev3.screen.clear()
timer.reset()

robot.Scurve(50,100, 50)
wait(1000)

# gira in senso antiorario di 45 gradi
#robot.spin(45) #45
#motoreDestro.run_angle(rotation_angle=180, speed = 300)
#motoreSinistro.run_angle(rotation_angle=180, speed = 300)
# vai avanti di 85mm
#robot.straight(90)

# gira in senso orario di 45 gradi
#robot.spin(-40)

# seguilinea per 135mm
robot.seguiLineaPerDistanza(130)

# leggi blocco ordine 1
#TODO read until valid
# eventualmente sterzatina verso sx per avvicinarsi al blocco
ev3.speaker.beep()
order1 = sensorBlocks.getColor()

ev3.screen.print("color1: "+str(order1))
#wait(1000)

# avanza di 55mm 
robot.seguiLineaPerDistanza(55)
# robot.straight(55)

# leggi blocco ordine 2
#TODO read until valid
ev3.speaker.beep()
order2 = sensorBlocks.getColor()
ev3.screen.print("color2: "+str(order2))
ev3.screen.print("time: "+str(timer.time()))
wait(5000)
