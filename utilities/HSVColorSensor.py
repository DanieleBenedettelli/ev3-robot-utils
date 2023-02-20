""" Python package for HSV Color Sensor

Author  : Daniele Benedettelli
Date    : February 2023
Version : 0.1

"""

from pybricks.ev3devices import ColorSensor
from pybricks.parameters import Port, Color
from pybricks.iodevices import Ev3devSensor  

class HSVColorSensor(Ev3devSensor):
    """ This class contains all general functionalities of Pixy.

    Keyword arguments:
    port        -- Port which the color sensor is connected to (Port.S1 ...)

    """

    def __init__(self, port=Port.S1):
        super().__init__(port)
        #self.sensor = ColorSensor(port)

    def getHSV(self):
        #r,g,b = self.rgb()
        (r,g,b) = self.read('RGB-RAW')
        _min = min(r,g,b)
        _max = max(r,g,b)
        #print("RGB RAW:%3d %3d %3d"%(r,g,b))

        value = int(_max)
        delta = _max - _min
        if delta == 0: 
            saturation = 0
            hue = 0
        else : 
            if _max > 0 :
                saturation = int(100.0* delta / _max)
            else :
                saturation = 0
                hue = 0
                
            if r == _max :
                hue = 0.0 + (g-b) / delta
            elif g == _max:
                hue = 2.0 + (b-r) / delta
            else : #b == _max
                hue = 4.0 + (r-g) / delta
            hue *= 60.0    
            while hue < 0 : 
                hue += 360
        hue = int(hue)
        return hue, saturation, value

    def getColor(self):

        h,s,v = self.getHSV()
        print("HSV   :%3d %3d %3d"%(h,s,v))
        reading = None
        if s > 80 or (s>45 and v > 10):
            if h > 70 and h < 160:
                reading = Color.GREEN
            elif h > 175 and h < 240:
                reading = Color.BLUE
        elif v>20:
            reading = Color.WHITE
            #else : 
            #    reading = Color.NONE
        return reading

