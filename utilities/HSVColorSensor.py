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
        # self.sensor = ColorSensor(port)

    def getHSV(self):
        # r,g,b = self.rgb()
        (r, g, b) = self.read('RGB-RAW')
        _min = min(r, g, b)
        _max = max(r, g, b)
        # print("RGB RAW:%3d %3d %3d"%(r,g,b))

        value = int(_max)
        delta = _max - _min
        if delta == 0:
            saturation = 0
            hue = 0
        else:
            if _max > 0:
                saturation = int(100.0 * delta / _max)
            else:
                saturation = 0
                hue = 0

            if r == _max:
                hue = 0.0 + (g-b) / delta
            elif g == _max:
                hue = 2.0 + (b-r) / delta
            else:  # b == _max
                hue = 4.0 + (r-g) / delta
            hue *= 60.0
            while hue < 0:
                hue += 360
        hue = int(hue)
        return hue, saturation, value

    def getColor(self, longRange=True, printHSV = False):
        h, s, v = self.getHSV()
        if printHSV:
            print("HSV   :%3d %3d %3d"%(h,s,v))
        reading = None
        if longRange: # solo per l'ordine
            if h > 65 and h < 185 and v > 2 and v < 150:
                reading = Color.GREEN
            elif h > 185 and h < 245 and v > 2 and v < 105:
                reading = Color.BLUE
            elif s > 14 and s < 55 and v > 20:
                reading = Color.WHITE                
            #elif h > 20 and h < 45 and s > 60 and v > 30:
            #    reading = Color.YELLOW     
            #elif (h < 10 or h > 330) and s > 70 and v > 22:
            #    reading = Color.RED                                

        else: #short range
            if h > 65 and h < 185 and s > 64 and v > 20:
                reading = Color.GREEN
            elif h > 190 and h < 245 and s > 45 and v > 30: # v>50
                reading = Color.BLUE
            elif h > 20 and h < 45 and s > 60 and v > 30:
                reading = Color.YELLOW 
            elif (h < 10 or h > 330) and s > 70 and v > 22:
                reading = Color.RED                       
            elif s < 25 and v > 50: # v > 40
                reading = Color.WHITE
            
        #print("color:", reading)
        return reading

    def getRobustColor(self, colors=[Color.BLUE, Color.GREEN], samples=20, longRange=True):
        count = [0]*len(colors)
        for i in range(samples):
            sample = self.getColor(longRange)
            for k in range(len(colors)):
                if sample == colors[k]:
                    # increase count for the color that is being detected
                    count[k] += 1
        i = count.index(max(count))
        return colors[i]
