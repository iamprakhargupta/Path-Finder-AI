"""
CSCI-630: Foundation of AI
Lab1 Code - pixel.py
Author: Prakhar Gupta
Username: pg9349

To store pixel informationn



"""

class Pixel:
    def __init__(self, speed=0, type=None, pixelvalue=(0, 0, 0), el=0):
        self.speed = speed
        self.type = type
        self.pixelvalue = pixelvalue
        self.el = el

    def __str__(self):
        """
        Return a string representation of the vertex and its direct neighbors:

            vertex-id connectedTo [neighbor-1-id, neighbor-2-id, ...]

        :return: The string
        """
        return str(self.speed) + " " + str(self.type) + " " + str(
            self.pixelvalue) + " " + str(self.el)