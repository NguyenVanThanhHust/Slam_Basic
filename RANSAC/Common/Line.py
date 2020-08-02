import math

from .Point import Point

class Line:
    def __init__(self, a, b, c):
        self.a = a
        self.b = b
        self.c = c 

    def display(self):
        s="A=%f B=%f C=%f") % (self.a,self.b,self.c)
        return s

    # distance between line and point
    def compute_distance(self, p:Point):
        numerator = math.abs(self.a * p.x + self.b * p.y + self.c)
        denominator = math.sqrt(self.a ** 2 + self.b**2)
        return numerator / denominator

    def get
    