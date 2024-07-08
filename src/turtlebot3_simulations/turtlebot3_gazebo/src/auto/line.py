#!/usr/bin/env python3
import numpy as np

class Line():
    def __init__(self, frameDimensions, color):
        self.frameDimensions = frameDimensions
        self.color = color
        self.w = frameDimensions[0]
        self.h = frameDimensions[1]
        self.x = []
        self.y = []
        self.poly = None

    def add(self, x0, y0, x1, y1):
        self.x.extend([x0, x1])
        self.y.extend([y0, y1])

    def clear(self):
        self.x = []
        self.y = []

    def fit(self):
        if len(self.x) > 0 and len(self.y) > 0:
            self.poly = np.poly1d(np.polyfit(self.y, self.x, deg=1))
        else:
            self.poly = None
        return self.poly

    def eval(self, y0, y1):
        y0Px = int(self.h * y0)
        y1Px = int(self.h * y1)
        if self.poly:
            return [int(self.poly(y0Px)), y0Px, int(self.poly(y1Px)), y1Px]
        else:
            return [0, y0Px, 0, y1Px]
