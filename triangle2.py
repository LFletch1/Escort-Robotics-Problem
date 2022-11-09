import numpy as np
import math

# import scikit-geometry modules
import skgeom as sg
from scikit_utils import *

class Triangle2:
    """
    A Triangle class composed of sg.Point2 objects
    """
    def __init__(self, a, b, c):
        if isinstance(a, sg.Point2):
            self._p1 = a
        else:
            # assuming it is a list, array, or np.array
            self._p1 = sg.Point2(a[0], a[1])

        if isinstance(b, sg.Point2):
            self._p2 = b
        else:
            # assuming it is a list, array, or np.array
            self._p2 = sg.Point2(b[0], b[1])

        if isinstance(c, sg.Point2):
            self._p3 = c
        else:
            # assuming it is a list, array, or np.array
            self._p3 = sg.Point2(c[0], c[1])

        # Use Heron's formula
        #   semi_perimeter: s = (p1 + p2 + p3)/2
        point1 = self.p1.to_numpy() #np.array([float(self.p1.x()), float(self.p1.y())])
        point2 = self.p2.to_numpy() #np.array([float(self.p2.x()), float(self.p2.y())])
        point3 = self.p3.to_numpy() #np.array([float(self.p3.x()), float(self.p3.y())])
        x = np.linalg.norm(point1 - point2)
        y = np.linalg.norm(point2 - point3)
        z = np.linalg.norm(point3 - point1)
        s = (x + y + z) / 2
        #   Area = sqrt(s (s-x)(s-y)(s-z))
        self._area = math.sqrt((s*(s-x)*(s-y)*(s-z)))

    @property
    def p1(self):
        return self._p1

    @property
    def p2(self):
        return self._p2

    @property
    def p3(self):
        return self._p3

    @property
    def area(self):
        return self._area

    def to_numpy(self):
        return np.array([ (float(self.p1.x()), float(self.p1.y())), (float(self.p2.x()), float(self.p2.y())), (float(self.p3.x()), float(self.p3.y()))])

    def __repr__(self):
        return f"Triangle({self.p1}, {self.p2}, {self.p3})"

    def random_point(self) -> {'type': sg.Point2, 'docstring': "Returns a random point in a Triangle"}:
        """
        Shape Distributions - Robert Osada, Thomas Funkhouser, Bernard Chazelle, and David Dobkin
        https://www.cs.princeton.edu/~funk/tog02.pdf -- Section 4.2 -- Equation 1
        """
        r1 = np.random.rand(1)
        r2 = np.random.rand(1)
        s1 = math.sqrt(r1)
        pts = self.to_numpy() # triangle flat numpy.ndarray
        pt = pts[0]* (1.0 - s1) + pts[1] * (1.0 - r2) * s1 + pts[2] * r2 * s1
        return sg.Point2(pt[0], pt[1])