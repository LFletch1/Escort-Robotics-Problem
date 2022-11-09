import skgeom as sg
from skgeom.draw import draw
import matplotlib.pyplot as plt

poly = sg.Polygon([sg.Point2(0, 0), sg.Point2(0, 3), sg.Point2(3, 3)])

draw(poly)

plt.show()