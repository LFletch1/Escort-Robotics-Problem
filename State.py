import skgeom as sg

class State:
    def __init__(self, position, contaminated_shadows, safezones, neighbors=[]):
        self.pos = position # Tuple (x,y)
        self.contaminated_shadows = contaminated_shadows # PolygonSet
        self.safezones = safezones  # PolygonSet
        self.neighbors = self.neighbors # List[(x,y),(x,y)]

    def as_point(self):
        return sg.Point2(self.pos[0], self.pos[1])