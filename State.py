import skgeom as sg
from scikit_utils import get_tuple_from_polygon_set

class State:
    def __init__(self, position, parent, contaminated_shadows, safezones, neighbors=[]):
        self.pos = position # Tuple (x,y)
        self.contaminated_shadows = contaminated_shadows # PolygonSet
        self.safezones = safezones  # PolygonSet
        self.neighbors_pos = neighbors # List[(x,y),(x,y)]
        self.safezones_tuple = get_tuple_from_polygon_set(self.safezones)
        self.shadows_tuple = get_tuple_from_polygon_set(self.contaminated_shadows)
        self.parent = parent

    def as_point(self):
        return sg.Point2(self.pos[0], self.pos[1])

    def __eq__(self, __o: object) -> bool:
        pass

    def __eq__(self, other):
        if type(self) != type(other):
            return False
        shadows_diff = self.contaminated_shadows.difference(other.contaminated_shadows)
        # print(shadows_diff)
        if shadows_diff:
            return False
        safezones_diff = self.safezones.difference(other.safezones)
        # print(safezones_diff)
        if safezones_diff:
            return False
        if self.pos[0] != other.pos[0] or self.pos[1] != other.pos[1]:
            return False
        return True

        # shadows_diff = self.contaminated_shadows.difference(other.contaminated_shadows)

        # return other and self.a == other.a and self.b == other.b
    # def get_polygon_set_tuples(self):


    def __ne__(self, other):
        return not self.__eq__(other)

    def __hash__(self):
        return hash((self.pos, self.safezones_tuple, self.shadows_tuple))