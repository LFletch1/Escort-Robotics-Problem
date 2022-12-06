import skgeom as sg
from skgeom.draw import draw
import matplotlib.pyplot as plt

# Notes: To access a segment's two points use seg.point(0) and seg.point(1)

def poly_from_coords(coords):
    point2List = []
    for coord in coords:
        point2List.append(sg.Point2(coord[0],coord[1]))
    return sg.Polygon(point2List)

def get_segments_from_coords(coords):
    segments = []
    for i in range(1,len(coords)):
        a = sg.Point2(coords[i-1][0],coords[i-1][1])
        b = sg.Point2(coords[i][0],coords[i][1])
        seg = sg.Segment2(a,b)
        segments.append(seg)
    return segments

def lines_share_endpoint(seg1, seg2):
    if seg1.point(0) == seg2.point(0) or seg1.point(0) == seg2.point(1):
        return True
    elif seg1.point(1) == seg2.point(0) or seg1.point(1) == seg2.point(1):
        return True
    else:
        return False

def get_shared_point(seg1, seg2):
    if seg1.point(0) == seg2.point(0):
        return seg1.point(0)
    elif seg1.point(0) == seg2.point(1):
        return seg1.point(0)
    elif seg1.point(1) == seg2.point(0):
        return seg1.point(1)
    elif seg1.point(1) == seg2.point(1):
        return seg1.point(1)
    else:
        return None

def reflex_angle_point(seg1, seg2, polygon_set):
    share_point = get_shared_point(seg1, seg2)
    if share_point == None:
        return None

    if seg1.point(0) == share_point:
        target_point = seg1.point(1)
    else:
        target_point = seg1.point(0)
    seg1_delta_x = target_point.x() - share_point.x()
    seg1_delta_y = target_point.y() - share_point.y()

    if seg2.point(0) == share_point:
        target_point = seg2.point(1)
    else:
        target_point = seg2.point(0)
    seg2_delta_x = target_point.x() - share_point.x()
    seg2_delta_y = target_point.y() - share_point.y()

    point_along_seg1 = sg.Point2((share_point.x() + (seg1_delta_x / 10)), (share_point.y() + (seg1_delta_y / 10)))
    point_along_seg2 = sg.Point2((share_point.x() + (seg2_delta_x / 10)), (share_point.y() + (seg2_delta_y / 10)))
    mid_x = (point_along_seg1.x() + point_along_seg2.x()) / 2
    mid_y = (point_along_seg1.y() + point_along_seg2.y()) / 2
    mid_point = sg.Point2(mid_x, mid_y)
    if not polygon_set.locate(mid_point):
        return share_point
    else:
        return None

# def slope(seg):
#     '''return slope of line segment'''
#     slop

def get_environment_reflex_angle(segments, env_polygon):
    '''Segments must be given in an order where adjacent vertexes in the segmetns list are adjacent'''
 
    polygon_set = sg.PolygonSet([env_polygon])
    reflex_angle_points = []
    for i in range(1,len(segments)):
        point = reflex_angle_point(segments[i-1], segments[i], polygon_set)
        if point:
            reflex_angle_points.append(point)

    # Check first segment last segment in segments list
    last_point = reflex_angle_point(segments[0], segments[1], polygon_set)
    if last_point:
        reflex_angle_points.append(last_point) 

    return reflex_angle_points


def get_environment_corner_lines(coords):
    poly = poly_from_coords(coords) 
    polygon_set = sg.PolygonSet([poly])

    env_edges = get_segments_from_coords(coords)
    env_rays = []
    for i in range(len(coords)):
        for j in range(i+1, len(coords)):

            a = sg.Point2(coords[i][0],coords[i][1])
            b = sg.Point2(coords[j][0],coords[j][1])
            line = sg.Segment2(a,b)

            no_intersections = True
            for edge in env_edges:
                intersect = sg.intersection(edge,line)

                if intersect != None:
                    # print(lines_share_endpoint(line,segment))
                    if not lines_share_endpoint(line, edge):
                        no_intersections = False
                        break
                
            if no_intersections:
                mid_x = (line.point(0).x() + line.point(1).x()) / 2
                mid_y = (line.point(0).y() + line.point(1).y()) / 2
                mid_point = sg.Point2(mid_x ,mid_y)
                if polygon_set.locate(mid_point):
                    env_rays.append(line)
    return env_rays
    

# coords = [(0,0),(0,6),(5,6),(5,7),(3,7),(3,9),(9,9),(9,7),(7,7),(7,6),\
#             (11,6),(11,4),(7,4),(7,3),(9,3),(9,1),(3,1),(3,3),(5,3),(5,4),(2,4),(2,0)]

coords = [(2,0),(2,4),(5,4),(5,3),(3,3),(3,1),(9,1),(9,3),(7,3),(7,4),(11,4),(11,6),(7,6),(7,7),(9,7),(9,9),(3,9),(3,7),(5,7),(5,6),(0,6),(0,0)]



test_poly = poly_from_coords(coords)
segments = get_segments_from_coords(coords)
# rays = get_environment_corner_lines(coords)
reflex_angle_points = get_environment_reflex_angle(segments, test_poly)


draw(test_poly)

for point in reflex_angle_points:
    draw(point, color='red')
    
plt.show()