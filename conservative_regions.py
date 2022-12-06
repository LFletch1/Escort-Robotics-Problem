import skgeom as sg
from skgeom.draw import draw
import matplotlib.pyplot as plt
import math

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
    if seg1.point(0) == seg2.point(0) or seg1.point(0) == seg2.point(1) or seg1.point(1) == seg2.point(0) or seg1.point(1) == seg2.point(1):
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

def get_environment_reflex_points(segments, env_polygon):
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

def closer_point(target, point_a, point_b):
    '''Return point that is closer to target'''
    dist_a = math.sqrt((float(target.x()) - float(point_a.x()))**2 + (float(target.y()) - float(point_a.y()))**2)
    dist_b = math.sqrt((float(target.x()) - float(point_b.x()))**2 + (float(target.y()) - float(point_b.y()))**2)
    if dist_a < dist_b:
        return point_a
    else:
        return point_b

def closest_point(target, list_of_points):
    '''Return point that is closer to target'''
    least_dist = 100000
    close_point = list_of_points[0]
    for point in list_of_points:
        dist = math.sqrt((float(target.x()) - float(point.x()))**2 + (float(target.y()) - float(point.y()))**2)
        if dist < least_dist:
            close_point = point
            least_dist = dist
    return close_point


def get_env_conservative_regions(env_segments, env_polygon):
    '''Inefficient but working method to calculate conservative regions'''
    reflex_points = get_environment_reflex_points(env_segments, env_polygon)

    conservative_region_edges = []
    for seg in env_segments:
        # If segment contains two environment reflexive points
        if seg.point(0) in reflex_points and seg.point(1) in reflex_points:
            # draw(seg, color='orange')
            vec1 = seg.point(1) - seg.point(0) # Vector in direction from point 1 to point 2
            vec2 = seg.point(0) - seg.point(1) # Vector Ray in direction of from point 2 to point 1
            ray1 = sg.Ray2(seg.point(0), vec1) # Ray starting at point 2
            ray2 = sg.Ray2(seg.point(1), vec2) # Ray starting at point 1
            intersecting1_points = []
            intersecting2_points = []
            for line_seg in env_segments:

                intersect1 = sg.intersection(ray1, line_seg)
                intersect2 = sg.intersection(ray2, line_seg)

                if isinstance(intersect1, sg.Segment2):
                    closest = closer_point(seg.point(1), intersect1.point(0), intersect1.point(1))
                    if not lines_share_endpoint(seg, line_seg):
                        intersecting1_points.append(closest)
                    
                elif isinstance(intersect1, sg.Point2):
                    if not lines_share_endpoint(seg, line_seg):
                        intersecting1_points.append(intersect1)

                if isinstance(intersect2, sg.Segment2):
                    closest = closer_point(seg.point(0), intersect2.point(0), intersect2.point(1))
                    if not lines_share_endpoint(seg, line_seg):
                        intersecting2_points.append(closest)
                    
                elif isinstance(intersect2, sg.Point2):
                    if not lines_share_endpoint(seg, line_seg):
                        intersecting2_points.append(intersect2)

            if len(intersecting1_points):
                closest_point1 = closest_point(seg.point(1), intersecting1_points)
                conservative_edge = sg.Segment2(closest_point1, seg.point(1))
                conservative_region_edges.append(conservative_edge)

            if len(intersecting2_points):
                closest_point2 = closest_point(seg.point(0), intersecting2_points)
                conservative_edge = sg.Segment2(closest_point2, seg.point(0))
                conservative_region_edges.append(conservative_edge)

        # If segment contains one environment reflexive points (previous if statement would catch both case)
        elif seg.point(0) in reflex_points or seg.point(1) in reflex_points:
            if seg.point(0) in reflex_points:
                r_point = seg.point(0)
                non_r_point = seg.point(1)
            else:
                r_point = seg.point(1)
                non_r_point = seg.point(0)
            vec = r_point - non_r_point
            ray = sg.Ray2(r_point, vec)
            intersection_points = []
            for line_seg in env_segments:
                intersect = sg.intersection(ray, line_seg)
                if isinstance(intersect, sg.Segment2):
                    closest = closer_point(r_point, intersect.point(0), intersect.point(1))
                    if not lines_share_endpoint(seg, line_seg):
                        intersection_points.append(closest)
                    
                elif isinstance(intersect, sg.Point2):
                    if not lines_share_endpoint(seg, line_seg):
                        intersection_points.append(intersect)

            if len(intersection_points):
                closest_point1 = closest_point(r_point, intersection_points)
                conservative_edge = sg.Segment2(closest_point1, r_point)
                conservative_region_edges.append(conservative_edge) 
            
    return conservative_region_edges

                    
                



# rooms environment
# coords = [(2,0),(2,4),(5,4),(5,3),(3,3),(3,1),(9,1),(9,3),(7,3),(8,4),(11,4),(11,6),(7,6),(7,7),(9,7),(9,9),(3,9),(3,7),(5,7),(5,6),(0,6),(0,0)]

# tetris environment
coords = [(0 , 4 ),
            (12, 4 ),
            (12, 0 ),
            (16, 0 ),
            (16, 8 ),
            (4 , 8 ),
            (4 , 12),
            (0 , 12)]


env_poly = poly_from_coords(coords)
env_segments = get_segments_from_coords(coords)
reflex_angle_points = get_environment_reflex_points(env_segments, env_poly)

draw(env_poly)

cons_edges = get_env_conservative_regions(env_segments, env_poly)


for edge in cons_edges:
    draw(edge, color='blue')

for point in reflex_angle_points:
    draw(point, color='red')


# a = sg.Point2(3, 2)
# b = sg.Point2(3, 3)

# # seg = sg.Segment2(sg.Point2(1,4),sg.Point2(8,4))
# seg = sg.Segment2(sg.Point2(3,5),sg.Point2(3,6))

# plt.clf()
# v = b - a 
# r = sg.Ray2(a,v)
# print(r.direction())
# print(r.is_horizontal())
# print(r.is_vertical())
# # print(r)
# intersect = sg.intersection(r, seg)
# print(type(intersect))
# draw(intersect)
# draw(seg)
    
plt.show()