import skgeom as sg
import json
from skgeom.draw import draw
import matplotlib.pyplot as plt
import math
from scikit_utils import *

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
    # append last segment
    segments.append(sg.Segment2(sg.Point2(coords[0][0], coords[0][1]),sg.Point2(coords[-1][0], coords[-1][1])))
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
    last_point = reflex_angle_point(segments[0], segments[-1], polygon_set)
    if last_point:
        reflex_angle_points.append(last_point) 

    return reflex_angle_points


def midpoint(segment):
    mid_x = (segment.point(0).x() + segment.point(1).x()) / 2
    mid_y = (segment.point(0).y() + segment.point(1).y()) / 2
    return sg.Point2(mid_x, mid_y)
    

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


def seg_in_segment_set(segment, segment_set):
    for seg in segment_set:
        if segment.point(0) == seg.point(0) and segment.point(1) == seg.point(1):
            return True
        elif segment.point(0) == seg.point(1) and segment.point(1) == seg.point(0):
            return True
    return False


def equal_segments(seg1, seg2):
    if seg1.point(0) == seg2.point(0) and seg1.point(1) == seg2.point(1):
        return True
    elif seg1.point(0) == seg2.point(1) and seg1.point(1) == seg2.point(0):
        return True
    return False


def get_env_conservative_edges(env_segments, env_polygon):
    '''Inefficient but working method to calculate conservative edges'''
    reflex_points = get_environment_reflex_points(env_segments, env_polygon)
    env_polygon_set = sg.PolygonSet([env_polygon])

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

    # Draw outward rays from reflex points that have are in the line of sight of each other
    lines_of_sight_reflex_points = []
    for j in range(len(reflex_points)):
        for k in range(j+1, len(reflex_points)):
            reflex1 = reflex_points[j]
            reflex2 = reflex_points[k]
            reflex_line = sg.Segment2(reflex1, reflex2)
            no_intersections = True
            for seg in env_segments:
                intersect = sg.intersection(reflex_line, seg)
                if isinstance(intersect, sg.Segment2):
                    no_intersections = False
                    break
                    
                m_p = midpoint(reflex_line)
                if isinstance(intersect, sg.Point2):
                    if intersect != seg.point(0) and intersect != seg.point(1):
                        no_intersections = False
                        break
                    elif not env_polygon_set.locate(m_p):
                        no_intersections = False
                        break

            if no_intersections:
                lines_of_sight_reflex_points.append(reflex_line)

    for seg in lines_of_sight_reflex_points:
        vec1 = seg.point(0) - seg.point(1) # Vector in direction from point 1 to point 2
        vec2 = seg.point(1) - seg.point(0) # Vector Ray in direction of from point 2 to point 1
        ray1 = sg.Ray2(seg.point(0), vec1) # Ray starting at point 1
        ray2 = sg.Ray2(seg.point(1), vec2) # Ray starting at point 2
        intersecting1_points = []
        intersecting2_points = []
        for line_seg in env_segments:
            intersect1 = sg.intersection(ray1, line_seg)
            intersect2 = sg.intersection(ray2, line_seg)

            if isinstance(intersect1, sg.Point2):
                if intersect1 != seg.point(0):
                    intersecting1_points.append(intersect1)

            if isinstance(intersect2, sg.Point2):
                if intersect2 != seg.point(1):
                    intersecting2_points.append(intersect2)

        if len(intersecting1_points):
            closest_point1 = closest_point(seg.point(0), intersecting1_points)
            conservative_edge = sg.Segment2(closest_point1, seg.point(0))
            if not seg_in_segment_set(conservative_edge, env_segments):
                m_p = midpoint(conservative_edge)
                if env_polygon_set.locate(m_p):
                    conservative_region_edges.append(conservative_edge)

        if len(intersecting2_points):
            closest_point2 = closest_point(seg.point(1), intersecting2_points)
            conservative_edge = sg.Segment2(closest_point2, seg.point(1))
            if not seg_in_segment_set(conservative_edge, env_segments):
                m_p = midpoint(conservative_edge)
                if env_polygon_set.locate(m_p):
                    conservative_region_edges.append(conservative_edge)

    # Some duplicates may have been created along the way. Remove them
    duplicate_indices = []
    for i in range(len(conservative_region_edges)):
        for j in range(i+1, len(conservative_region_edges)):
            if equal_segments(conservative_region_edges[i], conservative_region_edges[j]):
                duplicate_indices.append(j)
    duplicate_indices.sort(reverse=True)
    for idx in duplicate_indices:
        conservative_region_edges.pop(idx)

    return conservative_region_edges


def polys_share_edge(poly1, poly2):
    for edge1 in poly1.edges:
        for edge2 in poly2.edges:
            if equal_segments(edge1, edge2):
                return True
    return False


def coords_from_json(file_path):
    '''Returns simple tuple of coords from JSON'''
    try:
        with open( file_path ) as json_file :
            data = json.load(json_file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        print (f"Error opening file: {file_path}")
        return

    # Step 1: Read in the contours
    for contour in data["contours"]:
        coordinates = []

        for point in contour:
            xval = point["x"]
            yval = point["y"]
            coordinates.append((xval, yval))
        # Assumes only one contour, doesn't support polygons with holes
        return coordinates


def get_adj_list_of_conservative_centroid_nodes(coords):
    env_poly = poly_from_coords(coords)
    env_segments = get_segments_from_coords(coords)
    cons_edges = get_env_conservative_edges(env_segments, env_poly)
    arr = sg.arrangement.Arrangement()
    for seg in env_segments:
        arr.insert(seg)
    for edge in cons_edges:
        # draw(edge, color="blue")
        arr.insert(edge)

    poly_list = build_list_of_polygons_from_arrangement(arr)

    adjacency_list = {} 
    for poly in poly_list:
        centroid = sg.centroid(poly)
        adjacency_list[(float(centroid.x()), float(centroid.y()))] = []
    for i in range(len(poly_list)):
        for j in range(i+1, len(poly_list)):
            if polys_share_edge(poly_list[i], poly_list[j]):
                cent1 = sg.centroid(poly_list[i])
                cent2 = sg.centroid(poly_list[j])
                draw(sg.Segment2(cent1, cent2), color="red")
                draw(cent1, color="red")
                draw(cent2, color="red")
                adjacency_list[(float(cent1.x()),float(cent1.y()))].append((float(cent2.x()),float(cent2.y()))) 
                adjacency_list[(float(cent2.x()),float(cent2.y()))].append((float(cent1.x()),float(cent1.y()))) 
    return adjacency_list


def main():   
    coords = coords_from_json("Envs/rooms3.json")
    env_poly = poly_from_coords(coords)
    draw(env_poly)

    adj_list = get_adj_list_of_conservative_centroid_nodes(coords)
    
    plt.show()

if __name__ == "__main__":
    main()