from EscortProblem import EscortProblem
import matplotlib.pyplot as plt
import skgeom as sg
from skgeom.draw import draw
import numpy as np
from conservative_regions import *


def get_env_conservative_edges(env_segments, env_polygon):
    '''Inefficient but working method to calculate conservative edges'''
    reflex_points = get_environment_reflex_points(env_segments, env_polygon)
    env_polygon_set = sg.PolygonSet([env_polygon])

    conservative_region_edges = []
    for seg in env_segments:
        # If segment contains two environment reflexive points
        if seg.point(0) in reflex_points and seg.point(1) in reflex_points:
            #draw(seg, color='orange')
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
                        #draw(closest, color='orange')
                    
                elif isinstance(intersect1, sg.Point2):
                    if not lines_share_endpoint(seg, line_seg):
                        intersecting1_points.append(intersect1)
                        #draw(intersect1, color='orange')

                if isinstance(intersect2, sg.Segment2):
                    closest = closer_point(seg.point(0), intersect2.point(0), intersect2.point(1))
                    if not lines_share_endpoint(seg, line_seg):
                        intersecting2_points.append(closest)
                        #draw(closest, color='blue')
                    
                elif isinstance(intersect2, sg.Point2):
                    if not lines_share_endpoint(seg, line_seg):
                        intersecting2_points.append(intersect2)
                        #draw(intersect1, color='blue')

            if len(intersecting1_points):
                closest_point1 = closest_point(seg.point(1), intersecting1_points)
                conservative_edge = sg.Segment2(closest_point1, seg.point(1))
                conservative_region_edges.append(conservative_edge)
                #draw(conservative_edge, color='red')

            if len(intersecting2_points):
                closest_point2 = closest_point(seg.point(0), intersecting2_points)
                conservative_edge = sg.Segment2(closest_point2, seg.point(0))
                conservative_region_edges.append(conservative_edge)
                #draw(conservative_edge, color='red')

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
                #draw(conservative_edge, color='blue')

    # Draw outward rays from reflex points that have are in the line of sight of each other
    lines_of_sight_reflex_points = []
    
    for j in range(len(reflex_points)):
        for k in range(j+1, len(reflex_points)):
            reflex1 = reflex_points[j]
            reflex2 = reflex_points[k]
            reflex_line = sg.Segment2(reflex1, reflex2)
            #draw(reflex_line, color = "orange")
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

            draw(lines_of_sight_reflex_points, color = "orange")

    for seg in lines_of_sight_reflex_points:
        vec1 = seg.point(0) - seg.point(1) # Vector in direction from point 1 to point 2
        #draw(vec1, color = "pink")
        vec2 = seg.point(1) - seg.point(0) # Vector Ray in direction of from point 2 to point 1
        #draw(vec2, color = "pink")
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
            #draw(conservative_edge, color = "pink")
            if not seg_in_segment_set(conservative_edge, env_segments):
                m_p = midpoint(conservative_edge)
                if env_polygon_set.locate(m_p):
                    conservative_region_edges.append(conservative_edge)

        if len(intersecting2_points):
            closest_point2 = closest_point(seg.point(1), intersecting2_points)
            conservative_edge = sg.Segment2(closest_point2, seg.point(1))
            #draw(conservative_edge, color = "pink")
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

    return conservative_region_edges, lines_of_sight_reflex_points


def main():
    ep = EscortProblem("Envs/new_env3.json", (10.0, 16.667), (100, 45))
    draw(ep.environment.gp)


    coords = coords_from_json("Envs/new_env3.json")
    env_poly = poly_from_coords(coords)
    env_segments = get_segments_from_coords(coords)
    cons_edges, opp_edges = get_env_conservative_edges(env_segments, env_poly)

    for cons in cons_edges:
        print(cons)

    draw(cons_edges)
    #draw(opp_edges)

    


    plt.show()


if __name__ == "__main__":
    main()