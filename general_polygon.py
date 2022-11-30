#!/usr/bin/env python3

"""
Module containing the "general polygon" class.

Authors: Nick Stiffler
Version: June 13, 2022
"""

# import scikit-geometry modules
import skgeom as sg
from skgeom.draw import *

import triangle as tr # https://rufat.be/triangle/index.html

# import our local utils
from scikit_utils import *
from triangle2 import *

# import modules
import json
import itertools
import copy

gp_count = 0

class GeneralPolygon(sg.PolygonSet):
    """
    A generalized polygon, consisting of one or more polygonal boundaries.
    Sub-classes scikit-geometry's "PolygonSet" class.

    Note that the "polygons" data member is a list of PolygonWithHoles

    Attribute arrangement: The 2D arrangement of the general polygon
    Invariant: arrangement representation of 1 or more polygonal boundaries [polygons], or is None

    Attribute visibility: class used to compute the visibility from a specific point inside an arrangement
    Invariant: visibility has been initialized with an arrangement, or is None
    """

    def __init__(self, list_of_polys = []):
        # stuff for point location and visibility queries
        self.arrangement = None
        self.visibility = None
        self.triangulation = None
        self._triangles = None
        self._tri_weights = None
        super().__init__(list_of_polys)
        
        import traceback
        self.traceback = traceback.format_stack()[:-2]

        global gp_count
        gp_count += 1
        #print(f"+ GeneralPolygon count: {gp_count}")
    
    def __del__(self):
        global gp_count
        gp_count -= 1
        #print(f"- GeneralPolygon count: {gp_count}")
     

    @classmethod
    def load_from_json(cls, location, *,verbose=False):
        """
        Reads a json file and loads the contours as individual polygons.

        If you follow vertices in-order, then the area to the left of the
        traversal is "free-space", similarly, the area to the right is obstacle
        space.

        Example: an environment with an outer boundary would have vertices in CCW order.
        Any interior holes would be in CW order.

        Parameter location: relative json file location.
        Precondition: file exists and can be loaded as a json
        """
        try:
            with open( location ) as json_file :
                data = json.load(json_file)
        except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
            print (f"Error opening file: {location}")
            return


        # Step 1: Read in the contours
        contours = []
        for contour in data["contours"]:
            if verbose:
                # print ("Reading a contour")
                pass
            contour_points = []

            for point in contour:
                xval = point["x"]
                yval = point["y"]
                if verbose:
                    print (f"({xval:3}, {yval:3})")
                contour_points.append(sg.Point2(xval,yval))
            contours.append(sg.Polygon(contour_points))

        # Step 2: Build the PolygonSet ... which is a collection of PolygonWithHoles
        return cls(contours)

    # -------------------------------------------
    #  Class decorators
    # -------------------------------------------
    def require_contours(foo):
        def contour_decorator(self,*args, **kwargs):
            if self.is_empty:
                raise Exception("Error - No Polygons have been setup yet")
                return
            return foo(self, *args, **kwargs)
        return contour_decorator

    def require_arrangement(foo):
        def arrangement_decorator(self,*args, **kwargs):
            if self.arrangement is None:
                print("Warning - The \"Arrangement\" has not been set up yet. Building it now!!!")
                self.build_arrangement(verbose=True)
            return foo(self, *args, **kwargs)
        return arrangement_decorator

    def require_triangulation(foo):
        def triangulation_decorator(self,*args, **kwargs):
            if self.triangulation is None:
                print("Warning - The \"Triangulation\" has not been set up yet. Building it now!!!")
                self.build_triangulation(verbose=True)
            return foo(self, *args, **kwargs)
        return triangulation_decorator


    def require_visibility(foo):
        def visibility_decorator(self,*args, **kwargs):
            if self.visibility is None:
                print("Warning - The \"Visibility Object\" has not been set up yet. Building it now!!!")
                self.build_visibility()
            return foo(self, *args, **kwargs)
        return visibility_decorator

    # ---------------------------------
    # Actual methods
    # ---------------------------------
    def contains(self,x,y,*,verbose=False) -> {'type': bool, 'docstring':'Given a 2D point, does the general polygon contain the point'}:
        return True if self.locate(sg.Point2(x,y)) else False

    @require_contours
    def build_arrangement(self,*, verbose=False):
        # Arrangements
        self.arrangement = sg.arrangement.Arrangement()

        if verbose:
            pass
            # print(f"----------------------------------\n\tBuilding Arrangement\n----------------------------------")
        # Each poly is a PolygonWithHoles
        for i, poly in enumerate(self.polygons):
            boundary = poly.outer_boundary()
            if verbose:
                # print(f"PolygonWithHole at index ({i})\n\tOuter Boundary: {compact_to_string(boundary)}")
                pass
            for e in boundary.edges:
                self.arrangement.insert(e)

            for j, hole in enumerate(poly.holes):
                if verbose:
                    pass
                    # print(f"\tHole at index ({j}): {compact_to_string(hole)}")
                for e in hole.edges:
                    self.arrangement.insert(e)

        if verbose:
            # print face information from arrangement
            for f in self.arrangement.faces:
                # print("////////////////////////////////////")
                pass
                # print(face_details(f))
            # print("////////////////////////////////////")


    @require_contours
    def build_triangulation(self,*, tsettings='qpa1.0', verbose=False):

        def _tri_helper(poly, *, compute_hole=False) -> {'type': tuple[np.ndarray, np.ndarray, []], 'docstring': "Given a polygon boundary, return the pts, segs, and representative hole (if applicable)"}:
            i = np.arange(len(poly)) # How many points
            pts = np.array( [ point.to_numpy() for point in poly.vertices]) # list of points
            seg = np.stack([i, i + 1], axis=1) % len(poly) # A ndarray with shape (N, 2) corresponding to vertex indices of our segments
            if compute_hole:
                centroid = sg.centroid(poly)
                return pts, seg, centroid.to_numpy()
            else:
                return pts, seg, None

        if verbose:
            print(f"----------------------------------\n\tBuilding Triangulation\n----------------------------------")

            gp_pts = []   # list of np.arrays for  vertices
            gp_seg = []   # list of np.array's for contours (segments0
            gp_holes = [] # holes represented as a point in the hole
            idx = 0 # where does the next contour start
            for p in self.polygons:
                # p should be a polygon_with_holes (meaning an outer boundary and 0 or more holes)
                bpts, bsegs, bhole = _tri_helper(p.outer_boundary())
                gp_pts.append(bpts)
                gp_seg.append(bsegs)
                idx += bsegs.shape[0] # update idx
                for h in p.holes:
                    hpts, hsegs, hhole = _tri_helper(h, compute_hole=True)
                    gp_pts.append(hpts)
                    gp_seg.append(hsegs + idx)
                    gp_holes.append(hhole)
                    idx += hsegs.shape[0] # update idx

            # Stacks (may not need)
            stack_pts = np.vstack(gp_pts)
            stack_seg = np.vstack(gp_seg)

            if gp_holes:
                stack_holes = np.vstack(gp_holes)
                A = dict(vertices=stack_pts, segments=stack_seg, holes=stack_holes)
            else:
                A = dict(vertices=stack_pts, segments=stack_seg)


            self.triangulation = tr.triangulate(A, tsettings) # this builds the triangulation ... save it

            # extra bookkeeping to grab the triangles and turn it into something we can use
            vlist = self.triangulation['vertices'].tolist()
            tlist = self.triangulation['triangles'].tolist()

            self._triangles = [Triangle2(vlist[t[0]], vlist[t[1]], vlist[t[2]]) for t in tlist]
            # https://stackoverflow.com/questions/46539431/np-random-choice-probabilities-do-not-sum-to-1
            initial_weights = np.asarray([t.area for t in self._triangles]).astype('float64')
            self._tri_weights = initial_weights / np.sum(initial_weights)


    @require_arrangement
    def build_visibility(self,*, verbose=False):
        if verbose:
            print(f"----------------------------------\n\tBuilding Visibility\n----------------------------------")
        self.visibility = sg.TriangularExpansionVisibility(self.arrangement)

    @require_contours
    @require_arrangement
    @require_visibility
    def compute_vispoly(self,x,y,*,verbose=False) -> {'type': sg.PolygonWithHoles, 'docstring':'Given a 2D point, compute the visibility polygon from that point'}:
        if not self.contains(x,y):
            # logging
            from log2 import log, complete_log, html_online_log, html_offline_log, figures_log, silent_log
            with log.grouped():
                log.heading(f"Assertion error in compute_vispoly({x},{y})")
                log.pl_error(self,x,y)
            assert False, f"The query point ({x} , {y}) is not in the general polygon"

        point = sg.Point2(x,y)

        if verbose:
            print(f"----------------------------------\n\tComputing VisPoly {compact_to_string(point)}\n----------------------------------")

        pt_handle = self.arrangement.find(point)

        # --------------------
        # Case 1: Face (easy)
        # --------------------
        if isinstance(pt_handle , sg.arrangement.Face):
            return self.visibility.compute_visibility(point, pt_handle)

        # locate this point within the polygon set
        local_poly_with_holes = self.locate(point)

        # ----------------------------------------
        # Case 2: Halfedge (potentially ambiguous)
        # ----------------------------------------
        if(isinstance(pt_handle, sg.arrangement.Halfedge)):

            if local_poly_with_holes:
                # #######################################################
                # Case: query point contained within one of the
                #       PolygonWithHoles in this PolygonSet
                # #######################################################
                correct_boundary = local_poly_with_holes.outer_boundary()

                # construct a set of pts corresponding to the outer boundary vertices
                boundary_pts = {(float(x.x()),float(x.y())) for x in correct_boundary.vertices}

                # check if the original halfedge corresponds to the face we want
                original_boundary_pts = halfedge_outer_ccb_pts(pt_handle)
                if boundary_pts == original_boundary_pts:
                    return self.visibility.compute_visibility(point, pt_handle)

                # The initial handle didn't work!!! Test the twin
                twin_boundary_pts = halfedge_outer_ccb_pts(pt_handle.twin())
                if boundary_pts == twin_boundary_pts:
                    return self.visibility.compute_visibility(point, pt_handle.twin())

                ## Panic if we get here, bc neither the halfedge nor it's twin work
                raise Exception(f"We messed up something when computing a vispoly corresponding to a point on a halfedge: {compact_to_string(point)}")
            else:
                # #######################################################
                # Case: query point is not within any of our
                #       PolygonWithHoles ... return the "unbounded face"
                # #######################################################

                # Test if the original halfedge belongs to the unbounded face
                if pt_handle.face().is_unbounded():
                    return self.visibility.compute_visibility(point, pt_handle)
                # Test the twin
                if pt_handle.face().is_unbounded():
                    return self.visibility.compute_visibility(point, pt_handle.twin())

                ## Panic if we get here
                raise Exception(f"We messed up something when computing a vispoly corresponding to a point on a halfedge: {compact_to_string(point)}")

        # ----------------------------------------------------
        # Case 3: Vertex (really ambiguous)
        #         No direct 'compute_visibility' for vertices
        #         So we need an incident halfedge
        # ----------------------------------------------------
        if (isinstance(pt_handle,sg.arrangement.Vertex)):
            # Step 0: check for isolated vertex (This is not well defined so we should complain)
            incident = pt_handle.incident_halfedges
            inc_first = next(incident)
            inc_circ = next(incident)
            if inc_first == inc_circ:
                raise Exception(f"Error - Cannot compute a vispoly at an isolated vertex: {compact_to_string(point)}")

            # Step 1: locate this point within the polygon set
            local_poly_with_holes = self.locate(point)

            if local_poly_with_holes:
                # #######################################################
                # Case: query point contained within one of the
                #       PolygonWithHoles in this PolygonSet
                # #######################################################
                correct_boundary = local_poly_with_holes.outer_boundary()

                # construct a set of pts corresponding to the outer boundary vertices
                boundary_pts = {(float(x.x()),float(x.y())) for x in correct_boundary.vertices}

                # Iterate through the incident halfedges looking for a match
                while(inc_circ != inc_first):
                    # check if the original halfedge corresponds to the face we want
                    inc_boundary_pts = halfedge_outer_ccb_pts(inc_circ)
                    if boundary_pts == inc_boundary_pts:
                        return self.visibility.compute_visibility(point, inc_circ)
                    inc_circ = next(incident)
                # test the 1st element
                inc_boundary_pts = halfedge_outer_ccb_pts(inc_circ)
                if boundary_pts == inc_boundary_pts:
                    return self.visibility.compute_visibility(point, inc_circ)

                ## Panic if we get here, bc none of the incident halfedges work
                raise Exception(f"We messed up something when computing a vispoly corresponding to a point at a vertex: {compact_to_string(point)}")
            else:
                # #######################################################
                # Case: query point is not within any of our
                #       PolygonWithHoles ... return the "unbounded face"
                # #######################################################

                # find the unbounded face
                for f in self.arrangement.faces:
                    if f.is_unbounded():
                        return self.visibility.compute_visibility(point, f)

                ## Panic if we get here
                raise Exception(f"We messed up something when computing a vispoly corresponding to a point at a vertex: {compact_to_string(point)}")

        # ----------------------------------------------------
        # Case 4: Something went horribly wrong!!!
        # ----------------------------------------------------
        raise Exception(f"This object: {pt_handle} isn't a face, halfedge, or vertex. It is a {type(eh)}.")

    @require_contours
    @require_arrangement
    @require_visibility
    def compute_shadow_regions(self, pursuers:'pursuer positions'):
        """Should return a list of GeneralPolygon."""
        local = self
        for x in pursuers:
            if hasattr(x, 'x'):
                xvispoly = self.compute_vispoly(x.x(), x.y())
            else:
                xvispoly = self.compute_vispoly(x[0], x[1])
            x_polyset = build_polygon_set_from_arrangement(xvispoly)
            local = local.difference(x_polyset)

        gps = [GeneralPolygon([x]) for x in local.polygons]
        return gps

    @require_contours
    @require_triangulation
    def random_point(self) -> {'type': np.array, 'docstring':'Compute a random Point in the genpoly'}:
        # Step 1: pick a weighted triangle
        rand_triangle = np.random.choice(self._triangles, 1, p=self._tri_weights)[0]
        # Step 2: Calculate a random point in the triangle
        p = rand_triangle.random_point()
        return np.array([p.x(), p.y()]).astype('float64')

if __name__ == "__main__":
    gp = GeneralPolygon.load_from_json("Envs/octbrick_env.json", verbose=True)
    gp.build_arrangement(verbose=True)

    draw(gp, facecolor='lightblue', point_color='red')
    plt.show()

    # Draw the arrangement
    for he in gp.arrangement.halfedges:
        draw(he.curve(), visible_point=False)
    plt.show()

    # Test visibility polygon stuff
    a = sg.Point2(0, 1)   # Face
    b = sg.Point2(-8 , 5) # Halfedge
    c = sg.Point2(-8 , 2) # Vertex
    d = sg.Point2(-1 , 5) # Halfedge

    bad1 = sg.Point2(-5,5) # left box
    bad2 = sg.Point2(5,5)  # right box
    bad3 = sg.Point2(-12,5) # outside the env

    # locate this point within the polygon set
    print(f"Does the GenPoly contain {a}: {'True' if gp.contains(a.x(),a.y()) else 'False'}")
    print(f"Does the GenPoly contain {b}: {'True' if gp.contains(b.x(),b.y()) else 'False'}")
    print(f"Does the GenPoly contain {c}: {'True' if gp.contains(c.x(),c.y()) else 'False'}")
    print(f"Does the GenPoly contain {d}: {'True' if gp.contains(d.x(),d.y()) else 'False'}")
    print(f"Does the GenPoly contain {bad1}: {'True' if gp.contains(bad1.x(),bad1.y()) else 'False'}")
    print(f"Does the GenPoly contain {bad2}: {'True' if gp.contains(bad2.x(),bad2.y()) else 'False'}")
    print(f"Does the GenPoly contain {bad3}: {'True' if gp.contains(bad3.x(),bad3.y()) else 'False'}")

    print("-- Face")
    avispoly = gp.compute_vispoly(a.x(), a.y(), verbose=True)
    # Drawing
    for he in gp.arrangement.halfedges:
        draw(he.curve(), visible_point=False)
    for v in avispoly.halfedges:
        draw(v.curve(), color='red', visible_point=False)
    draw(a, color='magenta')
    plt.show()

    # locate this point within the polygon set
    # local_poly_with_holes = self.locate(point)

    print("-- Halfedge")
    bvispoly = gp.compute_vispoly(b.x(), b.y(), verbose=True)
    # Drawing
    for he in gp.arrangement.halfedges:
        draw(he.curve(), visible_point=False)
    for v in bvispoly.halfedges:
        draw(v.curve(), color='red', visible_point=False)
    draw(b, color='magenta')
    plt.show()


    dvispoly = gp.compute_vispoly(d.x(), d.y(), verbose=True)
    # Drawing
    for he in gp.arrangement.halfedges:
        draw(he.curve(), visible_point=False)
    for v in dvispoly.halfedges:
        draw(v.curve(), color='red', visible_point=False)
    draw(d, color='magenta')
    plt.show()

    print("-- Vertex")
    cvispoly = gp.compute_vispoly(c.x(), c.y(), verbose=True)
    # Drawing
    for he in gp.arrangement.halfedges:
        draw(he.curve(), visible_point=False)
    for v in cvispoly.halfedges:
        draw(v.curve(), color='red', visible_point=False)
    draw(c, color='magenta')
    plt.show()

    print("-- Compute Shadows")
    shadows = gp.compute_shadow_regions((b,d))
    # Drawing
    for he in gp.arrangement.halfedges:
        draw(he.curve(), visible_point=False)
    draw(shadows, facecolor='lightblue', point_color='red')
    draw(b, color='magenta')
    draw(d, color='magenta')
    plt.show()
