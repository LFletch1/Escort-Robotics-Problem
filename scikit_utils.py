#!/usr/bin/env python3

"""
Module with local helper functions for scikit-geometry.

Authors: Nick Stiffler
Version: June 14, 2022
"""

# import scikit-geometry modules
import skgeom as sg
import numpy as np
from skgeom.draw import draw

# ----------------------------
# Monkey Patching
# ----------------------------
def point2_to_numpy(self):
    assert isinstance(self, sg.Point2)
    return np.array([float(self.x()), float(self.y())])

sg.Point2.to_numpy = point2_to_numpy

def to_sg(obj):
    assert isinstance(obj, (np.ndarray, np.generic) )
    if obj.shape == (2,):
        return sg.Point2(obj[0,0], obj[0,1])
    elif obj.shape == (2,2):
        return sg.Segment2(sg.Point2(obj[0,0], obj[0,1]), sg.Point2(obj[1,0], obj[1,1]))
    elif obj.shape == (4,):
        return sg.Segment2(sg.Point2(obj[0,0], obj[0,1]), sg.Point2(obj[0,2], obj[0,3]))
    else:
        raise Exception("to_sg Error - np.array has incompatable dimension")
# -----------------------------
#  Utility methods
# -----------------------------
def compact_to_string(obj):
	if isinstance(obj, sg.Polygon):
		return '--'.join([f"({str(v.x())} , {str(v.y())})" for v in obj.vertices])
	elif isinstance(obj, sg.Segment2):
		return f"({str(obj.source().x())} , {str(obj.source().y())})--({str(obj.target().x())} , {str(obj.target().y())})"
	elif isinstance(obj, sg.Point2):
		return f"({str(obj.x())} , {str(obj.y())})"

def face_details(obj):
	assert isinstance(obj, sg.arrangement.Face), f"The object in question is a {type(obj)} and NOT a FACE"

	details = "This face "

	# Is this face unbounded?
	if obj.is_unbounded():
		details += f"is unbounded\n"

	# Info about the outer connect component boundary
	if obj.has_outer_ccb():
		details += f"has an outer ccb\n\t"
		outer_ccb_circulator = obj.outer_ccb
		first = next(outer_ccb_circulator)
		circ = next(outer_ccb_circulator)
		while circ != first:
			details += f"{compact_to_string(circ.source().point())} -- "
			# Move circulator
			circ = next(outer_ccb_circulator)
		details += f"{compact_to_string(circ.source().point())}"
		details += "\n"

	# Info about the isolated vertices
	num_isolated = obj.number_of_isolated_vertices()
	details += f"Number of isolated vertices {num_isolated}\n"
	if num_isolated > 0:
		details += "\t"
		for i in obj.isolated_vertices:
			details += f"{compact_to_string(i)}"
		details += "\n"

	# Info about the holes
	num_holes = obj.number_of_holes()
	details += f"Number of holes {num_holes}\n"
	if num_holes > 0:
		for h in obj.holes:
			details += "\t"
			hole_first = next(h)
			hole_circ = next(h)
			while hole_circ != hole_first:
				details += f"{compact_to_string(hole_circ.source().point())} -- "
				hole_circ = next(h)
			details += f"{compact_to_string(hole_circ.source().point())}"
			details += "\n"

	# return the full details
	return details

def halfedge_outer_ccb_pts(eh) -> {'type':set,'elements':'Point2','docstring':'Given a halfedge handle, determine the face containing the halfedge and return the points along the face\'s outer ccb'}:
	"""
	Given a halfedge handle, we can determine the face containing the halfedge.
	Return the points along this face's outer ccb (connected component boundary)

	Primary use: in "compute_vispoly" as a helper function

	Parameter eh: a halfedge handle
	Precondition: handle is not NONE
	"""
	assert isinstance(eh, sg.arrangement.Halfedge), f"The object in question is a {type(eh)} and NOT a HALFEDGE"

	ehf = eh.face() # edge handle face
	ehbp = set() # edge handle boundary pts
	if ehf.has_outer_ccb():
		# only test bounded faces!
		ehocc = ehf.outer_ccb # edge handle outer ccb circulator
		eh_first = next(ehocc)
		eh_circ = next(ehocc)
		while eh_circ != eh_first: # while loop grabs elements from 2 ... n
			ehbp.add( (float(eh_circ.source().point().x()), float(eh_circ.source().point().y())) )
			# Move circulator
			eh_circ = next(ehocc)
		# grab the 1st element
		ehbp.add( (float(eh_circ.source().point().x()), float(eh_circ.source().point().y())) )
	return ehbp

def build_polygon_set_from_arrangement(arr):
	polys = []
	# iterate over all of the faces
	for f in arr.faces:
		if f.is_unbounded():
			continue

		# Info about the outer connect component boundary
		if f.has_outer_ccb():
			poly_pts = []
			outer_ccb_circulator = f.outer_ccb
			first = next(outer_ccb_circulator)
			circ = next(outer_ccb_circulator)
			while circ != first:
				poly_pts.append(circ.source().point())
				# Move circulator
				circ = next(outer_ccb_circulator)
			poly_pts.append(circ.source().point())
			# p = sg.Polygon(poly_pts)
			# print(p)
			# draw(p)
			polys.append(sg.Polygon(poly_pts))

	return sg.PolygonSet(polys)


def build_list_of_polygons_from_arrangement(arr):
	polys = []
	# iterate over all of the faces
	for f in arr.faces:
		if f.is_unbounded():
			continue

		# Info about the outer connect component boundary
		if f.has_outer_ccb():
			poly_pts = []
			outer_ccb_circulator = f.outer_ccb
			first = next(outer_ccb_circulator)
			circ = next(outer_ccb_circulator)
			while circ != first:
				poly_pts.append(circ.source().point())
				# Move circulator
				circ = next(outer_ccb_circulator)
			poly_pts.append(circ.source().point())
			polys.append(sg.Polygon(poly_pts))

	return polys

def get_tuple_from_polygon_set(poly_set):
	'''Used to hash polysets'''
	# Returns tuple of tuples with order guarenteed to be the same if same polygon_set is input
	# Probably unneccsary to do a bunch of sorting if polygon_sets are alway returned as the same,
	# but I don't want to deal with the issue where that isn't true
	poly_set_list = []
	for poly in poly_set.polygons:
		single_poly = []
		for coord in poly.outer_boundary().coords:
			# print(type(coord))
			single_poly.append((coord[0], coord[1]))
		sorted_single_poly_tup = tuple(sorted(single_poly))
		poly_set_list.append(sorted_single_poly_tup)
	return tuple(sorted(poly_set_list))