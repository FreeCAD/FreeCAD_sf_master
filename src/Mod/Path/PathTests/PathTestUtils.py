# -*- coding: utf-8 -*-

# ***************************************************************************
# *                                                                         *
# *   Copyright (c) 2016 sliptonic <shopinthewoods@gmail.com>               *
# *                                                                         *
# *   This program is free software; you can redistribute it and/or modify  *
# *   it under the terms of the GNU Lesser General Public License (LGPL)    *
# *   as published by the Free Software Foundation; either version 2 of     *
# *   the License, or (at your option) any later version.                   *
# *   for detail see the LICENCE text file.                                 *
# *                                                                         *
# *   This program is distributed in the hope that it will be useful,       *
# *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
# *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
# *   GNU Library General Public License for more details.                  *
# *                                                                         *
# *   You should have received a copy of the GNU Library General Public     *
# *   License along with this program; if not, write to the Free Software   *
# *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  *
# *   USA                                                                   *
# *                                                                         *
# ***************************************************************************

import FreeCAD
import Part
import math
import unittest

from FreeCAD import Vector
from PathScripts.PathGeom import Side

class PathTestBase(unittest.TestCase):
    """Base test class with some additional asserts."""

    def assertRoughly(self, f1, f2):
        """Verify that two float values are approximately the same."""
        self.assertTrue(math.fabs(f1 - f2) < 0.00001, "%f != %f" % (f1, f2))

    def assertCoincide(self, pt1, pt2):
        """Verify that two points coincide - roughly speaking."""
        self.assertRoughly(pt1.x, pt2.x)
        self.assertRoughly(pt1.y, pt2.y)
        self.assertRoughly(pt1.z, pt2.z)

    def assertPlacement(self, p1, p2):
        """Verify that two placements are roughly identical."""
        self.assertCoincide(p1.Base, p2.Base)
        self.assertCoincide(p1.Rotation.Axis, p2.Rotation.Axis)
        self.assertTrue(p1.Rotation.isSame(p2.Rotation))

    def assertLine(self, edge, pt1, pt2):
        """Verify that edge is a line from pt1 to pt2."""
        # Depending on the setting of LineOld ....
        self.assertTrue(type(edge.Curve) is Part.Line or type(edge.Curve) is Part.LineSegment)
        self.assertCoincide(edge.valueAt(edge.FirstParameter), pt1)
        self.assertCoincide(edge.valueAt(edge.LastParameter), pt2)

    def assertLines(self, edgs, tail, points):
        """Verify that the edges match the polygon resulting from points."""
        edges = list(edgs)
        if tail:
            edges.append(tail)
        self.assertEqual(len(edges), len(points) - 1)

        for i in range(0, len(edges)):
            self.assertLine(edges[i], points[i], points[i+1])

    def assertArc(self, edge, pt1, pt2, direction = 'CW'):
        """Verify that edge is an arc between pt1 and pt2 with the given direction."""
        self.assertIs(type(edge.Curve), Part.Circle)
        self.assertCoincide(edge.valueAt(edge.FirstParameter), pt1)
        self.assertCoincide(edge.valueAt(edge.LastParameter), pt2)
        ptm = edge.valueAt((edge.LastParameter + edge.FirstParameter)/2)
        side = Side.of(pt2 - pt1, ptm - pt1)
        if 'CW' == direction:
            self.assertEqual(side, Side.Left)
        else:
            self.assertEqual(side, Side.Right)

    def assertCircle(self, edge, pt, r):
        """Verivy that edge is a circle at given location."""
        curve = edge.Curve
        self.assertIs(type(curve), Part.Circle)
        self.assertCoincide(curve.Center, Vector(pt.x, pt.y, pt.z))
        self.assertRoughly(curve.Radius, r)


    def assertCurve(self, edge, p1, p2, p3):
        """Verify that the edge goes through the given 3 points, representing start, mid and end point respectively."""
        self.assertCoincide(edge.valueAt(edge.FirstParameter), p1)
        self.assertCoincide(edge.valueAt(edge.LastParameter), p3)
        self.assertCoincide(edge.valueAt((edge.FirstParameter + edge.LastParameter)/2), p2)

    def assertCylinderAt(self, solid, pt, r, h):
        """Verify that solid is a cylinder at the specified location."""
        self.assertEqual(len(solid.Edges), 3)

        lid  = solid.Edges[0]
        hull = solid.Edges[1]
        base = solid.Edges[2]

        self.assertCircle(lid, Vector(pt.x, pt.y, pt.z+h), r)
        self.assertLine(hull, Vector(pt.x+r, pt.y, pt.z), Vector(pt.x+r, pt.y, pt.z+h))
        self.assertCircle(base, Vector(pt.x, pt.y, pt.z), r)

    def assertConeAt(self, solid, pt, r1, r2, h):
        """Verify that solid is a cone at the specified location."""
        self.assertEqual(len(solid.Edges), 3)

        lid  = solid.Edges[0]
        hull = solid.Edges[1]
        base = solid.Edges[2]

        self.assertCircle(lid, Vector(pt.x, pt.y, pt.z+h), r2)
        self.assertLine(hull, Vector(pt.x+r1, pt.y, pt.z), Vector(pt.x+r2, pt.y, pt.z+h))
        self.assertCircle(base, Vector(pt.x, pt.y, pt.z), r1)

    def assertCommandEqual(self, c1, c2):
        """Verify that the 2 commands are equivalent."""
        self.assertEqual(c1.Name, c2.Name)

        self.assertRoughly(c1.Parameters.get('X', 0), c2.Parameters.get('X', 0))
        self.assertRoughly(c1.Parameters.get('Y', 0), c2.Parameters.get('Y', 0))
        self.assertRoughly(c1.Parameters.get('Z', 0), c2.Parameters.get('Z', 0))

        self.assertRoughly(c1.Parameters.get('I', 0), c2.Parameters.get('I', 0))
        self.assertRoughly(c1.Parameters.get('J', 0), c2.Parameters.get('J', 0))
        self.assertRoughly(c1.Parameters.get('K', 0), c2.Parameters.get('K', 0))

    def assertEqualLocale(self,s1,s2):
        """Verify that the 2 strings are equivalent, but converts eventual , into . for the first string that may be affected by locale."""
        self.assertEqual(s1.replace(",","."), s2)
		
