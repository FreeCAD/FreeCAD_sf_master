# -*- coding: utf-8 -*-
# ***************************************************************************
# *   Copyright (c) 2021 Russell Johnson (russ4262) <russ4262@gmail.com>    *
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
import Path
import PathScripts.PathJob as PathJob
import PathScripts.PathSlot as PathSlot
from PathTests.PathTestUtils import PathTestBase


class TestPathSlot(PathTestBase):
    """Unit tests for the Slot operation."""

    @staticmethod
    def _setup_test_environment():
        """_setup_test_environment()...
        This method is upon instantiation of this test class.  Add code and objects here
        that are needed for the duration of the test() methods in this class.
        This method does not have access to the class `self` reference.
        """
        # doc = FreeCAD.open(FreeCAD.getHomePath() + 'Mod/Path/PathTests/test_slot_01.fcstd')
        doc_title = "TestSlot"
        doc = FreeCAD.newDocument(doc_title)

        # This is a real world test to make sure none of the tool chain broke
        box0 = doc.addObject('Part::Box', 'Box')
        box0.Length = 25.0
        box0.Width = 50.0
        box0.Height = 10.0
        box1 = doc.addObject('Part::Box', 'Box')
        box1.Length = 5.0  # X
        box1.Width = 15.0  # Y
        box1.Height = 5.0  # Z
        box1.Placement = FreeCAD.Placement(FreeCAD.Vector(5.0, 0.0, 5.0), FreeCAD.Rotation(FreeCAD.Vector(0,0,1), 0))
        doc.recompute()
        cut = doc.addObject('Part::Cut', 'Cut')
        cut.Base = box0
        cut.Tool = box1
        doc.recompute()

        # Create Job object
        PathJob.Create('Job', [doc.Cut], None)
        doc.recompute()

    @classmethod
    def setUpClass(cls):
        """setUpClass()...
        This method is called upon instantiation of this test class.  Add code and objects here
        that are needed for the duration of the test() methods in this class.
        This method does not have access to the class `self` reference.  This method
        is able to call static methods within this same class.
        """
        # FreeCAD.Console.PrintMessage("TestPathSlot.setUpClass()\n")
        cls._setup_test_environment()

    @classmethod
    def tearDownClass(cls):
        """tearDownClass()...
        This method is called prior to destruction of this test class.  Add code and objects here
        that cleanup the test environment after the test() methods in this class have been executed.
        This method does not have access to the class `self` reference.  This method
        is able to call static methods within this same class.
        """
        # FreeCAD.Console.PrintMessage("TestPathSlot.tearDownClass()\n")
        FreeCAD.closeDocument(FreeCAD.ActiveDocument.Name)

    def setUp(self):
        """setUp()...
        This method is called prior to each test() method.  Add code and objects here
        that are needed for multiple test() methods.
        """
        self.doc = FreeCAD.ActiveDocument
        self.con = FreeCAD.Console

    def tearDown(self):
        """tearDown()...
        This method is called after each test() method. Add cleanup instructions here.
        Such cleanup instructions will likely undo those in the setUp() method.
        """
        pass

    def _set_depths_and_heights(self, op):
        # Set Depths
        # set start and final depth in order to eliminate effects of stock (and its default values)
        op.setExpression('StartDepth', None)
        op.StartDepth.Value = 10.0
        op.setExpression('FinalDepth', None)
        op.FinalDepth.Value = 5.0
        op.setExpression('StepDown', None)
        op.StepDown.Value = 5.0

        # Set Heights
        # default values used

    def _format_point(self, cmd):
        """Accepts command dict and returns point string coordinate"""
        x = round(cmd["X"], 2)
        y = round(cmd["Y"], 2)
        return "({}, {})".format(x, y)

    def test00(self):
        '''test00()...
        Test Slot operation with:
            - horizontal rectangular face selection
            - ReverseDirection = True
            - LayerMode = Single-pass
        '''

        # Identify base feature(s) to be used for operation's Base Geometry
        base = self.doc.Cut
        for i in range(11):
            face = "Face%d" % (i+1)
            f = base.Shape.getElement(face)
            if (f.Surface.Axis == FreeCAD.Vector(0,0,1) and
                f.Orientation == 'Forward' and f.BoundBox.ZMin == 5.0):
                break

        # Instantiate a Slot operation and sets standard depths & heights
        slot = PathSlot.Create('Slot-test00')
        self._set_depths_and_heights(slot)

        # Set Base Geometry
        slot.Base = (base, [face])

        # Set Operation
        slot.ReverseDirection = True
        slot.LayerMode = "Single-pass"

        self.doc.recompute()
        
        pnts = list()
        for c in slot.Path.Commands:
            p = c.Parameters
            k = p.keys()
            if "'X', 'Y'" in str(k)[9:]:  # == "(['F', 'X', 'Y'])":
                pnts.append(self._format_point(p))
        pnts.sort()
        # self.con.PrintMessage("pnts: {}\n".format(pnts))

        # Verify point count
        self.assertEqual(len(pnts), 2)
        # Verify each line-segment point, excluding arcs
        verify_points = ['(7.5, 0.0)', '(7.5, 15.0)']
        for i in range(2):
            self.assertEqual(pnts[i], verify_points[i])

    def test01(self):
        '''test01()...
        Test Slot operation with:
            - two horizontal parallel bottom lines
            - LayerMode = Single-pass
        '''

        # Identify base feature to be used for operation's Base Geometry
        base = self.doc.Cut
        edge_list = list()
        for i in range(0, len(base.Shape.Edges)):
            edge = "Edge%d" % (i+1)
            e = base.Shape.getElement(edge)
            eBB = e.BoundBox
            if (e.Length == 15.0 and round(eBB.ZLength, 6) == 0.0 and round(eBB.ZMin, 6) == 5.0):
                edge_list.append(edge)

        # Instantiate a Slot operation and sets standard depths & heights
        slot = PathSlot.Create('Slot-test01')
        self._set_depths_and_heights(slot)

        # Set Base Geometry
        slot.Base = (base, edge_list)

        # Set Operation
        slot.LayerMode = "Single-pass"

        self.doc.recompute()
        
        pnts = list()
        for c in slot.Path.Commands:
            p = c.Parameters
            k = p.keys()
            if "'X', 'Y'" in str(k)[9:]:  # == "(['F', 'X', 'Y'])":
                pnts.append(self._format_point(p))
        pnts.sort()
        # self.con.PrintMessage("pnts: {}\n".format(pnts))

        # Verify point count
        self.assertEqual(len(pnts), 2)
        # Verify each line-segment point, excluding arcs
        verify_points = ['(7.5, 0.0)', '(7.5, 15.0)']
        for i in range(2):
            self.assertEqual(pnts[i], verify_points[i])

    def test02(self):
        '''test02()...
        Test Slot operation with:
            - two vertical parallel rectangular faces selected
            - ReverseDirection = True
            - LayerMode = "Single-pass"
            - PathOrientation = "Perpendicular"
            - ExtendPathStart.Value = 7.5
            - ExtendPathEnd.Value = 7.5
        '''

        # Identify base feature to be used for operation's Base Geometry
        base = self.doc.Cut
        subs_list = list()
        for i in range(0, len(base.Shape.Faces)):
            feat = "Face{}".format(i+1)
            f = base.Shape.getElement(feat)
            fBB = f.BoundBox
            if (f.Surface.Axis.z == 0.0 and round(fBB.ZMin, 6) == 5.0):
                subs_list.append(feat)

        # Instantiate a Slot operation and sets standard depths & heights
        slot = PathSlot.Create('Slot-test02')
        self._set_depths_and_heights(slot)

        # Set Base Geometry
        slot.Base = (base, subs_list)

        # Set Operation
        slot.ReverseDirection = True
        slot.LayerMode = "Single-pass"
        slot.PathOrientation = "Perpendicular"
        slot.ExtendPathStart.Value = 7.5
        slot.ExtendPathEnd.Value = 7.5

        self.doc.recompute()
        
        pnts = list()
        for c in slot.Path.Commands:
            p = c.Parameters
            k = p.keys()
            if "'X', 'Y'" in str(k)[9:]:  # == "(['F', 'X', 'Y'])":
                pnts.append(self._format_point(p))
        pnts.sort()
        # self.con.PrintMessage("pnts: {}\n".format(pnts))

        # Verify point count
        self.assertEqual(len(pnts), 2)
        # Verify each line-segment point, excluding arcs
        verify_points = ['(7.5, 0.0)', '(7.5, 15.0)']
        for i in range(2):
            self.assertEqual(pnts[i], verify_points[i])

