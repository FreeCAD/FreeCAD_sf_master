# ***************************************************************************
# *                                                                         *
# *   Copyright (c) 2018 Yorik van Havre <yorik@uncreated.net>              *
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


"""The BIM Box command"""

import os
import FreeCAD
import FreeCADGui

QT_TRANSLATE_NOOP = FreeCAD.Qt.QT_TRANSLATE_NOOP
translate = FreeCAD.Qt.translate


class BIM_Box:
    def GetResources(self):
        return {
            "Pixmap": "BIM_Box",
            "MenuText": QT_TRANSLATE_NOOP("BIM_Box", "Box"),
            "ToolTip": QT_TRANSLATE_NOOP(
                "BIM_Box", "Graphically creates a generic box in the current document"
            ),
        }

    def IsActive(self):
        v = hasattr(FreeCADGui.getMainWindow().getActiveWindow(), "getSceneGraph")
        return v

    def Activated(self):
        import draftguitools.gui_trackers as DraftTrackers

        # here we will store our points
        self.points = []
        # we build a special cube tracker which is a list of 4 rectangle trackers
        self.cubetracker = []
        self.LengthValue = 0
        self.WidthValue = 0
        self.HeightValue = 0
        self.currentpoint = None
        for i in range(4):
            self.cubetracker.append(DraftTrackers.rectangleTracker())
        if hasattr(FreeCADGui, "Snapper"):
            FreeCADGui.Snapper.getPoint(
                callback=self.PointCallback,
                movecallback=self.MoveCallback,
                extradlg=self.taskbox(),
            )

    def MoveCallback(self, point, snapinfo):
        import DraftGeomUtils
        import DraftVecUtils

        self.currentpoint = point
        if len(self.points) == 1:
            # we have the base point already
            self.Length.setText(
                FreeCAD.Units.Quantity(
                    self.points[-1].sub(point).Length,  FreeCAD.Units.Length
                ).UserString
            )
            self.Length.selectAll()
            self.Length.setFocus()
        elif len(self.points) == 2:
            # now we already have our base line, we update the 1st rectangle
            p = point
            v1 = point.sub(self.points[1])
            v4 = v1.cross(self.points[1].sub(self.points[0]))
            if v4 and v4.Length:
                n = (self.points[1].sub(self.points[0])).cross(v4)
                if n and n.Length:
                    n = DraftVecUtils.project(v1, n)
                    p = self.points[1].add(n)
            self.cubetracker[0].p3(p)
            self.Width.setText(
                FreeCAD.Units.Quantity(
                    self.cubetracker[0].getSize()[1], FreeCAD.Units.Length
                ).UserString
            )
            self.Width.selectAll()
            self.Width.setFocus()
        elif len(self.points) == 3:
            h = DraftGeomUtils.distance_to_plane(point, self.cubetracker[0].p3(), self.normal)
            w = self.normal * h
            # then we update all rectangles
            self.cubetracker[1].p3((self.cubetracker[0].p2()).add(w))
            self.cubetracker[2].p3((self.cubetracker[0].p4()).add(w))
            self.cubetracker[3].p1((self.cubetracker[0].p1()).add(w))
            self.cubetracker[3].p3((self.cubetracker[0].p3()).add(w))
            self.Height.setText(FreeCAD.Units.Quantity(h, FreeCAD.Units.Length).UserString)
            self.Height.selectAll()
            self.Height.setFocus()

    def PointCallback(self, point, snapinfo):
        if not point:
            # cancelled
            if hasattr(FreeCAD, "DraftWorkingPlane"):
                FreeCAD.DraftWorkingPlane.restore()
            FreeCADGui.Snapper.setGrid()
            for c in self.cubetracker:
                c.off()
            return

        if len(self.points) == 0:
            # this is our first clicked point, nothing to do just yet
            self.points.append(point)
            FreeCADGui.Snapper.getPoint(
                last=point,
                callback=self.PointCallback,
                movecallback=self.MoveCallback,
                extradlg=self.taskbox(),
            )
        elif len(self.points) == 1:
            # this is our second point
            baseline = point.sub(self.points[0])
            self.points.append(point)
            self._setupForWidthInput(baseline)
        elif len(self.points) == 2:
            # this is our third point
            self.points.append(point)
            self._setupForHeightInput()
        elif len(self.points) == 3:
            # finally we have all our points. Let's create the actual cube
            self._makeBox()
            self._finish()

    def taskbox(self):
        "sets up a taskbox widget"

        from PySide import QtCore, QtGui

        wid = QtGui.QWidget()
        ui = FreeCADGui.UiLoader()
        wid.setWindowTitle(translate("BIM", "Box dimensions"))
        grid = QtGui.QGridLayout(wid)

        label1 = QtGui.QLabel(translate("BIM", "Length"))
        self.Length = ui.createWidget("Gui::InputField")
        l = FreeCAD.Units.Quantity(self.LengthValue, FreeCAD.Units.Length)
        self.Length.setText(l.UserString)
        grid.addWidget(label1, 0, 0, 1, 1)
        grid.addWidget(self.Length, 0, 1, 1, 1)
        if self.LengthValue:
            self.Length.setEnabled(False)

        label2 = QtGui.QLabel(translate("BIM", "Width"))
        self.Width = ui.createWidget("Gui::InputField")
        l = FreeCAD.Units.Quantity(self.WidthValue, FreeCAD.Units.Length)
        self.Width.setText(l.UserString)
        grid.addWidget(label2, 1, 0, 1, 1)
        grid.addWidget(self.Width, 1, 1, 1, 1)
        if self.WidthValue or (not self.LengthValue):
            self.Width.setEnabled(False)

        label3 = QtGui.QLabel(translate("BIM", "Height"))
        self.Height = ui.createWidget("Gui::InputField")
        l = FreeCAD.Units.Quantity(self.HeightValue, FreeCAD.Units.Length)
        self.Height.setText(l.UserString)
        grid.addWidget(label3, 2, 0, 1, 1)
        grid.addWidget(self.Height, 2, 1, 1, 1)
        if not self.WidthValue:
            self.Height.setEnabled(False)

        self.Length.valueChanged.connect(self.setLength)
        self.Width.valueChanged.connect(self.setWidth)
        self.Height.valueChanged.connect(self.setHeight)
        self.Length.returnPressed.connect(self.setLengthUI)
        self.Width.returnPressed.connect(self.setWidthUI)
        self.Height.returnPressed.connect(self.setHeightUI)
        return wid

    def setLength(self, d):
        self.LengthValue = d

    def setWidth(self, d):
        self.WidthValue = d

    def setHeight(self, d):
        self.HeightValue = d

    def setLengthUI(self):
        if (len(self.points) == 1) and self.currentpoint and self.LengthValue:
            baseline = self.currentpoint.sub(self.points[0])
            baseline.normalize()
            baseline.multiply(self.LengthValue)
            p2 = self.points[0].add(baseline)
            self.points.append(p2)
            self._setupForWidthInput(baseline)

    def setWidthUI(self):
        if (len(self.points) == 2) and self.currentpoint and self.WidthValue:
            self.normal = self.cubetracker[0].getNormal()
            if self.normal:
                n = (self.points[1].sub(self.points[0])).cross(self.normal)
                if n and n.Length:
                    n.normalize()
                    n.multiply(self.WidthValue)
                    p2 = self.points[1].add(n)
            self.cubetracker[0].p3(p2)
            self.points.append(p2)
            self._setupForHeightInput()

    def setHeightUI(self):
        if (len(self.points) == 3) and self.HeightValue:
            self._makeBox()
            self._finish()

    def _setupForWidthInput(self, baseline):
        # we turn on only one of the rectangles
        self.cubetracker[0].setPlane(baseline)
        self.cubetracker[0].p1(self.points[0])
        self.cubetracker[0].on()
        FreeCADGui.Snapper.getPoint(
            last=self.points[-1],
            callback=self.PointCallback,
            movecallback=self.MoveCallback,
            extradlg=self.taskbox(),
        )

    def _setupForHeightInput(self):
        # we can get the cubes Z axis from our first rectangle
        self.normal = self.cubetracker[0].getNormal()
        # we can therefore define the (u,v) planes of all rectangles
        u = self.cubetracker[0].u
        v = self.cubetracker[0].v
        self.cubetracker[1].setPlane(u, self.normal)
        self.cubetracker[2].setPlane(u, self.normal)
        self.cubetracker[3].setPlane(u, v)
        # and the origin points of the vertical rectangles
        self.cubetracker[1].p1(self.cubetracker[0].p1())
        self.cubetracker[2].p1(self.cubetracker[0].p3())
        # finally we turn all rectangles on
        for r in self.cubetracker:
            r.on()
        if hasattr(FreeCAD, "DraftWorkingPlane"):
            FreeCAD.DraftWorkingPlane.save()
            FreeCAD.DraftWorkingPlane.position = self.cubetracker[0].p3()
            FreeCAD.DraftWorkingPlane.u = (
                self.cubetracker[0].p3().sub(self.cubetracker[0].p4())
            ).normalize()
            FreeCAD.DraftWorkingPlane.v = self.normal
            FreeCAD.DraftWorkingPlane.axis = (
                self.cubetracker[0].p2().sub(self.cubetracker[0].p3())
            ).normalize()
            FreeCADGui.Snapper.setGrid()
        FreeCADGui.Snapper.getPoint(
            last=self.cubetracker[0].p3(),
            callback=self.PointCallback,
            movecallback=self.MoveCallback,
            extradlg=self.taskbox(),
        )

    def _makeBox(self):
        import DraftGeomUtils

        p1 = self.cubetracker[0].p1()
        p2 = self.cubetracker[0].p2()
        p3 = self.cubetracker[0].p4()
        pla = DraftGeomUtils.placement_from_points(p1, p2, p3)
        if self.normal.isEqual(pla.Rotation.multVec(FreeCAD.Vector(0, 0, 1)), 1e-6):
            if self.HeightValue < 0.0:
                pla = DraftGeomUtils.placement_from_points(p1, p3, p2)
                self.LengthValue, self.WidthValue = self.WidthValue, self.LengthValue
        else:
            if self.HeightValue > 0.0:
                pla = DraftGeomUtils.placement_from_points(p1, p3, p2)
                self.LengthValue, self.WidthValue = self.WidthValue, self.LengthValue
        doc = FreeCAD.ActiveDocument
        doc.openTransaction(translate("Arch","Create Box"))
        cube = doc.addObject("Part::Box", "Cube")
        cube.Placement = pla
        cube.Length = self.LengthValue
        cube.Width = self.WidthValue
        cube.Height = abs(self.HeightValue)
        doc.commitTransaction()
        doc.recompute()

    def _finish(self):
        FreeCADGui.Snapper.getPoint()
        FreeCADGui.Snapper.off()
        for c in self.cubetracker:
            c.finalize()
        if hasattr(FreeCADGui, "draftToolBar"):
            FreeCADGui.draftToolBar.offUi()
        if hasattr(FreeCAD, "DraftWorkingPlane"):
            FreeCAD.DraftWorkingPlane.restore()
        FreeCADGui.Snapper.setGrid()


FreeCADGui.addCommand("BIM_Box", BIM_Box())
