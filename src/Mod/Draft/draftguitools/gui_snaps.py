"""Provide the Draft_Snap commands used by the snapping mechanism in Draft."""
## @package gui_snaps
# \ingroup DRAFT
# \brief Provide the Draft_Snap commands used by the snapping mechanism
# in Draft.

# ***************************************************************************
# *   (c) 2009, 2010 Yorik van Havre <yorik@uncreated.net>                  *
# *   (c) 2009, 2010 Ken Cline <cline@frii.com>                             *
# *   (c) 2020 Eliud Cabrera Castillo <e.cabrera-castillo@tum.de>           *
# *                                                                         *
# *   This file is part of the FreeCAD CAx development system.              *
# *                                                                         *
# *   This program is free software; you can redistribute it and/or modify  *
# *   it under the terms of the GNU Lesser General Public License (LGPL)    *
# *   as published by the Free Software Foundation; either version 2 of     *
# *   the License, or (at your option) any later version.                   *
# *   for detail see the LICENCE text file.                                 *
# *                                                                         *
# *   FreeCAD is distributed in the hope that it will be useful,            *
# *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
# *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
# *   GNU Library General Public License for more details.                  *
# *                                                                         *
# *   You should have received a copy of the GNU Library General Public     *
# *   License along with FreeCAD; if not, write to the Free Software        *
# *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  *
# *   USA                                                                   *
# *                                                                         *
# ***************************************************************************
import FreeCADGui as Gui
from PySide import QtGui
from PySide.QtCore import QT_TRANSLATE_NOOP

# UTILITIES -----------------------------------------------------------------

def sync_snap_toolbar_button(button, status):
    """set snap toolbar button to given state"""
    snap_toolbar = Gui.Snapper.get_snap_toolbar()
    #print(snap_toolbar)
    if not snap_toolbar:
        return
    for a in snap_toolbar.actions():
        if a.objectName() == button:
            if button == "Draft_Snap_Lock_Button":
                # for lock button
                snap_toolbar.actions()[0].setChecked(status)
                for a in snap_toolbar.actions()[1:]:
                    a.setEnabled(status)
            else:
                # for every other button
                a.setChecked(status)
                if a.isChecked():
                    a.setToolTip(a.toolTip().replace("OFF","ON"))
                else:
                    a.setToolTip(a.toolTip().replace("ON","OFF"))


# SNAP GUI TOOLS ------------------------------------------------------------

class Draft_Snap_Lock:
    """Command to activate or deactivate all snap commands."""

    def GetResources(self):
        """Set icon, menu and tooltip."""
        _menu = "Toggle On/Off"
        _tip = ("Activates or deactivates "
                "all snap tools at once")
        return {'Pixmap': 'Snap_Lock',
                'Accel': "Shift+S",
                'MenuText': QT_TRANSLATE_NOOP("Draft_Snap_Lock", _menu),
                'ToolTip': QT_TRANSLATE_NOOP("Draft_Snap_Lock", _tip)}

    def Activated(self):
        """Execute this when the command is called."""
        if hasattr(Gui, "Snapper"):
            # toggle the corresponding snap_index in Preferences/Mod/Draft/snapModes
            status = Gui.Snapper.toggle_snap('Lock')
            # change interface consistently
            sync_snap_toolbar_button("Draft_Snap_Lock"+"_Button", status)


Gui.addCommand('Draft_Snap_Lock', Draft_Snap_Lock())


class Draft_Snap_Midpoint:
    """Command to snap to the midpoint of an edge."""

    def GetResources(self):
        """Set icon, menu and tooltip."""
        _menu = "Midpoint"
        _tip = "Snaps to midpoints of edges"
        return {'Pixmap': 'Snap_Midpoint',
                'MenuText': QT_TRANSLATE_NOOP("Draft_Snap_Midpoint", _menu),
                'ToolTip': QT_TRANSLATE_NOOP("Draft_Snap_Midpoint", _tip)}

    def Activated(self):
        """Execute this when the command is called."""
        if hasattr(Gui, "Snapper"):
            # toggle the corresponding snap_index in Preferences/Mod/Draft/snapModes
            status = Gui.Snapper.toggle_snap('Midpoint')
            # change interface consistently
            sync_snap_toolbar_button("Draft_Snap_Midpoint"+"_Button", status)


Gui.addCommand('Draft_Snap_Midpoint', Draft_Snap_Midpoint())


class Draft_Snap_Perpendicular:
    """Command to snap to perdendicular of an edge."""

    def GetResources(self):
        """Set icon, menu and tooltip."""
        _menu = "Perpendicular"
        _tip = "Snaps to perpendicular points on edges"
        return {'Pixmap': 'Snap_Perpendicular',
                'MenuText': QT_TRANSLATE_NOOP("Draft_Snap_Perpendicular",
                                              _menu),
                'ToolTip': QT_TRANSLATE_NOOP("Draft_Snap_Perpendicular",
                                             _tip)}

    def Activated(self):
        """Execute this when the command is called."""
        if hasattr(Gui, "Snapper"):
            # toggle the corresponding snap_index in Preferences/Mod/Draft/snapModes
            status = Gui.Snapper.toggle_snap('Perpendicular')
            # change interface consistently
            sync_snap_toolbar_button("Draft_Snap_Perpendicular"+"_Button", status)


Gui.addCommand('Draft_Snap_Perpendicular', Draft_Snap_Perpendicular())


class Draft_Snap_Grid:
    """Command to snap to the intersection of grid lines."""

    def GetResources(self):
        """Set icon, menu and tooltip."""
        _tip = "Snaps to grid points"
        return {'Pixmap': 'Snap_Grid',
                'MenuText': QT_TRANSLATE_NOOP("Draft_Snap_Grid", "Grid"),
                'ToolTip': QT_TRANSLATE_NOOP("Draft_Snap_Grid", _tip)}

    def Activated(self):
        """Execute this when the command is called."""
        if hasattr(Gui, "Snapper"):
            # toggle the corresponding snap_index in Preferences/Mod/Draft/snapModes
            status = Gui.Snapper.toggle_snap('Grid')
            # change interface consistently
            sync_snap_toolbar_button("Draft_Snap_Grid"+"_Button", status)


Gui.addCommand('Draft_Snap_Grid', Draft_Snap_Grid())


class Draft_Snap_Intersection:
    """Command to snap to the intersection of two edges."""

    def GetResources(self):
        """Set icon, menu and tooltip."""
        _menu = "Intersection"
        _tip = "Snaps to edges intersections"
        return {'Pixmap': 'Snap_Intersection',
                'MenuText': QT_TRANSLATE_NOOP("Draft_Snap_Intersection",
                                              _menu),
                'ToolTip': QT_TRANSLATE_NOOP("Draft_Snap_Intersection",
                                             _tip)}

    def Activated(self):
        """Execute this when the command is called."""
        if hasattr(Gui, "Snapper"):
            # toggle the corresponding snap_index in Preferences/Mod/Draft/snapModes
            status = Gui.Snapper.toggle_snap('Intersection')
            # change interface consistently
            sync_snap_toolbar_button("Draft_Snap_Intersection"+"_Button", status)


Gui.addCommand('Draft_Snap_Intersection', Draft_Snap_Intersection())


class Draft_Snap_Parallel:
    """Command to snap to the parallel of an edge."""

    def GetResources(self):
        """Set icon, menu and tooltip."""
        _menu = "Parallel"
        _tip = "Snaps to parallel directions of edges"
        return {'Pixmap': 'Snap_Parallel',
                'MenuText': QT_TRANSLATE_NOOP("Draft_Snap_Parallel", _menu),
                'ToolTip': QT_TRANSLATE_NOOP("Draft_Snap_Parallel", _tip)}

    def Activated(self):
        """Execute this when the command is called."""
        if hasattr(Gui, "Snapper"):
            # toggle the corresponding snap_index in Preferences/Mod/Draft/snapModes
            status = Gui.Snapper.toggle_snap('Parallel')
            # change interface consistently
            sync_snap_toolbar_button("Draft_Snap_Parallel"+"_Button", status)


Gui.addCommand('Draft_Snap_Parallel', Draft_Snap_Parallel())


class Draft_Snap_Endpoint:
    """Command to snap to an endpoint of an edge."""

    def GetResources(self):
        """Set icon, menu and tooltip."""
        _menu = "Endpoint"
        _tip = "Snaps to endpoints of edges"
        return {'Pixmap': 'Snap_Endpoint',
                'MenuText': QT_TRANSLATE_NOOP("Draft_Snap_Endpoint", _menu),
                'ToolTip': QT_TRANSLATE_NOOP("Draft_Snap_Endpoint", _tip)}

    def Activated(self):
        """Execute this when the command is called."""
        if hasattr(Gui, "Snapper"):
            # toggle the corresponding snap_index in Preferences/Mod/Draft/snapModes
            status = Gui.Snapper.toggle_snap('Endpoint')
            # change interface consistently
            sync_snap_toolbar_button("Draft_Snap_Endpoint"+"_Button", status)


Gui.addCommand('Draft_Snap_Endpoint', Draft_Snap_Endpoint())


class Draft_Snap_Angle:
    """Command to snap to 90 degree angles."""

    def GetResources(self):
        """Set icon, menu and tooltip."""
        _tip = "Snaps to 45 and 90 degrees points on arcs and circles"
        return {'Pixmap': 'Snap_Angle',
                'MenuText': QT_TRANSLATE_NOOP("Draft_Snap_Angle", "Angles"),
                'ToolTip': QT_TRANSLATE_NOOP("Draft_Snap_Angle", _tip)}

    def Activated(self):
        """Execute this when the command is called."""
        if hasattr(Gui, "Snapper"):
            # toggle the corresponding snap_index in Preferences/Mod/Draft/snapModes
            status = Gui.Snapper.toggle_snap('Angle')
            # change interface consistently
            sync_snap_toolbar_button("Draft_Snap_Angle"+"_Button", status)


Gui.addCommand('Draft_Snap_Angle', Draft_Snap_Angle())


class Draft_Snap_Center:
    """Command to snap to the centers of arcs and circumferences."""

    def GetResources(self):
        """Set icon, menu and tooltip."""
        _tip = "Snaps to center of circles and arcs"
        return {'Pixmap': 'Snap_Center',
                'MenuText': QT_TRANSLATE_NOOP("Draft_Snap_Center", "Center"),
                'ToolTip': QT_TRANSLATE_NOOP("Draft_Snap_Center", _tip)}

    def Activated(self):
        """Execute this when the command is called."""
        if hasattr(Gui, "Snapper"):
            # toggle the corresponding snap_index in Preferences/Mod/Draft/snapModes
            status = Gui.Snapper.toggle_snap('Center')
            # change interface consistently
            sync_snap_toolbar_button("Draft_Snap_Center"+"_Button", status)


Gui.addCommand('Draft_Snap_Center', Draft_Snap_Center())


class Draft_Snap_Extension:
    """Command to snap to the extension of an edge."""

    def GetResources(self):
        """Set icon, menu and tooltip."""
        _menu = "Extension"
        _tip = "Snaps to extension of edges"
        return {'Pixmap': 'Snap_Extension',
                'MenuText': QT_TRANSLATE_NOOP("Draft_Snap_Extension", _menu),
                'ToolTip': QT_TRANSLATE_NOOP("Draft_Snap_Extension", _tip)}

    def Activated(self):
        """Execute this when the command is called."""
        if hasattr(Gui, "Snapper"):
            # toggle the corresponding snap_index in Preferences/Mod/Draft/snapModes
            status = Gui.Snapper.toggle_snap('Extension')
            # change interface consistently
            sync_snap_toolbar_button("Draft_Snap_Extension"+"_Button", status)


Gui.addCommand('Draft_Snap_Extension', Draft_Snap_Extension())


class Draft_Snap_Near:
    """Command to snap to the nearest point of an edge."""

    def GetResources(self):
        """Set icon, menu and tooltip."""
        _tip = "Snaps to nearest point on edges"
        return {'Pixmap': 'Snap_Near',
                'MenuText': QT_TRANSLATE_NOOP("Draft_Snap_Near", "Nearest"),
                'ToolTip': QT_TRANSLATE_NOOP("Draft_Snap_Near", _tip)}

    def Activated(self):
        """Execute this when the command is called."""
        if hasattr(Gui, "Snapper"):
            # toggle the corresponding snap_index in Preferences/Mod/Draft/snapModes
            status = Gui.Snapper.toggle_snap('Near')
            # change interface consistently
            sync_snap_toolbar_button("Draft_Snap_Near"+"_Button", status)


Gui.addCommand('Draft_Snap_Near', Draft_Snap_Near())


class Draft_Snap_Ortho:
    """Command to snap to the orthogonal directions."""

    def GetResources(self):
        """Set icon, menu and tooltip."""
        _tip = "Snaps to orthogonal and 45 degrees directions"
        return {'Pixmap': 'Snap_Ortho',
                'MenuText': QT_TRANSLATE_NOOP("Draft_Snap_Ortho", "Ortho"),
                'ToolTip': QT_TRANSLATE_NOOP("Draft_Snap_Ortho", _tip)}

    def Activated(self):
        """Execute this when the command is called."""
        if hasattr(Gui, "Snapper"):
            # toggle the corresponding snap_index in Preferences/Mod/Draft/snapModes
            status = Gui.Snapper.toggle_snap('Ortho')
            # change interface consistently
            sync_snap_toolbar_button("Draft_Snap_Ortho"+"_Button", status)


Gui.addCommand('Draft_Snap_Ortho', Draft_Snap_Ortho())


class Draft_Snap_Special:
    """Command to snap to the special point of an object."""

    def GetResources(self):
        """Set icon, menu and tooltip."""
        _menu = "Special"
        _tip = "Snaps to special locations of objects"
        return {'Pixmap': 'Snap_Special',
                'MenuText': QT_TRANSLATE_NOOP("Draft_Snap_Special", _menu),
                'ToolTip': QT_TRANSLATE_NOOP("Draft_Snap_Special", _tip)}

    def Activated(self):
        """Execute this when the command is called."""
        if hasattr(Gui, "Snapper"):
            # toggle the corresponding snap_index in Preferences/Mod/Draft/snapModes
            status = Gui.Snapper.toggle_snap('Special')
            # change interface consistently
            sync_snap_toolbar_button("Draft_Snap_Special"+"_Button", status)


Gui.addCommand('Draft_Snap_Special', Draft_Snap_Special())


class Draft_Snap_Dimensions:
    """Command to temporary show dimensions when snapping."""

    def GetResources(self):
        """Set icon, menu and tooltip."""
        _menu = "Dimensions"
        _tip = "Shows temporary dimensions when snapping to Arch objects"
        return {'Pixmap': 'Snap_Dimensions',
                'MenuText': QT_TRANSLATE_NOOP("Draft_Snap_Dimensions", _menu),
                'ToolTip': QT_TRANSLATE_NOOP("Draft_Snap_Dimensions", _tip)}

    def Activated(self):
        """Execute this when the command is called."""
        if hasattr(Gui, "Snapper"):
            # toggle the corresponding snap_index in Preferences/Mod/Draft/snapModes
            status = Gui.Snapper.toggle_snap('Dimensions')
            # change interface consistently
            sync_snap_toolbar_button("Draft_Snap_Dimensions"+"_Button", status)


Gui.addCommand('Draft_Snap_Dimensions', Draft_Snap_Dimensions())


class Draft_Snap_WorkingPlane:
    """Command to snap to a point in the current working plane."""

    def GetResources(self):
        """Set icon, menu and tooltip."""
        _menu = "Working plane"
        _tip = "Restricts the snapped point to the current working plane"
        return {'Pixmap': 'Snap_WorkingPlane',
                'MenuText': QT_TRANSLATE_NOOP("Draft_Snap_WorkingPlane",
                                              _menu),
                'ToolTip': QT_TRANSLATE_NOOP("Draft_Snap_WorkingPlane",
                                             _tip)}

    def Activated(self):
        """Execute this when the command is called."""
        if hasattr(Gui, "Snapper"):
            # toggle the corresponding snap_index in Preferences/Mod/Draft/snapModes
            status = Gui.Snapper.toggle_snap('WorkingPlane')
            # change interface consistently
            sync_snap_toolbar_button("Draft_Snap_WorkingPlane"+"_Button", status)


Gui.addCommand('Draft_Snap_WorkingPlane', Draft_Snap_WorkingPlane())
