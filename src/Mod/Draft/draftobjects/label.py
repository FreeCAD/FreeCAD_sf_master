# -*- coding: utf-8 -*-
# ***************************************************************************
# *   Copyright (c) 2009, 2010 Yorik van Havre <yorik@uncreated.net>        *
# *   Copyright (c) 2009, 2010 Ken Cline <cline@frii.com>                   *
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

"""This module provides the object code for Draft Label.
"""
## @package label
# \ingroup DRAFT
# \brief This module provides the object code for Draft Label.

from PySide.QtCore import QT_TRANSLATE_NOOP

import FreeCAD as App

from draftobjects.draft_annotation import DraftAnnotation


class Label(DraftAnnotation):
    """The Draft Label object"""

    def __init__(self, obj):
        super(Label, self).__init__(obj, "Label")

        self.init_properties(obj)

        obj.Proxy = self

    def init_properties(self, obj):
        """Add properties to the object and set them"""
        _tip = QT_TRANSLATE_NOOP("App::Property",
                "The placement of this object")
        obj.addProperty("App::PropertyPlacement", "Placement", "Base", _tip)

        _tip = QT_TRANSLATE_NOOP("App::Property","The length of the straight segment")
        obj.addProperty("App::PropertyDistance", "StraightDistance", "Base", _tip)

        _tip = QT_TRANSLATE_NOOP("App::Property", "The point indicated by this label")
        obj.addProperty("App::PropertyVector", "TargetPoint", "Base", _tip)

        _tip = QT_TRANSLATE_NOOP("App::Property", "The points defining the label polyline")
        obj.addProperty("App::PropertyVectorList", "Points", "Base", _tip)

        _tip = QT_TRANSLATE_NOOP("App::Property", "The direction of the straight segment")
        obj.addProperty("App::PropertyEnumeration", "StraightDirection", "Base", _tip)

        _tip = QT_TRANSLATE_NOOP("App::Property","The type of information shown by this label")
        obj.addProperty("App::PropertyEnumeration", "LabelType", "Base", _tip)

        _tip = QT_TRANSLATE_NOOP("App::Property","The target object of this label")
        obj.addProperty("App::PropertyLinkSub", "Target", "Base", _tip)

        _tip = QT_TRANSLATE_NOOP("App::Property", "The text to display when type is set to custom")
        obj.addProperty("App::PropertyStringList", "CustomText", "Base", _tip)

        _tip = QT_TRANSLATE_NOOP("App::Property", "The text displayed by this label")
        obj.addProperty("App::PropertyStringList", "Text", "Base", _tip)

        obj.StraightDirection = ["Horizontal","Vertical","Custom"]
        obj.LabelType = ["Custom","Name","Label","Position",
                         "Length","Area","Volume","Tag","Material"]
        obj.setEditorMode("Text",1)
        obj.StraightDistance = 1
        obj.TargetPoint = App.Vector(2,-1,0)
        obj.CustomText = "Label"

    def onDocumentRestored(self, obj):
        super(Label, self).onDocumentRestored(obj)

    def execute(self,obj):
        '''Do something when recompute object'''

        if obj.StraightDirection != "Custom":
            p1 = obj.Placement.Base
            if obj.StraightDirection == "Horizontal":
                p2 = App.Vector(obj.StraightDistance.Value,0,0)
            else:
                p2 = App.Vector(0,obj.StraightDistance.Value,0)
            p2 = obj.Placement.multVec(p2)
            # p3 = obj.Placement.multVec(obj.TargetPoint)
            p3 = obj.TargetPoint
            obj.Points = [p1,p2,p3]
        if obj.LabelType == "Custom":
            if obj.CustomText:
                obj.Text = obj.CustomText
        elif obj.Target and obj.Target[0]:
            if obj.LabelType == "Name":
                obj.Text = [obj.Target[0].Name]
            elif obj.LabelType == "Label":
                obj.Text = [obj.Target[0].Label]
            elif obj.LabelType == "Tag":
                if hasattr(obj.Target[0],"Tag"):
                    obj.Text = [obj.Target[0].Tag]
            elif obj.LabelType == "Material":
                if hasattr(obj.Target[0],"Material"):
                    if hasattr(obj.Target[0].Material,"Label"):
                        obj.Text = [obj.Target[0].Material.Label]
            elif obj.LabelType == "Position":
                p = obj.Target[0].Placement.Base
                if obj.Target[1]:
                    if "Vertex" in obj.Target[1][0]:
                        p = obj.Target[0].Shape.Vertexes[int(obj.Target[1][0][6:])-1].Point
                obj.Text = [App.Units.Quantity(x,App.Units.Length).UserString for x in tuple(p)]
            elif obj.LabelType == "Length":
                if hasattr(obj.Target[0],'Shape'):
                    if hasattr(obj.Target[0].Shape,"Length"):
                        obj.Text = [App.Units.Quantity(obj.Target[0].Shape.Length,App.Units.Length).UserString]
                    if obj.Target[1] and ("Edge" in obj.Target[1][0]):
                        obj.Text = [App.Units.Quantity(obj.Target[0].Shape.Edges[int(obj.Target[1][0][4:])-1].Length,App.Units.Length).UserString]
            elif obj.LabelType == "Area":
                if hasattr(obj.Target[0],'Shape'):
                    if hasattr(obj.Target[0].Shape,"Area"):
                        obj.Text = [App.Units.Quantity(obj.Target[0].Shape.Area,App.Units.Area).UserString.replace("^2","²")]
                    if obj.Target[1] and ("Face" in obj.Target[1][0]):
                        obj.Text = [App.Units.Quantity(obj.Target[0].Shape.Faces[int(obj.Target[1][0][4:])-1].Area,App.Units.Area).UserString]
            elif obj.LabelType == "Volume":
                if hasattr(obj.Target[0],'Shape'):
                    if hasattr(obj.Target[0].Shape,"Volume"):
                        obj.Text = [App.Units.Quantity(obj.Target[0].Shape.Volume,App.Units.Volume).UserString.replace("^3","³")]


    def onChanged(self,obj,prop):
        '''Do something when a property has changed'''

        return


# Alias for compatibility with v0.18 and earlier
DraftLabel = Label
