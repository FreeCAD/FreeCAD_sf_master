# ***************************************************************************
# *   Copyright (c) 2009, 2010 Yorik van Havre <yorik@uncreated.net>        *
# *   Copyright (c) 2009, 2010 Ken Cline <cline@frii.com>                   *
# *   Copyright (c) 2019, M G Berberich (berberic2, rynn)                   *
# *   Copyright (c) 2020 Eliud Cabrera Castillo <e.cabrera-castillo@tum.de> *
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
"""Provides the object code for the Draft Array object.

The `Array` class currently handles three types of arrays,
orthogonal, polar, and circular. In the future, probably they should be
split in separate classes so that they are easier to manage.
"""
## @package array
# \ingroup DRAFT
# \brief Provides the object code for the Draft Array object.

import math
from PySide.QtCore import QT_TRANSLATE_NOOP

import FreeCAD as App
import Part
import DraftVecUtils

from draftobjects.draftlink import DraftLink

class Array(DraftLink):
    """The Draft Array object.

    To Do
    -----
    The `Array` class currently handles three types of arrays,
    orthogonal, polar, and circular. In the future, probably they should be
    split in separate classes so that they are easier to manage.
    """

    def __init__(self, obj):
        super(Array, self).__init__(obj, "Array")

    def attach(self, obj):
        """Set up the properties when the object is attached."""
        self.set_properties(obj)
        super(Array, self).attach(obj)

    def set_properties(self, obj):
        """Set properties only if they don't exist."""
        self.set_ortho_properties(obj)
        self.set_polar_circular_properties(obj)
        self.set_polar_properties(obj)
        self.set_circular_properties(obj)

        # The ArrayType property (general) must be attached
        # after the other array properties so that onChanged works properly
        self.set_general_properties(obj)
        self.set_link_properties(obj)

    def set_general_properties(self, obj):
        """Set general properties only if they don't exist."""
        properties = obj.PropertiesList

        if "Base" not in properties:
            _tip = "The base object that will be duplicated"
            obj.addProperty("App::PropertyLink",
                            "Base",
                            "Objects",
                            QT_TRANSLATE_NOOP("App::Property", _tip))
            obj.Base = None

        if "ArrayType" not in properties:
            _tip = ("The type of array to create.\n"
                    "- Ortho: places the copies in the direction "
                    "of the global X, Y, Z axes.\n"
                    "- Polar: places the copies along a circular arc, "
                    "up to a specified angle, and with certain orientation "
                    "defined by a center and an axis.\n"
                    "- Circular: places the copies in concentric circular "
                    "layers around the base object.")
            obj.addProperty("App::PropertyEnumeration",
                            "ArrayType",
                            "Objects",
                            QT_TRANSLATE_NOOP("App::Property", _tip))
            obj.ArrayType = ['ortho', 'polar', 'circular']

        if "Fuse" not in properties:
            _tip = ("Specifies if the copies should be fused together "
                    "if they touch each other (slower)")
            obj.addProperty("App::PropertyBool",
                            "Fuse",
                            "Objects",
                            QT_TRANSLATE_NOOP("App::Property", _tip))
            obj.Fuse = False

    def set_ortho_properties(self, obj):
        """Set orthogonal properties only if they don't exist."""
        properties = obj.PropertiesList

        if "NumberX" not in properties:
            _tip = "Number of copies in X direction"
            obj.addProperty("App::PropertyInteger",
                            "NumberX",
                            "Orthogonal array",
                            QT_TRANSLATE_NOOP("App::Property", _tip))
            obj.NumberX = 2

        if "NumberY" not in properties:
            _tip = "Number of copies in Y direction"
            obj.addProperty("App::PropertyInteger",
                            "NumberY",
                            "Orthogonal array",
                            QT_TRANSLATE_NOOP("App::Property", _tip))
            obj.NumberY = 2

        if "NumberZ" not in properties:
            _tip = "Number of copies in Z direction"
            obj.addProperty("App::PropertyInteger",
                            "NumberZ",
                            "Orthogonal array",
                            QT_TRANSLATE_NOOP("App::Property", _tip))
            obj.NumberZ = 1

        if "IntervalX" not in properties:
            _tip = "Distance and orientation of intervals in X direction"
            obj.addProperty("App::PropertyVectorDistance",
                            "IntervalX",
                            "Orthogonal array",
                            QT_TRANSLATE_NOOP("App::Property", _tip))
            obj.IntervalX = App.Vector(50, 0, 0)

        if "IntervalY" not in properties:
            _tip = "Distance and orientation of intervals in Y direction"
            obj.addProperty("App::PropertyVectorDistance",
                            "IntervalY",
                            "Orthogonal array",
                            QT_TRANSLATE_NOOP("App::Property", _tip))
            obj.IntervalY = App.Vector(0, 50, 0)

        if "IntervalZ" not in properties:
            _tip = "Distance and orientation of intervals in Z direction"
            obj.addProperty("App::PropertyVectorDistance",
                            "IntervalZ",
                            "Orthogonal array",
                            QT_TRANSLATE_NOOP("App::Property", _tip))
            obj.IntervalZ = App.Vector(0, 0, 50)

    def set_polar_circular_properties(self, obj):
        """Set general polar and circular properties if they don't exist."""
        properties = obj.PropertiesList

        if "Axis" not in properties:
            _tip = ("The axis direction around which the elements in "
                    "a polar or a circular array will be created")
            obj.addProperty("App::PropertyVector",
                            "Axis",
                            "Polar/circular array",
                            QT_TRANSLATE_NOOP("App::Property", _tip))
            obj.Axis = App.Vector(0, 0, 1)

        if "Center" not in properties:
            _tip = ("Center point for polar and circular arrays.\n"
                    "The 'Axis' passes through this point.")
            obj.addProperty("App::PropertyVectorDistance",
                            "Center",
                            "Polar/circular array",
                            QT_TRANSLATE_NOOP("App::Property", _tip))
            obj.Center = App.Vector(0, 0, 0)

        # The AxisReference property must be attached after Axis and Center
        # so that onChanged works properly
        if "AxisReference" not in properties:
            _tip = ("The axis object that overrides the value of 'Axis' "
                    "and 'Center', for example, a datum line.\n"
                    "Its location and direction will be used "
                    "when creating polar and circular arrays.\n"
                    "Leave this property empty to be able to set "
                    "'Axis' and 'Center' manually.")
            obj.addProperty("App::PropertyLinkSubGlobal",
                            "AxisReference",
                            "Objects",
                            QT_TRANSLATE_NOOP("App::Property", _tip))
            obj.AxisReference = None

    def set_polar_properties(self, obj):
        """Set polar properties only if they don't exist."""
        properties = obj.PropertiesList

        if "NumberPolar" not in properties:
            _tip = "Number of copies in the polar direction"
            obj.addProperty("App::PropertyInteger",
                            "NumberPolar",
                            "Polar array",
                            QT_TRANSLATE_NOOP("App::Property", _tip))
            obj.NumberPolar = 5

        if "IntervalAxis" not in properties:
            _tip = "Distance and orientation of intervals in 'Axis' direction"
            obj.addProperty("App::PropertyVectorDistance",
                            "IntervalAxis",
                            "Polar array",
                            QT_TRANSLATE_NOOP("App::Property", _tip))
            obj.IntervalAxis = App.Vector(0, 0, 0)

        if "Angle" not in properties:
            _tip = "Angle to cover with copies"
            obj.addProperty("App::PropertyAngle",
                            "Angle",
                            "Polar array",
                            QT_TRANSLATE_NOOP("App::Property", _tip))
            obj.Angle = 360

    def set_circular_properties(self, obj):
        """Set circular properties only if they don't exist."""
        properties = obj.PropertiesList

        if "RadialDistance" not in properties:
            _tip = "Distance between circular layers"
            obj.addProperty("App::PropertyDistance",
                            "RadialDistance",
                            "Circular array",
                            QT_TRANSLATE_NOOP("App::Property", _tip))
            obj.RadialDistance = 50

        if "TangentialDistance" not in properties:
            _tip = "Distance between copies in the same circular layer"
            obj.addProperty("App::PropertyDistance",
                            "TangentialDistance",
                            "Circular array",
                            QT_TRANSLATE_NOOP("App::Property", _tip))
            obj.TangentialDistance = 25

        if "NumberCircles" not in properties:
            _tip = ("Number of circular layers. "
                    "The 'Base' object counts as one layer.")
            obj.addProperty("App::PropertyInteger",
                            "NumberCircles",
                            "Circular array",
                            QT_TRANSLATE_NOOP("App::Property", _tip))
            obj.NumberCircles = 3

        if "Symmetry" not in properties:
            _tip = ("A parameter that determines how many symmetry planes "
                    " the circular array will have.")
            obj.addProperty("App::PropertyInteger",
                            "Symmetry",
                            "Circular array",
                            QT_TRANSLATE_NOOP("App::Property", _tip))
            obj.Symmetry = 1

    def set_link_properties(self, obj):
        """Set link properties only if they don't exist."""
        properties = obj.PropertiesList

        if self.use_link:
            if "Count" not in properties:
                _tip = ("Total number of elements in the array.\n"
                        "This property is read-only, as the number depends "
                        "on the parameters of the array.")
                obj.addProperty("App::PropertyInteger",
                                "Count",
                                "Objects",
                                QT_TRANSLATE_NOOP("App::Property", _tip))
                obj.Count = 0
                obj.setEditorMode("Count", 1)  # Read only

            if "ExpandArray" not in properties:
                _tip = ("Show the individual array elements "
                        "(only for Link arrays)")
                obj.addProperty("App::PropertyBool",
                                "ExpandArray",
                                "Objects",
                                QT_TRANSLATE_NOOP("App::Property", _tip))
                obj.ExpandArray = False

    def linkSetup(self, obj):
        """Set up the object as a link object."""
        super(Array, self).linkSetup(obj)
        obj.configLinkProperty(ElementCount='Count')
        obj.setPropertyStatus('Count', 'Hidden')

    def onChanged(self, obj, prop):
        """Execute when a property is changed."""
        super(Array, self).onChanged(obj, prop)
        # print(prop, ": ", getattr(obj, prop))
        self.show_and_hide(obj, prop)

    def show_and_hide(self, obj, prop):
        """Show and hide the properties depending on the touched property."""
        if prop == "AxisReference":
            if obj.AxisReference:
                obj.setEditorMode("Center", 1)
                obj.setEditorMode("Axis", 1)
            else:
                obj.setEditorMode("Center", 0)
                obj.setEditorMode("Axis", 0)

        if prop == "ArrayType":
            if obj.ArrayType == "ortho":
                for pr in ("NumberX", "NumberY", "NumberZ",
                           "IntervalX", "IntervalY", "IntervalZ"):
                    obj.setPropertyStatus(pr, "-Hidden")

                for pr in ("Axis", "Center", "NumberPolar", "Angle",
                           "IntervalAxis", "NumberCircles",
                           "RadialDistance", "TangentialDistance",
                           "Symmetry"):
                    obj.setPropertyStatus(pr, "Hidden")

            if obj.ArrayType == "polar":
                for pr in ("Axis", "Center", "NumberPolar",
                           "Angle", "IntervalAxis"):
                    obj.setPropertyStatus(pr, "-Hidden")

                for pr in ("NumberX", "NumberY", "NumberZ",
                           "IntervalX", "IntervalY", "IntervalZ",
                           "NumberCircles", "RadialDistance",
                           "TangentialDistance", "Symmetry"):
                    obj.setPropertyStatus(pr, "Hidden")

            if obj.ArrayType == "circular":
                for pr in ("Axis", "Center", "NumberCircles",
                           "RadialDistance", "TangentialDistance",
                           "Symmetry"):
                    obj.setPropertyStatus(pr, "-Hidden")

                for pr in ("NumberX", "NumberY", "NumberZ",
                           "IntervalX", "IntervalY", "IntervalZ",
                           "NumberPolar", "Angle", "IntervalAxis"):
                    obj.setPropertyStatus(pr, "Hidden")

    def execute(self, obj):
        """Execture when the object is created or recomputed."""
        if not obj.Base:
            return

        pl = obj.Placement
        axis = obj.Axis
        center = obj.Center

        if hasattr(obj, "AxisReference") and obj.AxisReference:
            edge = obj.AxisReference[0].getSubObject(obj.AxisReference[1][0])
            if not hasattr(edge, "Curve"):
                _info = ("'AxisReference' has no 'Curve' attribute. "
                         "Please select a different object to use as "
                         "reference. It needs to be an edge.")
                raise TypeError(_info)

            if not isinstance(edge.Curve, Part.Line):
                _info = ("The 'AxisReference' 'Curve' attribute "
                         "does not contain an 'Part.Line' instance. "
                         "Please select a different object to use as "
                         "reference. It needs to be an edge.")
                raise TypeError(_info)

            axis = edge.Curve.Direction
            center = edge.Curve.Location

        if obj.ArrayType == "ortho":
            pls = rect_placements(obj.Base.Placement,
                                  obj.IntervalX,
                                  obj.IntervalY,
                                  obj.IntervalZ,
                                  obj.NumberX,
                                  obj.NumberY,
                                  obj.NumberZ)
        elif obj.ArrayType == "polar":
            av = obj.IntervalAxis if hasattr(obj, "IntervalAxis") else None
            pls = polar_placements(obj.Base.Placement,
                                   center, obj.Angle.Value,
                                   obj.NumberPolar, axis, av)
        elif obj.ArrayType == "circular":
            pls = circ_placements(obj.Base.Placement,
                                  obj.RadialDistance,
                                  obj.TangentialDistance,
                                  axis, center,
                                  obj.NumberCircles, obj.Symmetry)

        return super(Array, self).buildShape(obj, pl, pls)


_Array = Array


def rect_placements(base_placement,
                    xvector, yvector, zvector,
                    xnum, ynum, znum):
    """Determine the placements where the rectangular copies will be."""
    pl = base_placement
    placements = [pl.copy()]

    for xcount in range(xnum):
        currentxvector = App.Vector(xvector).multiply(xcount)
        if xcount != 0:
            npl = pl.copy()
            npl.translate(currentxvector)
            placements.append(npl)

        for ycount in range(ynum):
            currentyvector = App.Vector(currentxvector)
            _y_shift = App.Vector(yvector).multiply(ycount)
            currentyvector = currentyvector.add(_y_shift)
            if ycount != 0:
                npl = pl.copy()
                npl.translate(currentyvector)
                placements.append(npl)

            for zcount in range(znum):
                currentzvector = App.Vector(currentyvector)
                _z_shift = App.Vector(zvector).multiply(zcount)
                currentzvector = currentzvector.add(_z_shift)
                if zcount != 0:
                    npl = pl.copy()
                    npl.translate(currentzvector)
                    placements.append(npl)

    return placements


def polar_placements(base_placement,
                     center, angle,
                     number, axis, axisvector):
    """Determine the placements where the polar copies will be."""
    # print("angle ",angle," num ",num)
    placements = [base_placement.copy()]

    if number == 0:
        return placements

    spin = App.Placement(App.Vector(), base_placement.Rotation)
    pl = App.Placement(base_placement.Base, App.Rotation())
    center = center.sub(base_placement.Base)

    if angle == 360:
        fraction = float(angle)/number
    else:
        fraction = float(angle)/(number - 1)

    center_tuple = DraftVecUtils.tup(center)
    axis_tuple = DraftVecUtils.tup(axis)

    for i in range(number - 1):
        currangle = fraction + (i*fraction)
        npl = pl.copy()
        npl.rotate(center_tuple, axis_tuple, currangle)
        npl = npl.multiply(spin)
        if axisvector:
            if not DraftVecUtils.isNull(axisvector):
                npl.translate(App.Vector(axisvector).multiply(i+1))
        placements.append(npl)

    return placements


def circ_placements(base_placement,
                    r_distance, tan_distance,
                    axis, center,
                    circle_number, symmetry):
    """Determine the placements where the circular copies will be."""
    symmetry = max(1, symmetry)
    lead = (0, 1, 0)

    if axis.x == 0 and axis.z == 0:
        lead = (1, 0, 0)

    direction = axis.cross(App.Vector(lead)).normalize()
    placements = [base_placement.copy()]

    for xcount in range(1, circle_number):
        rc = xcount * r_distance
        c = 2 * rc * math.pi
        n = math.floor(c / tan_distance)
        n = int(math.floor(n / symmetry) * symmetry)
        if n == 0:
            continue

        angle = 360.0/n
        for ycount in range(0, n):
            npl = base_placement.copy()
            trans = App.Vector(direction).multiply(rc)
            npl.translate(trans)
            npl.rotate(npl.Rotation.inverted().multVec(center-trans),
                       axis,
                       ycount * angle)
            placements.append(npl)

    return placements
