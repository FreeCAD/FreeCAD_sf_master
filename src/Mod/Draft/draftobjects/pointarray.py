# ***************************************************************************
# *   Copyright (c) 2009, 2010 Yorik van Havre <yorik@uncreated.net>        *
# *   Copyright (c) 2009, 2010 Ken Cline <cline@frii.com>                   *
# *   Copyright (c) 2018 Benjamin Alterauge (ageeye)                        *
# *   Copyright (c) 2020 Carlo Pavan <carlopav@gmail.com>                   *
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
"""Provides the object code for the Draft PointArray object.

To Do
-----
This object currently uses the points inside a `Draft Block`,
a `Part::Compound`, or a `Sketch`. To make this more general,
this object could be augmented to extract a list of points
from the vertices of any object with a `Part::TopoShape` (2D or 3D).

See the `get_point_list` function for more information.
"""
## @package pointarray
# \ingroup DRAFT
# \brief Provides the object code for the Draft PointArray object.

from PySide.QtCore import QT_TRANSLATE_NOOP

import FreeCAD as App
import draftutils.utils as utils
import lazy_loader.lazy_loader as lz

from draftutils.messages import _wrn, _err
from draftutils.translate import translate, _tr
from draftobjects.base import DraftObject

# Delay import of module until first use because it is heavy
Part = lz.LazyLoader("Part", globals(), "Part")


class PointArray(DraftObject):
    """The Draft Point Array object."""

    def __init__(self, obj):
        super(PointArray, self).__init__(obj, "PointArray")
        self.set_properties(obj)

    def set_properties(self, obj):
        """Set properties only if they don't exist."""
        properties = obj.PropertiesList

        if "Base" not in properties:
            _tip = "Base object that will be duplicated"
            obj.addProperty("App::PropertyLink",
                            "Base",
                            "Objects",
                            QT_TRANSLATE_NOOP("App::Property", _tip))
            obj.Base = None

        if "PointObject" not in properties:
            _tip = ("Object containing points used to distribute "
                    "the base object, for example, a sketch or "
                    "a Part compound.\n"
                    "The sketch or compound must contain at least "
                    "one explicit point or vertex object.")
            obj.addProperty("App::PropertyLink",
                            "PointObject",
                            "Objects",
                            QT_TRANSLATE_NOOP("App::Property", _tip))
            obj.PointObject = None

        if "Count" not in properties:
            _tip = ("Total number of elements in the array.\n"
                    "This property is read-only, as the number depends "
                    "on the points contained within 'Point Object'.")
            obj.addProperty("App::PropertyInteger",
                            "Count",
                            "Objects",
                            QT_TRANSLATE_NOOP("App::Property", _tip))
            obj.Count = 0
            obj.setEditorMode("Count", 1)  # Read only

        if "ExtraPlacement" not in properties:
            _tip = ("Additional placement, shift and rotation, "
                    "that will be applied to each copy")
            obj.addProperty("App::PropertyPlacement",
                            "ExtraPlacement",
                            "Objects",
                            QT_TRANSLATE_NOOP("App::Property", _tip))
            obj.ExtraPlacement = App.Placement()

    def execute(self, obj):
        """Run when the object is created or recomputed."""
        if not hasattr(obj.Base, 'Shape'):
            _err(_tr("Base object doesn't have a 'Shape', "
                     "it cannot be used for an array."))
            obj.Count = 0
            return

        pt_list, count = get_point_list(obj.PointObject)
        shape = build_copies(obj.Base, pt_list, obj.ExtraPlacement)

        obj.Shape = shape
        obj.Count = count

    def onDocumentRestored(self, obj):
        """Execute code when the document is restored.

        Add properties that don't exist and migrate old properties.
        """
        # If the ExtraPlacement property has never been added before
        # it will add it first, and set it to the base object's position
        # in order to produce the same displacement as before.
        # Then all the other properties will be processed.
        properties = obj.PropertiesList

        if "ExtraPlacement" not in properties:
            _tip = ("Additional placement, shift and rotation, "
                    "that will be applied to each copy")
            obj.addProperty("App::PropertyPlacement",
                            "ExtraPlacement",
                            "Objects",
                            QT_TRANSLATE_NOOP("App::Property", _tip))
            obj.ExtraPlacement.Base = obj.Base.Placement.Base
            _info = "added property 'ExtraPlacement'"
            _wrn("v0.19, " + obj.Label + ", " + _tr(_info))

        self.set_properties(obj)
        self.migrate_properties_0v19(obj)

    def migrate_properties_0v19(self, obj):
        """Migrate properties."""
        # If the old name still exists, migrate it to the new
        # name and delete the old property
        properties = obj.PropertiesList

        if "PointList" in properties:
            obj.PointObject = obj.PointList
            obj.removeProperty("PointList")
            _info = "'PointList' property will be migrated to 'PointObject'"
            _wrn("v0.19, " + obj.Label + ", " + _tr(_info))


def get_point_list(point_object):
    """Extract a list of points from a point object.

    Parameters
    ----------
    point_object: Part::Feature
        Either a `Draft Block`, a `Part::Compound`,
        or a `Sketcher::SketchObject` containing points.

    Returns
    -------
    list, int
        A list of points that have `X`, `Y`, `Z` coordinates;
        the second element is the number of elements.
        If the list is empty, the second element is zero.

    To Do
    -----
    - This function currently extracts the points inside a `Draft Block`,
      a `Part::Compound`, or a `Sketch`. To make this more general,
      this function could be augmented to extract a list of points
      from the vertices of any object with a `Part::TopoShape` (2D or 3D).

    - If the input is a `Part::Compound`, it should handle all valid types
      of objects simultaneously, that is, `Draft Points`, `Part::Vertexes`,
      `Sketches` with points, and possibly any other object with
      a `Part::TopoShape` as in the previous point.
      It should recursively call itself to extract
      points contained in nested compounds.
    """
    # If its a clone, extract the real object
    while utils.get_type(point_object) == 'Clone':
        point_object = point_object.Objects[0]

    # If the point object doesn't have actual points
    # the point list will remain empty
    pt_list = list()

    if hasattr(point_object, 'Geometry'):
        # Intended for a Sketcher::SketchObject, which has this property
        place = point_object.Placement
        for geo in point_object.Geometry:
            # It must contain at least one Part::GeomPoint.
            if (hasattr(geo, 'X')
                    and hasattr(geo, 'Y') and hasattr(geo, 'Z')):
                point = geo.copy()
                point.translate(place.Base)
                point.rotate(place)
                pt_list.append(point)

        count = len(pt_list)
        return pt_list, count

    obj_list = list()
    if hasattr(point_object, 'Links'):
        # Intended for a Part::Compound, which has this property
        obj_list = point_object.Links
    elif hasattr(point_object, 'Components'):
        # Intended for a Draft Block, which has this property
        obj_list = point_object.Components

    # These compounds should have at least one discrete point object
    # like a Draft Point or a Part::Vertex
    for _obj in obj_list:
        if hasattr(_obj, 'X') and hasattr(_obj, 'Y') and hasattr(_obj, 'Z'):
            pt_list.append(_obj)

    count = len(pt_list)
    return pt_list, count


def build_copies(base_object, pt_list=None, placement=App.Placement()):
    """Build a compound of copies from the base object and list of points.

    Returns
    -------
    Part::TopoShape
        The compound shape created by `Part.makeCompound`.
    """
    if not pt_list:
        _err(translate("Draft",
                       "Point object doesn't have a discrete point, "
                       "it cannot be used for an array."))
        shape = base_object.Shape.copy()
        return shape

    copies = list()

    for point in pt_list:
        new_shape = base_object.Shape.copy()
        original_rotation = new_shape.Placement.Rotation

        # Reset the position of the copy, and combine the original rotation
        # with the provided rotation. Two rotations (quaternions)
        # are combined by multiplying them.
        new_shape.Placement.Base = placement.Base
        new_shape.Placement.Rotation = original_rotation * placement.Rotation

        if point.TypeId == "Part::Vertex":
            # For this object the final position is the value of the Placement
            # plus the value of the X, Y, Z properties
            place = App.Vector(point.X,
                               point.Y,
                               point.Z) + point.Placement.Base

        elif hasattr(point, 'Placement'):
            # If the point object has a placement (Draft Point), use it
            # to displace the copy of the shape
            place = point.Placement.Base

            # The following old code doesn't make much sense because it uses
            # the rotation value of the auxiliary point.
            # Even if the point does have a rotation property as part of its
            # placement, rotating a point by itself is a strange workflow.
            # We want to use the position of the point but not its rotation,
            # which will probably be zero anyway.

            # Old code:
            # place = point.Placement
            # new_shape.rotate(place.Base,
            #                  place.Rotation.Axis,
            #                  math.degrees(place.Rotation.Angle))
        else:
            # In other cases (Sketch with points)
            # translate by the X, Y, Z coordinates
            place = App.Vector(point.X, point.Y, point.Z)

        new_shape.translate(place)

        copies.append(new_shape)

    shape = Part.makeCompound(copies)
    return shape


_PointArray = PointArray
