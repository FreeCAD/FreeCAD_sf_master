# ***************************************************************************
# *   Copyright (c) 2009, 2010 Yorik van Havre <yorik@uncreated.net>        *
# *   Copyright (c) 2009, 2010 Ken Cline <cline@frii.com>                   *
# *   Copyright (c) 2020 FreeCAD Developers                                 *
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
"""This module provides the code for Draft make_copy function.
"""
## @package make_copy
# \ingroup DRAFT
# \brief This module provides the code for Draft make_copy function.

import FreeCAD as App

import draftutils.utils as utils
import draftutils.gui_utils as gui_utils



def make_copy(obj, force=None, reparent=False, simple_copy=False):
    """makeCopy(object, [force], [reparent])
    
    Make an exact copy of an object and return it.
    
    Parameters
    ----------
    obj :
        Object to copy.

    force :
        Obsolete, not used anymore.

    reparent :
        Group the new object in the same group of the old one.

    simple_copy :
        Create a simple copy of the object (a new non parametric 
        Part::Feature with the same Shape as the given object).
    """
    if not App.ActiveDocument:
        App.Console.PrintError("No active document. Aborting\n")
        return

    newobj = None

    if simple_copy and hasattr(obj, 'Shape'):
        _name = utils.get_real_name(obj.Name)
        newobj = App.ActiveDocument.addObject("Part::Feature", _name)
        newobj.Shape = obj.Shape
        gui_utils.format_object(newobj, obj)
    elif not simple_copy:
        newobj = App.ActiveDocument.copyObject(obj)

    if not newobj:
        return None

    if reparent:
        parents = obj.InList
        if parents:
            for par in parents:
                if par.isDerivedFrom("App::DocumentObjectGroup") or par.isDerivedFrom("App::Part"):
                    par.addObject(newobj)
                else:
                    # That's the case of Arch_BuildingParts or Draft_Layers for example
                    if "Group" in par.PropertiesList:
                        if obj in par.Group:
                            group = par.Group
                            group.append(newobj)
                            par.Group = group

    return newobj
    