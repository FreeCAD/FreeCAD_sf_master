# ***************************************************************************
# *                                                                         *
# *   Copyright (c) 2017 - Markus Hovorka <m.hovorka@live.de>               *
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


__title__ = "_ViewProviderFemSolverElmer"
__author__ = "Markus Hovorka"
__url__ = "http://www.freecadweb.org"


import FreeCADGui as Gui
import PyGui._TaskPanelFemSolverControl
import FemSolve


class _ViewProviderFemSolverElmer(object):
    """Proxy for FemSolverElmers View Provider."""

    def __init__(self, vobj):
        vobj.Proxy = self

    def getIcon(self):
        return ":/icons/fem-elmer.png"

    def setEdit(self, vobj, mode=0):
        machine = FemSolve.getMachine(vobj.Object)
        task = PyGui._TaskPanelFemSolverControl.ControlTaskPanel(machine)
        Gui.Control.showDialog(task)
        return True

    def unsetEdit(self, vobj, mode=0):
        Gui.Control.closeDialog()

    def doubleClicked(self, vobj):
        if Gui.Control.activeDialog():
            Gui.Control.closeDialog()
        Gui.ActiveDocument.setEdit(vobj.Object.Name)
        return True

    def attach(self, vobj):
        pass
