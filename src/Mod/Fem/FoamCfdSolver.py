#***************************************************************************
#*                                                                         *
#*   Copyright (c) 2015 - Qingfeng Xia <qingfeng.xia()eng.ox.ac.uk> *
#*                                                                         *
#*   This program is free software; you can redistribute it and/or modify  *
#*   it under the terms of the GNU Lesser General Public License (LGPL)    *
#*   as published by the Free Software Foundation; either version 2 of     *
#*   the License, or (at your option) any later version.                   *
#*   for detail see the LICENCE text file.                                 *
#*                                                                         *
#*   This program is distributed in the hope that it will be useful,       *
#*   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
#*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
#*   GNU Library General Public License for more details.                  *
#*                                                                         *
#*   You should have received a copy of the GNU Library General Public     *
#*   License along with this program; if not, write to the Free Software   *
#*   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  *
#*   USA                                                                   *
#*                                                                         *
#***************************************************************************

__title__ = "Command and Classes for New CAE Analysis"
__author__ = "Qingfeng Xia"
__url__ = "http://www.freecadweb.org"

import FreeCAD
if FreeCAD.GuiUp:
    import FreeCADGui
    from PySide import QtCore

class CaeSolver():
    """The Fem::FemSolver 's Proxy python type
    add solver specific Properties and Bring up SolverControlTaskPanel
    """
    def __init__(self, obj):
        self.Type = "CfdSolver"
        self.Object=obj #keep a ref to the DocObj for nonGui usage
        obj.Proxy = self #link between App::DocumentObject to  this object
        
        #general CFD properties
        TurbulenceModelList={"Laminar","KE","KW","LES"}
        #obj.addProperty("App::PropertyEnum", "TurbulenceModel", "CFD", "Laminar,KE,KW,LES,") #should be Enum
        #obj.addProperty("App::PropertyBool", "Compressible", "CFD", "Compressible air or Incompressible like liquid")
        #obj.addProperty("App::PropertyBool","ThermalAnalysisEnabled", "CFD","calc heat transfering")
        
        #adding solver specific properties
        obj.addProperty("App::PropertyString", "CommandLine", "OpenFOAM", "comand line string to solve analysis")
        
    def generate_cmdline(self):
        pass
    
    #following are the standard methods
    def execute(self, obj):
        return

    def onChanged(self, obj, prop):
        """updated Part should lead to recompution of the analysis"""
        return # to-do 

    def __getstate__(self):
        return self.Type

    def __setstate__(self, state):
        if state:
            self.Type = state

#this class could be moved into CaeSolver, as it can be shared by any solver
class ViewProviderCaeSolver:
    """A View Provider for the Solver object, base class for all derived solver
    derived solver should implement  a specific TaskPanel and set up solver and override setEdit()"""

    def __init__(self, vobj):
        vobj.Proxy = self

    def getIcon(self):
        return ":/icons/fem-solver.svg"

    def attach(self, vobj):
        self.ViewObject = vobj
        self.Object = vobj.Object

    def updateData(self, obj, prop):
        return

    def onChanged(self, vobj, prop):
        return

    def doubleClicked(self, vobj):
        if not FemGui.getActiveAnalysis() == self.Object:
            if FreeCADGui.activeWorkbench().name() != 'FemWorkbench':
                FreeCADGui.activateWorkbench("FemWorkbench")  
            FemGui.setActiveAnalysis(self.Object)
            return True
        else:
            #from import _SolverControlTaskPanel
            taskd = _SolverControlTaskPanel(self.Object)  
            FreeCADGui.Control.showDialog(taskd)
        return True
        
    def setEdit(self, vobj, mode):
        #import module if it is defined in another file
        taskd = _SolverControlTaskPanel(self.Object) 
        taskd.obj = vobj.Object
        FreeCADGui.Control.showDialog(taskd)
        return True

    def unsetEdit(self, vobj, mode):
        #FreeCADGui.Control.closeDialog() #Todo
        return

    def __getstate__(self):
        return None

    def __setstate__(self, state):
        return None

if FreeCAD.GuiUp:
    import FreeCADGui
    import FemGui
    from PySide import QtCore, QtGui
    from PySide.QtCore import Qt
    from PySide.QtGui import QApplication
    
class _SolverControlTaskPanel:
    def __init__(self, solver_object):
        #self.ui = App.getResourceDir() + "Mod/Fem/_SolverControlTaskPanel.ui"
        QtGui.QMessageBox.critical(None, "This task panel is not implement yet", "Please editor ")
        pass
        
    def accept(self):
        return True

    def reject(self):
        return True