# Fem gui init module
# (c) 2009 Juergen Riegel
#
# Gathering all the information to start FreeCAD
# This is the second one of three init scripts, the third one
# runs when the gui is up

#***************************************************************************
#*   (c) Juergen Riegel (juergen.riegel@web.de) 2009                       *
#*                                                                         *
#*   This file is part of the FreeCAD CAx development system.              *
#*                                                                         *
#*   This program is free software; you can redistribute it and/or modify  *
#*   it under the terms of the GNU Lesser General Public License (LGPL)    *
#*   as published by the Free Software Foundation; either version 2 of     *
#*   the License, or (at your option) any later version.                   *
#*   for detail see the LICENCE text file.                                 *
#*                                                                         *
#*   FreeCAD is distributed in the hope that it will be useful,            *
#*   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
#*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
#*   GNU Lesser General Public License for more details.                   *
#*                                                                         *
#*   You should have received a copy of the GNU Library General Public     *
#*   License along with FreeCAD; if not, write to the Free Software        *
#*   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  *
#*   USA                                                                   *
#*                                                                         *
#*   Juergen Riegel 2002                                                   *
#***************************************************************************/


class FemWorkbench (Workbench):
    "Fem workbench object"
    def __init__(self):
        self.__class__.Icon = FreeCAD.getResourceDir() + "Mod/Fem/Resources/icons/preferences-fem.svg"
        self.__class__.MenuText = "FEM"
        self.__class__.ToolTip = "FEM workbench"

    def Initialize(self):
        # load the module
        import Fem
        import FemGui
        
        import CaeAnalysis
        import CaeSolver
        """# FemGui::Workbench  may needs some modification to add Toolbar from python
        from PySide import QtCore
        cmdlst = ["Fem_NewCfdAnalysis","Fem_NewMechanicalAnalysis"]
        self.appendToolbar(str(QtCore.QT_TRANSLATE_NOOP("Fem", "Fem tools")), cmdlst)
        self.appendMenu(str(QtCore.QT_TRANSLATE_NOOP("Fem", "Fem")), cmdlst)
        """
        #The following code should be moved into SolverObject init function!
        import subprocess
        from platform import system
        ccx_path = FreeCAD.ParamGet("User parameter:BaseApp/Preferences/Mod/Fem").GetString("ccxBinaryPath")
        if not ccx_path:
            try:
                if system() == 'Linux':
                    p1 = subprocess.Popen(['which', 'ccx'], stdout=subprocess.PIPE)
                    if p1.wait() == 0:
                        ccx_path = p1.stdout.read().split('\n')[0]
                elif system() == 'Windows':
                    ccx_path = FreeCAD.getHomePath() + 'bin/ccx.exe'
                FreeCAD.ParamGet("User parameter:BaseApp/Preferences/Mod/Fem").SetString("ccxBinaryPath", ccx_path)
            except Exception as e:
                FreeCAD.Console.PrintError(e.message)

    def GetClassName(self):
        return "FemGui::Workbench"

Gui.addWorkbench(FemWorkbench())
