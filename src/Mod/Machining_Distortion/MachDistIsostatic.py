#***************************************************************************
#*                                                                         *
#*   Copyright (c) 2013 - Juergen Riegel <FreeCAD@juergen-riegel.net>      *  
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

import FreeCAD, Fem, Mesh

if FreeCAD.GuiUp:
    import FreeCADGui,FemGui
    from FreeCAD import Vector
    from PyQt4 import QtCore, QtGui
    from pivy import coin
    import PyQt4.uic as uic

__title__="Machine-Distortion Isostatic managment"
__author__ = "Juergen Riegel"
__url__ = "http://free-cad.sourceforge.net"




def getBoundaryCoditions(Mesh):
    BndBox = Mesh.BoundBox
    FirstLength = 10000.0
    FirstIndex  = -1
    SecondLength = 10000.0
    SecondIndex  = -1
    ThirdLength = 10000.0
    ThirdIndex  = -1
    
    for id,i in Mesh.Nodes.items():
        l = (i-FreeCAD.Vector(BndBox.XMin,BndBox.YMin,BndBox.ZMin)).Length
        if FirstLength > l:
            FirstLength = l
            FirstIndex = id
            
        l = (i-FreeCAD.Vector(BndBox.XMax,BndBox.YMin,BndBox.ZMin)).Length
        if SecondLength > l:
            SecondLength = l
            SecondIndex = id
            
        l = (i-FreeCAD.Vector(BndBox.XMin,BndBox.YMax,BndBox.ZMin)).Length
        if ThirdLength > l:
            ThirdLength = l
            ThirdIndex = id

        
    print FirstIndex,SecondIndex,ThirdIndex
    return (FirstIndex,SecondIndex,ThirdIndex)

def makeIsostatic(name):
    '''makeMaterial(name): makes an Material
    name there fore is a material name or an file name for a FCMat file'''
    obj = FreeCAD.ActiveDocument.addObject("App::FeaturePython",name)
    _IsostaticNodes(obj)
    _ViewProviderIsostaticNodes(obj.ViewObject)
    #FreeCAD.ActiveDocument.recompute()
    return obj


class _CommandIsostatic:
    "the MachDist Isostatic command definition"
    def GetResources(self):
        return {'Pixmap'  : 'MachDist_Isostatic',
                'MenuText': QtCore.QT_TRANSLATE_NOOP("MachDist_Isostatic","Machine-Distortion Isostatic"),
                'Accel': "A",
                'ToolTip': QtCore.QT_TRANSLATE_NOOP("MachDist_Isostatic","Add or edit a Machine-Distortion Isostatic")}
        
    def Activated(self):
        
        FreeCAD.ActiveDocument.openTransaction("Isostatic")

        obj = None
        FemMeshObj = None
        if FemGui.getActiveAnalysis():
            for i in FemGui.getActiveAnalysis().Member:
                if i.isDerivedFrom("App::FeaturePython"):
                    if i.Proxy.Type == 'MachDist_IsostaticNodes':
                        obj = i
                        break
        else: 
            return 
        
        for i in FemGui.getActiveAnalysis().Member:
            if i.isDerivedFrom("Fem::FemMeshObject"):
                FemMeshObj = i
            
        if not obj:
            FreeCADGui.addModule("MachDistIsostatic")
            FreeCADGui.doCommand("MachDistIsostatic.makeIsostatic('IsostaticNodes')")
            obj = FreeCAD.activeDocument().ActiveObject
            FreeCADGui.doCommand("FemGui.getActiveAnalysis().Member = FemGui.getActiveAnalysis().Member + [App.activeDocument().ActiveObject]")
        
        #node_numbers = Fem.getBoundary_Conditions(FemMeshObj.FemMesh)
        node_numbers = getBoundaryCoditions(FemMeshObj.FemMesh)
        obj.IsostaticNodes = node_numbers
        
        nodes = FemMeshObj.FemMesh.Nodes
        meshObj = None
        
        for i in FemGui.getActiveAnalysis().Member:
            if i.isDerivedFrom("Mesh::Feature"):
                meshObj = i
                break
                    
        if not meshObj:
            FreeCADGui.doCommand("App.activeDocument().addObject('Mesh::Feature','IsostaticPlane')")
            meshObj = FreeCAD.activeDocument().ActiveObject
            meshObj.ViewObject.ShapeColor = (0.0, 1.0, 0.0, 0.0)
            FreeCADGui.doCommand("FemGui.getActiveAnalysis().Member = FemGui.getActiveAnalysis().Member + [App.activeDocument().ActiveObject]")
        
        planarMesh = [
            # triangle 1
            nodes[node_numbers[0]],nodes[node_numbers[1]],nodes[node_numbers[2]],
            #triangle 2
            #[-0.5000,-0.5000,0.0000],[0.5000,-0.5000,0.0000],[0.5000,0.5000,0.0000],
            ]
        aMesh = Mesh.Mesh(planarMesh)
        meshObj.Mesh = aMesh

        taskd = _IsostaticTaskPanel(obj,meshObj,FemMeshObj)
        
        FreeCADGui.Control.showDialog(taskd)

    def IsActive(self):
        if FemGui.getActiveAnalysis():
            for i in FemGui.getActiveAnalysis().Member:
                if i.isDerivedFrom("Fem::FemMeshObject"):
                    return True
        else:
            return False
       
class _IsostaticNodes:
    "The IsostaticNodes object"
    def __init__(self,obj):
        self.Type = "MachDist_IsostaticNodes"
        obj.Proxy = self
        obj.addProperty("App::PropertyIntegerList","IsostaticNodes","Base",
                        "The isostatic node numbers")

        
    def execute(self,obj):
        return
        
    def onChanged(self,obj,prop):
        if prop in ["IsostaticNodes"]:
            return

    def __getstate__(self):
        return self.Type

    def __setstate__(self,state):
        if state:
            self.Type = state
        
class _ViewProviderIsostaticNodes:
    "A View Provider for the IsostaticNodes object"

    def __init__(self,vobj):
        #vobj.addProperty("App::PropertyLength","BubbleSize","Base", str(translate("MachDist","The size of the axis bubbles")))
        vobj.Proxy = self
       
    def getIcon(self):
        import machdist_rc
        return ":/icons/MachDist_Isostatic.svg"

    def attach(self, vobj):
        self.ViewObject = vobj
        self.Object = vobj.Object
  
    def setEdit(self,vobj,mode):
        FemMeshObj = None
        if FemGui.getActiveAnalysis():
            for i in FemGui.getActiveAnalysis().Member:
                if i.isDerivedFrom("Fem::FemMeshObject"):
                    FemMeshObj = i
                    break
        else: 
            return False
        
        for i in FemGui.getActiveAnalysis().Member:
            if i.isDerivedFrom("Fem::FemMeshObject"):
                FemMeshObj = i
        taskd = _IsostaticTaskPanel(self.Object, None,FemMeshObj)
        taskd.obj = vobj.Object
        taskd.update()
        FreeCADGui.Control.showDialog(taskd)
        return True
    
    def unsetEdit(self,vobj,mode):
        FreeCADGui.Control.closeDialog()
        return

 
class _IsostaticTaskPanel:
    '''The editmode TaskPanel for Material objects'''
    def __init__(self,obj,meshObj,femMeshObj):
        # the panel has a tree widget that contains categories
        # for the subcomponents, such as additions, subtractions.
        # the categories are shown only if they are not empty.
        form_class, base_class = uic.loadUiType(FreeCAD.getHomePath() + "Mod/Machining_Distortion/Isostatic.ui")

        self.obj = obj
        self.meshObj = meshObj
        self.femMeshObj = femMeshObj
        self.formUi = form_class()
        self.form = QtGui.QWidget()
        self.formUi.setupUi(self.form)
        self.params = FreeCAD.ParamGet("User parameter:BaseApp/Preferences/Mod/Machining_Distortion")
        

        #QtCore.QObject.connect(self.formUi.select_L_file, QtCore.SIGNAL("clicked()"), self.add_L_data)
        self.femMeshObj.ViewObject.Transparency = 50
        self.meshObj.ViewObject.Visibility=True
        
        self.update()
        

    def getStandardButtons(self):
        return int(QtGui.QDialogButtonBox.Ok) | int(QtGui.QDialogButtonBox.Cancel)
    
    def update(self):
        'fills the widgets'
        OutStr = 'Isostatic Plane:\n'
        
        IsoNodes = list(self.obj.IsostaticNodes)
        
        AllNodes = self.femMeshObj.FemMesh.Nodes
        GridNode1 = AllNodes[IsoNodes[0]]
        GridNode2 = AllNodes[IsoNodes[1]]
        GridNode3 = AllNodes[IsoNodes[2]]
        
        OutStr = OutStr + 'Nodes: '+`IsoNodes[0]`+', '+`IsoNodes[1]`+', '+`IsoNodes[2]`+'\n'
        OutStr = OutStr + '('+`GridNode1.x`[0:6]+', '+`GridNode1.y`[0:6]+', '+`GridNode1.z`[0:6]+')\n'
        OutStr = OutStr + '('+`GridNode2.x`[0:6]+', '+`GridNode2.y`[0:6]+', '+`GridNode2.z`[0:6]+')\n'
        OutStr = OutStr + '('+`GridNode3.x`[0:6]+', '+`GridNode3.y`[0:6]+', '+`GridNode3.z`[0:6]+')\n'
        

        self.formUi.textEdit.setText(OutStr)
        return 
                
    def accept(self):
        self.femMeshObj.ViewObject.Transparency = 0
        self.meshObj.ViewObject.Visibility=False
        FreeCAD.ActiveDocument.commitTransaction()
        FreeCADGui.Control.closeDialog()

                    
    def reject(self):
        self.femMeshObj.ViewObject.Transparency = 0
        self.meshObj.ViewObject.Visibility=False
        FreeCAD.ActiveDocument.abortTransaction()
        FreeCADGui.Control.closeDialog()


       
FreeCADGui.addCommand('MachDist_Isostatic',_CommandIsostatic())
