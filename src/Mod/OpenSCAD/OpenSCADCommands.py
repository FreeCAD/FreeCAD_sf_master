import FreeCAD,FreeCADGui
from PyQt4 import QtGui, QtCore

def translate(context,text):
    "convenience function for Qt translator"
    return QtGui.QApplication.translate(context, text, None, \
        QtGui.QApplication.UnicodeUTF8)
def utf8(unio):
    return unicode(unio).encode('UTF8')

class ColorCodeShape:
    "Change the Color of selected or all Shapes based on their validity"
    def Activated(self):
        import colorcodeshapes
        selection=FreeCADGui.Selection.getSelectionEx()
        if len(selection) > 0:
            objs=[selobj.Object for selobj in selection]

        else:
            objs=FreeCAD.ActiveDocument.Objects
        colorcodeshapes.colorcodeshapes(objs)
    def GetResources(self):
        return {'Pixmap'  : 'OpenSCAD_ColorCodeShape', 'MenuText': \
                QtCore.QT_TRANSLATE_NOOP('OpenSCAD_ColorCodeShape',\
                'Color Shapes'), 'ToolTip': \
                QtCore.QT_TRANSLATE_NOOP('OpenSCAD_ColorCodeShape',\
                'Color Shapes by validity and type')}

class Edgestofaces:
    def Activated(self):
        from OpenSCAD2Dgeom import edgestofaces,Overlappingfaces
        selection=FreeCADGui.Selection.getSelectionEx()
        edges=[]
        for selobj in selection:
            edges.extend(selobj.Object.Shape.Edges)
        Overlappingfaces(edgestofaces(edges,None)).makefeatures(FreeCAD.ActiveDocument)
        for selobj in selection:
            selobj.Object.ViewObject.hide()
        FreeCAD.ActiveDocument.recompute()

    def GetResources(self):
        return {'Pixmap'  : 'python', 'MenuText': QtCore.QT_TRANSLATE_NOOP(\
                'OpenSCAD_Edgestofaces','Convert Edges To Faces'),
                'ToolTip': QtCore.QT_TRANSLATE_NOOP('OpenSCAD',\
                'Convert Edges to Faces')}

class RefineShapeFeature:
    def Activated(self):
        import Part,OpenSCADFeatures
        selection=FreeCADGui.Selection.getSelectionEx()
        for selobj in selection:
            #newobj=FreeCAD.ActiveDocument.addObject("Part::FeaturePython",'refine')
            newobj=selobj.Document.addObject("Part::FeaturePython",'refine')
            OpenSCADFeatures.RefineShape(newobj,selobj.Object)
            OpenSCADFeatures.ViewProviderTree(newobj.ViewObject)
            newobj.Label='refine_%s' % selobj.Object.Label
            selobj.Object.ViewObject.hide()
        FreeCAD.ActiveDocument.recompute()
    def GetResources(self):
        return {'Pixmap'  : 'OpenSCAD_RefineShapeFeature', 'MenuText': \
                QtCore.QT_TRANSLATE_NOOP('OpenSCAD_RefineShapeFeature',\
                'Refine Shape Feature'), 'ToolTip': \
                QtCore.QT_TRANSLATE_NOOP('OpenSCAD_RefineShapeFeature',\
                'Create Refine Shape Feature')}


class ExpandPlacements:
    '''This should aid interactive repair in the future
    but currently it breaks extrusions, as axis, base and so on have to be
    recalculated'''
    def Activated(self):
        import expandplacements
        selobj=FreeCADGui.Selection.getSelectionEx()[0]
        expandplacements.expandplacements(selobj.Object,FreeCAD.Placement())
        FreeCAD.ActiveDocument.recompute()
    def GetResources(self):
        return {'Pixmap'  : 'python', 'MenuText': QtCore.QT_TRANSLATE_NOOP(\
                'OpenSCAD_ExpandPlacements','Expand Placements'), 'ToolTip': \
                QtCore.QT_TRANSLATE_NOOP('OpenSCAD_ExpandPlacements',\
                'Expand all placements downwards the FeatureTree')}

class ReplaceObject:
    def Activated(self):
        import replaceobj
        #objs=[selobj.Object for selobj in FreeCADGui.Selection.getSelectionEx()]
        objs=FreeCADGui.Selection.getSelection()
        if len(objs)==3:
            replaceobj.replaceobjfromselection(objs)
        else:
            FreeCAD.Console.PrintError(unicode(translate('OpenSCAD',\
                    'Please select 3 objects first'))+u'\n')
    def GetResources(self):
        return {'Pixmap'  : 'OpenSCAD_ReplaceObject', 'MenuText': \
                QtCore.QT_TRANSLATE_NOOP('OpenSCAD_ReplaceObject',\
                'Replace Object'), 'ToolTip': \
                QtCore.QT_TRANSLATE_NOOP('OpenSCAD_ReplaceObject',\
                'Replace an object in the Feature Tree. Please select old, new and parent object')}


class RemoveSubtree:
    def Activated(self):
        def addsubobjs(obj,toremoveset):
            toremove.add(obj)
            for subobj in obj.OutList:
                addsubobjs(subobj,toremoveset)

        import FreeCAD,FreeCADGui
        objs=FreeCADGui.Selection.getSelection()
        toremove=set()
        for obj in objs:
            addsubobjs(obj,toremove)
        checkinlistcomplete =False
        while not checkinlistcomplete:
            for obj in toremove:
                if (obj not in objs) and (frozenset(obj.InList) - toremove):
                    toremove.remove(obj)
                    break
            else:
                checkinlistcomplete = True
        for obj in toremove:
            obj.Document.removeObject(obj.Name)
    def GetResources(self):
        return {'Pixmap'  : 'OpenSCAD_RemoveSubtree', 'MenuText': \
                QtCore.QT_TRANSLATE_NOOP('OpenSCAD_RemoveSubtree',\
                'Remove Objects and their Children'), 'ToolTip': \
                QtCore.QT_TRANSLATE_NOOP('OpenSCAD_RemoveSubtree',\
                'Removes the selected objects and all children that are not referenced from other objects')}

class AddSCADWidget(QtGui.QWidget):
    def __init__(self,*args):
        QtGui.QWidget.__init__(self,*args)
        self.textEdit=QtGui.QTextEdit()
        self.buttonadd = QtGui.QPushButton(translate('OpenSCAD',u'Add'))
        self.buttonclear = QtGui.QPushButton(translate('OpenSCAD',u'Clear'))
        self.checkboxmesh = QtGui.QCheckBox(translate('OpenSCAD',u'as Mesh'))
        layouth=QtGui.QHBoxLayout()
        layouth.addWidget(self.buttonadd)
        layouth.addWidget(self.buttonclear)
        layout= QtGui.QVBoxLayout()
        layout.addLayout(layouth)
        layout.addWidget(self.checkboxmesh)
        layout.addWidget(self.textEdit)
        self.setLayout(layout)
        self.setWindowTitle(translate('OpenSCAD',u'Add OpenSCAD Element'))
        self.textEdit.setText(u'cube();')
        self.buttonclear.clicked.connect(self.textEdit.clear)

    def retranslateUi(self, widget=None):
        self.buttonadd.setText(translate('OpenSCAD',u'Add'))
        self.buttonclear.setText(translate('OpenSCAD',u'Clear'))
        self.checkboxmesh.setText(translate('OpenSCAD',u'as Mesh'))
        self.setWindowTitle(translate('OpenSCAD',u'Add OpenSCAD Element'))

class AddSCADTask:
    def __init__(self):
        self.form = AddSCADWidget()
        self.form.buttonadd.clicked.connect(self.addelement)
    def getStandardButtons(self):
        return int(QtGui.QDialogButtonBox.Close)

    def isAllowedAlterSelection(self):
        return True

    def isAllowedAlterView(self):
        return True

    def isAllowedAlterDocument(self):
        return True

    def addelement(self):
        scadstr=unicode(self.form.textEdit.toPlainText())
        asmesh=self.form.checkboxmesh.checkState()
        import OpenSCADUtils, os
        extension= 'stl' if asmesh else 'csg'
        tmpfilename=OpenSCADUtils.callopenscadstring(scadstr,extension)
        if tmpfilename:
            doc=FreeCAD.activeDocument() or FreeCAD.newDocument()
            if asmesh:
                import Mesh
                Mesh.insert(tmpfilename,doc.Name)
            else:
                import importCSG
                importCSG.insert(tmpfilename,doc.Name)
            os.unlink(tmpfilename)
        else:
            FreeCAD.Console.PrintError(unicode(translate('OpenSCAD','Running OpenSCAD failed'))+u'\n')

class AddOpenSCADElement:
    def Activated(self):
        panel = AddSCADTask()
        FreeCADGui.Control.showDialog(panel)
    def GetResources(self):
        return {'Pixmap'  : 'OpenSCAD_AddOpenSCADElement', 'MenuText': \
                QtCore.QT_TRANSLATE_NOOP('OpenSCAD_AddOpenSCADElement',\
                'Add OpenSCAD Element...'), 'ToolTip': \
                QtCore.QT_TRANSLATE_NOOP('OpenSCAD_AddOpenSCADElement',\
                'Add an OpenSCAD element by entering OpenSCAD code and executing the OpenSCAD binary')}


FreeCADGui.addCommand('ColorCodeShape',ColorCodeShape())
FreeCADGui.addCommand('Edgestofaces',Edgestofaces())
FreeCADGui.addCommand('RefineShapeFeature',RefineShapeFeature())
FreeCADGui.addCommand('ExpandPlacements',ExpandPlacements())
FreeCADGui.addCommand('ReplaceObject',ReplaceObject())
FreeCADGui.addCommand('RemoveSubtree',RemoveSubtree())
FreeCADGui.addCommand('AddOpenSCADElement',AddOpenSCADElement())
