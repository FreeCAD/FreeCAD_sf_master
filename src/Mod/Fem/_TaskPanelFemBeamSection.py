# ***************************************************************************
# *                                                                         *
# *   Copyright (c) 2015 - Bernd Hahnebach <bernd@bimstatik.org>            *
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

__title__ = "_TaskPanelFemBeamSection"
__author__ = "Bernd Hahnebach"
__url__ = "http://www.freecadweb.org"

## @package TaskPanelFemBeamSection
#  \ingroup FEM

import FreeCAD
import FreeCADGui
from PySide import QtGui
from PySide import QtCore


class _TaskPanelFemBeamSection:
    '''The TaskPanel for editing References property of FemBeamSection objects'''
    def __init__(self, obj):
        FreeCADGui.Selection.clearSelection()
        self.sel_server = None
        self.obj = obj
        self.references = []
        if self.obj.References:
            self.tuplereferences = self.obj.References
            self.get_references()

        self.form = FreeCADGui.PySideUic.loadUi(FreeCAD.getHomePath() + "Mod/Fem/TaskPanelFemBeamSection.ui")

        if self.obj.SectionType == 'Rectangular':
            self.form.rb_Rect.setChecked(True)
        elif self.obj.SectionType == 'Circular':
            self.form.rb_Circ.setChecked(True)
        elif self.obj.SectionType == 'Pipe':
            self.form.rb_Pipe.setChecked(True)

        QtCore.QObject.connect(self.form.pushButton_Reference, QtCore.SIGNAL("clicked()"), self.add_references)
        QtCore.QObject.connect(self.form.rb_Rect, QtCore.SIGNAL("clicked()"), self.rect_section)
        QtCore.QObject.connect(self.form.rb_Circ, QtCore.SIGNAL("clicked()"), self.circ_section)
        QtCore.QObject.connect(self.form.rb_Pipe, QtCore.SIGNAL("clicked()"), self.pipe_section)
        self.form.list_References.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
        self.form.list_References.connect(self.form.list_References, QtCore.SIGNAL("customContextMenuRequested(QPoint)"), self.references_list_right_clicked)

        self.rebuild_list_References()

    def accept(self):
        if self.sel_server:
            FreeCADGui.Selection.removeObserver(self.sel_server)
        self.obj.References = self.references
        FreeCADGui.ActiveDocument.resetEdit()
        FreeCAD.ActiveDocument.recompute()
        return True

    def reject(self):
        if self.sel_server:
            FreeCADGui.Selection.removeObserver(self.sel_server)
        FreeCADGui.ActiveDocument.resetEdit()
        return True

    def get_references(self):
        for ref in self.tuplereferences:
            for elem in ref[1]:
                self.references.append((ref[0], elem))

    def references_list_right_clicked(self, QPos):
        self.form.contextMenu = QtGui.QMenu()
        menu_item = self.form.contextMenu.addAction("Remove Reference")
        if not self.references:
            menu_item.setDisabled(True)
        self.form.connect(menu_item, QtCore.SIGNAL("triggered()"), self.remove_reference)
        parentPosition = self.form.list_References.mapToGlobal(QtCore.QPoint(0, 0))
        self.form.contextMenu.move(parentPosition + QPos)
        self.form.contextMenu.show()

    def remove_reference(self):
        if not self.references:
            return
        currentItemName = str(self.form.list_References.currentItem().text())
        for ref in self.references:
            refname_to_compare_listentry = ref[0].Name + ':' + ref[1]
            if refname_to_compare_listentry == currentItemName:
                self.references.remove(ref)
        self.rebuild_list_References()

    def add_references(self):
        '''Called if Button add_reference is triggered'''
        # in constraints EditTaskPanel the selection is active as soon as the taskpanel is open
        # here the addReference button EditTaskPanel has to be triggered to start selection mode
        FreeCADGui.Selection.clearSelection()
        # start SelectionObserver and parse the function to add the References to the widget
        print_message = "Select Edges by single click on them to add them to the list"
        import FemSelectionObserver
        self.sel_server = FemSelectionObserver.FemSelectionObserver(self.selectionParser, print_message)

    def rect_section(self):
        '''Called if Rectangular radio button is triggered'''
        self.obj.SectionType = 'Rectangular'
        self.obj.RectWidth = 20.0
        self.obj.RectHeight = 20.0

    def circ_section(self):
        '''Called if Circular radio button is triggered'''
        self.obj.SectionType = 'Circular'
        self.obj.CircRadius = 20.0

    def pipe_section(self):
        '''Called if Pipe radio button is triggered'''
        self.obj.SectionType = 'Pipe'
        self.obj.PipeRadius = 20.0
        self.obj.PipeThickness = 2.0

    def selectionParser(self, selection):
        # print('selection: ', selection[0].Shape.ShapeType, '  ', selection[0].Name, '  ', selection[1])
        if hasattr(selection[0], "Shape"):
            if selection[1]:
                elt = selection[0].Shape.getElement(selection[1])
                if elt.ShapeType == 'Edge':
                    if selection not in self.references:
                        self.references.append(selection)
                        self.rebuild_list_References()
                    else:
                        FreeCAD.Console.PrintMessage(selection[0].Name + ' --> ' + selection[1] + ' is in reference list already!\n')

    def rebuild_list_References(self):
        self.form.list_References.clear()
        items = []
        for ref in self.references:
            item_name = ref[0].Name + ':' + ref[1]
            items.append(item_name)
        for listItemName in sorted(items):
            self.form.list_References.addItem(listItemName)
