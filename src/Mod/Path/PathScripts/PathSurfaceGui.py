# -*- coding: utf-8 -*-

# ***************************************************************************
# *                                                                         *
# *   Copyright (c) 2017 sliptonic <shopinthewoods@gmail.com>               *
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

__title__ = "Path Surface Operation UI"
__author__ = "sliptonic (Brad Collette)"
__url__ = "http://www.freecadweb.org"
__doc__ = "Surface operation page controller and command implementation."

import FreeCAD
import FreeCADGui
import PathScripts.PathSurface as PathSurface
import PathScripts.PathGui as PathGui
import PathScripts.PathOpGui as PathOpGui

from PySide import QtCore


def debugMsg(msg):
    DEBUG = False
    if DEBUG:
        FreeCAD.Console.PrintMessage(__name__ + ':: ' + msg + '\n')


class TaskPanelOpPage(PathOpGui.TaskPanelPage):
    '''Page controller class for the Surface operation.'''

    def initPage(self, obj):
        self.setTitle("3D Surface - " + obj.Label)

    def getForm(self):
        '''getForm() ... returns UI'''
        return FreeCADGui.PySideUic.loadUi(":/panels/PageOpSurfaceEdit.ui")

    def getFields(self, obj):
        '''getFields(obj) ... transfers values from UI to obj's proprties'''
        self.updateToolController(obj, self.form.toolController)
        self.updateCoolant(obj, self.form.coolantController)

        if obj.BoundBox != str(self.form.boundBoxSelect.currentText()):
            obj.BoundBox = str(self.form.boundBoxSelect.currentText())

        if obj.ScanType != str(self.form.scanType.currentText()):
            obj.ScanType = str(self.form.scanType.currentText())

        if obj.LayerMode != str(self.form.layerMode.currentText()):
            obj.LayerMode = str(self.form.layerMode.currentText())

        """
        The following method of getting values from the UI form
            allows for translations of combobox options in the UI.
        The requirement is that the enumeration lists must
            be in the same order in both the opPropertyEnumerations() method
            and the UI panel QComboBox list.
        Another step to ensure sychronization of the two lists is to
            populate the list dynamically in this Gui module in `initPage()`
            using the property enumerations list when loading the UI panel.
            This type of dynamic combobox population is done for the
            Tool Controller selection.
        """
        val = self.propEnums['CutPattern'][self.form.cutPattern.currentIndex()]
        if obj.CutPattern != val:
            obj.CutPattern = val

        val = self.propEnums['ProfileEdges'][self.form.profileEdges.currentIndex()]
        if obj.ProfileEdges != val:
            obj.ProfileEdges = val

        if obj.AvoidLastX_Faces != self.form.avoidLastX_Faces.value():
            obj.AvoidLastX_Faces = self.form.avoidLastX_Faces.value()

        obj.DropCutterExtraOffset.x = FreeCAD.Units.Quantity(self.form.boundBoxExtraOffsetX.text()).Value
        obj.DropCutterExtraOffset.y = FreeCAD.Units.Quantity(self.form.boundBoxExtraOffsetY.text()).Value

        if obj.DropCutterDir != str(self.form.dropCutterDirSelect.currentText()):
            obj.DropCutterDir = str(self.form.dropCutterDirSelect.currentText())

        PathGui.updateInputField(obj, 'DepthOffset', self.form.depthOffset)

        if obj.StepOver != self.form.stepOver.value():
            obj.StepOver = self.form.stepOver.value()

        PathGui.updateInputField(obj, 'SampleInterval', self.form.sampleInterval)

        if obj.UseStartPoint != self.form.useStartPoint.isChecked():
            obj.UseStartPoint = self.form.useStartPoint.isChecked()

        if obj.OptimizeLinearPaths != self.form.optimizeEnabled.isChecked():
            obj.OptimizeLinearPaths = self.form.optimizeEnabled.isChecked()

        if obj.OptimizeStepOverTransitions != self.form.optimizeStepOverTransitions.isChecked():
            obj.OptimizeStepOverTransitions = self.form.optimizeStepOverTransitions.isChecked()

    def setFields(self, obj):
        '''setFields(obj) ... transfers obj's property values to UI'''
        debugMsg('setFields()')
        self.sync_combobox_with_enumerations()  # Also updates self.propEnums

        self.setupToolController(obj, self.form.toolController)
        self.setupCoolant(obj, self.form.coolantController)
        self.selectInComboBox(obj.BoundBox, self.form.boundBoxSelect)
        self.selectInComboBox(obj.ScanType, self.form.scanType)
        self.selectInComboBox(obj.LayerMode, self.form.layerMode)

        """
        The following method of setting values in the UI form
            allows for translations of combobox options in the UI.
        The requirement is that the enumeration lists must
            be in the same order in both the operation's enumeration list
            and the UI panel QComboBox list.
        The original method is commented out below.
        """
        idx = self.propEnums['CutPattern'].index(obj.CutPattern)
        self.form.cutPattern.setCurrentIndex(idx)
        idx = self.propEnums['ProfileEdges'].index(obj.ProfileEdges)
        self.form.profileEdges.setCurrentIndex(idx)
        # self.selectInComboBox(obj.CutPattern, self.form.cutPattern)
        # self.selectInComboBox(obj.ProfileEdges, self.form.profileEdges)

        self.form.avoidLastX_Faces.setValue(obj.AvoidLastX_Faces)
        self.form.boundBoxExtraOffsetX.setText(FreeCAD.Units.Quantity(obj.DropCutterExtraOffset.x, FreeCAD.Units.Length).UserString)
        self.form.boundBoxExtraOffsetY.setText(FreeCAD.Units.Quantity(obj.DropCutterExtraOffset.y, FreeCAD.Units.Length).UserString)
        self.selectInComboBox(obj.DropCutterDir, self.form.dropCutterDirSelect)
        self.form.depthOffset.setText(FreeCAD.Units.Quantity(obj.DepthOffset.Value, FreeCAD.Units.Length).UserString)
        self.form.stepOver.setValue(obj.StepOver)
        self.form.sampleInterval.setText(FreeCAD.Units.Quantity(obj.SampleInterval.Value, FreeCAD.Units.Length).UserString)

        if obj.UseStartPoint:
            self.form.useStartPoint.setCheckState(QtCore.Qt.Checked)
        else:
            self.form.useStartPoint.setCheckState(QtCore.Qt.Unchecked)

        if obj.OptimizeLinearPaths:
            self.form.optimizeEnabled.setCheckState(QtCore.Qt.Checked)
        else:
            self.form.optimizeEnabled.setCheckState(QtCore.Qt.Unchecked)

        if obj.OptimizeStepOverTransitions:
            self.form.optimizeStepOverTransitions.setCheckState(QtCore.Qt.Checked)
        else:
            self.form.optimizeStepOverTransitions.setCheckState(QtCore.Qt.Unchecked)

        self.updateVisibility()

    def getSignalsForUpdate(self, obj):
        '''getSignalsForUpdate(obj) ... return list of signals for updating obj'''
        signals = []
        signals.append(self.form.toolController.currentIndexChanged)
        signals.append(self.form.coolantController.currentIndexChanged)
        signals.append(self.form.boundBoxSelect.currentIndexChanged)
        signals.append(self.form.scanType.currentIndexChanged)
        signals.append(self.form.layerMode.currentIndexChanged)
        signals.append(self.form.cutPattern.currentIndexChanged)
        signals.append(self.form.profileEdges.currentIndexChanged)
        signals.append(self.form.avoidLastX_Faces.editingFinished)
        signals.append(self.form.boundBoxExtraOffsetX.editingFinished)
        signals.append(self.form.boundBoxExtraOffsetY.editingFinished)
        signals.append(self.form.dropCutterDirSelect.currentIndexChanged)
        signals.append(self.form.depthOffset.editingFinished)
        signals.append(self.form.stepOver.editingFinished)
        signals.append(self.form.sampleInterval.editingFinished)
        signals.append(self.form.useStartPoint.stateChanged)
        signals.append(self.form.optimizeEnabled.stateChanged)
        signals.append(self.form.optimizeStepOverTransitions.stateChanged)

        return signals

    def on_Base_Geometry_change(self):
        '''on_Base_Geometry_change()...
        Called with a change made in Base Geometry.
        '''
        debugMsg('on_Base_Geometry_change()')
        self.sync_combobox_with_enumerations()  # located in gui_features module
        debugMsg(' -call updateVisibility()')
        self.updateVisibility()

    def setObjectMaps(self):
        # visibilityMap is for editor modes
        self.visibilityMap = {
            'CutPattern': 'cutPattern',
            'OptimizeStepOverTransitions': 'optimizeStepOverTransitions',
            'ProfileEdges': 'profileEdges',
            'AvoidLastX_Faces': 'avoidLastX_Faces',
            'DropCutterDir': 'dropCutterDirSelect'
        }
        # enumerationMap is for combo boxes
        self.enumerationMap = {}

    def custom_editor_mode_actions(self, modes_dict):
        '''custom_editor_mode_actions(modes_dict) ...
        Custom modifications to editor modes and related UI panel elements,
        and custom actions based on updated editor modes.
        The visibility of UI `customPoints` frame is dependent
        upon use of Base Geometry: `Reference1` and `Reference2`.
        '''
        if modes_dict['DropCutterExtraOffset'] == 2:
            self.form.boundBoxExtraOffsetX.hide()
            self.form.boundBoxExtraOffsetY.hide()
            self.form.boundBoxExtraOffset_label.hide()
        else:
            self.form.boundBoxExtraOffsetX.show()
            self.form.boundBoxExtraOffsetY.show()
            self.form.boundBoxExtraOffset_label.show()

    def updateVisibility(self):
        '''updateVisibility()... Updates visibility of Tasks panel objects.'''
        # debugMsg('updateVisibility()')
        self.apply_prop_editor_modes()  # located in gui_features module

    def registerSignalHandlers(self, obj):
        self.form.scanType.currentIndexChanged.connect(self.updateVisibility)


Command = PathOpGui.SetupOperation('Surface',
        PathSurface.Create,
        TaskPanelOpPage,
        'Path-3DSurface',
        QtCore.QT_TRANSLATE_NOOP("Surface", "3D Surface"),
        QtCore.QT_TRANSLATE_NOOP("Surface", "Create a 3D Surface Operation from a model"),
        PathSurface.SetupProperties)

FreeCAD.Console.PrintLog("Loading PathSurfaceGui... done\n")
