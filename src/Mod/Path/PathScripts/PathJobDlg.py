# -*- coding: utf-8 -*-

# ***************************************************************************
# *                                                                         *
# *   Copyright (c) 2018 sliptonic <shopinthewoods@gmail.com>               *
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

import FreeCAD
import FreeCADGui
import PathScripts.PathJob as PathJob
import PathScripts.PathLog as PathLog
import PathScripts.PathPreferences as PathPreferences
import PathScripts.PathStock as PathStock
import PathScripts.PathUtil as PathUtil
import glob
import os

from PySide import QtCore, QtGui

# Qt tanslation handling
def translate(context, text, disambig=None):
    return QtCore.QCoreApplication.translate(context, text, disambig)

if False:
    PathLog.setLevel(PathLog.Level.DEBUG, PathLog.thisModule())
    PathLog.trackModule(PathLog.thisModule())
else:
    PathLog.setLevel(PathLog.Level.INFO, PathLog.thisModule())

class JobCreate:
    DataObject = QtCore.Qt.ItemDataRole.UserRole

    def __init__(self, parent=None, sel=None):
        self.dialog = FreeCADGui.PySideUic.loadUi(":/panels/DlgJobCreate.ui")
        self.items = []
        self.dialog.templateGroup.hide()
        self.dialog.modelGroup.hide()

    def setupTitle(self, title):
        self.dialog.setWindowTitle(title)

    def setupModel(self, job = None):
        if job:
            sel = [PathUtil.getPublicObject(job.Proxy.baseObject(job, obj)) for obj in job.Model.Group]
            xxx = job.Model.Group + [job.Stock]
        else:
            sel = FreeCADGui.Selection.getSelection()
            xxx = []

        if sel:
            selected = [s.Label for s in sel]
        else:
            selected = []

        index = 0
        for base in sorted(PathJob.ObjectJob.baseCandidates(), key=lambda o: o.Label):
            if not base in xxx and not PathJob.isResourceClone(job, base, None):
                item = QtGui.QListWidgetItem(base.Label)
                item.setData(self.DataObject, base)
                item.setCheckState(QtCore.Qt.CheckState.Checked if base.Label in selected else QtCore.Qt.CheckState.Unchecked)
                if PathUtil.isSolid(base):
                    self.dialog.solidList.addItem(item)
                else:
                    self.dialog.twoDList.addItem(item)
                self.items.append(item)
        self.dialog.modelGroup.show()


    def setupTemplate(self):
        templateFiles = []
        for path in PathPreferences.searchPaths():
            templateFiles.extend(self.templateFilesIn(path))

        template = {}
        for tFile in templateFiles:
            name = os.path.split(os.path.splitext(tFile)[0])[1][4:]
            if name in template:
                basename = name
                i = 0
                while name in template:
                    i = i + 1
                    name = basename + " (%s)" % i
            PathLog.track(name, tFile)
            template[name] = tFile
        selectTemplate = PathPreferences.defaultJobTemplate()
        index = 0
        self.dialog.jobTemplate.addItem('<none>', '')
        for name in sorted(template.keys()):
            if template[name] == selectTemplate:
                index = self.dialog.jobTemplate.count()
            self.dialog.jobTemplate.addItem(name, template[name])
        self.dialog.jobTemplate.setCurrentIndex(index)
        self.dialog.templateGroup.show()

    def templateFilesIn(self, path):
        '''templateFilesIn(path) ... answer all file in the given directory which fit the job template naming convention.
        PathJob template files are name job_*.json'''
        PathLog.track(path)
        return glob.glob(path + '/job_*.json')

    def getModels(self):
        '''answer the base models selected for the job'''
        return [item.data(self.DataObject) for item in self.items if item.checkState() == QtCore.Qt.CheckState.Checked]

    def getTemplate(self):
        '''answer the file name of the template to be assigned'''
        return self.dialog.jobTemplate.itemData(self.dialog.jobTemplate.currentIndex())

    def exec_(self):
        return self.dialog.exec_()


class JobTemplateExport:
    DataObject = QtCore.Qt.ItemDataRole.UserRole

    def __init__(self, job, parent=None):
        self.job = job
        self.dialog = FreeCADGui.PySideUic.loadUi(":/panels/DlgJobTemplateExport.ui")

        if job.PostProcessor:
            ppHint = "%s %s %s" % (job.PostProcessor, job.PostProcessorArgs, job.PostProcessorOutputFile)
            self.dialog.postProcessingHint.setText(ppHint)
        else:
            self.dialog.postProcessingGroup.setEnabled(False)
            self.dialog.postProcessingGroup.setChecked(False)

        if job.Stock and not PathJob.isResourceClone(job, 'Stock', 'Stock'):
            stockType = PathStock.StockType.FromStock(job.Stock)
            if stockType == PathStock.StockType.FromBase:
                seHint = translate('PathJob', "Base -/+ %.2f/%.2f %.2f/%.2f %.2f/%.2f") % (job.Stock.ExtXneg, job.Stock.ExtXpos, job.Stock.ExtYneg, job.Stock.ExtYpos, job.Stock.ExtZneg, job.Stock.ExtZpos)
                self.dialog.stockPlacement.setChecked(False)
            elif stockType == PathStock.StockType.CreateBox:
                seHint = translate('PathJob', "Box: %.2f x %.2f x %.2f") % (job.Stock.Length, job.Stock.Width, job.Stock.Height)
            elif stockType == PathStock.StockType.CreateCylinder:
                seHint = translate('PathJob', "Cylinder: %.2f x %.2f") % (job.Stock.Radius, job.Stock.Height)
            else:
                seHint = '-'
                PathLog.error(translate('PathJob', 'Unsupported stock type'))
            self.dialog.stockExtentHint.setText(seHint)
            spHint = "%s" % job.Stock.Placement
            self.dialog.stockPlacementHint.setText(spHint)

        rapidChanged = not job.SetupSheet.Proxy.hasDefaultToolRapids()
        depthsChanged = not job.SetupSheet.Proxy.hasDefaultOperationDepths()
        heightsChanged = not job.SetupSheet.Proxy.hasDefaultOperationHeights()
        settingsChanged = rapidChanged or depthsChanged or heightsChanged
        self.dialog.settingsGroup.setChecked(settingsChanged)
        self.dialog.settingToolRapid.setChecked(rapidChanged)
        self.dialog.settingOperationDepths.setChecked(depthsChanged)
        self.dialog.settingOperationHeights.setChecked(heightsChanged)

        for tc in sorted(job.ToolController, key=lambda o: o.Label):
            item = QtGui.QListWidgetItem(tc.Label)
            item.setData(self.DataObject, tc)
            item.setCheckState(QtCore.Qt.CheckState.Checked)
            self.dialog.toolsList.addItem(item)

        self.dialog.toolsGroup.clicked.connect(self.checkUncheckTools)

    def checkUncheckTools(self):
        state = QtCore.Qt.CheckState.Checked if self.dialog.toolsGroup.isChecked() else QtCore.Qt.CheckState.Unchecked
        for i in range(self.dialog.toolsList.count()):
            self.dialog.toolsList.item(i).setCheckState(state)

    def includePostProcessing(self):
        return self.dialog.postProcessingGroup.isChecked()

    def includeToolControllers(self):
        tcs = []
        for i in range(self.dialog.toolsList.count()):
            item = self.dialog.toolsList.item(i)
            if item.checkState() == QtCore.Qt.CheckState.Checked:
                tcs.append(item.data(self.DataObject))
        return tcs

    def includeStock(self):
        return self.dialog.stockGroup.isChecked()
    def includeStockExtent(self):
        return self.dialog.stockExtent.isChecked()
    def includeStockPlacement(self):
        return self.dialog.stockPlacement.isChecked()

    def includeSettings(self):
        return self.dialog.settingsGroup.isChecked()
    def includeSettingToolRapid(self):
        return self.dialog.settingToolRapid.isChecked()
    def includeSettingOperationHeights(self):
        return self.dialog.settingOperationHeights.isChecked()
    def includeSettingOperationDepths(self):
        return self.dialog.settingOperationDepths.isChecked()

    def exec_(self):
        return self.dialog.exec_()

