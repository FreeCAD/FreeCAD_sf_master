# -*- coding: utf-8 -*-

# ***************************************************************************
# *                                                                         *
# *   Copyright (c) 2014 Yorik van Havre <yorik@uncreated.net>              *
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

import ArchPanel
import Draft
import FreeCAD
import PathScripts.PathLog as PathLog
import PathScripts.PathToolController as PathToolController
import PathScripts.PathUtil as PathUtil
import xml.etree.ElementTree as xml

from PathScripts.PathPreferences import PathPreferences
from PathScripts.PathPostProcessor import PostProcessor
from PySide import QtCore

if True:
    PathLog.setLevel(PathLog.Level.DEBUG, PathLog.thisModule())
    PathLog.trackModule(PathLog.thisModule())
else:
    PathLog.setLevel(PathLog.Level.INFO, PathLog.thisModule())

"""Path Job object and FreeCAD command"""

# Qt tanslation handling
def translate(context, text, disambig=None):
    return QtCore.QCoreApplication.translate(context, text, disambig)

class JobTemplate:
    '''Attribute and sub element strings for template export/import.'''
    Job = 'Job'
    PostProcessor = 'post'
    PostProcessorArgs = 'post_args'
    PostProcessorOutputFile = 'output'
    GeometryTolerance = 'tol'
    Description = 'desc'
    ToolController = 'ToolController'

class ObjectJob:

    def __init__(self, obj, base, template = None):
        self.obj = obj
        obj.addProperty("App::PropertyFile", "PostProcessorOutputFile", "Output", QtCore.QT_TRANSLATE_NOOP("App::Property","The NC output file for this project"))
        obj.addProperty("App::PropertyEnumeration", "PostProcessor", "Output", QtCore.QT_TRANSLATE_NOOP("App::Property","Select the Post Processor"))
        obj.addProperty("App::PropertyString", "PostProcessorArgs", "Output", QtCore.QT_TRANSLATE_NOOP("App::Property", "Arguments for the Post Processor (specific to the script)"))

        obj.addProperty("App::PropertyString", "Description", "Path", QtCore.QT_TRANSLATE_NOOP("App::Property","An optional description for this job"))
        obj.addProperty("App::PropertyDistance", "GeometryTolerance", "Geometry", QtCore.QT_TRANSLATE_NOOP("App::Property", "For computing Paths; smaller increases accuracy, but slows down computation"))

        obj.addProperty("App::PropertyLink", "Base", "Base", QtCore.QT_TRANSLATE_NOOP("PathJob", "The base object for all operations"))
        obj.addProperty("App::PropertyLink", "Stock", "Base", QtCore.QT_TRANSLATE_NOOP("PathJob", "Solid object to be used as stock."))
        obj.addProperty("App::PropertyLink", "Operations", "Base", QtCore.QT_TRANSLATE_NOOP("PathJob", "Compound path of all operations in the order they are processed."))
        obj.addProperty("App::PropertyLinkList", "ToolController", "Base", QtCore.QT_TRANSLATE_NOOP("PathJob", "Collection of tool controllers available for this job."))

        obj.PostProcessorOutputFile = PathPreferences.defaultOutputFile()
        #obj.setEditorMode("PostProcessorOutputFile", 0)  # set to default mode
        obj.PostProcessor = postProcessors = PathPreferences.allEnabledPostProcessors()
        defaultPostProcessor = PathPreferences.defaultPostProcessor()
        # Check to see if default post processor hasn't been 'lost' (This can happen when Macro dir has changed)
        if defaultPostProcessor in postProcessors:
            obj.PostProcessor = defaultPostProcessor
        else:
            obj.PostProcessor = postProcessors[0]
        obj.PostProcessorArgs = PathPreferences.defaultPostProcessorArgs()
        obj.GeometryTolerance = PathPreferences.defaultGeometryTolerance()

        ops = FreeCAD.ActiveDocument.addObject("Path::FeatureCompoundPython", "Operations")
        obj.Operations = ops
        obj.setEditorMode('Operations', 2) # hide
        obj.setEditorMode('Placement', 2)

        obj.Base = base
        obj.Proxy = self

        self.assignTemplate(obj, template)

    def onDelete(self, obj, arg2=None):
        PathLog.track(obj, arg2)
        for tc in obj.ToolController:
            FreeCAD.ActiveDocument.removeObject(tc.Name)
        obj.ToolController = []
        for op in obj.Operations.Group:
            FreeCAD.ActiveDocument.removeObject(op.Name)
        obj.Operations.Group = []
        FreeCAD.ActiveDocument.removeObject(obj.Operations.Name)
        obj.Operations = None

    def isResourceClone(self, obj, propName, resourceName):
        if hasattr(obj, propName):
            prop =  getattr(obj, propName)
            if hasattr(prop, 'PathResource') and resourceName == getattr(prop, 'PathResource'):
                return True
        return False

    def fixupResourceClone(self, obj, name):
        if not self.isResourceClone(obj, name, name):
            orig = getattr(obj, name)
            if orig:
                clone = Draft.clone(orig)
                clone.Label = name
                clone.addProperty('App::PropertyString', 'PathResource')
                clone.PathResource = name
                setattr(obj, name, clone)

    def onBeforeChange(self, obj, prop):
        PathLog.track(obj.Label, prop)
        if prop == 'Base' and obj.Base and hasattr(obj.Base, 'PathResource'):
            PathLog.info('removing old base clone')
            obj.Base.removeProperty('PathResource')
            obj.Document.removeObject(obj.Base.Name)

    def onChanged(self, obj, prop):
        PathLog.track(obj.Label, prop)
        if prop == "PostProcessor" and obj.PostProcessor:
            processor = PostProcessor.load(obj.PostProcessor)
            self.tooltip = processor.tooltip
            self.tooltipArgs = processor.tooltipArgs

        if prop == 'Base':
            self.fixupResourceClone(obj, 'Base')
        if prop == 'Stock':
            self.fixupResourceClone(obj, 'Stock')

    def onDocumentRestored(self, obj):
        self.fixupResourceClone(obj, 'Base')
        self.fixupResourceClone(obj, 'Stock')

    def assignTemplate(self, obj, template):
        '''assignTemplate(obj, template) ... extract the properties from the given template file and assign to receiver.
        This will also create any TCs stored in the template.'''
        tcs = []
        if template:
            tree = xml.parse(template)
            for job in tree.getroot().iter(JobTemplate.Job):
                if job.get(JobTemplate.GeometryTolerance):
                    obj.GeometryTolerance = float(job.get(JobTemplate.GeometryTolerance))
                if job.get(JobTemplate.PostProcessor):
                    obj.PostProcessor = job.get(JobTemplate.PostProcessor)
                    if job.get(JobTemplate.PostProcessorArgs):
                        obj.PostProcessorArgs = job.get(JobTemplate.PostProcessorArgs)
                    else:
                        obj.PostProcessorArgs = ''
                if job.get(JobTemplate.PostProcessorOutputFile):
                    obj.PostProcessorOutputFile = job.get(JobTemplate.PostProcessorOutputFile)
                if job.get(JobTemplate.Description):
                    obj.Description = job.get(JobTemplate.Description)
            for tc in tree.getroot().iter(JobTemplate.ToolController):
                tcs.append(PathToolController.FromTemplate(tc))
        else:
            tcs.append(PathToolController.Create(obj.Name))
        PathLog.debug("setting tool controllers (%d)" % len(tcs))
        obj.ToolController = tcs

    def templateAttrs(self, obj):
        '''templateAttrs(obj) ... answer a dictionary with all properties of the receiver that should be stored in a template file.'''
        attrs = {}
        if obj.PostProcessor:
            attrs[JobTemplate.PostProcessor]           = obj.PostProcessor
            attrs[JobTemplate.PostProcessorArgs]       = obj.PostProcessorArgs
        if obj.PostProcessorOutputFile:
            attrs[JobTemplate.PostProcessorOutputFile] = obj.PostProcessorOutputFile
        attrs[JobTemplate.GeometryTolerance]           = str(obj.GeometryTolerance.Value)
        if obj.Description:
            attrs[JobTemplate.Description]             = obj.Description
        return attrs

    def __getstate__(self):
        return None

    def __setstate__(self, state):
        for obj in FreeCAD.ActiveDocument.Objects:
            if hasattr(obj, 'Proxy') and obj.Proxy == self:
                self.obj = obj
                break
        return None

    def execute(self, obj):
        obj.Path = obj.Operations.Path

    def addOperation(self, op):
        group = self.obj.Operations.Group
        if op not in group:
            group.append(op)
            self.obj.Operations.Group = group

    def addToolController(self, tc):
        group = self.obj.ToolController
        PathLog.info("addToolController(%s): %s" % (tc.Label, [t.Label for t in group]))
        if tc.Name not in [str(t.Name) for t in group]:
            group.append(tc)
            self.obj.ToolController = group


    @classmethod
    def baseCandidates(cls):
        '''Answer all objects in the current document which could serve as a Base for a job.'''
        return sorted(filter(lambda obj: cls.isBaseCandidate(obj) , FreeCAD.ActiveDocument.Objects), key=lambda o: o.Label)

    @classmethod
    def isBaseCandidate(cls, obj):
        '''Answer true if the given object can be used as a Base for a job.'''
        return PathUtil.isValidBaseObject(obj) or (hasattr(obj, 'Proxy') and isinstance(obj.Proxy, ArchPanel.PanelSheet))

def Create(name, base, template = None):
    obj = FreeCAD.ActiveDocument.addObject("Path::FeaturePython", name)
    proxy = ObjectJob(obj, base, template)
    return obj

