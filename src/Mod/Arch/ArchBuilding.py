# -*- coding: utf8 -*-

#***************************************************************************
#*                                                                         *
#*   Copyright (c) 2011                                                    *
#*   Yorik van Havre <yorik@uncreated.net>                                 *
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

import FreeCAD,Draft,ArchCommands,ArchFloor
if FreeCAD.GuiUp:
    import FreeCADGui
    from PySide import QtCore, QtGui
    from DraftTools import translate
else:
    def translate(ctxt,txt):
        return txt

__title__="FreeCAD Building"
__author__ = "Yorik van Havre"
__url__ = "http://www.freecadweb.org"

BuildingTypes = ['Undefined',
'Agricultural - Barn',
'Agricultural - Chicken coop or chickenhouse',
'Agricultural - Cow-shed',
'Agricultural - Farmhouse',
'Agricultural - Granary',
'Agricultural - Greenhouse',
'Agricultural - Hayloft',
'Agricultural - Pigpen or sty',
'Agricultural - Root cellar',
'Agricultural - Shed',
'Agricultural - Silo',
'Agricultural - Stable',
'Agricultural - Storm cellar',
'Agricultural - Well house',
'Agricultural - Underground pit',

'Commercial - Automobile repair shop',
'Commercial - Bank',
'Commercial - Car wash',
'Commercial - Convention center',
'Commercial - Forum',
'Commercial - Gas station',
'Commercial - Hotel',
'Commercial - Market',
'Commercial - Market house',
'Commercial - Skyscraper',
'Commercial - Shop',
'Commercial - Shopping mall',
'Commercial - Supermarket',
'Commercial - Warehouse',
'Commercial - Restaurant',

'Residential - Apartment block',
'Residential - Asylum',
'Residential - Condominium',
'Residential - Dormitory',
'Residential - Duplex',
'Residential - House',
'Residential - Nursing home',
'Residential - Townhouse',
'Residential - Villa',
'Residential - Bungalow',

'Educational - Archive',
'Educational - College classroom building',
'Educational - College gymnasium',
'Educational - College students union',
'Educational - School',
'Educational - Library',
'Educational - Museum',
'Educational - Art gallery',
'Educational - Theater',
'Educational - Amphitheater',
'Educational - Concert hall',
'Educational - Cinema',
'Educational - Opera house',
'Educational - Boarding school',

'Government - Capitol',
'Government - City hall',
'Government - Consulate',
'Government - Courthouse',
'Government - Embassy',
'Government - Fire station',
'Government - Meeting house',
'Government - Moot hall',
'Government - Palace',
'Government - Parliament',
'Government - Police station',
'Government - Post office',
'Government - Prison',

'Industrial - Brewery',
'Industrial - Factory',
'Industrial - Foundry',
'Industrial - Power plant',
'Industrial - Mill',

'Military - Arsenal',
'Military -Barracks',

'Parking - Boathouse',
'Parking - Garage',
'Parking - Hangar',

'Storage - Silo',
'Storage - Hangar',

'Religious - Church',
'Religious - Basilica',
'Religious - Cathedral',
'Religious - Chapel',
'Religious - Oratory',
'Religious - Martyrium',
'Religious - Mosque',
'Religious - Mihrab',
'Religious - Surau',
'Religious - Imambargah',
'Religious - Monastery',
'Religious - Mithraeum',
'Religious - Fire temple',
'Religious - Shrine',
'Religious - Synagogue',
'Religious - Temple',
'Religious - Pagoda',
'Religious - Gurdwara',
'Religious - Hindu temple',

'Transport - Airport terminal',
'Transport - Bus station',
'Transport - Metro station',
'Transport - Taxi station',
'Transport - Railway station',
'Transport - Signal box',
'Transport - Lighthouse',

'Infrastructure - Data centre',

'Power station - Fossil-fuel power station',
'Power station - Nuclear power plant',
'Power station - Geothermal power',
'Power station - Biomass-fuelled power plant',
'Power station - Waste heat power plant',
'Power station - Renewable energy power station',
'Power station - Atomic energy plant',

'Other - Apartment',
'Other - Clinic',
'Other - Community hall',
'Other - Eatery',
'Other - Folly',
'Other - Food court',
'Other - Hospice',
'Other - Hospital',
'Other - Hut',
'Other - Bathhouse',
'Other - Workshop',
'Other - World trade centre'
]


def makeBuilding(objectslist=None,baseobj=None,name="Building"):
    '''makeBuilding(objectslist): creates a building including the
    objects from the given list.'''
    obj = FreeCAD.ActiveDocument.addObject("App::DocumentObjectGroupPython",name)
    _Building(obj)
    if FreeCAD.GuiUp:
        _ViewProviderBuilding(obj.ViewObject)
    if objectslist:
        obj.Group = objectslist
    obj.Label = translate("Arch",name)
    return obj

class _CommandBuilding:
    "the Arch Building command definition"
    def GetResources(self):
        return {'Pixmap'  : 'Arch_Building',
                'MenuText': QtCore.QT_TRANSLATE_NOOP("Arch_Building","Building"),
                'Accel': "B, U",
                'ToolTip': QtCore.QT_TRANSLATE_NOOP("Arch_Building","Creates a building object including selected objects.")}

    def IsActive(self):
        return not FreeCAD.ActiveDocument is None

    def Activated(self):
        sel = FreeCADGui.Selection.getSelection()
        p = FreeCAD.ParamGet("User parameter:BaseApp/Preferences/Mod/Arch")
        link = p.GetBool("FreeLinking",False)
        buildingobj = []
        warning = False
        for obj in sel :
            if not Draft.getType(obj) in ["Site", "Building"] :
                buildingobj.append(obj)
            else :
                if link == True :
                    buildingobj.append(obj)
                else:
                    warning = True
        if warning :
            message = translate( "Arch" , "You can put anything but Site and Building object in a Building object.\n\
Building object are not allowed to accept Site and Building object.\n\
Site and Building objects will be removed from the selection.\n\
You can change that in the preferences.\n" )
            ArchCommands.printMessage( message )
        if sel and len(buildingobj) == 0:
            message = translate( "Arch" , "There is no valid object in the selection.\n\
Building creation aborted.\n" )
            ArchCommands.printMessage( message )
        else :
            ss = "[ "
            for o in buildingobj:
                ss += "FreeCAD.ActiveDocument." + o.Name + ", "
            ss += "]"
            FreeCAD.ActiveDocument.openTransaction(translate("Arch","Create Building"))
            FreeCADGui.addModule("Arch")
            FreeCADGui.doCommand("Arch.makeBuilding("+ss+")")
            FreeCAD.ActiveDocument.commitTransaction()
            FreeCAD.ActiveDocument.recompute()

class _Building(ArchFloor._Floor):
    "The Building object"
    def __init__(self,obj):
        ArchFloor._Floor.__init__(self,obj)
        obj.addProperty("App::PropertyEnumeration","BuildingType","Arch","The type of this building")
        self.Type = "Building"
        obj.setEditorMode('Height',2)
        obj.BuildingType = BuildingTypes

class _ViewProviderBuilding(ArchFloor._ViewProviderFloor):
    "A View Provider for the Building object"
    def __init__(self,vobj):
        ArchFloor._ViewProviderFloor.__init__(self,vobj)

    def getIcon(self):
        import Arch_rc
        return ":/icons/Arch_Building_Tree.svg"

if FreeCAD.GuiUp:
    FreeCADGui.addCommand('Arch_Building',_CommandBuilding())
