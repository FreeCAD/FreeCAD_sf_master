/***************************************************************************
 *   Copyright (c) 2008 J�rgen Riegel (juergen.riegel@web.de)              *
 *                                                                         *
 *   This file is part of the FreeCAD CAx development system.              *
 *                                                                         *
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Library General Public           *
 *   License as published by the Free Software Foundation; either          *
 *   version 2 of the License, or (at your option) any later version.      *
 *                                                                         *
 *   This library  is distributed in the hope that it will be useful,      *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Library General Public License for more details.                  *
 *                                                                         *
 *   You should have received a copy of the GNU Library General Public     *
 *   License along with this library; see the file COPYING.LIB. If not,    *
 *   write to the Free Software Foundation, Inc., 59 Temple Place,         *
 *   Suite 330, Boston, MA  02111-1307, USA                                *
 *                                                                         *
 ***************************************************************************/


#include "PreCompiled.h"
#ifndef _PreComp_
# include <QMessageBox>
#endif

#include <Gui/Application.h>
#include <Gui/Command.h>
#include <Gui/MainWindow.h>
#include <Gui/FileDialog.h>
#include <Gui/Selection.h>

#include <Mod/Assembly/App/ItemAssembly.h>


using namespace std;

extern Assembly::Item *ActiveAsmObject;


//===========================================================================

DEF_STD_CMD(CmdAssemblyAddNewPart);

CmdAssemblyAddNewPart::CmdAssemblyAddNewPart()
	:Command("Assembly_AddNewPart")
{
    sAppModule      = "Assembly";
    sGroup          = QT_TR_NOOP("Assembly");
    sMenuText       = QT_TR_NOOP("Add new Part");
    sToolTipText    = QT_TR_NOOP("Add a new Part into the active Assembly");
    sWhatsThis      = sToolTipText;
    sStatusTip      = sToolTipText;
    sPixmap         = "Part_Box";
}


void CmdAssemblyAddNewPart::activated(int iMsg)
{
    Assembly::ItemAssembly *dest = 0;

    unsigned int n = getSelection().countObjectsOfType(Assembly::ItemAssembly::getClassTypeId());
    if (n >= 1) {
        std::vector<App::DocumentObject*> Sel = getSelection().getObjectsOfType(Assembly::ItemAssembly::getClassTypeId());
        dest = dynamic_cast<Assembly::ItemAssembly*>(Sel.front());
    }else if(ActiveAsmObject && ActiveAsmObject->getTypeId().isDerivedFrom(Assembly::ItemAssembly::getClassTypeId())) {
        dest = dynamic_cast<Assembly::ItemAssembly*>(ActiveAsmObject);
    }else {

        return;
    }

    openCommand("Insert Part");
    std::string PartName = getUniqueObjectName("Part");
    doCommand(Doc,"App.activeDocument().addObject('Assembly::ItemPart','%s')",PartName.c_str());
    if(dest){
        std::string fatherName = dest->getNameInDocument();
        doCommand(Doc,"App.activeDocument().%s.Items = App.activeDocument().%s.Items + [App.activeDocument().%s] ",fatherName.c_str(),fatherName.c_str(),PartName.c_str());
    }
    Command::addModule(App,"PartDesign");
    Command::addModule(Gui,"PartDesignGui");

#if 1  // test code for children nesting 
    Command::addModule(App,"Part");
    std::string BodyName = getUniqueObjectName("Box");
    doCommand(Doc,"App.activeDocument().addObject('Part::Box','%s')",BodyName.c_str());
    doCommand(Doc,"App.activeDocument().%s.Model = App.activeDocument().%s ",PartName.c_str(),BodyName.c_str(),BodyName.c_str());
#else

    std::string BodyName = getUniqueObjectName("Body");
    // add the standard planes 
    std::string Plane1Name = BodyName + "_PlaneXY";
    std::string Plane2Name = BodyName + "_PlaneYZ";
    std::string Plane3Name = BodyName + "_PlaneXZ";
    doCommand(Doc,"App.activeDocument().addObject('App::Plane','%s')",Plane1Name.c_str());
    doCommand(Doc,"App.activeDocument().addObject('App::Plane','%s')",Plane2Name.c_str());
    doCommand(Doc,"App.activeDocument().%s.Placement = App.Placement(App.Vector(0.000000,0.000000,0.000000),App.Rotation(-0.707107,0.000000,0.000000,-0.707107))",Plane2Name.c_str());
    doCommand(Doc,"App.activeDocument().addObject('App::Plane','%s')",Plane3Name.c_str());
    doCommand(Doc,"App.activeDocument().%s.Annotation = [App.activeDocument().%s,App.activeDocument().%s,App.activeDocument().%s] ",PartName.c_str(),Plane1Name.c_str(),Plane2Name.c_str(),Plane3Name.c_str());
    // add the main body
    doCommand(Doc,"App.activeDocument().addObject('PartDesign::Body','%s')",BodyName.c_str());
    doCommand(Doc,"App.activeDocument().%s.Model = App.activeDocument().%s ",PartName.c_str(),BodyName.c_str(),BodyName.c_str());
#endif #
    this->updateActive();
}

//===========================================================================

DEF_STD_CMD(CmdAssemblyAddNewComponent);

CmdAssemblyAddNewComponent::CmdAssemblyAddNewComponent()
	:Command("Assembly_AddNewComponent")
{
    sAppModule      = "Assembly";
    sGroup          = QT_TR_NOOP("Assembly");
    sMenuText       = QT_TR_NOOP("Add new Component");
    sToolTipText    = QT_TR_NOOP("Add a new Component into the active Assembly");
    sWhatsThis      = sToolTipText;
    sStatusTip      = sToolTipText;
    sPixmap         = "Part_Box";
}


void CmdAssemblyAddNewComponent::activated(int iMsg)
{
    Assembly::ItemAssembly *dest = 0;

    unsigned int n = getSelection().countObjectsOfType(Assembly::ItemAssembly::getClassTypeId());
    if (n >= 1) {
        std::vector<App::DocumentObject*> Sel = getSelection().getObjectsOfType(Assembly::ItemAssembly::getClassTypeId());
        dest = dynamic_cast<Assembly::ItemAssembly*>(Sel.front());
    }else if(ActiveAsmObject && ActiveAsmObject->getTypeId().isDerivedFrom(Assembly::ItemAssembly::getClassTypeId())) {
        dest = dynamic_cast<Assembly::ItemAssembly*>(ActiveAsmObject);
    }

    openCommand("Insert Component");
    std::string CompName = getUniqueObjectName("Product.0");
    doCommand(Doc,"App.activeDocument().addObject('Assembly::ItemAssembly','%s')",CompName.c_str());
    if(dest){
        std::string fatherName = dest->getNameInDocument();
        doCommand(Doc,"App.activeDocument().%s.Items = App.activeDocument().%s.Items + [App.activeDocument().%s] ",fatherName.c_str(),fatherName.c_str(),CompName.c_str());
    }
}

//===========================================================================

DEF_STD_CMD(CmdAssemblyAddExistingComponent);

CmdAssemblyAddExistingComponent::CmdAssemblyAddExistingComponent()
	:Command("Assembly_AddExistingComponent")
{
    sAppModule      = "Assembly";
    sGroup          = QT_TR_NOOP("Assembly");
    sMenuText       = QT_TR_NOOP("Add existing Component...");
    sToolTipText    = QT_TR_NOOP("Add a existing Component or File into the active Assembly");
    sWhatsThis      = sToolTipText;
    sStatusTip      = sToolTipText;
    sPixmap         = "Part_Box";
}


void CmdAssemblyAddExistingComponent::activated(int iMsg)
{
    Assembly::ItemAssembly *dest = 0;

    unsigned int n = getSelection().countObjectsOfType(Assembly::ItemAssembly::getClassTypeId());
    if (n >= 1) {
        std::vector<App::DocumentObject*> Sel = getSelection().getObjectsOfType(Assembly::ItemAssembly::getClassTypeId());
        dest = dynamic_cast<Assembly::ItemAssembly*>(Sel.front());
    }else if(ActiveAsmObject && ActiveAsmObject->getTypeId().isDerivedFrom(Assembly::ItemAssembly::getClassTypeId())) {
        dest = dynamic_cast<Assembly::ItemAssembly*>(ActiveAsmObject);
    }else {

        return;
    }

    openCommand("Insert TestPart");
    std::string PartName = getUniqueObjectName("Part");
    doCommand(Doc,"App.activeDocument().addObject('Assembly::ItemPart','%s')",PartName.c_str());
    if(dest){
        std::string fatherName = dest->getNameInDocument();
        doCommand(Doc,"App.activeDocument().%s.Items = App.activeDocument().%s.Items + [App.activeDocument().%s] ",fatherName.c_str(),fatherName.c_str(),PartName.c_str());
    }
    Command::addModule(App,"PartDesign");
    Command::addModule(Gui,"PartDesignGui");
    std::string BodyName = getUniqueObjectName("Body");
    doCommand(Doc,"App.activeDocument().addObject('PartDesign::Body','%s')",BodyName.c_str());
    doCommand(Doc,"App.activeDocument().%s.Model = App.activeDocument().%s ",PartName.c_str(),BodyName.c_str(),BodyName.c_str());
      
}

void CreateAssemblyCommands(void)
{
    Gui::CommandManager &rcCmdMgr = Gui::Application::Instance->commandManager();

    rcCmdMgr.addCommand(new CmdAssemblyAddNewPart());
    rcCmdMgr.addCommand(new CmdAssemblyAddNewComponent());
    rcCmdMgr.addCommand(new CmdAssemblyAddExistingComponent());
 }
