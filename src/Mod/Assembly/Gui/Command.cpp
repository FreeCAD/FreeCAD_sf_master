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
#endif

#include <Gui/Application.h>
#include <Gui/Command.h>
#include <Gui/MainWindow.h>
#include <Gui/FileDialog.h>


using namespace std;



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
    sPixmap         = "actions/Axle_constraint";
}


void CmdAssemblyAddNewPart::activated(int iMsg)
{
    // load the file with the module
    //Command::doCommand(Command::Gui, "import Assembly, AssemblyGui");
      
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
    sPixmap         = "actions/Axle_constraint";
}


void CmdAssemblyAddNewComponent::activated(int iMsg)
{
    // load the file with the module
    //Command::doCommand(Command::Gui, "import Assembly, AssemblyGui");
      
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
    sPixmap         = "actions/Axle_constraint";
}


void CmdAssemblyAddExistingComponent::activated(int iMsg)
{
    // load the file with the module
    //Command::doCommand(Command::Gui, "import Assembly, AssemblyGui");
      
}

void CreateAssemblyCommands(void)
{
    Gui::CommandManager &rcCmdMgr = Gui::Application::Instance->commandManager();

    rcCmdMgr.addCommand(new CmdAssemblyAddNewPart());
    rcCmdMgr.addCommand(new CmdAssemblyAddNewComponent());
    rcCmdMgr.addCommand(new CmdAssemblyAddExistingComponent());
 }
