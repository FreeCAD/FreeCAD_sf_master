/***************************************************************************
 *   Copyright (c) 2008 Jürgen Riegel (juergen.riegel@web.de)              *
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
# include <Python.h>
# include <Standard_math.hxx>
#endif

#include <Base/Console.h>
#include <Base/Interpreter.h>
#include <Gui/Application.h>
#include <Gui/WidgetFactory.h>
#include <Gui/Language/Translator.h>
#include "PropertyFemMeshItem.h"
#include "DlgSettingsFemImp.h"
#include "ViewProviderFemMesh.h"
#include "ViewProviderFemMeshShape.h"
#include "ViewProviderFemMeshShapeNetgen.h"
#include "ViewProviderAnalysis.h"
#include "ViewProviderSolver.h"
#include "ViewProviderSetNodes.h"
#include "ViewProviderSetElements.h"
#include "ViewProviderSetFaces.h"
#include "ViewProviderSetGeometry.h"
#include "ViewProviderFemConstraint.h"
#include "ViewProviderFemConstraintBearing.h"
#include "ViewProviderFemConstraintFixed.h"
#include "ViewProviderFemConstraintForce.h"
#include "ViewProviderFemConstraintPressure.h"
#include "ViewProviderFemConstraintGear.h"
#include "ViewProviderFemConstraintPulley.h"
#include "ViewProviderFemConstraintDisplacement.h"
#include "ViewProviderFemConstraintContact.h"
#include "ViewProviderResult.h"
#include "Workbench.h"

#ifdef FC_USE_VTK
#include "ViewProviderFemPostObject.h"
#include "ViewProviderFemPostPipeline.h"
#include "ViewProviderFemPostFunction.h"
#include "ViewProviderFemPostFilter.h"
#endif

#ifdef FC_USE_VTK
#include "ViewProviderFemPostObject.h"
#endif


// use a different name to CreateCommand()
void CreateFemCommands(void);

void loadFemResource()
{
    // add resources and reloads the translators
    Q_INIT_RESOURCE(Fem);
    Gui::Translator::instance()->refresh();
}

namespace FemGui {
extern PyObject* initModule();
}


/* Python entry */
PyMODINIT_FUNC initFemGui()
{
    if (!Gui::Application::Instance) {
        PyErr_SetString(PyExc_ImportError, "Cannot load Gui module in console application.");
        return;
    }

    (void) FemGui::initModule();
    Base::Console().Log("Loading GUI of Fem module... done\n");

    // instantiating the commands
    CreateFemCommands();

    // addition objects
    FemGui::Workbench                             ::init();
    FemGui::ViewProviderFemAnalysis               ::init();
    FemGui::ViewProviderFemAnalysisPython         ::init();
    FemGui::ViewProviderFemMesh                   ::init();
    FemGui::ViewProviderFemMeshShape              ::init();
    FemGui::ViewProviderFemMeshShapeNetgen        ::init();
    FemGui::ViewProviderSolver                    ::init();
    FemGui::ViewProviderSolverPython              ::init();
    FemGui::ViewProviderSetNodes                  ::init();
    FemGui::ViewProviderSetElements               ::init();
    FemGui::ViewProviderSetFaces                  ::init();
    FemGui::ViewProviderSetGeometry               ::init();
    FemGui::ViewProviderFemConstraint             ::init();
    FemGui::ViewProviderFemConstraintBearing      ::init();
    FemGui::ViewProviderFemConstraintFixed        ::init();
    FemGui::ViewProviderFemConstraintForce        ::init();
    FemGui::ViewProviderFemConstraintPressure     ::init();
    FemGui::ViewProviderFemConstraintGear         ::init();
    FemGui::ViewProviderFemConstraintPulley       ::init();
    FemGui::ViewProviderFemConstraintDisplacement ::init();
    FemGui::ViewProviderFemConstraintContact       ::init();
    FemGui::ViewProviderResult                    ::init();
    FemGui::ViewProviderResultPython              ::init();
    FemGui::PropertyFemMeshItem                   ::init();
    
#ifdef FC_USE_VTK
    FemGui::ViewProviderFemPostObject          ::init();
    FemGui::ViewProviderFemPostPipeline        ::init();
    FemGui::ViewProviderFemPostFunction        ::init();
    FemGui::ViewProviderFemPostFunctionProvider::init();
    FemGui::ViewProviderFemPostPlaneFunction   ::init();
    FemGui::ViewProviderFemPostSphereFunction  ::init();
    FemGui::ViewProviderFemPostClip            ::init();
    FemGui::ViewProviderFemPostScalarClip      ::init();
    FemGui::ViewProviderFemPostWarpVector      ::init();
    FemGui::ViewProviderFemPostCut             ::init();
#endif


    // register preferences pages
    new Gui::PrefPageProducer<FemGui::DlgSettingsFemImp> ("FEM");

     // add resources and reloads the translators
    loadFemResource();
}
