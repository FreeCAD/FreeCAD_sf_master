/***************************************************************************
 *   Copyright (c) 2013 J¨¹rgen Riegel (FreeCAD@juergen-riegel.net)         *
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

#include "FemSolverObject.h"
#include <App/DocumentObjectPy.h>

using namespace Fem;
using namespace App;

PROPERTY_SOURCE(Fem::FemSolverObject, App::DocumentObject)


FemSolverObject::FemSolverObject()
{
    ADD_PROPERTY_TYPE(SolverName,("Calculix"), "Data",Prop_None,"Solver program name");
    ADD_PROPERTY_TYPE(AnalysisType,("Static"), "Data",Prop_None,"Solver program name");
    ADD_PROPERTY_TYPE(WorkingDir,("./"), "Data",Prop_None,"Solver working director");
    ADD_PROPERTY_TYPE(ExternalCaseEditor,(""), "Data",Prop_None,"Solver program name");
    ADD_PROPERTY_TYPE(ExternalResultViewer,(""), "Data",Prop_None,"Solver program name");

}

FemSolverObject::~FemSolverObject()
{
}

short FemSolverObject::mustExecute(void) const
{
    return 0;
}

PyObject *FemSolverObject::getPyObject()
{
    if (PythonObject.is(Py::_None())){
        // ref counter is set to 1
        PythonObject = Py::Object(new DocumentObjectPy(this),true);
    }
    return Py::new_reference_to(PythonObject); 
}

// Python feature ---------------------------------------------------------

namespace App {
/// @cond DOXERR
PROPERTY_SOURCE_TEMPLATE(Fem::FemSolverObjectPython, Fem::FemSolverObject)
template<> const char* Fem::FemSolverObjectPython::getViewProviderName(void) const {
    return "FemGui::ViewProviderSolverPython";
}
/// @endcond

// explicit template instantiation
template class AppFemExport FeaturePythonT<Fem::FemSolverObject>;

}