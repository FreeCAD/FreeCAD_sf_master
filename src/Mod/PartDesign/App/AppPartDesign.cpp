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
#endif

#include <Base/Console.h>
#include <Base/Interpreter.h>
 
#include "FeaturePad.h"
#include "FeatureSolid.h"
#include "FeaturePocket.h"
#include "FeatureFillet.h"
#include "FeatureSketchBased.h"
#include "FeatureRevolution.h"
#include "FeatureGroove.h"
#include "Body.h"
#include "FeatureDressUp.h"
#include "FeatureChamfer.h"
#include "FeatureDraft.h"
#include "FeatureSubtractive.h"
#include "FeatureAdditive.h"
#include "FeatureTransformed.h"
#include "FeatureMirrored.h"
#include "FeatureLinearPattern.h"
#include "FeaturePolarPattern.h"
#include "FeatureScaled.h"
#include "FeatureMultiTransform.h"
#include "FeatureHole.h"
#include "DatumPlane.h"
#include "DatumLine.h"
#include "DatumPoint.h"
#include "FeatureBoolean.h"

namespace PartDesign {
extern PyObject* initModule();
}

/* Python entry */
PyMODINIT_FUNC init_PartDesign()
{
    // load dependent module
    try {
        Base::Interpreter().runString("import Part");
        Base::Interpreter().runString("import Sketcher");
    }
    catch(const Base::Exception& e) {
        PyErr_SetString(PyExc_ImportError, e.what());
        return;
    }

    (void)PartDesign::initModule();
    Base::Console().Log("Loading PartDesign module... done\n");


    // NOTE: To finish the initialization of our own type objects we must
    // call PyType_Ready, otherwise we run into a segmentation fault, later on.
    // This function is responsible for adding inherited slots from a type's base class.
 
    PartDesign::Feature            ::init();
    PartDesign::Solid              ::init();
    PartDesign::DressUp            ::init();
    PartDesign::SketchBased        ::init();
    PartDesign::Subtractive        ::init();
    PartDesign::Additive           ::init();
    PartDesign::Transformed        ::init();
    PartDesign::Mirrored           ::init();
    PartDesign::LinearPattern      ::init();
    PartDesign::PolarPattern       ::init();
    PartDesign::Scaled             ::init();
    PartDesign::MultiTransform     ::init();
    PartDesign::Hole               ::init();
    PartDesign::Body               ::init();
    PartDesign::Pad                ::init();
    PartDesign::Pocket             ::init();
    PartDesign::Fillet             ::init();
    PartDesign::Revolution         ::init();
    PartDesign::Groove             ::init();
    PartDesign::Chamfer            ::init();
    PartDesign::Draft              ::init();
    PartDesign::Plane              ::init();
    PartDesign::Line               ::init();
    PartDesign::Point              ::init();
    PartDesign::Boolean            ::init();

    PartDesign::Point::initHints();
    PartDesign::Line ::initHints();
    PartDesign::Plane::initHints();
}
