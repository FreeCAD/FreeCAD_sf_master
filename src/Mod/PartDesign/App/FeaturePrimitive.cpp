/***************************************************************************
 *   Copyright (c) 2015 Stefan Tröger <stefantroeger@gmx.net>              *
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


#include "FeaturePrimitive.h"
#include "DatumPoint.h"
#include "DatumCS.h"
#include <Mod/Part/App/modelRefine.h>
#include <Base/Exception.h>
#include <App/Document.h>
#include <App/Application.h>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepBuilderAPI_GTransform.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <QObject>

using namespace PartDesign;

namespace PartDesign {

PROPERTY_SOURCE(PartDesign::FeaturePrimitive, PartDesign::FeatureAddSub)

FeaturePrimitive::FeaturePrimitive()
  :  primitiveType(Box)
{
    ADD_PROPERTY_TYPE(CoordinateSystem, (0), "Primitive", App::Prop_None, "References to build the location of the primitive");
}

TopoDS_Shape FeaturePrimitive::refineShapeIfActive(const TopoDS_Shape& oldShape) const
{
    Base::Reference<ParameterGrp> hGrp = App::GetApplication().GetUserParameter()
        .GetGroup("BaseApp")->GetGroup("Preferences")->GetGroup("Mod/PartDesign");
    if (hGrp->GetBool("RefineModel", false)) {
        Part::BRepBuilderAPI_RefineModel mkRefine(oldShape);
        TopoDS_Shape resShape = mkRefine.Shape();
        return resShape;
    }

    return oldShape;
}

App::DocumentObjectExecReturn* FeaturePrimitive::execute(const TopoDS_Shape& primitiveShape)
{
    try {
        //transform the primitive in the correct coordinance
        App::DocumentObject* cs =  CoordinateSystem.getValue();
        if(cs && cs->getTypeId() == PartDesign::CoordinateSystem::getClassTypeId())
            Placement.setValue(static_cast<PartDesign::CoordinateSystem*>(cs)->Placement.getValue());
        else 
            Placement.setValue(Base::Placement());
        
        //if we have no base we just add the standart primitive shape
        TopoDS_Shape base;
        try{
             //if we have a base shape we need to make sure that it does not get our transformation to
             BRepBuilderAPI_Transform trsf(getBaseShape(), getLocation().Transformation().Inverted(), true);
             base = trsf.Shape();
        }
        catch(const Base::Exception&) {

             if(getAddSubType() == FeatureAddSub::Additive)
                 Shape.setValue(primitiveShape);
             else 
                 return new App::DocumentObjectExecReturn("Cannot subtract primitive feature without base feature");
                 
             AddSubShape.setValue(base);
             return  App::DocumentObject::StdReturn;
        }
         
        if(getAddSubType() == FeatureAddSub::Additive) {
            
            BRepAlgoAPI_Fuse mkFuse(base, primitiveShape);
            if (!mkFuse.IsDone())
                return new App::DocumentObjectExecReturn("Adding the primitive failed");
            // we have to get the solids (fuse sometimes creates compounds)
            TopoDS_Shape boolOp = this->getSolid(mkFuse.Shape());
            // lets check if the result is a solid
            if (boolOp.IsNull())
                return new App::DocumentObjectExecReturn("Resulting shape is not a solid");
            
            boolOp = refineShapeIfActive(boolOp);
            Shape.setValue(boolOp);
            AddSubShape.setValue(primitiveShape);
        }
        else if(getAddSubType() == FeatureAddSub::Subtractive) {
            
            BRepAlgoAPI_Cut mkCut(base, primitiveShape);
            if (!mkCut.IsDone())
                return new App::DocumentObjectExecReturn("Subtracting the primitive failed");
            // we have to get the solids (fuse sometimes creates compounds)
            TopoDS_Shape boolOp = this->getSolid(mkCut.Shape());
            // lets check if the result is a solid
            if (boolOp.IsNull())
                return new App::DocumentObjectExecReturn("Resulting shape is not a solid");
            
            boolOp = refineShapeIfActive(boolOp);
            Shape.setValue(boolOp);
            AddSubShape.setValue(primitiveShape);
        }
        
        
    }
    catch (Standard_Failure) {
        Handle_Standard_Failure e = Standard_Failure::Caught();
        return new App::DocumentObjectExecReturn(e->GetMessageString());
    }

    return App::DocumentObject::StdReturn;
}

void FeaturePrimitive::onChanged(const App::Property* prop)
{
    FeatureAddSub::onChanged(prop);
}

PROPERTY_SOURCE(PartDesign::Box, PartDesign::FeaturePrimitive)

Box::Box()
{
    ADD_PROPERTY_TYPE(Length,(10.0f),"Box",App::Prop_None,"The length of the box");
    ADD_PROPERTY_TYPE(Width ,(10.0f),"Box",App::Prop_None,"The width of the box");
    ADD_PROPERTY_TYPE(Height,(10.0f),"Box",App::Prop_None,"The height of the box");
    
    primitiveType = FeaturePrimitive::Box;
}

App::DocumentObjectExecReturn* Box::execute(void)
{
    double L = Length.getValue();
    double W = Width.getValue();
    double H = Height.getValue();

    if (L < Precision::Confusion())
        return new App::DocumentObjectExecReturn("Length of box too small");

    if (W < Precision::Confusion())
        return new App::DocumentObjectExecReturn("Width of box too small");

    if (H < Precision::Confusion())
        return new App::DocumentObjectExecReturn("Height of box too small");

    try {
        // Build a box using the dimension attributes
        BRepPrimAPI_MakeBox mkBox(L, W, H);
        return FeaturePrimitive::execute(mkBox.Shape());
    }
    catch (Standard_Failure) {
        Handle_Standard_Failure e = Standard_Failure::Caught();
        return new App::DocumentObjectExecReturn(e->GetMessageString());
    }
}

short int Box::mustExecute() const
{
     if ( Length.isTouched() ||
          Height.isTouched() ||
          Width.isTouched() )
        return 1;
     
    return FeaturePrimitive::mustExecute();
}

PROPERTY_SOURCE(PartDesign::AdditiveBox, PartDesign::Box)
PROPERTY_SOURCE(PartDesign::SubtractiveBox, PartDesign::Box)


PROPERTY_SOURCE(PartDesign::Cylinder, PartDesign::FeaturePrimitive)

Cylinder::Cylinder()
{
    ADD_PROPERTY_TYPE(Radius,(10.0f),"Cylinder",App::Prop_None,"The radius of the cylinder");
    ADD_PROPERTY_TYPE(Angle,(10.0f),"Cylinder",App::Prop_None,"The closing angel of the cylinder ");
    ADD_PROPERTY_TYPE(Height,(10.0f),"Cylinder",App::Prop_None,"The height of the cylinder");
    
    primitiveType = FeaturePrimitive::Cylinder;
}

App::DocumentObjectExecReturn* Cylinder::execute(void)
{
    // Build a cylinder
    if (Radius.getValue() < Precision::Confusion())
        return new App::DocumentObjectExecReturn("Radius of cylinder too small");
    if (Height.getValue() < Precision::Confusion())
        return new App::DocumentObjectExecReturn("Height of cylinder too small");
    try {
        BRepPrimAPI_MakeCylinder mkCylr(Radius.getValue(),
                                        Height.getValue(),
                                        Angle.getValue()/180.0f*M_PI);
        
        return FeaturePrimitive::execute(mkCylr.Shape());
    }
    catch (Standard_Failure) {
        Handle_Standard_Failure e = Standard_Failure::Caught();
        return new App::DocumentObjectExecReturn(e->GetMessageString());
    }

    return App::DocumentObject::StdReturn;
}

short int Cylinder::mustExecute() const
{
     if ( Radius.isTouched() ||
          Height.isTouched() ||
          Angle.isTouched() )
        return 1;
     
    return FeaturePrimitive::mustExecute();
}

PROPERTY_SOURCE(PartDesign::AdditiveCylinder, PartDesign::Cylinder)
PROPERTY_SOURCE(PartDesign::SubtractiveCylinder, PartDesign::Cylinder)


PROPERTY_SOURCE(PartDesign::Sphere, PartDesign::FeaturePrimitive)

Sphere::Sphere()
{
    ADD_PROPERTY_TYPE(Radius,(5.0),"Sphere",App::Prop_None,"The radius of the sphere");
    ADD_PROPERTY_TYPE(Angle1,(-90.0f),"Sphere",App::Prop_None,"The angle of the sphere");
    ADD_PROPERTY_TYPE(Angle2,(90.0f),"Sphere",App::Prop_None,"The angle of the sphere");
    ADD_PROPERTY_TYPE(Angle3,(360.0f),"Sphere",App::Prop_None,"The angle of the sphere");
    
    primitiveType = FeaturePrimitive::Sphere;
}

App::DocumentObjectExecReturn* Sphere::execute(void)
{
   // Build a sphere
    if (Radius.getValue() < Precision::Confusion())
        return new App::DocumentObjectExecReturn("Radius of sphere too small");
    try {
        BRepPrimAPI_MakeSphere mkSphere(Radius.getValue(),
                                        Angle1.getValue()/180.0f*M_PI,
                                        Angle2.getValue()/180.0f*M_PI,
                                        Angle3.getValue()/180.0f*M_PI);
        return FeaturePrimitive::execute(mkSphere.Shape());
    }
    catch (Standard_Failure) {
        Handle_Standard_Failure e = Standard_Failure::Caught();
        return new App::DocumentObjectExecReturn(e->GetMessageString());
    }

    return App::DocumentObject::StdReturn;
}

short int Sphere::mustExecute() const
{
     if ( Radius.isTouched() ||
          Angle1.isTouched() ||
          Angle2.isTouched() ||
          Angle3.isTouched())
        return 1;
     
    return FeaturePrimitive::mustExecute();
}

PROPERTY_SOURCE(PartDesign::AdditiveSphere, PartDesign::Sphere)
PROPERTY_SOURCE(PartDesign::SubtractiveSphere, PartDesign::Sphere)
}
