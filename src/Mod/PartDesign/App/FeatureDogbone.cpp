/***************************************************************************
 *   Copyright (c) 2010 Juergen Riegel <FreeCAD@juergen-riegel.net>        *
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
# include <BRepAlgo.hxx>
# include <BRepPrimAPI_MakeCylinder.hxx>
# include <BRepAlgoAPI_Cut.hxx>
# include <TopExp.hxx>
# include <TopExp_Explorer.hxx>
# include <TopoDS.hxx>
# include <TopoDS_Edge.hxx>
# include <TopTools_IndexedMapOfShape.hxx>
# include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
# include <TopTools_ListOfShape.hxx>
# include <BRep_Tool.hxx>
# include <ShapeFix_Shape.hxx>
# include <ShapeFix_ShapeTolerance.hxx>
# include <Standard_Version.hxx>
#endif

#include <Base/Console.h>
#include <Base/Exception.h>
#include <Base/Reader.h>
#include <Mod/Part/App/TopoShape.h>

#include <gce_MakeDir.hxx>
#include <BRepGProp.hxx>
#include <ShapeAnalysis_Edge.hxx>
#include <GProp_GProps.hxx>

#include "FeatureDogbone.h"


using namespace PartDesign;


PROPERTY_SOURCE(PartDesign::Dogbone, PartDesign::DressUp)

const App::PropertyQuantityConstraint::Constraints floatRadius = {0.0,FLT_MAX,0.1};

Dogbone::Dogbone()
{
    ADD_PROPERTY(Radius,(1.0));
    Radius.setUnit(Base::Unit::Length);
    Radius.setConstraints(&floatRadius);
}

short Dogbone::mustExecute() const
{
    if (Placement.isTouched() || Radius.isTouched())
        return 1;
    return DressUp::mustExecute();
}

App::DocumentObjectExecReturn *Dogbone::execute(void)
{
    // NOTE: Normally the Base property and the BaseFeature property should point to the same object.
    // The only difference is that the Base property also stores the edges that are to be chamfered
    Part::TopoShape TopShape;
    try {
        TopShape = getBaseShape();
    } catch (Base::Exception& e) {
        return new App::DocumentObjectExecReturn(e.what());
    }

    std::vector<std::string> SubNames = std::vector<std::string>(Base.getSubValues());
    getContiniusEdges(TopShape, SubNames);

    if (SubNames.size() == 0)
        return new App::DocumentObjectExecReturn("No edges specified");

    double size = Radius.getValue();
    if (size <= 0)
        return new App::DocumentObjectExecReturn("Radius must be greater than zero");

    this->positionByBaseFeature();
    // create an untransformed copy of the basefeature shape
    Part::TopoShape baseShape(TopShape);
    baseShape.setTransform(Base::Matrix4D());
    try {
        GProp_GProps edgeProps;
        ShapeAnalysis_Edge edgeAnalysis;

        Part::TopoShape result = baseShape;

        for (std::vector<std::string>::const_iterator it=SubNames.begin(); it != SubNames.end(); ++it) {
            TopoDS_Edge edge = TopoDS::Edge(baseShape.getSubShape(it->c_str()));
            BRepGProp::LinearProperties(edge, edgeProps);
            TopExp_Explorer exp(edge, TopAbs_VERTEX);

            gp_Pnt first = BRep_Tool::Pnt(edgeAnalysis.FirstVertex(edge));
            gp_Pnt last = BRep_Tool::Pnt(edgeAnalysis.LastVertex(edge));

            gp_Ax2 ax2(first, gce_MakeDir(first, last).Value());

            std::cerr << "Pos: " << first.X() << " " << first.Y() << " " << first.Z() << std::endl;

            BRepPrimAPI_MakeCylinder mkCylr(ax2, Radius.getValue(), edgeProps.Mass());
            TopoDS_Shape cyl = mkCylr.Shape();

            BRepAlgoAPI_Cut mkCut(result.getShape(), cyl);
            result = mkCut.Shape();
            if (!mkCut.IsDone()) {
                return new App::DocumentObjectExecReturn("Failed to create Dogbone");
            }
            if (result.isNull()) {
                return new App::DocumentObjectExecReturn("Resulting shape is null");
            }
        }

        TopoDS_Shape resultShape = result.getShape();

        TopTools_ListOfShape aLarg;
        aLarg.Append(baseShape.getShape());
        if (!BRepAlgo::IsValid(aLarg, resultShape, Standard_False, Standard_False)) {
            ShapeFix_ShapeTolerance aSFT;
            aSFT.LimitTolerance(resultShape, Precision::Confusion(), Precision::Confusion(), TopAbs_SHAPE);
            Handle(ShapeFix_Shape) aSfs = new ShapeFix_Shape(resultShape);
            aSfs->Perform();
            result = aSfs->Shape();
            if (!BRepAlgo::IsValid(aLarg, resultShape, Standard_False, Standard_False)) {
                return new App::DocumentObjectExecReturn("Resulting shape is invalid");
            }
        }
        int solidCount = countSolids(resultShape);
        if (solidCount > 1) {
            return new App::DocumentObjectExecReturn("Dogbone: Result has multiple solids. This is not supported at this time.");
        }

        this->Shape.setValue(getSolid(resultShape));
        return App::DocumentObject::StdReturn;
    }
    catch (Standard_Failure& e) {
        return new App::DocumentObjectExecReturn(e.GetMessageString());
    }
}

void Dogbone::Restore(Base::XMLReader &reader)
{
    reader.readElement("Properties");
    int Cnt = reader.getAttributeAsInteger("Count");

    for (int i=0 ;i<Cnt ;i++) {
        reader.readElement("Property");
        const char* PropName = reader.getAttribute("name");
        const char* TypeName = reader.getAttribute("type");
        App::Property* prop = getPropertyByName(PropName);

        try {
            if (prop && strcmp(prop->getTypeId().getName(), TypeName) == 0) {
                prop->Restore(reader);
            }
            else if (prop && strcmp(TypeName,"App::PropertyFloatConstraint") == 0 &&
                     strcmp(prop->getTypeId().getName(), "App::PropertyQuantityConstraint") == 0) {
                App::PropertyFloatConstraint p;
                p.Restore(reader);
                static_cast<App::PropertyQuantityConstraint*>(prop)->setValue(p.getValue());
            }
        }
        catch (const Base::XMLParseException&) {
            throw; // re-throw
        }
        catch (const Base::Exception &e) {
            Base::Console().Error("%s\n", e.what());
        }
        catch (const std::exception &e) {
            Base::Console().Error("%s\n", e.what());
        }
        reader.readEndElement("Property");
    }
    reader.readEndElement("Properties");
}
