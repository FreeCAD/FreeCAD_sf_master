/***************************************************************************
 *   Copyright (c) 2011 Juergen Riegel <juergen.riegel@web.de>             *
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
# include <sstream>
# include <Bnd_Box.hxx>
# include <Poly_Polygon3D.hxx>
# include <BRepBndLib.hxx>
# include <BRepBuilderAPI_MakeVertex.hxx>
# include <BRepExtrema_DistShapeShape.hxx>
# include <BRepMesh_IncrementalMesh.hxx>
# include <BRep_Tool.hxx>
# include <BRepTools.hxx>
# include <BRepAdaptor_Curve.hxx>
# include <BRepAdaptor_Surface.hxx>
# include <GeomLib.hxx>
# include <GeomAbs_CurveType.hxx>
# include <GeomAbs_SurfaceType.hxx>
# include <Geom_BezierCurve.hxx>
# include <Geom_BSplineCurve.hxx>
# include <Geom_BezierSurface.hxx>
# include <Geom_BSplineSurface.hxx>
# include <GeomAPI_ProjectPointOnSurf.hxx>
# include <GeomLProp_SLProps.hxx>
# include <gp_Trsf.hxx>
# include <Handle_Poly_Triangulation.hxx>
# include <Poly_Array1OfTriangle.hxx>
# include <Poly_Triangulation.hxx>
# include <Poly_Connect.hxx>
# include <Standard_Version.hxx>
# include <TColgp_Array1OfPnt.hxx>
# include <TopoDS.hxx>
# include <TopoDS_Edge.hxx>
# include <TopoDS_Wire.hxx>
# include <TopoDS_Face.hxx>
# include <TopoDS_Shape.hxx>
# include <TopoDS_Vertex.hxx>
# include <TopoDS_Iterator.hxx>
# include <TopExp_Explorer.hxx>
# include <TopExp.hxx>
# include <TopTools_IndexedMapOfShape.hxx>
# include <Poly_PolygonOnTriangulation.hxx>
# include <TColStd_Array1OfInteger.hxx>
# include <TColgp_Array1OfDir.hxx>
# include <TColgp_Array1OfPnt2d.hxx>
# include <TopTools_ListOfShape.hxx>
# include <TShort_Array1OfShortReal.hxx>
# include <TShort_HArray1OfShortReal.hxx>
# include <Precision.hxx>

# include <Inventor/SoPickedPoint.h>
# include <Inventor/details/SoFaceDetail.h>
# include <Inventor/details/SoLineDetail.h>
# include <Inventor/details/SoPointDetail.h>
# include <Inventor/events/SoMouseButtonEvent.h>
# include <Inventor/nodes/SoBaseColor.h>
# include <Inventor/nodes/SoCoordinate3.h>
# include <Inventor/nodes/SoDrawStyle.h>
# include <Inventor/nodes/SoIndexedFaceSet.h>
# include <Inventor/nodes/SoIndexedLineSet.h>
# include <Inventor/nodes/SoLocateHighlight.h>
# include <Inventor/nodes/SoMaterial.h>
# include <Inventor/nodes/SoMaterialBinding.h>
# include <Inventor/nodes/SoNormal.h>
# include <Inventor/nodes/SoNormalBinding.h>
# include <Inventor/nodes/SoPointSet.h>
# include <Inventor/nodes/SoPolygonOffset.h>
# include <Inventor/nodes/SoShapeHints.h>
# include <Inventor/nodes/SoSwitch.h>
# include <Inventor/nodes/SoGroup.h>
# include <Inventor/nodes/SoSphere.h>
# include <Inventor/nodes/SoScale.h>
# include <Inventor/nodes/SoLightModel.h>
# include <QAction>
# include <QMenu>
#endif

/// Here the FreeCAD includes sorted by Base,App,Gui......
#include <Base/Console.h>
#include <Base/Parameter.h>
#include <Base/Exception.h>
#include <Base/TimeInfo.h>

#include <App/Application.h>
#include <App/Document.h>

#include <Gui/SoFCUnifiedSelection.h>
#include <Gui/Selection.h>
#include <Gui/View3DInventorViewer.h>
#include <Gui/Utilities.h>
#include <Gui/Control.h>

#include "ViewProviderExt.h"
#include "SoBrepPointSet.h"
#include "SoBrepEdgeSet.h"
#include "SoBrepFaceSet.h"
#include "TaskFaceColors.h"

#include <Mod/Part/App/PartFeature.h>
#include <Mod/Part/App/PrimitiveFeature.h>


using namespace PartGui;

PROPERTY_SOURCE(PartGui::ViewProviderPartExt, Gui::ViewProviderGeometryObject)


void ViewProviderPartExt::GetNormals(const TopoDS_Face&  theFace,
             const Handle(Poly_Triangulation)& aPolyTri,
             TColgp_Array1OfDir& theNormals)
{
    Poly_Connect thePolyConnect(aPolyTri);
    const TColgp_Array1OfPnt&         aNodes   = aPolyTri->Nodes();

    if(aPolyTri->HasNormals())
    {
        // normals pre-computed in triangulation structure
        const TShort_Array1OfShortReal& aNormals = aPolyTri->Normals();
        const Standard_ShortReal*       aNormArr = &(aNormals.Value(aNormals.Lower()));

        for(Standard_Integer aNodeIter = aNodes.Lower(); aNodeIter <= aNodes.Upper(); ++aNodeIter)
        {
            const Standard_Integer anId = 3 * (aNodeIter - aNodes.Lower());
            const gp_Dir aNorm(aNormArr[anId + 0],
                               aNormArr[anId + 1],
                               aNormArr[anId + 2]);
            theNormals(aNodeIter) = aNorm;
        }

        if(theFace.Orientation() == TopAbs_REVERSED)
        {
            for(Standard_Integer aNodeIter = aNodes.Lower(); aNodeIter <= aNodes.Upper(); ++aNodeIter)
            {
                theNormals.ChangeValue(aNodeIter).Reverse();
            }
        }

        return;
    }

    // take in face the surface location
    const TopoDS_Face      aZeroFace = TopoDS::Face(theFace.Located(TopLoc_Location()));
    Handle(Geom_Surface)   aSurf     = BRep_Tool::Surface(aZeroFace);
    const Standard_Real    aTol      = Precision::Confusion();
    Handle(TShort_HArray1OfShortReal) aNormals = new TShort_HArray1OfShortReal(1, aPolyTri->NbNodes() * 3);
    const Poly_Array1OfTriangle& aTriangles = aPolyTri->Triangles();
    const TColgp_Array1OfPnt2d*  aNodesUV   = aPolyTri->HasUVNodes() && !aSurf.IsNull()
            ? &aPolyTri->UVNodes()
            : NULL;
    Standard_Integer aTri[3];

    for(Standard_Integer aNodeIter = aNodes.Lower(); aNodeIter <= aNodes.Upper(); ++aNodeIter)
    {
        // try to retrieve normal from real surface first, when UV coordinates are available
        if(aNodesUV == NULL
                || GeomLib::NormEstim(aSurf, aNodesUV->Value(aNodeIter), aTol, theNormals(aNodeIter)) > 1)
        {
            // compute flat normals
            gp_XYZ eqPlan(0.0, 0.0, 0.0);

            for(thePolyConnect.Initialize(aNodeIter); thePolyConnect.More(); thePolyConnect.Next())
            {
                aTriangles(thePolyConnect.Value()).Get(aTri[0], aTri[1], aTri[2]);
                const gp_XYZ v1(aNodes(aTri[1]).Coord() - aNodes(aTri[0]).Coord());
                const gp_XYZ v2(aNodes(aTri[2]).Coord() - aNodes(aTri[1]).Coord());
                const gp_XYZ vv = v1 ^ v2;
                const Standard_Real aMod = vv.Modulus();

                if(aMod >= aTol)
                {
                    eqPlan += vv / aMod;
                }
            }

            const Standard_Real aModMax = eqPlan.Modulus();
            theNormals(aNodeIter) = (aModMax > aTol) ? gp_Dir(eqPlan) : gp::DZ();
        }

        const Standard_Integer anId = (aNodeIter - aNodes.Lower()) * 3;
        aNormals->SetValue(anId + 1, (Standard_ShortReal)theNormals(aNodeIter).X());
        aNormals->SetValue(anId + 2, (Standard_ShortReal)theNormals(aNodeIter).Y());
        aNormals->SetValue(anId + 3, (Standard_ShortReal)theNormals(aNodeIter).Z());
    }

    aPolyTri->SetNormals(aNormals);

    if(theFace.Orientation() == TopAbs_REVERSED)
    {
        for(Standard_Integer aNodeIter = aNodes.Lower(); aNodeIter <= aNodes.Upper(); ++aNodeIter)
        {
            theNormals.ChangeValue(aNodeIter).Reverse();
        }
    }
}

//**************************************************************************
// Construction/Destruction

App::PropertyFloatConstraint::Constraints ViewProviderPartExt::sizeRange = {1.0,64.0,1.0};
App::PropertyFloatConstraint::Constraints ViewProviderPartExt::tessRange = {0.0001,100.0,0.01};
App::PropertyQuantityConstraint::Constraints ViewProviderPartExt::angDeflectionRange = {0.0,180.0,0.05};
const char* ViewProviderPartExt::LightingEnums[]= {"One side","Two side",NULL};
const char* ViewProviderPartExt::DrawStyleEnums[]= {"Solid","Dashed","Dotted","Dashdot",NULL};

ViewProviderPartExt::ViewProviderPartExt() 
{
    VisualTouched = true;

    ParameterGrp::handle hGrp = App::GetApplication().GetParameterGroupByPath("User parameter:BaseApp/Preferences/View");
    unsigned long lcol = hGrp->GetUnsigned("DefaultShapeLineColor",421075455UL); // dark grey (25,25,25)
    float r,g,b;
    r = ((lcol >> 24) & 0xff) / 255.0; g = ((lcol >> 16) & 0xff) / 255.0; b = ((lcol >> 8) & 0xff) / 255.0;
    int lwidth = hGrp->GetInt("DefaultShapeLineWidth",2);
    App::Material mat;
    mat.ambientColor.set(0.2f,0.2f,0.2f);
    mat.diffuseColor.set(r,g,b);
    mat.specularColor.set(0.0f,0.0f,0.0f);
    mat.emissiveColor.set(0.0f,0.0f,0.0f);
    mat.shininess = 1.0f;
    mat.transparency = 0.0f;
    ADD_PROPERTY(LineMaterial,(mat));
    ADD_PROPERTY(PointMaterial,(mat));
    ADD_PROPERTY(LineColor,(mat.diffuseColor));
    ADD_PROPERTY(PointColor,(mat.diffuseColor));
    ADD_PROPERTY(DiffuseColor,(ShapeColor.getValue()));
    ADD_PROPERTY(LineColorArray,(LineColor.getValue()));
    ADD_PROPERTY(LineWidth,(lwidth));
    LineWidth.setConstraints(&sizeRange);
    PointSize.setConstraints(&sizeRange);
    ADD_PROPERTY(PointSize,(lwidth));
    ADD_PROPERTY(Deviation,(0.5f));
    Deviation.setConstraints(&tessRange);
    ADD_PROPERTY(AngularDeflection,(28.65));
    AngularDeflection.setConstraints(&angDeflectionRange);
    ADD_PROPERTY(Lighting,(1));
    Lighting.setEnums(LightingEnums);
    ADD_PROPERTY(DrawStyle,((long int)0));
    DrawStyle.setEnums(DrawStyleEnums);

    coords = new SoCoordinate3();
    coords->ref();
    faceset = new SoBrepFaceSet();
    faceset->ref();
    norm = new SoNormal;
    norm->ref();
    normb = new SoNormalBinding;
    normb->value = SoNormalBinding::PER_VERTEX_INDEXED;
    normb->ref();
    lineset = new SoBrepEdgeSet();
    lineset->ref();
    nodeset = new SoBrepPointSet();
    nodeset->ref();

    pcShapeBind = new SoMaterialBinding();
    pcShapeBind->ref();

    pcLineBind = new SoMaterialBinding();
    pcLineBind->ref();
    pcLineMaterial = new SoMaterial;
    pcLineMaterial->ref();
    LineMaterial.touch();

    pcPointMaterial = new SoMaterial;
    pcPointMaterial->ref();
    PointMaterial.touch();

    pcLineStyle = new SoDrawStyle();
    pcLineStyle->ref();
    pcLineStyle->style = SoDrawStyle::LINES;
    pcLineStyle->lineWidth = LineWidth.getValue();

    pcPointStyle = new SoDrawStyle();
    pcPointStyle->ref();
    pcPointStyle->style = SoDrawStyle::POINTS;
    pcPointStyle->pointSize = PointSize.getValue();

    pShapeHints = new SoShapeHints;
    pShapeHints->shapeType = SoShapeHints::UNKNOWN_SHAPE_TYPE;
    pShapeHints->ref();
    Lighting.touch();
    DrawStyle.touch();

    sPixmap = "Tree_Part";
    loadParameter();
}

ViewProviderPartExt::~ViewProviderPartExt()
{
    pcShapeBind->unref();
    pcLineBind->unref();
    pcLineMaterial->unref();
    pcPointMaterial->unref();
    pcLineStyle->unref();
    pcPointStyle->unref();
    pShapeHints->unref();
    coords->unref();
    faceset->unref();
    norm->unref();
    normb->unref();
    lineset->unref();
    nodeset->unref();
}

void ViewProviderPartExt::onChanged(const App::Property* prop)
{
    Part::Feature* feature = dynamic_cast<Part::Feature*>(pcObject);
    
    if (prop == &Deviation) {
        if(Visibility.getValue() && feature && !feature->Shape.getValue().IsNull()) 
            updateVisual(feature->Shape.getValue());
        else
            VisualTouched = true;
    }
    if (prop == &AngularDeflection) {
        if(Visibility.getValue() && feature && !feature->Shape.getValue().IsNull()) 
            updateVisual(feature->Shape.getValue());
        else
            VisualTouched = true;
    }
    if (prop == &LineWidth) {
        pcLineStyle->lineWidth = LineWidth.getValue();
    }
    else if (prop == &PointSize) {
        pcPointStyle->pointSize = PointSize.getValue();
    }
    else if (prop == &LineColor) {
        const App::Color& c = LineColor.getValue();
        pcLineMaterial->diffuseColor.setValue(c.r,c.g,c.b);
        if (c != LineMaterial.getValue().diffuseColor)
            LineMaterial.setDiffuseColor(c);
        LineColorArray.setValue(LineColor.getValue());
    }
    else if (prop == &PointColor) {
        const App::Color& c = PointColor.getValue();
        pcPointMaterial->diffuseColor.setValue(c.r,c.g,c.b);
        if (c != PointMaterial.getValue().diffuseColor)
            PointMaterial.setDiffuseColor(c);
        PointColorArray.setValue(PointColor.getValue());
    }
    else if (prop == &LineMaterial) {
        const App::Material& Mat = LineMaterial.getValue();
        if (LineColor.getValue() != Mat.diffuseColor)
            LineColor.setValue(Mat.diffuseColor);
        pcLineMaterial->ambientColor.setValue(Mat.ambientColor.r,Mat.ambientColor.g,Mat.ambientColor.b);
        pcLineMaterial->diffuseColor.setValue(Mat.diffuseColor.r,Mat.diffuseColor.g,Mat.diffuseColor.b);
        pcLineMaterial->specularColor.setValue(Mat.specularColor.r,Mat.specularColor.g,Mat.specularColor.b);
        pcLineMaterial->emissiveColor.setValue(Mat.emissiveColor.r,Mat.emissiveColor.g,Mat.emissiveColor.b);
        pcLineMaterial->shininess.setValue(Mat.shininess);
        pcLineMaterial->transparency.setValue(Mat.transparency);
    }
    else if (prop == &PointMaterial) {
        const App::Material& Mat = PointMaterial.getValue();
        if (PointColor.getValue() != Mat.diffuseColor)
        PointColor.setValue(Mat.diffuseColor);
        pcPointMaterial->ambientColor.setValue(Mat.ambientColor.r,Mat.ambientColor.g,Mat.ambientColor.b);
        pcPointMaterial->diffuseColor.setValue(Mat.diffuseColor.r,Mat.diffuseColor.g,Mat.diffuseColor.b);
        pcPointMaterial->specularColor.setValue(Mat.specularColor.r,Mat.specularColor.g,Mat.specularColor.b);
        pcPointMaterial->emissiveColor.setValue(Mat.emissiveColor.r,Mat.emissiveColor.g,Mat.emissiveColor.b);
        pcPointMaterial->shininess.setValue(Mat.shininess);
        pcPointMaterial->transparency.setValue(Mat.transparency);
    }
    else if (prop == &PointColorArray) {
        const std::vector<App::Color>& c = PointColorArray.getValues();
        unsigned int size = (int)c.size();
        if (size > 1) {
            // FIXME: Check for size mismatch between number of points and number of colors
            pcShapeBind->value = SoMaterialBinding::PER_VERTEX;
            pcPointMaterial->diffuseColor.setNum(size);
            SbColor* ca = pcPointMaterial->diffuseColor.startEditing();
            for (unsigned int i=0; i < size; ++i)
                ca[i].setValue(c[i].r,c[i].g,c[i].b);
            pcPointMaterial->diffuseColor.finishEditing();
        }
        else if (size == 1) {
            pcShapeBind->value = SoMaterialBinding::OVERALL;
            pcPointMaterial->diffuseColor.setValue(c[0].r,c[0].g,c[0].b);
        }
    }
    else if (prop == &LineColorArray) {
        const std::vector<App::Color>& c = LineColorArray.getValues();
        int size = (int)c.size();
        if (size > 1) {
            pcLineBind->value = SoMaterialBinding::PER_PART;
            const int32_t* cindices = this->lineset->coordIndex.getValues(0);
            int numindices = this->lineset->coordIndex.getNum();
            pcLineMaterial->diffuseColor.setNum(size);
            SbColor* ca = pcLineMaterial->diffuseColor.startEditing();
            int linecount = 0;

            for (int i = 0; i < numindices; ++i) {
                if (cindices[i] < 0) {
                    ca[linecount].setValue(c[linecount].r,c[linecount].g,c[linecount].b);
                    linecount++;
                    if (linecount >= size)
                        break;
                }
            }

            pcLineMaterial->diffuseColor.finishEditing();
        }
        else if (size == 1) {
            pcLineBind->value = SoMaterialBinding::OVERALL;
            pcLineMaterial->diffuseColor.setValue(c[0].r,c[0].g,c[0].b);
        }
    }
    // For testing
    else if (prop == &DiffuseColor) {
        const std::vector<App::Color>& c = DiffuseColor.getValues();
        int size = static_cast<int>(c.size());
        if (size > 1 && size == this->faceset->partIndex.getNum()) {
            pcShapeBind->value = SoMaterialBinding::PER_PART;
            pcShapeMaterial->diffuseColor.setNum(size);
            SbColor* ca = pcShapeMaterial->diffuseColor.startEditing();
            for (int i=0; i < size; i++)
                ca[i].setValue(c[i].r,c[i].g,c[i].b);
            pcShapeMaterial->diffuseColor.finishEditing();
        }
        else if (c.size() == 1) {
            pcShapeBind->value = SoMaterialBinding::OVERALL;
            pcShapeMaterial->diffuseColor.setValue(c[0].r,c[0].g,c[0].b);
        }
    }
    else if (prop == &ShapeMaterial || prop == &ShapeColor) {
        pcShapeBind->value = SoMaterialBinding::OVERALL;
        ViewProviderGeometryObject::onChanged(prop);
        DiffuseColor.setValue(ShapeColor.getValue());
    }
    else if (prop == &Transparency) {
        const App::Material& Mat = ShapeMaterial.getValue();
        long value = (long)(100*Mat.transparency);
        if (value != Transparency.getValue()) {
            float trans = Transparency.getValue()/100.0f;
            if (pcShapeBind->value.getValue() == SoMaterialBinding::PER_PART) {
                int cnt = pcShapeMaterial->diffuseColor.getNum();
                pcShapeMaterial->transparency.setNum(cnt);
                float *t = pcShapeMaterial->transparency.startEditing();
                for (int i=0; i<cnt; i++)
                    t[i] = trans;
                pcShapeMaterial->transparency.finishEditing();
            }
            else {
                pcShapeMaterial->transparency = trans;
            }

            App::PropertyContainer* parent = ShapeMaterial.getContainer();
            ShapeMaterial.setContainer(0);
            ShapeMaterial.setTransparency(trans);
            ShapeMaterial.setContainer(parent);
        }
    }
    else if (prop == &Lighting) {
        if (Lighting.getValue() == 0)
            pShapeHints->vertexOrdering = SoShapeHints::UNKNOWN_ORDERING;
        else
            pShapeHints->vertexOrdering = SoShapeHints::COUNTERCLOCKWISE;
    }
    else if (prop == &DrawStyle) {
        if (DrawStyle.getValue() == 0)
            pcLineStyle->linePattern = 0xffff;
        else if (DrawStyle.getValue() == 1)
            pcLineStyle->linePattern = 0xf00f;
        else if (DrawStyle.getValue() == 2)
            pcLineStyle->linePattern = 0x0f0f;
        else
            pcLineStyle->linePattern = 0xff88;
    }
    else {
        // if the object was invisible and has been changed, recreate the visual
        if (prop == &Visibility && Visibility.getValue() && VisualTouched) {
            updateVisual(feature->Shape.getValue());
            // The material has to be checked again (#0001736)
            onChanged(&DiffuseColor);
        }
    }

    ViewProviderGeometryObject::onChanged(prop);
}

void ViewProviderPartExt::attach(App::DocumentObject *pcFeat)
{
    // call parent attach method
    ViewProviderGeometryObject::attach(pcFeat);

    // Workaround for #0000433, i.e. use SoSeparator instead of SoGroup
    SoGroup* pcNormalRoot = new SoSeparator();
    SoGroup* pcFlatRoot = new SoSeparator();
    SoGroup* pcWireframeRoot = new SoSeparator();
    SoGroup* pcPointsRoot = new SoSeparator();

    // enable two-side rendering
    pShapeHints->vertexOrdering = SoShapeHints::COUNTERCLOCKWISE;
    pShapeHints->shapeType = SoShapeHints::UNKNOWN_SHAPE_TYPE;

    // Avoid any Z-buffer artefacts, so that the lines always appear on top of the faces
    // The correct order is Edges, Polygon offset, Faces.
    SoPolygonOffset* offset = new SoPolygonOffset();

    // wireframe node
    SoSeparator* wireframe = new SoSeparator();
    wireframe->addChild(pcLineBind);
    wireframe->addChild(pcLineMaterial);
    wireframe->addChild(pcLineStyle);
    wireframe->addChild(lineset);

    // normal viewing with edges and points
    pcNormalRoot->addChild(wireframe);
    pcNormalRoot->addChild(offset);
    pcNormalRoot->addChild(pcFlatRoot);
    pcNormalRoot->addChild(pcPointsRoot);

    // just faces with no edges or points
    pcFlatRoot->addChild(pShapeHints);
    pcFlatRoot->addChild(pcShapeBind);
    pcFlatRoot->addChild(pcShapeMaterial);
    SoDrawStyle* pcFaceStyle = new SoDrawStyle();
    pcFaceStyle->style = SoDrawStyle::FILLED;
    pcFlatRoot->addChild(pcFaceStyle);
    pcFlatRoot->addChild(norm);
    pcFlatRoot->addChild(normb);
    pcFlatRoot->addChild(faceset);

    // edges and points
    pcWireframeRoot->addChild(wireframe);
    pcWireframeRoot->addChild(pcPointsRoot);

    // normal viewing with edges and points
    pcPointsRoot->addChild(pcPointMaterial);
    pcPointsRoot->addChild(pcPointStyle);
    pcPointsRoot->addChild(nodeset);

    // Move 'coords' before the switch
    pcRoot->insertChild(coords,pcRoot->findChild(pcModeSwitch));

    // putting all together with the switch
    addDisplayMaskMode(pcNormalRoot, "Flat Lines");
    addDisplayMaskMode(pcFlatRoot, "Shaded");
    addDisplayMaskMode(pcWireframeRoot, "Wireframe");
    addDisplayMaskMode(pcPointsRoot, "Point");
}

void ViewProviderPartExt::setDisplayMode(const char* ModeName)
{
    if ( strcmp("Flat Lines",ModeName)==0 )
        setDisplayMaskMode("Flat Lines");
    else if ( strcmp("Shaded",ModeName)==0 )
        setDisplayMaskMode("Shaded");
    else if ( strcmp("Wireframe",ModeName)==0 )
        setDisplayMaskMode("Wireframe");
    else if ( strcmp("Points",ModeName)==0 )
        setDisplayMaskMode("Point");

    ViewProviderGeometryObject::setDisplayMode( ModeName );
}

std::vector<std::string> ViewProviderPartExt::getDisplayModes(void) const
{
    // get the modes of the father
    std::vector<std::string> StrList = ViewProviderGeometryObject::getDisplayModes();

    // add your own modes
    StrList.push_back("Flat Lines");
    StrList.push_back("Shaded");
    StrList.push_back("Wireframe");
    StrList.push_back("Points");

    return StrList;
}

std::string ViewProviderPartExt::getElement(const SoDetail* detail) const
{
    std::stringstream str;
    if (detail) {
        if (detail->getTypeId() == SoFaceDetail::getClassTypeId()) {
            const SoFaceDetail* face_detail = static_cast<const SoFaceDetail*>(detail);
            int face = face_detail->getPartIndex() + 1;
            str << "Face" << face;
        }
        else if (detail->getTypeId() == SoLineDetail::getClassTypeId()) {
            const SoLineDetail* line_detail = static_cast<const SoLineDetail*>(detail);
            int edge = line_detail->getLineIndex() + 1;
            str << "Edge" << edge;
        }
        else if (detail->getTypeId() == SoPointDetail::getClassTypeId()) {
            const SoPointDetail* point_detail = static_cast<const SoPointDetail*>(detail);
            int vertex = point_detail->getCoordinateIndex() - nodeset->startIndex.getValue() + 1;
            str << "Vertex" << vertex;
        }
    }

    return str.str();
}

SoDetail* ViewProviderPartExt::getDetail(const char* subelement) const
{
    std::string element = subelement;
    std::string::size_type pos = element.find_first_of("0123456789");
    int index = -1;
    if (pos != std::string::npos) {
        index = std::atoi(element.substr(pos).c_str());
        element = element.substr(0,pos);
    }

    SoDetail* detail = 0;
    if (index < 0)
        return detail;
    if (element == "Face") {
        detail = new SoFaceDetail();
        static_cast<SoFaceDetail*>(detail)->setPartIndex(index - 1);
    }
    else if (element == "Edge") {
        detail = new SoLineDetail();
        static_cast<SoLineDetail*>(detail)->setLineIndex(index - 1);
    }
    else if (element == "Vertex") {
        detail = new SoPointDetail();
        static_cast<SoPointDetail*>(detail)->setCoordinateIndex(index + nodeset->startIndex.getValue() - 1);
    }

    return detail;
}

std::vector<Base::Vector3d> ViewProviderPartExt::getModelPoints(const SoPickedPoint* pp) const
{
    try {
        std::vector<Base::Vector3d> pts;
        std::string element = this->getElement(pp->getDetail());
        const Part::TopoShape& shape = static_cast<Part::Feature*>(getObject())->Shape.getShape();

        TopoDS_Shape subShape = shape.getSubShape(element.c_str());

        // get the point of the vertex directly
        if (subShape.ShapeType() == TopAbs_VERTEX) {
            const TopoDS_Vertex& v = TopoDS::Vertex(subShape);
            gp_Pnt p = BRep_Tool::Pnt(v);
            pts.push_back(Base::Vector3d(p.X(),p.Y(),p.Z()));
        }
        // get the nearest point on the edge
        else if (subShape.ShapeType() == TopAbs_EDGE) {
            const SbVec3f& vec = pp->getPoint();
            BRepBuilderAPI_MakeVertex mkVert(gp_Pnt(vec[0],vec[1],vec[2]));
            BRepExtrema_DistShapeShape distSS(subShape, mkVert.Vertex(), 0.1);
            if (distSS.NbSolution() > 0) {
                gp_Pnt p = distSS.PointOnShape1(1);
                pts.push_back(Base::Vector3d(p.X(),p.Y(),p.Z()));
            }
        }
        // get the nearest point on the face
        else if (subShape.ShapeType() == TopAbs_FACE) {
            const SbVec3f& vec = pp->getPoint();
            BRepBuilderAPI_MakeVertex mkVert(gp_Pnt(vec[0],vec[1],vec[2]));
            BRepExtrema_DistShapeShape distSS(subShape, mkVert.Vertex(), 0.1);
            if (distSS.NbSolution() > 0) {
                gp_Pnt p = distSS.PointOnShape1(1);
                pts.push_back(Base::Vector3d(p.X(),p.Y(),p.Z()));
            }
        }

        return pts;
    }
    catch (...) {
    }

    // if something went wrong returns an empty array
    return std::vector<Base::Vector3d>();
}

std::vector<Base::Vector3d> ViewProviderPartExt::getSelectionShape(const char* Element) const
{
    return std::vector<Base::Vector3d>();
}

bool ViewProviderPartExt::loadParameter()
{
    bool changed = false;
    ParameterGrp::handle hGrp = App::GetApplication().GetParameterGroupByPath
        ("User parameter:BaseApp/Preferences/Mod/Part");
    float deviation = hGrp->GetFloat("MeshDeviation",0.2);
    float angularDeflection = hGrp->GetFloat("MeshAngularDeflection",28.65);
    bool novertexnormals = hGrp->GetBool("NoPerVertexNormals",false);
    bool qualitynormals = hGrp->GetBool("QualityNormals",false);

    if (Deviation.getValue() != deviation) {
        Deviation.setValue(deviation);
        changed = true;
    }
    if (AngularDeflection.getValue() != angularDeflection ) {
        AngularDeflection.setValue(angularDeflection);
    }
    if (this->noPerVertexNormals != novertexnormals) {
        this->noPerVertexNormals = novertexnormals;
        changed = true;
    }
    if (this->qualityNormals != qualitynormals) {
        this->qualityNormals = qualitynormals;
        changed = true;
    }

    return changed;
}

void ViewProviderPartExt::reload()
{
    if (loadParameter()) {
        App::Property* shape     = pcObject->getPropertyByName("Shape");
        if (shape) update(shape);
    }
}

void ViewProviderPartExt::updateData(const App::Property* prop)
{
    if (prop->getTypeId() == Part::PropertyPartShape::getClassTypeId()) {
        // get the shape to show
        const TopoDS_Shape &cShape = static_cast<const Part::PropertyPartShape*>(prop)->getValue();

        // calculate the visual only if visible
        if (Visibility.getValue())
            updateVisual(cShape);
        else
            VisualTouched = true;

        if (!VisualTouched) {
            if (this->faceset->partIndex.getNum() > 
                this->pcShapeMaterial->diffuseColor.getNum()) {
                this->pcShapeBind->value = SoMaterialBinding::OVERALL;
            }
        }
    }
    Gui::ViewProviderGeometryObject::updateData(prop);
}

void ViewProviderPartExt::setupContextMenu(QMenu* menu, QObject* receiver, const char* member)
{
    Gui::ViewProviderGeometryObject::setupContextMenu(menu, receiver, member);
    QAction* act = menu->addAction(QObject::tr("Set colors..."), receiver, member);
    act->setData(QVariant((int)ViewProvider::Color));
}

bool ViewProviderPartExt::setEdit(int ModNum)
{
    if (ModNum == ViewProvider::Color) {
        // When double-clicking on the item for this pad the
        // object unsets and sets its edit mode without closing
        // the task panel
        Gui::TaskView::TaskDialog *dlg = Gui::Control().activeDialog();
        if (dlg) {
            Gui::Control().showDialog(dlg);
            return false;
        }

        Gui::Selection().clearSelection();
        Gui::Control().showDialog(new TaskFaceColors(this));
        return true;
    }
    else {
        return Gui::ViewProviderGeometryObject::setEdit(ModNum);
    }
}

void ViewProviderPartExt::unsetEdit(int ModNum)
{
    if (ModNum == ViewProvider::Color) {
    }
    else {
        Gui::ViewProviderGeometryObject::unsetEdit(ModNum);
    }
}

void ViewProviderPartExt::updateVisual(const TopoDS_Shape& inputShape)
{
    // Clear selection
    Gui::SoSelectionElementAction saction(Gui::SoSelectionElementAction::None);
    saction.apply(this->faceset);
    saction.apply(this->lineset);
    saction.apply(this->nodeset);

    // Clear highlighting
    Gui::SoHighlightElementAction haction;
    haction.apply(this->faceset);
    haction.apply(this->lineset);
    haction.apply(this->nodeset);

    TopoDS_Shape cShape(inputShape);
    if (cShape.IsNull()) {
        coords  ->point      .setNum(0);
        norm    ->vector     .setNum(0);
        faceset ->coordIndex .setNum(0);
        faceset ->partIndex  .setNum(0);
        lineset ->coordIndex .setNum(0);
        nodeset ->startIndex .setValue(0);
        VisualTouched = false;
        return;
    }

    // time measurement and book keeping
    Base::TimeInfo start_time;
    int numTriangles=0,numNodes=0,numNorms=0,numFaces=0,numEdges=0,numLines=0;
    std::set<int> faceEdges;

    try {
        // calculating the deflection value
        Bnd_Box bounds;
        BRepBndLib::Add(cShape, bounds);
        bounds.SetGap(0.0);
        Standard_Real xMin, yMin, zMin, xMax, yMax, zMax;
        bounds.Get(xMin, yMin, zMin, xMax, yMax, zMax);
        Standard_Real deflection = ((xMax-xMin)+(yMax-yMin)+(zMax-zMin))/300.0 *
            Deviation.getValue();

        // create or use the mesh on the data structure
#if OCC_VERSION_HEX >= 0x060600
        Standard_Real AngDeflectionRads = AngularDeflection.getValue() / 180.0 * M_PI;
        BRepMesh_IncrementalMesh(cShape,deflection,Standard_False,
                AngDeflectionRads,Standard_True);
#else
        BRepMesh_IncrementalMesh(cShape,deflection);
#endif
        // We must reset the location here because the transformation data
        // are set in the placement property
        TopLoc_Location aLoc;
        cShape.Location(aLoc);

        // count triangles and nodes in the mesh
        TopExp_Explorer Ex;
        for (Ex.Init(cShape,TopAbs_FACE);Ex.More();Ex.Next()) {
            Handle (Poly_Triangulation) mesh = BRep_Tool::Triangulation(TopoDS::Face(Ex.Current()), aLoc);
            // Note: we must also count empty faces
            if (!mesh.IsNull()) {
                numTriangles += mesh->NbTriangles();
                numNodes     += mesh->NbNodes();
                numNorms     += mesh->NbNodes();
            }

            TopExp_Explorer xp;
            for (xp.Init(Ex.Current(),TopAbs_EDGE);xp.More();xp.Next())
                faceEdges.insert(xp.Current().HashCode(INT_MAX));
            numFaces++;
        }

        // get an indexed map of edges
        TopTools_IndexedMapOfShape edgeMap;
        TopExp::MapShapes(cShape, TopAbs_EDGE, edgeMap);

         // key is the edge number, value the coord indexes. This is needed to keep the same order as the edges.
        std::map<int, std::vector<int32_t> > lineSetMap;
        std::set<int>          edgeIdxSet;
        std::vector<int32_t>   edgeVector;

        // count and index the edges
        for (int i=1; i <= edgeMap.Extent(); i++) {
            edgeIdxSet.insert(i);
            numEdges++;

            const TopoDS_Edge& aEdge = TopoDS::Edge(edgeMap(i));
            TopLoc_Location aLoc;

            // handling of the free edge that are not associated to a face
            // Note: The assumption that if for an edge BRep_Tool::Polygon3D
            // returns a valid object is wrong. This e.g. happens for ruled
            // surfaces which gets created by two edges or wires.
            // So, we have to store the hashes of the edges associated to a face.
            // If the hash of a given edge is not in this list we know it's really
            // a free edge.
            int hash = aEdge.HashCode(INT_MAX);
            if (faceEdges.find(hash) == faceEdges.end()) {
                Handle(Poly_Polygon3D) aPoly = BRep_Tool::Polygon3D(aEdge, aLoc);
                if (!aPoly.IsNull()) {
                    int nbNodesInEdge = aPoly->NbNodes();
                    numNodes += nbNodesInEdge;
                }
            }
        }

        // handling of the vertices
        TopTools_IndexedMapOfShape vertexMap;
        TopExp::MapShapes(cShape, TopAbs_VERTEX, vertexMap);
        numNodes += vertexMap.Extent();

        // create memory for the nodes and indexes
        coords  ->point      .setNum(numNodes);
        norm    ->vector     .setNum(numNorms);
        faceset ->coordIndex .setNum(numTriangles*4);
        faceset ->partIndex  .setNum(numFaces);
        // get the raw memory for fast fill up
        SbVec3f* verts = coords  ->point       .startEditing();
        SbVec3f* norms = norm    ->vector      .startEditing();
        int32_t* index = faceset ->coordIndex  .startEditing();
        int32_t* parts = faceset ->partIndex   .startEditing();

        // preset the normal vector with null vector
        for (int i=0;i < numNorms;i++)
            norms[i]= SbVec3f(0.0,0.0,0.0);

        int ii = 0,faceNodeOffset=0,faceTriaOffset=0;
        for (Ex.Init(cShape, TopAbs_FACE); Ex.More(); Ex.Next(),ii++) {
            TopLoc_Location aLoc;
            const TopoDS_Face &actFace = TopoDS::Face(Ex.Current());
            // get the mesh of the shape
            Handle (Poly_Triangulation) mesh = BRep_Tool::Triangulation(actFace,aLoc);
            if (mesh.IsNull()) continue;

            // getting the transformation of the shape/face
            gp_Trsf myTransf;
            Standard_Boolean identity = true;
            if (!aLoc.IsIdentity()) {
                identity = false;
                myTransf = aLoc.Transformation();
            }

            // getting size of node and triangle array of this face
            int nbNodesInFace = mesh->NbNodes();
            int nbTriInFace   = mesh->NbTriangles();
            // check orientation
            TopAbs_Orientation orient = actFace.Orientation();


            // cycling through the poly mesh
            const Poly_Array1OfTriangle& Triangles = mesh->Triangles();
            const TColgp_Array1OfPnt& Nodes = mesh->Nodes();
            TColgp_Array1OfDir Normals (Nodes.Lower(), Nodes.Upper());
            GetNormals(actFace, mesh, Normals);
            
            for (int g=1;g<=nbTriInFace;g++) {
                // Get the triangle
                Standard_Integer N1,N2,N3;
                Triangles(g).Get(N1,N2,N3);

                // change orientation of the triangle if the face is reversed
                if ( orient != TopAbs_FORWARD ) {
                    Standard_Integer tmp = N1;
                    N1 = N2;
                    N2 = tmp;
                }

                // get the 3 points of this triangle
                gp_Pnt V1(Nodes(N1)), V2(Nodes(N2)), V3(Nodes(N3));

                // get the 3 normals of this triangle
                gp_Dir NV1(Normals(N1)), NV2(Normals(N2)), NV3(Normals(N3));                

                // transform the vertices and normals to the place of the face
                if(!identity) {
                    V1.Transform(myTransf);
                    V2.Transform(myTransf);
                    V3.Transform(myTransf);
                    NV1.Transform(myTransf);
                    NV2.Transform(myTransf);
                    NV3.Transform(myTransf);
                }

                // add the normals for all points of this triangle
                norms[faceNodeOffset+N1-1] += SbVec3f(NV1.X(),NV1.Y(),NV1.Z());
                norms[faceNodeOffset+N2-1] += SbVec3f(NV2.X(),NV2.Y(),NV2.Z());
                norms[faceNodeOffset+N3-1] += SbVec3f(NV3.X(),NV3.Y(),NV3.Z());

                // set the vertices
                verts[faceNodeOffset+N1-1].setValue((float)(V1.X()),(float)(V1.Y()),(float)(V1.Z()));
                verts[faceNodeOffset+N2-1].setValue((float)(V2.X()),(float)(V2.Y()),(float)(V2.Z()));
                verts[faceNodeOffset+N3-1].setValue((float)(V3.X()),(float)(V3.Y()),(float)(V3.Z()));

                // set the index vector with the 3 point indexes and the end delimiter
                index[faceTriaOffset*4+4*(g-1)]   = faceNodeOffset+N1-1;
                index[faceTriaOffset*4+4*(g-1)+1] = faceNodeOffset+N2-1;
                index[faceTriaOffset*4+4*(g-1)+2] = faceNodeOffset+N3-1;
                index[faceTriaOffset*4+4*(g-1)+3] = SO_END_FACE_INDEX;
            }

            parts[ii] = nbTriInFace; // new part

            // handling the edges lying on this face
            TopExp_Explorer Exp;
            for(Exp.Init(actFace,TopAbs_EDGE);Exp.More();Exp.Next()) {
                const TopoDS_Edge &curEdge = TopoDS::Edge(Exp.Current());
                // get the overall index of this edge
                int edgeIndex = edgeMap.FindIndex(curEdge);
                edgeVector.push_back((int32_t)edgeIndex-1);
                // already processed this index ?
                if (edgeIdxSet.find(edgeIndex)!=edgeIdxSet.end()) {
                    
                    // this holds the indices of the edge's triangulation to the current polygon
                    Handle(Poly_PolygonOnTriangulation) aPoly = BRep_Tool::PolygonOnTriangulation(curEdge, mesh, aLoc);
                    if (aPoly.IsNull())
                        continue; // polygon does not exist
                    
                    // getting the indexes of the edge polygon
                    const TColStd_Array1OfInteger& indices = aPoly->Nodes();
                    for (Standard_Integer i=indices.Lower();i <= indices.Upper();i++) {
                        int nodeIndex = indices(i);
                        int index = faceNodeOffset+nodeIndex-1;
                        lineSetMap[edgeIndex].push_back(index);

                        // usually the coordinates for this edge are already set by the
                        // triangles of the face this edge belongs to. However, there are
                        // rare cases where some points are only referenced by the polygon
                        // but not by any triangle. Thus, we must apply the coordinates to
                        // make sure that everything is properly set.
                        gp_Pnt p(Nodes(nodeIndex));
                        if (!identity)
                            p.Transform(myTransf);
                        verts[index].setValue((float)(p.X()),(float)(p.Y()),(float)(p.Z()));
                    }

                    // remove the handled edge index from the set
                    edgeIdxSet.erase(edgeIndex);
                }
            }

            edgeVector.push_back(-1);
            
            // counting up the per Face offsets
            faceNodeOffset += nbNodesInFace;
            faceTriaOffset += nbTriInFace;
        }

        // handling of the free edges
        for (int i=1; i <= edgeMap.Extent(); i++) {
            const TopoDS_Edge& aEdge = TopoDS::Edge(edgeMap(i));
            Standard_Boolean identity = true;
            gp_Trsf myTransf;
            TopLoc_Location aLoc;

            // handling of the free edge that are not associated to a face
            int hash = aEdge.HashCode(INT_MAX);
            if (faceEdges.find(hash) == faceEdges.end()) {
                Handle(Poly_Polygon3D) aPoly = BRep_Tool::Polygon3D(aEdge, aLoc);
                if (!aPoly.IsNull()) {
                    if (!aLoc.IsIdentity()) {
                        identity = false;
                        myTransf = aLoc.Transformation();
                    }

                    const TColgp_Array1OfPnt& aNodes = aPoly->Nodes();
                    int nbNodesInEdge = aPoly->NbNodes();

                    gp_Pnt pnt;
                    for (Standard_Integer j=1;j <= nbNodesInEdge;j++) {
                        pnt = aNodes(j);
                        if (!identity)
                            pnt.Transform(myTransf);
                        int index = faceNodeOffset+j-1;
                        verts[index].setValue((float)(pnt.X()),(float)(pnt.Y()),(float)(pnt.Z()));
                        lineSetMap[i].push_back(index);
                    }

                    faceNodeOffset += nbNodesInEdge;
                }
            }
        }

        nodeset->startIndex.setValue(faceNodeOffset);
        for (int i=0; i<vertexMap.Extent(); i++) {
            const TopoDS_Vertex& aVertex = TopoDS::Vertex(vertexMap(i+1));
            gp_Pnt pnt = BRep_Tool::Pnt(aVertex);
            verts[faceNodeOffset+i].setValue((float)(pnt.X()),(float)(pnt.Y()),(float)(pnt.Z()));
        }

        // normalize all normals 
        for (int i = 0; i< numNorms ;i++)
            norms[i].normalize();
        
        std::vector<int32_t> lineSetCoords;
        for (std::map<int, std::vector<int32_t> >::iterator it = lineSetMap.begin(); it != lineSetMap.end(); ++it) {
            lineSetCoords.insert(lineSetCoords.end(), it->second.begin(), it->second.end());
            lineSetCoords.push_back(-1);
        }

        // preset the index vector size
        numLines =  lineSetCoords.size();
        lineset ->coordIndex .setNum(numLines);
        int32_t* lines = lineset ->coordIndex  .startEditing();

        int l=0;
        for (std::vector<int32_t>::const_iterator it=lineSetCoords.begin();it!=lineSetCoords.end();++it,l++)
            lines[l] = *it;

        // end the editing of the nodes
        coords  ->point       .finishEditing();
        norm    ->vector      .finishEditing();
        faceset ->coordIndex  .finishEditing();
        faceset ->partIndex   .finishEditing();
        lineset ->coordIndex  .finishEditing();
    }
    catch (...) {
        Base::Console().Error("Cannot compute Inventor representation for the shape of %s.\n",pcObject->getNameInDocument());
    }

#   ifdef FC_DEBUG
        // printing some informations
        Base::Console().Log("ViewProvider update time: %f s\n",Base::TimeInfo::diffTimeF(start_time,Base::TimeInfo()));
        Base::Console().Log("Shape tria info: Faces:%d Edges:%d Nodes:%d Triangles:%d IdxVec:%d\n",numFaces,numEdges,numNodes,numTriangles,numLines);
#   endif
    VisualTouched = false;
}
