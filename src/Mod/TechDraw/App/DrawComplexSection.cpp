/***************************************************************************
 *   Copyright (c) 2022 WandererFan <wandererfan@gmail.com>                *
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
//DrawComplexSection processing overview
//for Strategy = Offset, DCS is much the same as DVS
//for Strategy = Aligned, there are many differences

//execute
//    sectionExec(getShapeToCut()*)

//sectionExec
//    makeSectionCut(baseShape)

//makeSectionCut (separate thread)
//    note that the section cut is not required for Aligned strategy,
//    but it is useful for debugging
//    m_cuttingTool = makeCuttingTool* (DVSTool.brep)
//    m_cutPieces = (baseShape - m_cuttingTool) (DVSCutPieces.brep)

//onSectionCutFinished
//    m_preparedShape = prepareShape(m_cutPieces)* - centered, scaled, rotated
//    geometryObject = DVP::buildGeometryObject(m_preparedShape)  (HLR)

//postHlrTasks
//    faceIntersections = findSectionPlaneIntersections
//    m_sectionTopoDSFaces = alignSectionFaces(faceIntersections)
//    m_tdSectionFaces = makeTDSectionFaces(m_sectionTopoDSFaces)

//* for Aligned, we use a different ShapeToCut, as the standard one will
//  cause many coincident face problems later
//* the cutting tool is built up from the profile, instead of the simple plane in DVS
//* for Aligned, preparing the shape is much different than Offset or DVS
//    - most of the work is done in makeAlignedPieces
//    - for each segment of the profile, make a cutting tool, then get the boolean
//      intersection of the tool and the shape to cut
//    - align and distribute the intersections along an "effective" section plane
//      which is a flattened version of the profile

#include "PreCompiled.h"

#ifndef _PreComp_
#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgo_NormalProjection.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepGProp.hxx>
#include <BRepLProp_SLProps.hxx>
#include <BRepLib.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeHalfSpace.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <Bnd_Box.hxx>
#include <Bnd_OBB.hxx>
#include <GProp_GProps.hxx>
#include <Geom_Plane.hxx>
#include <ShapeExtend_WireData.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Vertex.hxx>
#include <gp_Ax2.hxx>
#include <gp_Ax3.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#endif

#define _USE_MATH_DEFINES
#include <cmath>

#include <chrono>
#include <sstream>

#include <App/Application.h>
#include <App/Document.h>
#include <App/Material.h>
#include <Base/BoundBox.h>
#include <Base/Console.h>
#include <Base/Exception.h>
#include <Base/FileInfo.h>
#include <Base/Interpreter.h>
#include <Base/Parameter.h>

#include <Mod/Part/App/PartFeature.h>

#include "DrawComplexSection.h"
#include "DrawUtil.h"
#include "GeometryObject.h"

using namespace TechDraw;
using namespace std;
using DU = DrawUtil;

//class to store geometry of points where the section line changes direction
ChangePoint::ChangePoint(QPointF location, QPointF preDirection, QPointF postDirection)
{
    m_location = location;
    m_preDirection = preDirection;
    m_postDirection = postDirection;
}

ChangePoint::ChangePoint(gp_Pnt location, gp_Dir preDirection, gp_Dir postDirection)
{
    m_location.setX(location.X());
    m_location.setY(location.Y());
    m_preDirection.setX(preDirection.X());
    m_preDirection.setY(preDirection.Y());
    m_postDirection.setX(postDirection.X());
    m_postDirection.setY(postDirection.Y());
}

void ChangePoint::scale(double scaleFactor)
{
    m_location = m_location * scaleFactor;
    m_preDirection = m_preDirection * scaleFactor;
    m_postDirection = m_postDirection * scaleFactor;
}

//===========================================================================
// DrawComplexSection
//===========================================================================

PROPERTY_SOURCE(TechDraw::DrawComplexSection, TechDraw::DrawViewSection)

const char *DrawComplexSection::ProjectionStrategyEnums[] = {"Offset", "Aligned", "NoParallel",
                                                             nullptr};

DrawComplexSection::DrawComplexSection()
    : m_toolFaceShape(TopoDS_Shape()), m_profileWire(TopoDS_Wire())
{
    static const char *fgroup = "Cutting Tool";

    ADD_PROPERTY_TYPE(CuttingToolWireObject, (nullptr), fgroup, App::Prop_None,
                      "A sketch that describes the cutting tool");
    CuttingToolWireObject.setScope(App::LinkScope::Global);
    ProjectionStrategy.setEnums(ProjectionStrategyEnums);
    ADD_PROPERTY_TYPE(ProjectionStrategy, ((long)0), fgroup, App::Prop_None,
                      "Make a single cut, or use the profile in pieces");
}

TopoDS_Shape DrawComplexSection::getShapeToCut()
{
    //    Base::Console().Message("DCS::getShapeToCut()\n");
    App::DocumentObject *base = BaseView.getValue();
    TopoDS_Shape shapeToCut;
    if (base && base == this) {
        shapeToCut = getSourceShape();
        if (FuseBeforeCut.getValue()) {
            shapeToCut = getSourceShapeFused();
        }
        return shapeToCut;
    }
    if (!base
        || !base->getTypeId().isDerivedFrom(
            TechDraw::DrawViewPart::getClassTypeId())) {//is second clause necessary?
        //Complex section is based on 3d objects, need to get our own shapes since we can't ask a dvp
        shapeToCut = getSourceShape();
        if (FuseBeforeCut.getValue()) {
            shapeToCut = getSourceShapeFused();
        }
        return shapeToCut;
    }
    //complex section is based on a DVP, so get the shape the normal way
    return DrawViewSection::getShapeToCut();
}

void DrawComplexSection::makeSectionCut(TopoDS_Shape &baseShape)
{
    //    Base::Console().Message("DCS::makeSectionCut() - %s\n", getNameInDocument(), baseShape.IsNull());
    if (ProjectionStrategy.getValue() == 0) {
        //Offset. Use regular section behaviour
        DrawViewSection::makeSectionCut(baseShape);
        return;
    }
    //Aligned strategy
    if (debugSection()) {
        //only useful for debugging with Aligned strategy
        DrawViewSection::makeSectionCut(baseShape);
    }
    return;
}

TopoDS_Shape DrawComplexSection::makeCuttingTool(double dMax)
{
    //    Base::Console().Message("DCS::makeCuttingTool()\n");
    App::DocumentObject *toolObj = CuttingToolWireObject.getValue();
    if (!isProfileObject(toolObj)) {
        return TopoDS_Shape();
    }
    TopoDS_Wire profileWire = makeProfileWire(toolObj);
    m_profileWire = profileWire;
    BRepBuilderAPI_Copy BuilderCopy(profileWire);
    m_profileWire = TopoDS::Wire(BuilderCopy.Shape());
    if (debugSection()) {
        //the nose to tail version of the profile
        BRepTools::Write(m_profileWire, "DCSProfileWire.brep");//debug
    }

    gp_Ax2 sectionCS = getSectionCS();
    gp_Dir gClosestBasis;//direction perpendicular to profile & section normal
    bool isPositionOK = validateProfilePosition(profileWire, sectionCS, gClosestBasis);
    if (!isPositionOK) {
        //profile is not in a good position.  Result might not be right.
        Base::Console().Warning("DCS::makeCuttingTool - %s - profile is outside shape box\n",
                                getNameInDocument());
    }

    //move the profile wire to one side of the shape
    gp_Trsf mov;
    mov.SetTranslation(gp_Vec(gClosestBasis) * (-dMax));
    TopLoc_Location loc(mov);
    profileWire.Move(loc);

    gp_Vec extrudeDir(0.0, 0.0, 1.0);//arbitrary default
    if (BRep_Tool::IsClosed(profileWire)) {
        // Wire is closed, so make a face from it and extrude "vertically"
        BRepBuilderAPI_MakeFace mkFace(profileWire);
        TopoDS_Face toolFace = mkFace.Face();
        if (toolFace.IsNull()) {
            return TopoDS_Shape();
        }
        gp_Dir gpNormal = getFaceNormal(toolFace);
        extrudeDir = 2.0 * dMax * gpNormal;
        return BRepPrimAPI_MakePrism(toolFace, extrudeDir).Shape();
    }

    //if the wire is open we need to make a "face" from the wire by extruding it
    //in the direction of gClosestBasis , then extrude the face in the direction of the section normal
    m_toolFaceShape = extrudeWireToFace(profileWire, gClosestBasis, 2.0 * dMax);
    if (debugSection()) {
        BRepTools::Write(m_toolFaceShape, "DCSToolFaceShape.brep");//debug
    }
    extrudeDir = dMax * sectionCS.Direction();
    TopoDS_Shape roughTool = BRepPrimAPI_MakePrism(m_toolFaceShape, extrudeDir).Shape();
    if (roughTool.ShapeType() == TopAbs_COMPSOLID) {
        //Composite Solids do not cut well if they contain "solids" with no volume. This
        //happens if the profile has segments parallel to the extrude direction.
        //We need to disassemble it and only keep the real solids.
        BRep_Builder builder;
        TopoDS_Compound comp;
        builder.MakeCompound(comp);
        TopExp_Explorer expSolids(roughTool, TopAbs_SOLID);
        for (; expSolids.More(); expSolids.Next()) {
            TopoDS_Solid solid = TopoDS::Solid(expSolids.Current());
            GProp_GProps gprops;
            BRepGProp::VolumeProperties(solid, gprops);
            double volume = gprops.Mass();
            if (volume > EWTOLERANCE) {
                builder.Add(comp, solid);
            }
        }
        return comp;
    }

    return BRepPrimAPI_MakePrism(m_toolFaceShape, extrudeDir).Shape();
}

TopoDS_Shape DrawComplexSection::getShapeToPrepare() const
{
    if (ProjectionStrategy.getValue() == 0) {
        //Offset. Use regular section behaviour
        return DrawViewSection::getShapeToPrepare();
    }
    //Aligned strategy
    return m_saveShape;//the original input shape
}

//get the shape ready for projection and cut surface finding
TopoDS_Shape DrawComplexSection::prepareShape(const TopoDS_Shape &cutShape, double shapeSize)
{
    //    Base::Console().Message("DCS::prepareShape() - strat: %d\n", ProjectionStrategy.getValue());
    if (ProjectionStrategy.getValue() == 0) {
        //Offset. Use regular section behaviour
        return DrawViewSection::prepareShape(cutShape, shapeSize);
    }

    //"Aligned" projection (Aligned Section)
    TopoDS_Shape alignedResult = makeAlignedPieces(cutShape, m_toolFaceShape, shapeSize);

    if (alignedResult.IsNull()) {
        return TopoDS_Shape();
    }

    return scaleShape(alignedResult, getScale());
}

//for Aligned strategy, cut the rawShape by each segment of the tool
//TODO: this process should replace the "makeSectionCut" from DVS
TopoDS_Shape DrawComplexSection::makeAlignedPieces(const TopoDS_Shape &rawShape,
                                                   const TopoDS_Shape &toolFaceShape,
                                                   double extrudeDistance)
{
    //    Base::Console().Message("DCS::makeAlignedPieces()\n");
    std::vector<TopoDS_Shape> pieces;
    std::vector<double> pieceXSize;//size in sectionCS.XDirection (width)
    std::vector<double> pieceYSize;//size in sectionCS.Direction (depth)
    std::vector<double> pieceZSize;//size in sectionCS.YDirection (height)
    std::vector<double> pieceDirections;
    //make a "real" CS from the section projection CS
    gp_Ax3 alignedCS(gp_Pnt(0.0, 0.0, 0.0),
                     getSectionCS().YDirection(), //up and down
                     getSectionCS().XDirection());//left to right
    gp_Ax3 stdCS;                                 //OXYZ
    gp_Vec gProjectionUnit = gp_Vec(getSectionCS().Direction());

    //get a vector that describes the profile's orientation in paper space.
    gp_Vec gProfileVec = projectProfileWire(m_profileWire, gp_Ax3(getSectionCS()));
    if (fabs(gProfileVec.Dot(getProjectionCS().Direction()) == 1.0)) {
        Base::Console().Error(
            "DCS::makeAlignedPieces - %s - profile is parallel to SectionNormal\n",
            getNameInDocument());
        throw Base::RuntimeError("Profile orientation error");
    }

    //convert the profileVector with OXYZ.
    gp_Trsf xProfileOXYZ;
    gp_Ax3 OXYZ;
    xProfileOXYZ.SetTransformation(OXYZ, gp_Ax3(getSectionCS()));
    gp_Vec profileVecOXYZ = gProfileVec.Transformed(xProfileOXYZ);

    bool isVertical = true;
    if (fabs(profileVecOXYZ.Dot(gp::OY().Direction().XYZ())) != 1.0) {
        //profile is not parallel with stdY (paper space Up).
        //this test is not good enough for "vertical-ish" diagonal profiles
        isVertical = false;
    }

    double leftToRight = 1.0;//profile vector points to right, so we move to right
    if (profileVecOXYZ.Dot(gp_Vec(gp::OX().Direction().XYZ())) < 0.0) {
        //profileVec does not point towards stdX (right in paper space)
        leftToRight = -1.0;
    }
    double topToBottom = 1.0;//profile vector points to top, so we move to top
    if (profileVecOXYZ.Dot(gp_Vec(gp::OY().Direction().XYZ())) < 0.0) {
        //profileVec does not point towards stdY (up in paper space)
        topToBottom = -1.0;
    }

    gp_Vec rotateAxis = getSectionCS().Direction().Crossed(gProfileVec);


    //make a tool for each segment of the toolFaceShape and intersect it with the
    //raw shape
    TopExp_Explorer expFaces(toolFaceShape, TopAbs_FACE);
    for (int iPiece = 0; expFaces.More(); expFaces.Next(), iPiece++) {
        TopoDS_Face face = TopoDS::Face(expFaces.Current());
        gp_Vec segmentNormal = gp_Vec(getFaceNormal(face));
        if (!showSegment(gp_Dir(segmentNormal))) {
            //skip this segment of the profile
            continue;
        }
        //we only want to reverse the segment normal if it is not perpendicular to section normal
        if (segmentNormal.Dot(-gProjectionUnit) != 0.0
            && segmentNormal.Angle(gProjectionUnit) <= M_PI_2) {
            segmentNormal.Reverse();
        }

        gp_Vec extrudeDir = segmentNormal * extrudeDistance;
        BRepPrimAPI_MakePrism mkPrism(face, extrudeDir);
        TopoDS_Shape segmentTool = mkPrism.Shape();
        TopoDS_Shape intersect = shapeShapeIntersect(segmentTool, rawShape);

        gp_Pnt pieceCentroid = findCentroid(intersect);
        double faceAngle =
            gp_Vec(getSectionCS().Direction().Reversed()).AngleWithRef(segmentNormal, rotateAxis);

        //move intersection shape to the origin
        gp_Trsf xPieceCenter;
        xPieceCenter.SetTranslation(gp_Vec(findCentroid(intersect).XYZ()) * -1.0);
        BRepBuilderAPI_Transform mkTransXLate(intersect, xPieceCenter, true);
        TopoDS_Shape pieceCentered = mkTransXLate.Shape();

        //rotate the intersection so interesting face is aligned with paper plane
        pieceCentroid = findCentroid(pieceCentered);
        gp_Ax1 faceAxis(gp_Pnt(0.0, 0.0, 0.0), rotateAxis);
        gp_Ax3 pieceCS;//XYZ tipped so face is aligned with sectionCS
        pieceCS.Rotate(faceAxis, faceAngle);
        gp_Trsf xPieceRotate;
        xPieceRotate.SetTransformation(stdCS, pieceCS);
        BRepBuilderAPI_Transform mkTransRotate(pieceCentered, xPieceRotate, true);
        TopoDS_Shape pieceRotated = mkTransRotate.Shape();

        //align a copy of the piece with OXYZ so we can use bounding box to get
        //width, depth, height of the piece. We copy the piece so the transformation
        //does not affect the original.
        BRepBuilderAPI_Copy BuilderPieceCopy(pieceRotated);
        TopoDS_Shape copyPieceRotatedShape = BuilderPieceCopy.Shape();
        gp_Trsf xPieceAlign;
        xPieceAlign.SetTransformation(stdCS, alignedCS);
        BRepBuilderAPI_Transform mkTransAlign(copyPieceRotatedShape, xPieceAlign);
        TopoDS_Shape pieceAligned = mkTransAlign.Shape();

        Bnd_Box shapeBox;
        shapeBox.SetGap(0.0);
        BRepBndLib::AddOptimal(pieceAligned, shapeBox);
        double xMin = 0, xMax = 0, yMin = 0, yMax = 0, zMin = 0, zMax = 0;
        shapeBox.Get(xMin, yMin, zMin, xMax, yMax, zMax);
        double pieceWidth(xMax - xMin);
        double pieceDepth(yMax - yMin);
        double pieceHeight(zMax - zMin);
        pieceXSize.push_back(pieceWidth);
        pieceYSize.push_back(pieceDepth);
        pieceZSize.push_back(pieceHeight);

        //now we need to move the piece so that the interesting face is coincident
        //with the paper plane
        gp_Vec depthVector(gp::OY().Direction().XYZ() * pieceDepth / 2.0);//move "back"
        //only aligned to paper plane
        gp_Vec netDisplacement = -1.0 * gp_Vec(findCentroid(pieceAligned).XYZ()) + depthVector;
        //if we are going to space along X, we need to bring the pieces into alignment
        //with the XY plane.
        gp_Vec xyDisplacement = isVertical ? gp_Vec(0.0, 0.0, 0.0) : gp_Vec(gp::OZ().Direction());
        xyDisplacement = xyDisplacement * topToBottom * pieceHeight / 2.0;
        netDisplacement = netDisplacement + xyDisplacement;

        gp_Trsf xPieceDisplace;
        xPieceDisplace.SetTranslation(netDisplacement);
        BRepBuilderAPI_Transform mkTransDisplace(pieceAligned, xPieceDisplace, true);
        TopoDS_Shape pieceDisplaced = mkTransDisplace.Shape();
        //piece is now centered on X, aligned with XZ plane (which will be the effective
        //cutting plane)
        pieces.push_back(pieceDisplaced);
    }

    if (pieces.empty()) {
        return TopoDS_Compound();
    }

    int pieceCount = pieces.size();
    if (pieceCount < 2) {
        //no need to space out the pieces
        return TopoDS::Compound(pieces.front());
    }

    //space the pieces "horizontally" (stdX) or "vertically" (stdZ)
    double movementReverser = isVertical ? topToBottom : leftToRight;
    //TODO: non-cardinal profiles!
    gp_Vec movementAxis = isVertical ? gp_Vec(gp::OZ().Direction()) : gp_Vec(gp::OX().Direction());
    gp_Vec gMovementVector = movementAxis * movementReverser;

    int stopAt = pieces.size();
    double distanceToMove = 0.0;
    for (int iPiece = 0; iPiece < stopAt; iPiece++) {
        double pieceSize = pieceXSize.at(iPiece);
        if (isVertical) {
            pieceSize = pieceZSize.at(iPiece);
        }
        double myDistanceToMove = distanceToMove + pieceSize / 2.0;
        gp_Trsf xPieceDistribute;
        xPieceDistribute.SetTranslation(gMovementVector * myDistanceToMove);
        BRepBuilderAPI_Transform mkTransDistribute(pieces.at(iPiece), xPieceDistribute, true);
        pieces.at(iPiece) = mkTransDistribute.Shape();
        distanceToMove += pieceSize;
    }

    //make a compound of the aligned pieces
    BRep_Builder builder;
    TopoDS_Compound comp;
    builder.MakeCompound(comp);
    for (auto &piece : pieces) { builder.Add(comp, piece); }

    //center the compound along SectionCS XDirection
    Base::Vector3d centerVector = DU::toVector3d(gMovementVector) * distanceToMove / -2.0;
    TopoDS_Shape centeredCompound = moveShape(comp, centerVector);
    if (debugSection()) {
        BRepTools::Write(centeredCompound, "DCSfinCenteredCompound.brep");//debug
    }
    //realign with SectionCS
    gp_Trsf xPieceAlign;
    xPieceAlign.SetTransformation(alignedCS, stdCS);
    BRepBuilderAPI_Transform mkTransAlign(centeredCompound, xPieceAlign);
    TopoDS_Shape alignedCompound = mkTransAlign.Shape();

    if (debugSection()) {
        BRepTools::Write(alignedCompound, "DCSAlignedResult.brep");//debug
    }

    return alignedCompound;
}

//! tries to find the intersection faces of the cut shape and the cutting tool.
//! Aligned complex sections need to intersect the final cut shape (which in this
//! case is a compound of individual cuts) with the "effective" (flattened) section plane.
//! Profiles containing curves need special handling.
TopoDS_Compound
DrawComplexSection::findSectionPlaneIntersections(const TopoDS_Shape &shapeToIntersect)
{
    //    Base::Console().Message("DCS::findSectionPlaneIntersections() - %s\n", getNameInDocument());
    if (shapeToIntersect.IsNull()) {
        // this shouldn't happen
        Base::Console().Warning("DCS::findSectionPlaneInter - %s - cut shape is Null\n",
                                getNameInDocument());
        return TopoDS_Compound();
    }
    if (ProjectionStrategy.getValue() == 0) {//Offset
        return singleToolIntersections(shapeToIntersect);
    }

    return piecewiseToolIntersections(shapeToIntersect);
}

//Intersect cutShape with each segment of the cutting tool
TopoDS_Compound DrawComplexSection::singleToolIntersections(const TopoDS_Shape &cutShape)
{
    //    Base::Console().Message("DCS::singleToolInterSections()\n");
    App::DocumentObject *toolObj = CuttingToolWireObject.getValue();
    if (!isLinearProfile(toolObj)) {
        //TODO: special handling here
        //        Base::Console().Message("DCS::singleToolIntersection - profile has curves\n");
    }

    BRep_Builder builder;
    TopoDS_Compound result;
    builder.MakeCompound(result);

    if (debugSection()) {
        BRepTools::Write(cutShape, "DCSOffsetCutShape.brep");              //debug
        BRepTools::Write(m_toolFaceShape, "DCSOffsetCuttingToolFace.brep");//debug
    }

    if (m_toolFaceShape.IsNull()) {
        return result;
    }

    TopExp_Explorer expFaces(cutShape, TopAbs_FACE);
    for (; expFaces.More(); expFaces.Next()) {
        TopoDS_Face face = TopoDS::Face(expFaces.Current());
        if (!boxesIntersect(face, m_toolFaceShape)) {
            continue;
        }
        std::vector<TopoDS_Face> commonFaces = faceShapeIntersect(face, m_toolFaceShape);
        for (auto &cFace : commonFaces) { builder.Add(result, cFace); }
    }
    return result;
}

//Intersect cutShape with the effective (flattened) cutting plane to generate cut surface faces
TopoDS_Compound DrawComplexSection::piecewiseToolIntersections(const TopoDS_Shape &cutShape)
{
    //    Base::Console().Message("DCS::piecewiseToolIntersections()\n");
    BRep_Builder builder;
    TopoDS_Compound result;
    builder.MakeCompound(result);

    App::DocumentObject *toolObj = CuttingToolWireObject.getValue();
    if (!isLinearProfile(toolObj)) {
        //TODO: special handling here
        //        Base::Console().Message("DCS::pieceWiseToolIntersection - profile has curves\n");
    }

    gp_Pln effectivePlane = getSectionPlane();
    //piecewise result can be much wider than the shape itself, so we use an
    //infinite face.
    BRepBuilderAPI_MakeFace mkFace(effectivePlane, -Precision::Infinite(), Precision::Infinite(),
                                   -Precision::Infinite(), Precision::Infinite());
    TopoDS_Face cuttingFace = mkFace.Face();

    TopExp_Explorer expFaces(cutShape, TopAbs_FACE);
    for (; expFaces.More(); expFaces.Next()) {
        TopoDS_Face face = TopoDS::Face(expFaces.Current());
        if (!boxesIntersect(face, cuttingFace)) {
            continue;
        }
        std::vector<TopoDS_Face> commonFaces = faceShapeIntersect(face, cuttingFace);
        for (auto &cFace : commonFaces) { builder.Add(result, cFace); }
    }
    if (debugSection()) {
        BRepTools::Write(cuttingFace, "DCSAlignedCuttingFace.brep");  //debug
        BRepTools::Write(cutShape, "DCSAlignedCutShape.brep");        //debug
        BRepTools::Write(result, "DCSAlignedIntersectionResult.brep");//debug
    }
    return result;
}

TopoDS_Compound DrawComplexSection::alignSectionFaces(TopoDS_Shape faceIntersections)
{
    //    Base::Console().Message("DCS::alignSectionFaces()\n");
    if (ProjectionStrategy.getValue() == 0) {
        //Offset. Use regular section behaviour
        return DrawViewSection::alignSectionFaces(faceIntersections);
    }

    return TopoDS::Compound(mapToPage(faceIntersections));
}

TopoDS_Shape DrawComplexSection::getShapeToIntersect()
{
    if (ProjectionStrategy.getValue() == 0) {//Offset
        return DrawViewSection::getShapeToIntersect();
    }
    //Aligned
    return m_preparedShape;
}

TopoDS_Wire DrawComplexSection::makeProfileWire(App::DocumentObject *toolObj)
{
    TopoDS_Shape toolShape = Part::Feature::getShape(toolObj);
    if (toolShape.IsNull()) {
        return TopoDS_Wire();
    }

    TopoDS_Wire profileWire;
    if (toolShape.ShapeType() == TopAbs_WIRE) {
        profileWire = makeNoseToTailWire(TopoDS::Wire(toolShape));
    }
    else {//we have already checked that the shape is a wire or an edge in isProfileObject
        TopoDS_Edge edge = TopoDS::Edge(toolShape);
        profileWire = BRepBuilderAPI_MakeWire(edge).Wire();
    }
    return profileWire;
}

//methods related to section line

//project the profile onto the paper and convert to the working CS
gp_Dir DrawComplexSection::projectProfileWire(TopoDS_Wire profileWire, gp_Ax3 paperCS)
{
    //    Base::Console().Message("DCS::projectProfileWire()\n");
    gp_Pln plane(paperCS);
    TopoDS_Face paper = BRepBuilderAPI_MakeFace(plane);
    BRepAlgo_NormalProjection projector(paper);
    projector.Add(profileWire);
    projector.Build();
    TopoDS_Shape projectedShape = projector.Projection();

    TopoDS_Edge projectedSegment;
    //we only need 1 projected edge to determine direction
    TopExp_Explorer expEdges(projectedShape, TopAbs_EDGE);
    for (; expEdges.More(); expEdges.Next()) {
        projectedSegment = TopoDS::Edge(expEdges.Current());
        break;
    }
    if (debugSection()) {
        BRepTools::Write(projectedSegment, "DCSprojectedSegment.brep");//debug
    }
    if (projectedSegment.IsNull()) {
        Base::Console().Warning("DCS::projectProfileWire - projection of profile failed\n");
        return gp_Dir(1.0, 0.0, 0.0);
    }
    gp_Pnt gpProfileFirst = BRep_Tool::Pnt(TopExp::FirstVertex(projectedSegment));
    gp_Pnt gpProfileLast = BRep_Tool::Pnt(TopExp::LastVertex(projectedSegment));
    gp_Vec gProfileVec(gpProfileFirst, gpProfileLast);
    gProfileVec.Normalize();
    return gp_Dir(gProfileVec);
}

//make drawable td geometry for section line
BaseGeomPtrVector DrawComplexSection::makeSectionLineGeometry()
{
    //    Base::Console().Message("DCS::makeSectionLineGeometry()\n");
    BaseGeomPtrVector result;
    DrawViewPart *baseDvp = dynamic_cast<DrawViewPart *>(BaseView.getValue());
    if (baseDvp) {
        TopoDS_Wire lineWire = makeSectionLineWire();
        TopoDS_Shape projectedWire =
            GeometryObject::projectSimpleShape(lineWire, baseDvp->getProjectionCS());
        TopExp_Explorer expEdges(projectedWire, TopAbs_EDGE);
        for (; expEdges.More(); expEdges.Next()) {
            BaseGeomPtr edge = BaseGeom::baseFactory(TopoDS::Edge(expEdges.Current()));
            result.push_back(edge);
        }
    }
    return result;
}

//get the end points of the section wire
std::pair<Base::Vector3d, Base::Vector3d> DrawComplexSection::sectionLineEnds()
{
    //    Base::Console().Message("DCS::sectionLineEnds()\n");
    std::pair<Base::Vector3d, Base::Vector3d> result;
    TopoDS_Wire lineWire = makeSectionLineWire();
    if (lineWire.IsNull()) {
        return result;
    }

    TopoDS_Vertex vFirst, vLast;
    TopExp::Vertices(lineWire, vFirst, vLast);
    gp_Pnt gpFirst = BRep_Tool::Pnt(vFirst);
    gp_Pnt gpLast = BRep_Tool::Pnt(vLast);
    Base::Vector3d first = Base::Vector3d(gpFirst.X(), gpFirst.Y(), gpFirst.Z());
    Base::Vector3d last = Base::Vector3d(gpLast.X(), gpLast.Y(), gpLast.Z());

    DrawViewPart *baseDvp = dynamic_cast<DrawViewPart *>(BaseView.getValue());
    if (baseDvp) {
        first = baseDvp->projectPoint(first);
        last = baseDvp->projectPoint(last);
    }
    result.first = first;
    result.second = last;
    return result;
}

//get directions of the section line arrows
std::pair<Base::Vector3d, Base::Vector3d> DrawComplexSection::sectionArrowDirs()
{
    //    Base::Console().Message("DCS::sectionArrowDirs()\n");
    std::pair<Base::Vector3d, Base::Vector3d> result;
    App::DocumentObject *toolObj = CuttingToolWireObject.getValue();
    TopoDS_Wire profileWire = makeProfileWire(toolObj);
    if (profileWire.IsNull()) {
        return result;
    }

    TopoDS_Vertex tvFirst, tvLast;
    TopExp::Vertices(profileWire, tvFirst, tvLast);
    gp_Pnt gpFirst = BRep_Tool::Pnt(tvFirst);
    gp_Pnt gpLast = BRep_Tool::Pnt(tvLast);
    gp_Vec gProfileVector = gp_Vec(gpLast.XYZ()) - gp_Vec(gpFirst.XYZ());
    gp_Vec gSectionNormal = gp_Vec(DU::togp_Dir(SectionNormal.getValue()));
    gp_Vec gExtrudeVector = (gSectionNormal.Crossed(gProfileVector)).Normalized();
    Base::Vector3d vClosestBasis = DrawUtil::closestBasis(gp_Dir(gExtrudeVector), getSectionCS());
    gp_Dir gExtrudeDir = gp_Dir(vClosestBasis.x, vClosestBasis.y, vClosestBasis.z);

    TopoDS_Shape toolFaceShape = extrudeWireToFace(profileWire, gExtrudeDir, 100.0);
    if (toolFaceShape.IsNull()) {
        return result;
    }

    std::vector<TopoDS_Face> faces;
    TopExp_Explorer expl(toolFaceShape, TopAbs_FACE);
    for (; expl.More(); expl.Next()) { faces.push_back(TopoDS::Face(expl.Current())); }

    gp_Vec gDir0 = gp_Vec(getFaceNormal(faces.front()));
    gp_Vec gDir1 = gp_Vec(getFaceNormal(faces.back()));
    if (gDir0.Dot(gSectionNormal) > 0.0) {
        //face normal is pointing generally in section normal direction, so we
        //want the reverse for a view direction
        gDir0.Reverse();
    }
    if (gDir1.Dot(gSectionNormal) > 0.0) {
        //face normal is pointing generally in section normal direction, so we
        //want the reverse for a view direction
        gDir1.Reverse();
    }

    //TODO: similar code elsewhere. Opportunity for reuse.
    Base::Vector3d vDir0 = DU::toVector3d(gDir0);
    Base::Vector3d vDir1 = DU::toVector3d(gDir1);
    vDir0.Normalize();
    vDir1.Normalize();
    DrawViewPart *baseDvp = dynamic_cast<DrawViewPart *>(BaseView.getValue());
    if (baseDvp) {
        vDir0 = baseDvp->projectPoint(vDir0, true);
        vDir1 = baseDvp->projectPoint(vDir1, true);
    }

    result.first = vDir0;
    result.second = vDir1;
    return result;
}

//make a wire suitable for projection on a base view
TopoDS_Wire DrawComplexSection::makeSectionLineWire()
{
    TopoDS_Wire lineWire;
    App::DocumentObject *toolObj = CuttingToolWireObject.getValue();
    DrawViewPart *baseDvp = dynamic_cast<DrawViewPart *>(BaseView.getValue());
    if (baseDvp) {
        Base::Vector3d centroid = baseDvp->getCurrentCentroid();
        TopoDS_Shape sTrans =
            TechDraw::moveShape(Part::Feature::getShape(toolObj), centroid * -1.0);
        TopoDS_Shape sScaled = TechDraw::scaleShape(sTrans, baseDvp->getScale());
        //we don't mirror the scaled shape here as it will be mirrored by the projection

        if (sScaled.ShapeType() == TopAbs_WIRE) {
            lineWire = makeNoseToTailWire(TopoDS::Wire(sScaled));
        }
        else if (sScaled.ShapeType() == TopAbs_EDGE) {
            TopoDS_Edge edge = TopoDS::Edge(sScaled);
            lineWire = BRepBuilderAPI_MakeWire(edge).Wire();
        }
        else {
            //probably can't happen as cut profile has been checked before this
            Base::Console().Message("DCS::makeSectionLineGeometry - profile is type: %d\n",
                                    sScaled.ShapeType());
            return TopoDS_Wire();
        }
    }
    return lineWire;
}

//find the points where the section line changes direction, and the direction
//of the profile before and after the point
ChangePointVector DrawComplexSection::getChangePointsFromSectionLine()
{
    //    Base::Console().Message("DCS::getChangePointsFromSectionLine()\n");
    ChangePointVector result;
    std::vector<gp_Pnt> allPoints;
    DrawViewPart *baseDvp = dynamic_cast<DrawViewPart *>(BaseView.getValue());
    if (baseDvp) {
        TopoDS_Wire lineWire = makeSectionLineWire();
        TopoDS_Shape projectedWire =
            GeometryObject::projectSimpleShape(lineWire, baseDvp->getProjectionCS());
        if (projectedWire.IsNull()) {
            return result;
        }
        //get UNIQUE points along the projected profile
        TopExp_Explorer expVertex(projectedWire, TopAbs_VERTEX);
        gp_Pnt previousPoint(Precision::Infinite(), Precision::Infinite(), Precision::Infinite());
        for (; expVertex.More(); expVertex.Next()) {
            TopoDS_Vertex vert = TopoDS::Vertex(expVertex.Current());
            gp_Pnt gPoint = BRep_Tool::Pnt(vert);
            if (gPoint.IsEqual(previousPoint, 2.0 * EWTOLERANCE)) {
                continue;
            }
            allPoints.push_back(gPoint);
            previousPoint = gPoint;
        }

        //make the intermediate marks
        for (size_t iPoint = 1; iPoint < allPoints.size() - 1; iPoint++) {
            gp_Pnt location = allPoints.at(iPoint);
            gp_Dir preDir = gp_Dir(allPoints.at(iPoint - 1).XYZ() - allPoints.at(iPoint).XYZ());
            gp_Dir postDir = gp_Dir(allPoints.at(iPoint + 1).XYZ() - allPoints.at(iPoint).XYZ());
            ChangePoint point(location, preDir, postDir);
            result.push_back(point);
        }

        //make start and end marks
        gp_Pnt location0 = allPoints.at(0);
        gp_Pnt location1 = allPoints.at(1);
        gp_Dir postDir = gp_Dir(location1.XYZ() - location0.XYZ());
        gp_Dir preDir = postDir.Reversed();
        ChangePoint startPoint(location0, preDir, postDir);
        result.push_back(startPoint);
        location0 = allPoints.at(allPoints.size() - 1);
        location1 = allPoints.at(allPoints.size() - 2);
        preDir = gp_Dir(location0.XYZ() - location1.XYZ());
        postDir = preDir.Reversed();
        ChangePoint endPoint(location0, preDir, postDir);
        result.push_back(endPoint);
    }
    return result;
}

gp_Ax2 DrawComplexSection::getCSFromBase(const std::string sectionName) const
{
    //    Base::Console().Message("DCS::getCSFromBase()\n");
    App::DocumentObject *base = BaseView.getValue();
    if (!base
        || !base->getTypeId().isDerivedFrom(
            TechDraw::DrawViewPart::getClassTypeId())) {//is second clause necessary?
        //if this DCS does not have a baseView, we must use the existing SectionCS
        return getSectionCS();
    }
    return DrawViewSection::getCSFromBase(sectionName);
}

//get the "effective" (flattened) section plane for Aligned and
//the regular sectionPlane for Offset.
gp_Pln DrawComplexSection::getSectionPlane() const
{
    if (ProjectionStrategy.getValue() == 0) {
        //Offset. Use regular section behaviour
        return DrawViewSection::getSectionPlane();
    }

    //"Aligned" projection (Aligned Section)
    //this is the same as DVS::getSectionPlane except that the plane origin is not the SectionOrigin
    Base::Vector3d vSectionNormal = SectionNormal.getValue();
    gp_Dir gSectionNormal(vSectionNormal.x, vSectionNormal.y, vSectionNormal.z);
    gp_Pnt gOrigin(0.0, 0.0, 0.0);
    gp_Ax3 gPlaneCS(gOrigin, gSectionNormal);

    return gp_Pln(gPlaneCS);
}

bool DrawComplexSection::isBaseValid() const
{
    App::DocumentObject *base = BaseView.getValue();
    if (!base) {
        //complex section is not based on an existing DVP
        return true;
    }
    if (!base->getTypeId().isDerivedFrom(TechDraw::DrawViewPart::getClassTypeId())) {
        //this is probably an error somewhere. the valid options are base = a DVP,
        //or no base
        return false;
    }
    //have a base and it is a DVP
    return true;
}

//if the profile is not nicely positioned within the vertical span of the shape, we might not overlap
//the shape after extrusion.  As long as the profile is within the extent of the shape in the
//extrude direction we should be ok. the extrude direction has to be perpendicular to the profile and SectionNormal
bool DrawComplexSection::validateProfilePosition(TopoDS_Wire profileWire, gp_Ax2 sectionCS,
                                                 gp_Dir &gClosestBasis) const
{
    //    Base::Console().Message("DCS::validateProfilePosition()\n");
    TopoDS_Vertex tvFirst, tvLast;
    TopExp::Vertices(profileWire, tvFirst, tvLast);
    gp_Pnt gpFirst = BRep_Tool::Pnt(tvFirst);//a position point for the wire
    gp_Pnt gpLast = BRep_Tool::Pnt(tvLast);
    gp_Vec gProfileVector = gp_Vec(gpLast.XYZ()) - gp_Vec(gpFirst.XYZ());

    //since bounding boxes are aligned with the cardinal directions, we need to find
    //the appropriate direction to use when validating the profile position
    gp_Vec gSectionVector = getSectionCS().Direction().Reversed();
    gp_Vec gExtrudeVector = gSectionVector.Crossed(gProfileVector);
    Base::Vector3d vClosestBasis = DrawUtil::closestBasis(gp_Dir(gExtrudeVector), sectionCS);
    gClosestBasis = gp_Dir(vClosestBasis.x, vClosestBasis.y, vClosestBasis.z);

    Bnd_Box shapeBox;
    shapeBox.SetGap(0.0);
    BRepBndLib::AddOptimal(m_saveShape, shapeBox);
    double xMin = 0, xMax = 0, yMin = 0, yMax = 0, zMin = 0, zMax = 0;
    shapeBox.Get(xMin, yMin, zMin, xMax, yMax, zMax);
    double spanLow = xMin;
    double spanHigh = xMax;
    double spanCheck = gpFirst.X();
    if (gClosestBasis.IsParallel(sectionCS.YDirection(), Precision::Angular())) {
        spanLow = yMin;
        spanHigh = yMax;
        spanCheck = gpFirst.Y();
    }
    else if (gClosestBasis.IsParallel(sectionCS.Direction(), Precision::Angular())) {
        spanLow = zMin;
        spanHigh = zMax;
        spanCheck = gpFirst.Z();
    }

    if (spanLow > spanCheck || spanHigh < spanCheck) {
        //profile is not within span of shape
        return false;
    }
    //profile is within span of shape
    return true;
}

bool DrawComplexSection::showSegment(gp_Dir segmentNormal) const
{
    if (ProjectionStrategy.getValue() < 2) {
        //Offset or Aligned are always true
        return true;
    }

    Base::Vector3d vSectionNormal = SectionNormal.getValue();
    gp_Dir gSectionNormal(vSectionNormal.x, vSectionNormal.y, vSectionNormal.z);
    if (DU::fpCompare(fabs(gSectionNormal.Dot(segmentNormal)), 0.0)) {
        //segment normal is perpendicular to section normal, so segment is parallel to section normal,
        //and for ProjectionStrategy "NoParallel", we don't display these segments.
        return false;
    }
    return true;
}

// general purpose geometry methods

//make a "face" (not necessarily a TopoDS_Face since the extrusion of a wire is a shell)
//from a single open wire by displacing the wire extruding it
TopoDS_Shape DrawComplexSection::extrudeWireToFace(TopoDS_Wire &wire, gp_Dir extrudeDir,
                                                   double extrudeDist)
{
    gp_Trsf mov;
    mov.SetTranslation(gp_Vec(extrudeDir) * (-extrudeDist));
    TopLoc_Location loc(mov);
    wire.Move(loc);

    BRepPrimAPI_MakePrism mkPrism(wire, gp_Vec(extrudeDir) * 2.0 * extrudeDist);

    return mkPrism.Shape();
}

//returns the normal of the face to be extruded into a cutting tool
//the face is expected to be planar
gp_Dir DrawComplexSection::getFaceNormal(TopoDS_Face &face)
{
    BRepAdaptor_Surface adapt(face);
    double uParmFirst = adapt.FirstUParameter();
    double uParmLast = adapt.LastUParameter();
    double vParmFirst = adapt.FirstVParameter();
    double vParmLast = adapt.LastVParameter();
    double uMid = (uParmFirst + uParmLast) / 2.0;
    double vMid = (vParmFirst + vParmLast) / 2.0;

    BRepLProp_SLProps prop(adapt, uMid, vMid, 1, 0.01);
    gp_Dir normalDir(0.0, 0.0, 1.0);//default
    if (prop.IsNormalDefined()) {
        normalDir = prop.Normal();
    }
    return normalDir;
}

bool DrawComplexSection::boxesIntersect(TopoDS_Face &face, TopoDS_Shape &shape)
{
    Bnd_Box box0, box1;
    BRepBndLib::Add(face, box0);
    box0.SetGap(0.1);//generous
    BRepBndLib::Add(shape, box1);
    box1.SetGap(0.1);
    if (box0.IsOut(box1)) {
        return false;//boxes don't intersect
    }
    return true;
}

TopoDS_Shape DrawComplexSection::shapeShapeIntersect(const TopoDS_Shape &shape0,
                                                     const TopoDS_Shape &shape1)
{
    BRepAlgoAPI_Common anOp;
    anOp.SetFuzzyValue(EWTOLERANCE);
    TopTools_ListOfShape anArg1, anArg2;
    anArg1.Append(shape0);
    anArg2.Append(shape1);
    anOp.SetArguments(anArg1);
    anOp.SetTools(anArg2);
    anOp.Build();
    return anOp.Shape();//always a compound
}

//find all the intersecting regions of face and shape
std::vector<TopoDS_Face> DrawComplexSection::faceShapeIntersect(const TopoDS_Face &face,
                                                                const TopoDS_Shape &shape)
{
    //    Base::Console().Message("DCS::faceShapeIntersect()\n");
    TopoDS_Shape intersect = shapeShapeIntersect(face, shape);
    if (intersect.IsNull()) {
        return std::vector<TopoDS_Face>();
    }
    std::vector<TopoDS_Face> intersectFaceList;
    TopExp_Explorer expFaces(intersect, TopAbs_FACE);
    for (int i = 1; expFaces.More(); expFaces.Next(), i++) {
        intersectFaceList.push_back(TopoDS::Face(expFaces.Current()));
    }
    return intersectFaceList;
}

//ensure that the edges in the output wire are in nose to tail order
TopoDS_Wire DrawComplexSection::makeNoseToTailWire(TopoDS_Wire inWire)
{
    if (inWire.IsNull()) {
        return inWire;
    }

    std::list<TopoDS_Edge> inList;
    TopExp_Explorer expEdges(inWire, TopAbs_EDGE);
    for (; expEdges.More(); expEdges.Next()) {
        TopoDS_Edge edge = TopoDS::Edge(expEdges.Current());
        inList.push_back(edge);
    }

    std::list<TopoDS_Edge> sortedList;
    if (inList.empty() || inList.size() == 1) {
        return inWire;
    }
    else {
        sortedList = DrawUtil::sort_Edges(EWTOLERANCE, inList);
    }

    BRepBuilderAPI_MakeWire mkWire;
    for (auto &edge : sortedList) { mkWire.Add(edge); }
    return mkWire.Wire();
}

//static
bool DrawComplexSection::isProfileObject(App::DocumentObject *obj)
{
    //if the object's shape is a wire or an edge, then it can be a profile object
    TopoDS_Shape shape = Part::Feature::getShape(obj);
    if (shape.IsNull()) {
        return false;
    }
    if (shape.ShapeType() == TopAbs_WIRE || shape.ShapeType() == TopAbs_EDGE) {
        return true;
    }
    //don't know what this is, but it isn't suitable as a profile
    return false;
}

bool DrawComplexSection::isMultiSegmentProfile(App::DocumentObject *obj)
{
    //if the object's shape is a wire or an edge, then it can be a profile object
    TopoDS_Shape shape = Part::Feature::getShape(obj);
    if (shape.IsNull()) {
        return false;
    }
    if (shape.ShapeType() == TopAbs_EDGE) {
        //only have 1 edge, can't be multisegment;
        return false;
    }
    if (shape.ShapeType() == TopAbs_WIRE) {
        std::vector<TopoDS_Edge> edgesInWire;
        TopExp_Explorer expEdges(shape, TopAbs_EDGE);
        for (; expEdges.More(); expEdges.Next()) {
            TopoDS_Edge edge = TopoDS::Edge(expEdges.Current());
            BRepAdaptor_Curve adapt(edge);
            if (adapt.GetType() == GeomAbs_Line) {
                edgesInWire.push_back(edge);
            }
        }
        if (edgesInWire.size() > 1) {
            return true;
        }
    }
    return false;
}

//check if the profile has curves in it
bool DrawComplexSection::isLinearProfile(App::DocumentObject *obj)
{
    TopoDS_Shape shape = Part::Feature::getShape(obj);
    if (shape.IsNull()) {
        return false;
    }
    if (shape.ShapeType() == TopAbs_EDGE) {
        //only have 1 edge
        TopoDS_Edge edge = TopoDS::Edge(shape);
        BRepAdaptor_Curve adapt(edge);
        if (adapt.GetType() == GeomAbs_Line) {
            return true;
        }
        return false;
    }

    if (shape.ShapeType() == TopAbs_WIRE) {
        std::vector<TopoDS_Edge> edgesInWire;
        TopExp_Explorer expEdges(shape, TopAbs_EDGE);
        for (; expEdges.More(); expEdges.Next()) {
            TopoDS_Edge edge = TopoDS::Edge(expEdges.Current());
            BRepAdaptor_Curve adapt(edge);
            if (adapt.GetType() != GeomAbs_Line) {
                return false;
            }
        }
        //all the edges in the wire are lines
        return true;
    }

    //this shouldn't happen
    return false;
}

// Python Drawing feature ---------------------------------------------------------

namespace App
{
/// @cond DOXERR
PROPERTY_SOURCE_TEMPLATE(TechDraw::DrawComplexSectionPython, TechDraw::DrawComplexSection)
template<> const char *TechDraw::DrawComplexSectionPython::getViewProviderName() const
{
    return "TechDrawGui::ViewProviderDrawingView";
}
/// @endcond

// explicit template instantiation
template class TechDrawExport FeaturePythonT<TechDraw::DrawComplexSection>;
}// namespace App
