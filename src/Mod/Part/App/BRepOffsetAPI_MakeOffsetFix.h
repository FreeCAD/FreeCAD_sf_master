/***************************************************************************
 *   Copyright (c) 2019 Werner Mayer <wmayer[at]users.sourceforge.net>     *
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

#ifndef PART_BREPOFFSETAPI_MAKEOFFSETFIX_H
#define PART_BREPOFFSETAPI_MAKEOFFSETFIX_H

#include <list>
#include <map>

#include <Mod/Part/PartGlobal.h>

#include <BRepOffsetAPI_MakeOffset.hxx>
#include <GeomAbs_CurveType.hxx>


namespace Part {
/*!
 * \brief The BRepOffsetAPI_MakeOffsetFix class
 * This class works around a limitation of the BRepOffsetAPI_MakeOffset which
 * returns unexpected results when an input wire has set a placement and consists
 * of a single edge only.
 */
class PartExport BRepOffsetAPI_MakeOffsetFix : public BRepBuilderAPI_MakeShape
{
public:
    BRepOffsetAPI_MakeOffsetFix();
    BRepOffsetAPI_MakeOffsetFix(const GeomAbs_JoinType Join, const Standard_Boolean IsOpenResult);
    ~BRepOffsetAPI_MakeOffsetFix() override;

    //! Initializes the algorithm to construct parallels to the wire Spine.
    void AddWire (const TopoDS_Wire& Spine);

    //! Computes a parallel to the spine at distance Offset and
    //! at an altitude Alt from the plane of the spine in relation
    //! to the normal to the spine.
    //! Exceptions: StdFail_NotDone if the offset is not built.
    void Perform (const Standard_Real Offset, const Standard_Real Alt = 0.0);

    //! Builds the resulting shape (redefined from MakeShape).
    void Build() override;

    //! Initializes the algorithm to construct parallels to the spine Spine.
    //! Join defines the type of parallel generated by the
    //! salient vertices of the spine.
    //! The default type is GeomAbs_Arc where the vertices generate
    //! sections of a circle.
    //! If join type is GeomAbs_Intersection, the edges that
    //! intersect in a salient vertex generate the edges
    //! prolonged until intersection.
    void Init(const TopoDS_Face& Spine, const GeomAbs_JoinType Join = GeomAbs_Arc, const Standard_Boolean IsOpenResult = Standard_False);

    //! Initialize the evaluation of Offsetting.
    void Init(const GeomAbs_JoinType Join = GeomAbs_Arc, const Standard_Boolean IsOpenResult = Standard_False);

    Standard_Boolean IsDone() const override;

    //! Returns a shape built by the shape construction algorithm.
    //! Raises exception StdFail_NotDone if the shape was not built.
    const TopoDS_Shape& Shape() override;

    //! returns a list of the created shapes
    //! from the shape <S>.
    const TopTools_ListOfShape& Generated (const TopoDS_Shape& S) override;

    //! Returns the list  of shapes modified from the shape
    //! <S>.
    const TopTools_ListOfShape& Modified (const TopoDS_Shape& S) override;

    //! Returns true if the shape S has been deleted.
    Standard_Boolean IsDeleted (const TopoDS_Shape& S) override;

    //! Replaces the given curve type with a B-Spline. Input shape <S>
    //! must be a wire or a compound of wires
    TopoDS_Shape Replace(GeomAbs_CurveType, const TopoDS_Shape& S) const;

private:
    TopoDS_Wire ReplaceEdges(GeomAbs_CurveType, const TopoDS_Wire& S) const;
    void MakeWire(TopoDS_Shape&);

private:
    BRepOffsetAPI_MakeOffset mkOffset;
    std::list<std::pair<TopoDS_Shape, TopLoc_Location> > myLocations;
    TopoDS_Shape myResult;
};

}

#endif // PART_BREPOFFSETAPI_MAKEOFFSETFIX_H
