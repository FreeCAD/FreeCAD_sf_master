/******************************************************************************
 *   Copyright (c) 2012 Jan Rheinländer <jrheinlaender@users.sourceforge.net> *
 *                                                                            *
 *   This file is part of the FreeCAD CAx development system.                 *
 *                                                                            *
 *   This library is free software; you can redistribute it and/or            *
 *   modify it under the terms of the GNU Library General Public              *
 *   License as published by the Free Software Foundation; either             *
 *   version 2 of the License, or (at your option) any later version.         *
 *                                                                            *
 *   This library  is distributed in the hope that it will be useful,         *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of           *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the            *
 *   GNU Library General Public License for more details.                     *
 *                                                                            *
 *   You should have received a copy of the GNU Library General Public        *
 *   License along with this library; see the file COPYING.LIB. If not,       *
 *   write to the Free Software Foundation, Inc., 59 Temple Place,            *
 *   Suite 330, Boston, MA  02111-1307, USA                                   *
 *                                                                            *
 ******************************************************************************/


#ifndef PARTDESIGN_FeatureMultiTransform_H
#define PARTDESIGN_FeatureMultiTransform_H

#include "FeatureTransformed.h"


namespace PartDesign
{

class PartDesignExport MultiTransform : public PartDesign::Transformed
{
    PROPERTY_HEADER_WITH_OVERRIDE(PartDesign::MultiTransform);

public:
    MultiTransform();

    App::PropertyLinkList Transformations;

   /** @name methods override feature */
    //@{
    short mustExecute() const override;

    /// returns the type name of the view provider
    const char* getViewProviderName() const override {
        return "PartDesignGui::ViewProviderMultiTransform";
    }
    //@}

    std::vector<App::DocumentObject*> getOriginals() const { return Originals.getValues(); }

    /** Apply all subfeature transformations
      * Returns a list containing the shapes resulting from applying all transformations of
      * the subfeatures, given by the Transformations property, after each other.
      * Subfeatures can be Mirrored, LinearPattern, PolarPattern and Scaled.
      */
    std::vector<TopoDS_Shape> applyTransformation(std::vector<TopoDS_Shape> shapes) const override;

protected:
    void positionBySupport() override;
};

} //namespace PartDesign


#endif // PARTDESIGN_FeatureMultiTransform_H
