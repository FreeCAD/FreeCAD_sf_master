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


#ifndef PARTDESIGN_Body_H
#define PARTDESIGN_Body_H

#include <App/PropertyStandard.h>
#include <Mod/Part/App/BodyBase.h>


namespace PartDesign
{

class Feature;

class Body : public Part::BodyBase
{
    PROPERTY_HEADER(PartDesign::Body);

public:

    /// True if this body feature is active or was active when the document was last closed
    App::PropertyBool IsActive;

    Body();

    /** @name methods override feature */
    //@{
    /// recalculate the feature
    App::DocumentObjectExecReturn *execute(void);
    short mustExecute() const;
    /// returns the type name of the view provider
    const char* getViewProviderName(void) const {
        return "PartDesignGui::ViewProviderBody";
    }
    //@}

    /// Get the tip shape
    const Part::TopoShape getTipShape();

    /**
      * Return Tip feature if it is a solid. Otherwise, go backwards in the Model and
      * find the first feature that is a solid (i.e. Sketches are skipped)
      * This is used by SketchBased features to determine the shape they should fuse with or cut out off
      * NOTE: Currently only PartDesign features are accepted as TipSolidFeatures
      */
    App::DocumentObject *getTipSolidFeature();

    /**
      * Return the next solid feature after the Tip feature (so this only makes sense in insert mode)
      * This is used by Sketchbased features in insert mode to re-route the Base property
      * NOTE: Currently only PartDesign features are accepted as nextSolidFeatures
      */
    App::DocumentObject *getNextSolidFeature();

    /// Return the shape of the feature preceding this feature
    const Part::TopoShape getPreviousSolid(const PartDesign::Feature* f);

    /// Return true if the feature belongs to this body
    const bool hasFeature(const App::DocumentObject *f);

    /// Returns true if we are inserting into the feature tree instead of appending at the end
    const bool insertMode();

    PyObject *getPyObject(void);


};

} //namespace PartDesign


#endif // PART_Body_H
