/***************************************************************************
 *   Copyright (c) 2011 Juergen Riegel <FreeCAD@juergen-riegel.net>        *
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


#ifndef ASSEMBLYGUI_ViewProviderPart_H
#define ASSEMBLYGUI_ViewProviderPart_H

#include "ViewProvider.h"
#include <Inventor/nodes/SoAnnotation.h>
#include <Inventor/nodes/SoMarkerSet.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoSwitch.h>


namespace AssemblyGui {

class AssemblyGuiExport ViewProviderItemPart : public AssemblyGui::ViewProviderItem
{
    PROPERTY_HEADER(PartGui::ViewProviderItemPart);

public:
    /// constructor
    ViewProviderItemPart();
    /// destructor
    virtual ~ViewProviderItemPart();

    virtual bool doubleClicked(void);

    virtual void attach(App::DocumentObject *);
    virtual void setDisplayMode(const char* ModeName);
    /// returns a list of all possible modes
    virtual std::vector<std::string> getDisplayModes(void) const;

    virtual std::vector<App::DocumentObject*> claimChildren(void)const;

    virtual std::vector<App::DocumentObject*> claimChildren3D(void)const;
    
#ifdef ASSEMBLY_DEBUG_FACILITIES
    //draw the dcm points
    SoAnnotation*  m_anno;
    SoSwitch*      m_switch;
    SoMaterial*    m_material;
    SoCoordinate3* m_pointsCoordinate;
    SoMarkerSet*   m_points;
    virtual void onChanged(const App::Property* prop);
    
    App::PropertyBool ShowScalePoints;
#endif
};



} // namespace AssemblyGui


#endif // ASSEMBLYGUI_ViewProviderPart_H
