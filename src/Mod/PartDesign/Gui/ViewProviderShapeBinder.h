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


#ifndef PARTGUI_ViewProviderShapeBinder_H
#define PARTGUI_ViewProviderShapeBinder_H

#include <Mod/Part/Gui/ViewProvider.h>

namespace PartDesignGui {

// TODO may be derive from something else e.g. ViewProviderGeometryObject (2015-09-11, Fat-Zer)
class PartDesignGuiExport ViewProviderShapeBinder : public PartGui::ViewProviderPart
{
    PROPERTY_HEADER(PartDesignGui::ViewProviderShapeBinder);

public:
    /// Constructor
    ViewProviderShapeBinder();
    virtual ~ViewProviderShapeBinder();

    void highlightReferences(const bool on, bool auxillery);    
    
protected:
    virtual bool setEdit(int ModNum);
    virtual void unsetEdit(int ModNum);
    
private:
    std::vector<App::Color> originalLineColors;
    std::vector<App::Color> originalFaceColors;

};

} // namespace PartDesignGui


#endif // PARTGUI_ViewProviderShapeBinder_H
