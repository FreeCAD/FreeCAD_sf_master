/***************************************************************************
 *   Copyright (c) Stefan Tröger          (stefantroeger@gmx.net) 2016     *
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


#ifndef GUI_VIEWPROVIDEREXTENSION_H
#define GUI_VIEWPROVIDEREXTENSION_H

#include "App/Extension.h"
#include "ViewProvider.h"
#include "ViewProviderDocumentObject.h"

namespace Gui {
    
/**
 * @brief Extension with special viewprovider calls
 * 
 */
class GuiExport ViewProviderExtension : public App::Extension
{

    //The cass does not have properties itself, but it is important to provide the property access
    //functions.
    PROPERTY_HEADER(Gui::ViewProviderExtension);

public:

    ViewProviderExtension ();
    virtual ~ViewProviderExtension ();

    Gui::ViewProviderDocumentObject*       getExtendedViewProvider();
    const Gui::ViewProviderDocumentObject* getExtendedViewProvider() const;
   
    virtual std::vector<App::DocumentObject*> extensionClaimChildren3D(void) const { 
        return std::vector<App::DocumentObject*>(); }
        
    virtual bool extensionOnDelete(const std::vector<std::string> &subNames){ return true;}
 
    virtual std::vector<App::DocumentObject*> extensionClaimChildren(void) const { 
        return std::vector<App::DocumentObject*>(); }

    virtual bool extensionCanDragObjects() const { return false; }
    virtual bool extensionCanDragObject(App::DocumentObject*) const { return true; }
    virtual void extensionDragObject(App::DocumentObject*) { }
    virtual bool extensionCanDropObjects() const { return false; }
    virtual bool extensionCanDropObject(App::DocumentObject*) const { return true; }
    virtual void extensionDropObject(App::DocumentObject*) { }

    /// Hides the view provider
    virtual void extensionHide(void) { };
    /// Shows the view provider
    virtual void extensionShow(void) { };
    
    virtual void extensionRestore(Base::XMLReader& reader) { };
    
private:
    Gui::ViewProviderDocumentObject* m_viewBase = nullptr;
};

} //App

#endif // GUI_VIEWPROVIDEREXTENSION_H
