/***************************************************************************
 *   Copyright (c) 2006 Werner Mayer <wmayer[at]users.sourceforge.net>     *
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

#include "DocumentObjectGroup.h"
#include "DocumentObjectGroupPy.h"
#include "Document.h"
#include "FeaturePythonPyImp.h"

using namespace App;

PROPERTY_SOURCE(App::GroupExtension, App::Extension)

GroupExtension::GroupExtension()
{
    m_extensionType = GroupExtension::getClassTypeId();
    
    ADD_PROPERTY_TYPE(Group,(0),"Base",(App::PropertyType)(Prop_Output),"List of referenced objects");
}

GroupExtension::~GroupExtension()
{
}

DocumentObject* GroupExtension::addObject(const char* sType, const char* pObjectName)
{
    DocumentObject* obj = getExtendedObject()->getDocument()->addObject(sType, pObjectName);
    if (obj) addObject(obj);
    return obj;
}

void GroupExtension::addObject(DocumentObject* obj)
{
    if (!hasObject(obj)) {
        std::vector<DocumentObject*> grp = Group.getValues();
        grp.push_back(obj);
        Group.setValues(grp);
    }
}

void GroupExtension::removeObject(DocumentObject* obj)
{
    const std::vector<DocumentObject*> & grp = Group.getValues();
    std::vector<DocumentObject*> newGrp;

    std::remove_copy (grp.begin(), grp.end(), std::back_inserter (newGrp), obj);
    if (grp.size() != newGrp.size()) {
        Group.setValues (newGrp);
    }
}

void GroupExtension::removeObjectsFromDocument()
{
    const std::vector<DocumentObject*> & grp = Group.getValues();
    // Use set so iterate on each linked object exactly one time (in case of multiple links to the same document)
    std::set<DocumentObject*> grpSet (grp.begin(), grp.end());

    for (std::set<DocumentObject*>::iterator it = grpSet.begin(); it != grpSet.end(); ++it) {
        removeObjectFromDocument(*it);
    }
}

void GroupExtension::removeObjectFromDocument(DocumentObject* obj)
{
    // remove all children
    if (obj->hasExtension(GroupExtension::getClassTypeId())) {
        GroupExtension *grp = static_cast<GroupExtension*>(obj->getExtension(GroupExtension::getClassTypeId()));
        // recursive call to remove all subgroups
        grp->removeObjectsFromDocument();
    }

    getExtendedObject()->getDocument()->remObject(obj->getNameInDocument());
}

DocumentObject *GroupExtension::getObject(const char *Name) const
{
    DocumentObject* obj = getExtendedObject()->getDocument()->getObject(Name);
    if (obj && hasObject(obj))
        return obj;
    return 0;
}

bool GroupExtension::hasObject(const DocumentObject* obj, bool recursive) const
{
    const std::vector<DocumentObject*>& grp = Group.getValues();
    for (std::vector<DocumentObject*>::const_iterator it = grp.begin(); it != grp.end(); ++it) {
        if (*it == obj) {
            return true;
        } else if ( recursive && (*it)->hasExtension(GroupExtension::getClassTypeId()) ) {
            App::GroupExtension *subGroup = static_cast<App::GroupExtension *> ((*it)->getExtension(GroupExtension::getClassTypeId()));
            if (subGroup->hasObject (obj, recursive)) {
                return true;
            }
        }
    }

    return false;
}

bool GroupExtension::isChildOf(const GroupExtension* group) const
{
    const std::vector<DocumentObject*>& grp = group->Group.getValues();
    for (std::vector<DocumentObject*>::const_iterator it = grp.begin(); it != grp.end(); ++it) {
        if (*it == getExtendedObject())
            return true;
        if ((*it)->hasExtension(GroupExtension::getClassTypeId())) {
            if (this->isChildOf(static_cast<GroupExtension*>((*it)->getExtension(GroupExtension::getClassTypeId()))))
                return true;
        }
    }

    return false;
}

std::vector<DocumentObject*> GroupExtension::getObjects() const
{
    return Group.getValues();
}

std::vector<DocumentObject*> GroupExtension::getObjectsOfType(const Base::Type& typeId) const
{
    std::vector<DocumentObject*> type;
    const std::vector<DocumentObject*>& grp = Group.getValues();
    for (std::vector<DocumentObject*>::const_iterator it = grp.begin(); it != grp.end(); ++it) {
        if ( (*it)->getTypeId().isDerivedFrom(typeId))
            type.push_back(*it);
    }

    return type;
}

int GroupExtension::countObjectsOfType(const Base::Type& typeId) const
{
    int type=0;
    const std::vector<DocumentObject*>& grp = Group.getValues();
    for (std::vector<DocumentObject*>::const_iterator it = grp.begin(); it != grp.end(); ++it) {
        if ( (*it)->getTypeId().isDerivedFrom(typeId))
            type++;
    }

    return type;
}

DocumentObject* GroupExtension::getGroupOfObject(const DocumentObject* obj)
{
    const Document* doc = obj->getDocument();
    std::vector<DocumentObject*> grps = doc->getObjectsOfType(GroupExtension::getClassTypeId());
    for (std::vector<DocumentObject*>::const_iterator it = grps.begin(); it != grps.end(); ++it) {
        GroupExtension* grp = (GroupExtension*)(*it);
        if (grp->hasObject(obj))
            return *it;
    }

    return 0;
}


PROPERTY_SOURCE(App::DocumentObjectGroup, App::DocumentObject)

DocumentObjectGroup::DocumentObjectGroup(void): DocumentObject(), GroupExtension() {

    setExtendedObject(this);
}

DocumentObjectGroup::~DocumentObjectGroup() {

}



// Python feature ---------------------------------------------------------

namespace App {
PROPERTY_SOURCE_TEMPLATE(App::GroupExtensionPython, App::GroupExtension)
    
/// @cond DOXERR
PROPERTY_SOURCE_TEMPLATE(App::DocumentObjectGroupPython, App::DocumentObjectGroup)
template<> const char* App::DocumentObjectGroupPython::getViewProviderName(void) const {
    return "Gui::ViewProviderGroupExtensionPython";
}
template<> PyObject* App::DocumentObjectGroupPython::getPyObject(void) {
    if (PythonObject.is(Py::_None())) {
        // ref counter is set to 1
        PythonObject = Py::Object(new FeaturePythonPyT<App::DocumentObjectGroupPy>(this),true);
    }
    return Py::new_reference_to(PythonObject);
}
/// @endcond

// explicit template instantiation
template class AppExport FeaturePythonT<App::DocumentObjectGroup>;
}
