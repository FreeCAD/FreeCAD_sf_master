/***************************************************************************
 *   Copyright (c) 2008 Werner Mayer <wmayer[at]users.sourceforge.net>     *
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
# include <boost/uuid/uuid_io.hpp>
#endif

#include "MaterialPy.h"
#include "MaterialPy.cpp"
#include "Materials.h"


using namespace Materials;

// returns a string which represents the object e.g. when printed in python
std::string MaterialPy::representation() const
{
    MaterialPy::PointerType ptr = getMaterialPtr();
    std::stringstream str;
    str << "Property [Name=(";
    str << ptr->getName().toStdString();
    str << "), UUID=(";
    str << ptr->getUUID().toStdString();
    str << "), Library Name=(";
    str << ptr->getLibrary().getName().toStdString();
    str << "), Library Root=(";
    str << ptr->getLibrary().getDirectoryPath().toStdString();
     str << "), Library Icon=(";
    str << ptr->getLibrary().getIconPath().toStdString();
    str << "), Relative Path=(";
    str << ptr->getRelativePath().toStdString();
    str << "), Directory=(";
    str << ptr->getDirectory().absolutePath().toStdString();
    // str << "), URL=(";
    // str << ptr->getURL();
    // str << "), DOI=(";
    // str << ptr->getDOI();
    // str << "), Description=(";
    // str << ptr->getDescription();
    // str << "), Inherits=[";
    // const std::vector<std::string> &inherited = getMaterialPtr()->getInheritance();
    // for (auto it = inherited.begin(); it != inherited.end(); it++)
    // {
    //     std::string uuid = *it;
    //     if (it != inherited.begin())
    //         str << "), UUID=(";
    //     else
    //         str << "UUID=(";
    //     str << uuid << ")";
    // }
    // str << "]]";
    str << ")]";

    return str.str();
}

PyObject *MaterialPy::PyMake(struct _typeobject *, PyObject *, PyObject *)  // Python wrapper
{
    // never create such objects with the constructor
    return new MaterialPy(new Material());
}

// constructor method
int MaterialPy::PyInit(PyObject* /*args*/, PyObject* /*kwd*/)
{
    return 0;
}

Py::String MaterialPy::getLibraryName() const
{
    return Py::String(getMaterialPtr()->getLibrary().getName().toStdString());
}

Py::String MaterialPy::getLibraryRoot() const
{
    return Py::String(getMaterialPtr()->getLibrary().getDirectoryPath().toStdString());
}

Py::String MaterialPy::getRelativePath() const
{
    return Py::String(getMaterialPtr()->getRelativePath().toStdString());
}

Py::String MaterialPy::getLibraryIcon() const
{
    return Py::String(getMaterialPtr()->getLibrary().getIconPath().toStdString());
}

Py::String MaterialPy::getName() const
{
    return Py::String(getMaterialPtr()->getName().toStdString());
}

Py::String MaterialPy::getDirectory() const
{
    return Py::String(getMaterialPtr()->getDirectory().absolutePath().toStdString());
}

Py::String MaterialPy::getUUID() const
{
    return Py::String(getMaterialPtr()->getUUID().toStdString());
}

Py::String MaterialPy::getDescription() const
{
    return Py::String(getMaterialPtr()->getDescription().toStdString());
}

Py::String MaterialPy::getURL() const
{
    return Py::String(getMaterialPtr()->getURL().toStdString());
}

Py::String MaterialPy::getReference() const
{
    return Py::String(getMaterialPtr()->getReference().toStdString());
}

Py::String MaterialPy::getParent() const
{
    return Py::String(getMaterialPtr()->getParentUUID().toStdString());
}

Py::String MaterialPy::getAuthorAndLicense() const
{
    return Py::String(getMaterialPtr()->getAuthorAndLicense().toStdString());
}

Py::List MaterialPy::getPhysicalModels() const
{
    const std::vector<QString> *models = getMaterialPtr()->getPhysicalModels();
    Py::List list;

    for (auto it = models->begin(); it != models->end(); it++)
    {
        QString uuid = *it;

        list.append(Py::String(uuid.toStdString()));
    }

    return list;
}

Py::List MaterialPy::getTags() const
{
    const std::list<QString> &tags = getMaterialPtr()->getTags();
    Py::List list;

    for (auto it = tags.begin(); it != tags.end(); it++)
    {
        QString uuid = *it;

        list.append(Py::String(uuid.toStdString()));
    }

    return list;
}

PyObject *MaterialPy::getCustomAttributes(const char* /*attr*/) const
{
    return nullptr;
}

int MaterialPy::setCustomAttributes(const char* /*attr*/, PyObject* /*obj*/)
{
    return 0;
}
