/***************************************************************************
 *   Copyright (c) 2023 David Carter <dcarter@david.carter.ca>             *
 *                                                                         *
 *   This file is part of FreeCAD.                                         *
 *                                                                         *
 *   FreeCAD is free software: you can redistribute it and/or modify it    *
 *   under the terms of the GNU Lesser General Public License as           *
 *   published by the Free Software Foundation, either version 2.1 of the  *
 *   License, or (at your option) any later version.                       *
 *                                                                         *
 *   FreeCAD is distributed in the hope that it will be useful, but        *
 *   WITHOUT ANY WARRANTY; without even the implied warranty of            *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU      *
 *   Lesser General Public License for more details.                       *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with FreeCAD. If not, see                               *
 *   <https://www.gnu.org/licenses/>.                                      *
 *                                                                         *
 **************************************************************************/

#include "PreCompiled.h"

#include "Model.h"
#include "ModelPropertyPy.h"

#include "ModelPropertyPy.cpp"

using namespace Materials;

// returns a string which represents the object e.g. when printed in python
std::string ModelPropertyPy::representation() const
{
    std::stringstream str;
    str << "<ModelProperty object at " << getModelPropertyPtr() << ">";

    return str.str();
}

PyObject* ModelPropertyPy::PyMake(struct _typeobject*, PyObject*, PyObject*)  // Python wrapper
{
    // never create such objects with the constructor
    return new ModelPropertyPy(new ModelProperty());
}

// constructor method
int ModelPropertyPy::PyInit(PyObject* /*args*/, PyObject* /*kwd*/)
{
    return 0;
}

Py::String ModelPropertyPy::getName() const
{
    return Py::String(getModelPropertyPtr()->getName().toStdString());
}

Py::String ModelPropertyPy::getDisplayName() const
{
    return Py::String(getModelPropertyPtr()->getDisplayName().toStdString());
}

Py::String ModelPropertyPy::getType() const
{
    return Py::String(getModelPropertyPtr()->getPropertyType().toStdString());
}

Py::String ModelPropertyPy::getUnits() const
{
    return Py::String(getModelPropertyPtr()->getUnits().toStdString());
}

Py::String ModelPropertyPy::getURL() const
{
    return Py::String(getModelPropertyPtr()->getURL().toStdString());
}

Py::String ModelPropertyPy::getDescription() const
{
    return Py::String(getModelPropertyPtr()->getDescription().toStdString());
}

Py::List ModelPropertyPy::getColumns() const
{
    Py::List list;

    auto columns = getModelPropertyPtr()->getColumns();
    for (auto& column : columns) {
        PyObject* modelPropertyPy = new ModelPropertyPy(new ModelProperty(column));
        list.append(Py::Object(modelPropertyPy, true));
    }

    return list;
}

Py::String ModelPropertyPy::getInheritance() const
{
    return Py::String(getModelPropertyPtr()->getInheritance().toStdString());
}

Py::Boolean ModelPropertyPy::getInherited() const
{
    return getModelPropertyPtr()->isInherited();
}

PyObject* ModelPropertyPy::getCustomAttributes(const char* /*attr*/) const
{
    return nullptr;
}

int ModelPropertyPy::setCustomAttributes(const char* /*attr*/, PyObject* /*obj*/)
{
    return 0;
}