/***************************************************************************
 *   Copyright (c) 2023 David Carter <dcarter@david.carter.ca>             *
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

#include <App/Application.h>
#include <Base/Interpreter.h>

#include <QDirIterator>
#include <QFileInfo>
#include <QString>

#include "PropertyModel.h"
#include "Model.h"


using namespace Materials;

TYPESYSTEM_SOURCE(Materials::PropertyModel , App::Property)

PropertyModel::PropertyModel() = default;

PropertyModel::~PropertyModel() = default;

PyObject *PropertyModel::getPyObject()
{
    // return new MaterialPy(new Material(_cMat));
    return nullptr;
}

void PropertyModel::setPyObject(PyObject *)
{
    // if (PyObject_TypeCheck(value, &(MaterialPy::Type))) {
    //     setValue(*static_cast<MaterialPy*>(value)->getMaterialPtr());
    // }
    // else {
    //     QString error = QString("type must be 'Material', not ");
    //     error += value->ob_type->tp_name;
    //     throw Base::TypeError(error);
    // }
}

void PropertyModel::setValue(const Model &model)
{
    aboutToSetValue();
    _model = model;
    hasSetValue();
}

const Model& PropertyModel::getValue() const
{
    return _model;
}

const QString PropertyModel::getBase() const
{
    return _model.getBase();
}

void PropertyModel::setName(const QString& name)
{
    aboutToSetValue();
    _model.setName(name);
    hasSetValue();
}

const QString& PropertyModel::getName() const
{
    return _model.getName();
}

void PropertyModel::setType(Model::ModelType type)
{
    aboutToSetValue();
    _model.setType(type);
    hasSetValue();
}

Model::ModelType PropertyModel::getType() const
{
    return _model.getType();
}

void PropertyModel::setDirectory(const QString& directory)
{
    aboutToSetValue();
    _model.setDirectory(directory);
    hasSetValue();
}

const QString PropertyModel::getDirectory() const
{
    return _model.getDirectory().absolutePath();
}

void PropertyModel::setUUID(const QString& uuid)
{
    aboutToSetValue();
    _model.setUUID(uuid);
    hasSetValue();
}

const QString& PropertyModel::getUUID() const
{
    return _model.getUUID();
}

void PropertyModel::Save (Base::Writer &writer) const
{
    writer.Stream() << writer.ind() << "<PropertyModel UUID=\""
        << _model.getUUID().toStdString()
        << "\"/>" << std::endl;
}

void PropertyModel::Restore(Base::XMLReader &reader)
{
    // read my Element
    reader.readElement("PropertyModel");

    // get the value of my Attribute
    aboutToSetValue();
    auto uuid = reader.getAttribute("UUID");

    // Set _model according to UUID

    hasSetValue();
}

const char* PropertyModel::getEditorName() const
{
    return "";
}

Property *PropertyModel::Copy() const
{
    PropertyModel *p= new PropertyModel();
    p->_model = _model;
    return p;
}

void PropertyModel::Paste(const Property &from)
{
    aboutToSetValue();
    _model = static_cast<const PropertyModel&>(from)._model;
    hasSetValue();
}

#include "moc_PropertyModel.cpp"
