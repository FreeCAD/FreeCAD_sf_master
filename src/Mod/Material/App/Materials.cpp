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

#include <QMetaType>
#include <App/Application.h>
#include <Gui/MetaTypes.h>

#include "Materials.h"
#include "ModelManager.h"


using namespace Materials;

/* TRANSLATOR Material::Materials */

TYPESYSTEM_SOURCE(Materials::MaterialProperty, Materials::ModelProperty)

MaterialProperty::MaterialProperty()
{
    _valuePtr = new MaterialValue(MaterialValue::None);
}

MaterialProperty::MaterialProperty(const ModelProperty &property) :
    ModelProperty(property), _valuePtr(nullptr)
{
    setType(getPropertyType());
    for (auto it : property.getColumns()) {
        MaterialProperty prop(it);
        addColumn(prop);
    }

    if (_valuePtr->getType() == MaterialValue::Array2D)
    {
        reinterpret_cast<Material2DArray *>(_valuePtr)->setDefault(getColumnNull(0));
    }
    else if (_valuePtr->getType() == MaterialValue::Array3D)
    {
        reinterpret_cast<Material3DArray *>(_valuePtr)->setDefault(getColumnNull(0));
    }
}

MaterialProperty::~MaterialProperty()
{

}

void MaterialProperty::setModelUUID(const QString& uuid)
{
    _modelUUID = uuid;
}

const QVariant MaterialProperty::getValue(void) const
{
    return _valuePtr->getValue();
}

MaterialValue* MaterialProperty::getMaterialValue(void)
{
    return _valuePtr;
}

const QString MaterialProperty::getString(void) const
{
    if (getType() == MaterialValue::Quantity)
    {
        Base::Quantity quantity = getValue().value<Base::Quantity>();
        return quantity.getUserString();
    }
    return getValue().toString();
}

void MaterialProperty::setPropertyType(const QString& type)
{
    ModelProperty::setPropertyType(type);
    setType(type);
}

void MaterialProperty::setType(const QString& type)
{
    if (_valuePtr)
        delete _valuePtr;

    if (type == QString::fromStdString("String"))
    {
        _valuePtr = new MaterialValue(MaterialValue::String);
    } else if (type == QString::fromStdString("Boolean"))
    {
        _valuePtr = new MaterialValue(MaterialValue::Boolean);
    } else if (type == QString::fromStdString("Integer"))
    {
        _valuePtr = new MaterialValue(MaterialValue::Integer);
    } else if (type == QString::fromStdString("Float"))
    {
        _valuePtr = new MaterialValue(MaterialValue::Float);
    } else if (type == QString::fromStdString("URL"))
    {
        _valuePtr = new MaterialValue(MaterialValue::URL);
    } else if (type == QString::fromStdString("Quantity"))
    {
        _valuePtr = new MaterialValue(MaterialValue::Quantity);
    } else if (type == QString::fromStdString("Color"))
    {
        _valuePtr = new MaterialValue(MaterialValue::Color);
    } else if (type == QString::fromStdString("File"))
    {
        _valuePtr = new MaterialValue(MaterialValue::File);
    } else if (type == QString::fromStdString("Image"))
    {
        _valuePtr = new MaterialValue(MaterialValue::Image);
    } else if (type == QString::fromStdString("List"))
    {
        _valuePtr = new MaterialValue(MaterialValue::List);
    } else if (type == QString::fromStdString("2DArray"))
    {
        _valuePtr = new Material2DArray();
    } else if (type == QString::fromStdString("3DArray"))
    {
        _valuePtr = new Material3DArray();
    } else {
        // Error. Throw something
        _valuePtr = new MaterialValue(MaterialValue::None);
        std::string stringType = type.toStdString();
        std::string name = getName().toStdString();
        throw UnknownValueType();
    }
}

MaterialProperty &MaterialProperty::getColumn(int column)
{
    try {
        return _columns.at(column);
    } catch (std::out_of_range const&) {
        throw InvalidColumn();
    }
}
    
MaterialValue::ValueType MaterialProperty::getColumnType(int column) const
{
    try {
        return _columns.at(column).getType();
    } catch (std::out_of_range const&) {
        throw InvalidColumn();
    }
}

QString MaterialProperty::getColumnUnits(int column) const
{
    try {
        return _columns.at(column).getUnits();
    } catch (std::out_of_range const&) {
        throw InvalidColumn();
    }
}

QVariant MaterialProperty::getColumnNull(int column) const
{
    MaterialValue::ValueType valueType = getColumnType(column);

    switch (valueType)
    {
        case MaterialValue::Quantity:
        {
            Base::Quantity q = Base::Quantity(0, getColumnUnits(column));
            return QVariant::fromValue(q);
        }

        case MaterialValue::Float:
        case MaterialValue::Integer:
            return QVariant(0);

        default:
            break;
    }

    return QVariant(QString());
}

void MaterialProperty::setValue(const QVariant& value)
{
    // _valueType = MaterialValue::String;
    _valuePtr->setValue(value);
}

void MaterialProperty::setValue(const QString& value)
{
    if (_valuePtr->getType() == MaterialValue::Boolean)
        setBoolean(value);
    else if (_valuePtr->getType() == MaterialValue::Integer)
        setInt(value);
    else if (_valuePtr->getType() == MaterialValue::Float)
        setFloat(value);
    else if (_valuePtr->getType() == MaterialValue::URL)
        setURL(value);
    else if (_valuePtr->getType() == MaterialValue::Quantity)
    {
        Base::Console().Log("\tParse quantity '%s'\n", value.toStdString().c_str());
        try
        {
            setQuantity(Base::Quantity::parse(value));
        }
        catch(const Base::ParserError& e)
        {
            Base::Console().Log("Error '%s'\n", e.what());
            // Save as a string
            setString(value);
        }
    }
    else
        setString(value);
}

void MaterialProperty::setString(const QString& value)
{
    // _valueType = MaterialValue::String;
    _valuePtr->setValue(QVariant(value));
}

void MaterialProperty::setBoolean(bool value)
{
    // _valueType = MaterialValue::Boolean;
    _valuePtr->setValue(QVariant(value));
}

void MaterialProperty::setBoolean(int value)
{
    // _valueType = MaterialValue::Boolean;
    _valuePtr->setValue(QVariant(value != 0));
}

void MaterialProperty::setBoolean(const QString& value)
{
    // _valueType = MaterialValue::Boolean;
    bool boolean;
    std::string val = value.toStdString();
    if ((val == "true") || (val == "True"))
       boolean = true;
    else if ((val == "false") || (val == "False"))
       boolean = false;
    else
        boolean = (std::stoi(val) != 0);

    setBoolean(boolean);
}

void MaterialProperty::setInt(int value)
{
    _valuePtr->setValue(QVariant(value));
}

void MaterialProperty::setInt(const QString& value)
{
    _valuePtr->setValue(value.toInt());
}

void MaterialProperty::setFloat(double value)
{
    _valuePtr->setValue(QVariant(value));
}

void MaterialProperty::setFloat(const QString& value)
{
    _valuePtr->setValue(QVariant(value.toFloat()));
}

void MaterialProperty::setQuantity(const Base::Quantity& value)
{
    _valuePtr->setValue(QVariant(QVariant::fromValue(value)));
}

void MaterialProperty::setQuantity(double value, const QString& units)
{
    setQuantity(Base::Quantity(value, units));
}

void MaterialProperty::setQuantity(const QString& value)
{
    setQuantity(Base::Quantity::parse(value));
}

void MaterialProperty::setURL(const QString& value)
{
    _valuePtr->setValue(QVariant(value));
}


TYPESYSTEM_SOURCE(Materials::Material, Base::BaseClass)

Material::Material() :
    _dereferenced(false)
{}

Material::Material(const MaterialLibrary &library, const QString& directory,
            const QString& uuid, const QString& name) :
    _library(library), _uuid(uuid), _name(name), _dereferenced(false)
{
    setDirectory(directory);
}

Material::Material(const MaterialLibrary &library, const QDir& directory,
            const QString& uuid, const QString& name) :
    _library(library), _directory(directory), _uuid(uuid), _name(name), _dereferenced(false)
{}

/*
 *  Destroys the object and frees any allocated resources
 */
Material::~Material()
{
    // no need to delete child widgets, Qt does it all for us
}

void Material::addModel(const QString &uuid)
{
    for (QString modelUUID: _allUuids)
        if (modelUUID == uuid)
            return;

    _allUuids.push_back(uuid);

    ModelManager *manager = ModelManager::getManager();

    try {
        const Model &model = manager->getModel(uuid);
        auto inheritance = model.getInheritance();
        for (auto inherits = inheritance.begin(); inherits != inheritance.end(); inherits++)
            addModel(*inherits);
    } catch (ModelNotFound const &) {
    }
}

void Material::addPhysical(const QString &uuid)
{
    if (hasPhysicalModel(uuid))
        return;

    ModelManager *manager = ModelManager::getManager();

    try {
        const Model &model = manager->getModel(uuid);

        _physicalUuids.push_back(uuid);
        addModel(uuid);

        for (auto it = model.begin(); it != model.end(); it++)
        {
            QString propertyName = it->first;
            ModelProperty property = it->second;

            try
            {
                _physical[propertyName] = MaterialProperty(property);
            }
            catch(const UnknownValueType &)
            {
                Base::Console().Error("Property '%s' has unknown type '%s'. Ignoring\n", 
                                    property.getName().toStdString().c_str(),
                                    property.getPropertyType().toStdString().c_str());
            }
            
        }
    } catch (ModelNotFound const &) {
    }
}

void Material::addAppearance(const QString &uuid)
{
    if (hasAppearanceModel(uuid))
        return;

    ModelManager *manager = ModelManager::getManager();

    try {
        const Model &model = manager->getModel(uuid);

        _appearanceUuids.push_back(uuid);
        addModel(uuid);

        for (auto it = model.begin(); it != model.end(); it++)
        {
            QString propertyName = it->first;
            ModelProperty property = it->second;

            _appearance[propertyName] = MaterialProperty(property);
        }
    } catch (ModelNotFound const &) {
    }
}


void Material::setPhysicalValue(const QString& name, const QString &value)
{
    _physical[name].setValue(value); // may not be a string type
}

void Material::setPhysicalValue(const QString& name, int value)
{
    _physical[name].setInt(value);
}

void Material::setPhysicalValue(const QString& name, double value)
{
    _physical[name].setFloat(value);
}

void Material::setPhysicalValue(const QString& name, const Base::Quantity value)
{
    _physical[name].setQuantity(value);
}

void Material::setAppearanceValue(const QString& name, const QString &value)
{
    _appearance[name].setValue(value); // may not be a string type
}

MaterialProperty &Material::getPhysicalProperty(const QString &name)
{
    try {
        return _physical.at(name);
    } catch (std::out_of_range const &) {
        throw PropertyNotFound();
    }
}

const MaterialProperty &Material::getPhysicalProperty(const QString &name) const
{
    try {
        return _physical.at(name);
    } catch (std::out_of_range const &) {
        throw PropertyNotFound();
    }
}

MaterialProperty &Material::getAppearanceProperty(const QString &name)
{
    try {
        return _appearance.at(name);
    } catch (std::out_of_range const &) {
        throw PropertyNotFound();
    }
}

const MaterialProperty &Material::getAppearanceProperty(const QString &name) const
{
    try {
        return _appearance.at(name);
    } catch (std::out_of_range const &) {
        throw PropertyNotFound();
    }
}

const QVariant Material::getPhysicalValue(const QString &name) const
{
    try {
        return _physical.at(name).getValue();
    } catch (std::out_of_range const &) {
        throw PropertyNotFound();
    }
}

const QString Material::getPhysicalValueString(const QString &name) const
{
    try {
        return _physical.at(name).getValue().toString();
    } catch (std::out_of_range const &) {
        throw PropertyNotFound();
    }
}

const QVariant Material::getAppearanceValue(const QString &name) const
{
    try {
        return _appearance.at(name).getValue();
    } catch (std::out_of_range const &) {
        throw PropertyNotFound();
    }
}

const QString Material::getAppearanceValueString(const QString &name) const
{
    try {
        return _appearance.at(name).getValue().toString();
    } catch (std::out_of_range const &) {
        throw PropertyNotFound();
    }
}

bool Material::hasPhysicalProperty(const QString& name) const
{
    try {
        static_cast<void>(_physical.at(name));
    } catch (std::out_of_range const &) {
        return false;
    }
    return true;
}

bool Material::hasAppearanceProperty(const QString& name) const
{
    try {
        static_cast<void>(_appearance.at(name));
    } catch (std::out_of_range const &) {
        return false;
    }
    return true;
}

bool Material::hasModel(const QString& uuid) const
{
    for (QString modelUUID: _allUuids)
        if (modelUUID == uuid)
            return true;

    return false;
}

bool Material::hasPhysicalModel(const QString& uuid) const
{
    if (!hasModel(uuid))
        return false;

    ModelManager *manager = ModelManager::getManager();

    try {
        const Model &model = manager->getModel(uuid);
        if (model.getType() == Model::ModelType_Physical)
            return true;

    } catch (ModelNotFound const &) {
    }

    return false;
}

bool Material::hasAppearanceModel(const QString& uuid) const
{
    if (!hasModel(uuid))
        return false;

    ModelManager *manager = ModelManager::getManager();

    try {
        const Model &model = manager->getModel(uuid);
        if (model.getType() == Model::ModelType_Appearance)
            return true;

    } catch (ModelNotFound const &) {
    }

    return false;
}

bool Material::isPhysicalModelComplete(const QString& uuid) const
{
    if (!hasPhysicalModel(uuid))
        return false;

    ModelManager *manager = ModelManager::getManager();

    try {
        const Model &model = manager->getModel(uuid);
        for (auto it = model.begin(); it != model.end(); it++)
        {
            QString propertyName = it->first;
            const MaterialProperty& property = getPhysicalProperty(propertyName);

            if (property.isNull())
                return false;
        }
    } catch (ModelNotFound const &) {
        return false;
    }

    return true;
}

bool Material::isAppearanceModelComplete(const QString& uuid) const
{
    if (!hasAppearanceModel(uuid))
        return false;

    ModelManager *manager = ModelManager::getManager();

    try {
        const Model &model = manager->getModel(uuid);
        for (auto it = model.begin(); it != model.end(); it++)
        {
            QString propertyName = it->first;
            const MaterialProperty& property = getAppearanceProperty(propertyName);

            if (property.isNull())
                return false;
        }
    } catch (ModelNotFound const &) {
        return false;
    }

    return true;
}

#include "moc_Materials.cpp"
