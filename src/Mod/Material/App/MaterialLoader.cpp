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

#include "Model.h"
#include "MaterialLoader.h"
#include "MaterialConfigLoader.h"
#include "ModelManager.h"


using namespace Materials;

MaterialEntry::MaterialEntry():
    _dereferenced(false)
{}

MaterialEntry::MaterialEntry(const MaterialLibrary &library, const std::string &modelName, const QDir &dir, 
        const std::string &modelUuid):
    _library(library), _name(modelName), _directory(dir), _uuid(modelUuid), _dereferenced(false)
{}

MaterialEntry::~MaterialEntry()
{}

MaterialYamlEntry::MaterialYamlEntry(const MaterialLibrary &library, const std::string &modelName, const QDir &dir, 
        const std::string &modelUuid, const YAML::Node &modelData):
    MaterialEntry(library, modelName, dir, modelUuid), _model(modelData)
{}

MaterialYamlEntry::~MaterialYamlEntry()
{}

std::string MaterialYamlEntry::yamlValue(const YAML::Node &node, const std::string &key, const std::string &defaultValue)
{
    if (node[key])
        return node[key].as<std::string>();
    return defaultValue;
}

void MaterialYamlEntry::addToTree(std::map<std::string, Material*> *modelMap)
{
    std::set<std::string> exclude;
    exclude.insert("General");
    exclude.insert("Inherits");

    auto yamlModel = getModel();
    auto library = getLibrary();
    auto name = getName();
    auto directory = getDirectory();
    auto uuid = getUUID();

    std::string version = yamlValue(yamlModel["General"], "Version", "");
    std::string authorAndLicense = yamlValue(yamlModel["General"], "AuthorAndLicense", "");
    std::string description = yamlValue(yamlModel["General"], "Description", "");

    Material *finalModel = new Material(library, directory, uuid, name);
    finalModel->setVersion(version);
    finalModel->setAuthorAndLicense(authorAndLicense);
    finalModel->setDescription(description);

    // Add inheritance list
    if (yamlModel["Inherits"]) {
        auto inherits = yamlModel["Inherits"];
        for(auto it = inherits.begin(); it != inherits.end(); it++) {
            std::string nodeName = (*it)["UUID"].as<std::string>();

            finalModel->addInheritance(nodeName);
        }
    }

    ModelManager modelManager;
    // Add material models
    if (yamlModel["Models"]) {
        auto models = yamlModel["Models"];
        for(auto it = models.begin(); it != models.end(); it++) {
            std::string modelName = (it->first).as<std::string>();
            Base::Console().Log("Model: '%s'\n", modelName.c_str());
            MaterialLoader::showYaml(it->second);
            auto modelNode = models[modelName];
            std::string uuid = modelNode["UUID"].as<std::string>();
            auto materialModel = modelManager.getModel(uuid);
            Base::Console().Log("\tModel: '%s', UUID '%s'\n", materialModel.getName().c_str(), materialModel.getUUID().c_str());

            // finalModel->addInheritance(nodeName);
        }
    }

    // // Add property list
    // auto yamlProperties = yamlModel[base];
    // for(auto it = yamlProperties.begin(); it != yamlProperties.end(); it++) {
    //     std::string propName = it->first.as<std::string>();
    //     if (exclude.count(propName) == 0) {
    //         // showYaml(it->second);
    //         auto yamlProp = yamlProperties[propName];
    //         auto propType = yamlProp["Type"].as<std::string>();
    //         auto propUnits = yamlProp["Units"].as<std::string>();
    //         auto propURL = yamlProp["URL"].as<std::string>();
    //         auto propDescription = yamlProp["Description"].as<std::string>();

    //         ModelProperty property(propName, propType,
    //                        propUnits, propURL,
    //                        propDescription);

    //         finalModel->addProperty(property);
    //     }
    // }

    (*modelMap)[uuid] = finalModel;
}

std::map<std::string, MaterialEntry*> *MaterialLoader::_MaterialEntryMap = nullptr;

MaterialLoader::MaterialLoader(std::map<std::string, Material*> *modelMap, std::list<MaterialLibrary*> *libraryList) :
    _modelMap(modelMap), _libraryList(libraryList)
{
    loadLibraries();
}

/*
 *  Destroys the object and frees any allocated resources
 */
MaterialLoader::~MaterialLoader()
{}

void MaterialLoader::addLibrary(MaterialLibrary *model)
{
    _libraryList->push_back(model);
}

const std::string MaterialLoader::getUUIDFromPath(const std::string &path)
{
    YAML::Node yamlroot = YAML::LoadFile(path);
    const std::string uuid = yamlroot["General"]["UUID"].as<std::string>();
    return uuid;
}

MaterialEntry *MaterialLoader::getMaterialFromPath(const MaterialLibrary &library, const std::string &path) const
{
    QDir modelDir(QString::fromStdString(path));
    MaterialEntry* model = nullptr;

    if (MaterialConfigLoader::isConfigStyle(path))
    {
        Base::Console().Log("Old format .FCMat file: '%s'\n", path.c_str());
        return model;
    }

    try {
        YAML::Node yamlroot = YAML::LoadFile(path);

        const std::string uuid = yamlroot["General"]["UUID"].as<std::string>();
        const std::string name = yamlroot["General"]["Name"].as<std::string>();

        model = new MaterialYamlEntry(library, name, modelDir, uuid, yamlroot);
        showYaml(yamlroot);
    } catch (YAML::Exception ex) {
        Base::Console().Log("YAML parsing error: '%s'\n", path.c_str());

        // Perhaps try parsing the older format?
    }
   

    return model;
}

void MaterialLoader::showYaml(const YAML::Node &yaml)
{
    std::stringstream out;

    out << yaml;
    std::string logData = out.str();
    Base::Console().Log("%s\n", logData.c_str());
}

void MaterialLoader::dereference(MaterialEntry *parent, const MaterialEntry *child)
{
    // auto parentPtr = parent->getModelPtr();
    // auto childYaml = child->getModel();
    // auto childBase = child->getBase();

    // std::set<std::string> exclude;
    // exclude.insert("Name");
    // exclude.insert("UUID");
    // exclude.insert("URL");
    // exclude.insert("Description");
    // exclude.insert("DOI");
    // exclude.insert("Inherits");

    // auto parentProperties = (*parentPtr)[parentBase];
    // auto childProperties = childYaml[childBase];
    // for(auto it = childProperties.begin(); it != childProperties.end(); it++) {
    //     std::string name = it->first.as<std::string>();
    //     if (exclude.count(name) == 0) {
    //         // showYaml(it->second);
    //         if (!parentProperties[name])
    //             parentProperties[name] = it->second;
    //     }
    // }
    // showYaml(*parentPtr);
}


void MaterialLoader::dereference(MaterialEntry *model)
{
    // // Avoid recursion
    // if (model->getDereferenced())
    //     return;

    // auto yamlModel = model->getModel();
    // auto base = model->getBase();
    // if (yamlModel[base]["Inherits"]) {
    //     auto inherits = yamlModel[base]["Inherits"];
    //     for(auto it = inherits.begin(); it != inherits.end(); it++) {
    //         // auto nodeName = it->first.as<std::string>();
    //         std::string nodeName = (*it)["UUID"].as<std::string>();

    //         // This requires that all models have already been loaded undereferenced
    //         try {
    //             const MaterialYamlEntry *child = (*_MaterialYamlEntryMap)[nodeName];
    //             dereference(model, child);
    //         }
    //         catch (const std::out_of_range& oor) {
    //             Base::Console().Log("Unable to find '%s' in model map\n", nodeName.c_str());
    //         }
    //     }
    // }

    // model->markDereferenced();
}

void MaterialLoader::loadLibrary(const MaterialLibrary &library)
{
    if (_MaterialEntryMap == nullptr)
        _MaterialEntryMap = new std::map<std::string, MaterialEntry*>();

    QDirIterator it(library.getDirectory(), QDirIterator::Subdirectories);
    while (it.hasNext()) {
        auto pathname = it.next();
        QFileInfo file(pathname);
        if (file.isFile()) {
            if (file.suffix().toStdString() == "FCMat") {
                std::string libraryName = file.baseName().toStdString();

                auto model = getMaterialFromPath(library, file.canonicalFilePath().toStdString());
                if (model) {
                    (*_MaterialEntryMap)[model->getUUID()] = model;
                }
            }
        }
    }

    for (auto it = _MaterialEntryMap->begin(); it != _MaterialEntryMap->end(); it++) {
        dereference(it->second);
    }

    for (auto it = _MaterialEntryMap->begin(); it != _MaterialEntryMap->end(); it++) {
        it->second->addToTree(_modelMap);
    }

}

void MaterialLoader::loadLibraries(void)
{
    std::list<MaterialLibrary*>* _libraryList = getMaterialLibraries();
    if (_libraryList) {
        for (auto it = _libraryList->begin(); it != _libraryList->end(); it++) {
            loadLibrary(**it);
        }
    }
}

std::list<MaterialLibrary *> *MaterialLoader::getMaterialLibraries()
{
    auto param =
        App::GetApplication().GetParameterGroupByPath("User parameter:BaseApp/Preferences/Mod/Material/Resources");
    bool useBuiltInMaterials = param->GetBool("UseBuiltInMaterials", true);
    bool useMatFromModules = param->GetBool("UseMaterialsFromWorkbenches", true);
    bool useMatFromConfigDir = param->GetBool("UseMaterialsFromConfigDir", true);
    bool useMatFromCustomDir = param->GetBool("UseMaterialsFromCustomDir", true);

    if (useBuiltInMaterials)
    {
        QString resourceDir = QString::fromStdString(App::Application::getResourceDir() + "/Mod/Material/Resources/Materials");
        QDir materialDir(resourceDir);
        auto libData = new MaterialLibrary("System", materialDir, ":/icons/freecad.svg");
        _libraryList->push_back(libData);
    }

    if (useMatFromModules)
    {
        auto moduleParam = App::GetApplication().GetParameterGroupByPath(
            "User parameter:BaseApp/Preferences/Mod/Material/Resources/Modules");
        for (auto &group : moduleParam->GetGroups()) {
            // auto module = moduleParam->GetGroup(group->GetGroupName());
            auto moduleName = group->GetGroupName();
            auto materialDir = group->GetASCII("ModuleDir", "");
            auto materialIcon = group->GetASCII("ModuleIcon", "");

            if (materialDir.length() > 0) {
                QDir dir(QString::fromStdString(materialDir));
                auto libData = new MaterialLibrary(moduleName, dir, materialIcon);
                if (dir.exists()) {
                    _libraryList->push_back(libData);
                }
            }
        }
    }

    if (useMatFromConfigDir)
    {
        QString resourceDir = QString::fromStdString(App::Application::getUserAppDataDir() + "/Material");
        QDir materialDir(resourceDir);
        auto libData = new MaterialLibrary("User", materialDir, ":/icons/preferences-general.svg");
        if (materialDir.exists()) {
            _libraryList->push_back(libData);
        }
    }

    if (useMatFromCustomDir)
    {
        QString resourceDir = QString::fromStdString(param->GetASCII("CustomMaterialsDir", ""));
        QDir materialDir(resourceDir);
        auto libData = new MaterialLibrary("Custom", materialDir, ":/icons/user.svg");
        if (materialDir.exists()) {
            _libraryList->push_back(libData);
        }
    }

    return _libraryList;
}


#include "moc_MaterialLoader.cpp"
