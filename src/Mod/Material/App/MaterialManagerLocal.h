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

#ifndef MATERIAL_MATERIALMANAGERLOCAL_H
#define MATERIAL_MATERIALMANAGERLOCAL_H

#include <memory>

#include <boost/filesystem.hpp>

#include <Mod/Material/MaterialGlobal.h>

#include "FolderTree.h"
#include "Materials.h"

#include "MaterialFilter.h"
#include "MaterialLibrary.h"

namespace fs = boost::filesystem;

class QMutex;

namespace App
{
class Material;
}

namespace Materials
{

class MaterialsExport MaterialManagerLocal: public Base::BaseClass
{
    TYPESYSTEM_HEADER_WITH_OVERRIDE();

public:
    MaterialManagerLocal();
    ~MaterialManagerLocal() override = default;

    static void cleanup();
    static void refresh();

    std::shared_ptr<std::vector<Library>> getLibraries();
    void createLibrary(const QString& libraryName,
                       const QString& directory,
                       const QString& icon,
                       bool readOnly = true);
    void renameLibrary(const QString& libraryName, const QString& newName);
    void changeIcon(const QString& libraryName, const QString& icon);
    void removeLibrary(const QString& libraryName);
    std::shared_ptr<std::vector<std::tuple<QString, QString, QString>>>
    libraryMaterials(const QString& libraryName);

    std::shared_ptr<std::map<QString, std::shared_ptr<Material>>> getLocalMaterials() const
    {
        return _materialMap;
    }
    std::shared_ptr<Material> getMaterial(const QString& uuid) const;
    std::shared_ptr<Material> getMaterialByPath(const QString& path) const;
    std::shared_ptr<Material> getMaterialByPath(const QString& path, const QString& library) const;
    std::shared_ptr<MaterialLibrary> getLibrary(const QString& name) const;
    bool exists(const QString& uuid) const;
    bool exists(const std::shared_ptr<MaterialLibrary>& library, const QString& uuid) const;

    // Library management
    std::shared_ptr<std::list<std::shared_ptr<MaterialLibrary>>> getMaterialLibraries() const;
    std::shared_ptr<std::list<std::shared_ptr<MaterialLibrary>>> getLocalMaterialLibraries() const;
    std::shared_ptr<std::map<QString, std::shared_ptr<MaterialTreeNode>>>
    getMaterialTree(const std::shared_ptr<MaterialLibrary>& library,
                    const std::shared_ptr<Materials::MaterialFilter>& filter) const
    {
        MaterialFilterOptions options;
        return library->getMaterialTree(filter, options);
    }
    std::shared_ptr<std::map<QString, std::shared_ptr<MaterialTreeNode>>>
    getMaterialTree(const std::shared_ptr<MaterialLibrary>& library,
                    const std::shared_ptr<Materials::MaterialFilter>& filter,
                    const MaterialFilterOptions& options) const
    {
        return library->getMaterialTree(filter, options);
    }
    std::shared_ptr<std::map<QString, std::shared_ptr<MaterialTreeNode>>>
    getMaterialTree(const std::shared_ptr<MaterialLibrary>& library) const
    {
        std::shared_ptr<Materials::MaterialFilter> filter;
        MaterialFilterOptions options;
        return library->getMaterialTree(filter, options);
    }
    std::shared_ptr<std::list<QString>>
    getMaterialFolders(const std::shared_ptr<MaterialLibrary>& library) const;
    void remove(const QString& uuid) const
    {
        _materialMap->erase(uuid);
    }

    void saveMaterial(const std::shared_ptr<MaterialLibrary>& library,
                      const std::shared_ptr<Material>& material,
                      const QString& path,
                      bool overwrite,
                      bool saveAsCopy,
                      bool saveInherited) const;

    bool isMaterial(const fs::path& p) const;
    bool isMaterial(const QFileInfo& file) const;

    std::shared_ptr<std::map<QString, std::shared_ptr<Material>>>
    materialsWithModel(const QString& uuid) const;
    std::shared_ptr<std::map<QString, std::shared_ptr<Material>>>
    materialsWithModelComplete(const QString& uuid) const;
    void dereference(std::shared_ptr<Material> material) const;
    void dereference() const;

private:
    static std::shared_ptr<std::list<std::shared_ptr<MaterialLibrary>>> _libraryList;
    static std::shared_ptr<std::map<QString, std::shared_ptr<Material>>> _materialMap;
    static QMutex _mutex;

    static void initLibraries();
};

}  // namespace Materials

#endif  // MATERIAL_MATERIALMANAGERLOCAL_H