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

#ifndef MATERIAL_MATERIALMANAGER_H
#define MATERIAL_MATERIALMANAGER_H

#include <boost/filesystem.hpp>

#include "Materials.h"

namespace fs = boost::filesystem;

namespace Materials {

class MaterialsExport MaterialManager : public Base::BaseClass
{
    TYPESYSTEM_HEADER();

public:
    explicit MaterialManager();
    virtual ~MaterialManager();

    std::map<std::string, Material *> *getMaterials() { return _materialMap; }
    const Material& getMaterial(const std::string& uuid) const;
    const Material &getMaterialByPath(const std::string &path) const;
    const Material &getMaterialByPath(const std::string &path, const std::string &libraryPath) const;

    // Library management
    static std::list<MaterialLibrary*>* getMaterialLibraries();
    void createPath(MaterialLibrary* library, const std::string& path) { library->createPath(path); }
    void saveMaterial(MaterialLibrary* library, const Material& material, const std::string& path) {
        library->saveMaterial(material, path);
    }

    static bool isMaterial(const fs::path &p);

private:
    const std::string getUUIDFromPath(const std::string& path) const;

    static std::list<MaterialLibrary *> *_libraryList;
    static std::map<std::string, Material *> *_materialMap;
    static std::map<std::string, Material *> *_materialPathMap;
};

} // namespace Materials

#endif // MATERIAL_MATERIALMANAGER_H
