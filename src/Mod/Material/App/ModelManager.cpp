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

#include "Model.h"
#include "ModelManager.h"
#include "ModelLoader.h"


using namespace Material;

ModelManager *ModelManager::manager = nullptr;
std::list<ModelLibrary*> *ModelManager::_libraryList = nullptr;
std::map<std::string, Model*> *ModelManager::modelMap = nullptr;

TYPESYSTEM_SOURCE(Material::ModelManager, Base::BaseClass)

ModelManager::ModelManager()
{
    if (modelMap == nullptr) {
        modelMap = new std::map<std::string, Model*>();
    if (_libraryList == nullptr)
        _libraryList = new std::list<ModelLibrary*>();

        // Load the libraries
        ModelLoader loader(modelMap, _libraryList);
    }
}

/*
 *  Destroys the object and frees any allocated resources
 */
ModelManager::~ModelManager()
{
}

ModelManager *ModelManager::getManager()
{
    if (manager == nullptr)
        manager = new ModelManager();
    return manager;
}


#include "moc_ModelManager.cpp"
