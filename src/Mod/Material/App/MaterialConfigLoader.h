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

#ifndef MATERIAL_MATERIALCONFIGLOADER_H
#define MATERIAL_MATERIALCONFIGLOADER_H

#include <QDir>
#include <QString>
#include <QSettings>

#include "Materials.h"

namespace Materials {

class MaterialConfigLoader
{
public:
    MaterialConfigLoader();
    virtual ~MaterialConfigLoader();


    static bool isConfigStyle(const QString& path);
    static Material *getMaterialFromPath(const MaterialLibrary &library, const QString &path);

private:
    static QString value(const QSettings &fcmat, const std::string &name, const std::string &defaultValue)
    {
        return fcmat.value(QString::fromStdString(name), QString::fromStdString(defaultValue)).toString();
    }

    static void setPhysicalValue(Material *finalModel, const std::string &name, const QString &value)
    {
        if (value.length() > 0)
            finalModel->setPhysicalValue(QString::fromStdString(name), value);
    }
    static void setAppearanceValue(Material *finalModel, const std::string &name, const QString &value)
    {
        if (value.length() > 0)
            finalModel->setAppearanceValue(QString::fromStdString(name), value);
    }

    static QString getAuthorAndLicense(const QString& path);
    static void addMechanical(const QSettings &fcmat, Material *finalModel);
    static void addFluid(const QSettings &fcmat, Material *finalModel);
    static void addThermal(const QSettings &fcmat, Material *finalModel);
    static void addElectromagnetic(const QSettings &fcmat, Material *finalModel);
    static void addArchitectural(const QSettings &fcmat, Material *finalModel);
    static void addCosts(const QSettings &fcmat, Material *finalModel);
    static void addRendering(const QSettings &fcmat, Material *finalModel);
    static void addVectorRendering(const QSettings &fcmat, Material *finalModel);
};

} // namespace Materials

#endif // MATERIAL_MATERIALCONFIGLOADER_H
