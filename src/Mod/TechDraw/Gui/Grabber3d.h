/***************************************************************************
 *   Copyright (c) 2019 WandererFan <wandererfan@gmail.com>                *
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

#ifndef Grabber3d_h_
#define Grabber3d_h_

#include <Mod/TechDraw/TechDrawGlobal.h>

#include <QColor>
#include <QImage>

namespace App {
class Document;
class DocumentObject;
}
namespace Gui {
class Document;
class View3DInventorViewer;
}

namespace TechDrawGui
{

/// Utility functions for obtaining 3d window image
class TechDrawGuiExport Grabber3d {
public:
    static void quickView(App::Document* appDoc,
                          int outWidth, int outHeight,
                          const QColor bgColor,
                          QImage &image);
};

} //end namespace TechDrawGui
#endif
