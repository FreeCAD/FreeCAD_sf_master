/***************************************************************************
 *   Copyright (c) 2017 Kustaa Nyholm  <kustaa.nyholm@sparetimelabs.com>   *
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

#ifndef SRC_GUI_NAVICUBE_H_
#define SRC_GUI_NAVICUBE_H_

#include <CXX/Extensions.hxx>
#include <QColor>
#include <FCGlobal.h>

class SoEvent;

namespace Gui {
class View3DInventorViewer;
}

class NaviCubeImplementation;

class GuiExport NaviCube {
public:
    enum Corner {
        TopLeftCorner,
        TopRightCorner,
        BottomLeftCorner,
        BottomRightCorner
    };
    NaviCube(Gui::View3DInventorViewer* viewer);
    virtual ~NaviCube();
    void drawNaviCube();
    void createContextMenu(const std::vector<std::string>& cmd);
    bool processSoEvent(const SoEvent* ev);
    void setCorner(Corner);
    void setSize(int size);
    void setNaviRotateToNearest(bool toNearest);
    void setNaviStepByTurn(int steps);
    void setFont(std::string font);
    void setFontSize(int size);
    void setTextColor(QColor TextColor);
    void setFrontColor(QColor FrontColor);
    void setHiliteColor(QColor HiliteColor);
    void setButtonColor(QColor ButtonColor);
    void setBorderWidth(double BorderWidth);
    void setBorderColor(QColor BorderColor);
    static QString getDefaultSansserifFont();
    int getDefaultFontSize();
    static void setNaviCubeCommands(const std::vector<std::string>& cmd);
    static void setNaviCubeLabels(const std::vector<std::string>& labels);

private:
    NaviCubeImplementation* m_NaviCubeImplementation;
};

#endif /* SRC_GUI_NAVICUBE_H_ */
