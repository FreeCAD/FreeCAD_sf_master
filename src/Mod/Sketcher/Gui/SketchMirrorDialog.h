/***************************************************************************
 *   Copyright (c) 2015 Abdullah Tahiri <abdullah.tahiri.yo@gmail.com>     *
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

#ifndef SKETCHERGUI_SketchMirrorDialog_H
#define SKETCHERGUI_SketchMirrorDialog_H

#include <QDialog>

#include <Mod/Sketcher/App/GeoEnum.h>


namespace SketcherGui
{

class Ui_SketchMirrorDialog;
class SketchMirrorDialog: public QDialog
{
    Q_OBJECT

public:
    SketchMirrorDialog();
    ~SketchMirrorDialog() override;

    int RefGeoid;
    Sketcher::PointPos RefPosid;

    void accept() override;

private:
    std::unique_ptr<Ui_SketchMirrorDialog> ui;
};

}// namespace SketcherGui

#endif// SKETCHERGUI_SketchMirrorDialog_H
