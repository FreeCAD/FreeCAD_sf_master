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
# include <QMessageBox>
#endif

#include <Gui/MainWindow.h>

#include <Mod/Material/App/Materials.h>
#include "Array3D.h"
#include "ui_Array3D.h"


using namespace MatGui;

Array3D::Array3D(const QString &propertyName, Materials::Material *material, QWidget* parent)
  : QDialog(parent), ui(new Ui_Array3D)
{
    ui->setupUi(this);

    Base::Console().Log("Material '%s'\n", material->getName().toStdString().c_str());
    Base::Console().Log("\tproperty '%s'\n", propertyName.toStdString().c_str());

    connect(ui->standardButtons, &QDialogButtonBox::accepted,
            this, &Array3D::accept);
    connect(ui->standardButtons, &QDialogButtonBox::rejected,
            this, &Array3D::reject);
}

Array3D::~Array3D()
{
    // no need to delete child widgets, Qt does it all for us
}

void Array3D::accept()
{
    QDialog::accept();
}

void Array3D::reject()
{
    QDialog::reject();
}

#include "moc_Array3D.cpp"
