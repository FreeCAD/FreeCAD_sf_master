/***************************************************************************
 *   Copyright (c) 2011 Juergen Riegel <juergen.riegel@web.de>             *
 *   Copyright (c) 2015 Eivind Kvedalen <eivind@kvedalen.name>             *
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
#include <QMenu>
#include <QString>
#include <sstream>
#endif

#include <Gui/Application.h>
#include <Gui/BitmapFactory.h>
#include <Gui/CommandT.h>
#include <Gui/Document.h>
#include <Gui/FileDialog.h>
#include <Gui/MainWindow.h>
#include <Gui/View3DInventor.h>
#include <Mod/Spreadsheet/App/Sheet.h>

#include "ViewProviderSpreadsheet.h"
#include "ViewProviderSpreadsheetPy.h"


using namespace Base;
using namespace Gui;
using namespace App;
using namespace SpreadsheetGui;
using namespace Spreadsheet;

PROPERTY_SOURCE(SpreadsheetGui::ViewProviderSheet, Gui::ViewProviderDocumentObject)

ViewProviderSheet::ViewProviderSheet() = default;

ViewProviderSheet::~ViewProviderSheet()
{
    if (!view.isNull()) {
        Gui::getMainWindow()->removeWindow(view);
    }
}

void ViewProviderSheet::setDisplayMode(const char* ModeName)
{
    ViewProviderDocumentObject::setDisplayMode(ModeName);
}

std::vector<std::string> ViewProviderSheet::getDisplayModes() const
{
    std::vector<std::string> StrList;
    StrList.emplace_back("Spreadsheet");
    return StrList;
}

QIcon ViewProviderSheet::getIcon() const
{
    // clang-format off
    static const char* const Points_Feature_xpm[] = {
        "16 16 3 1",
        "       c None",
        ".      c #000000",
        "+      c #FFFFFF",
        "                ",
        "                ",
        "................",
        ".++++.++++.++++.",
        ".++++.++++.++++.",
        "................",
        ".++++.++++.++++.",
        ".++++.++++.++++.",
        "................",
        ".++++.++++.++++.",
        ".++++.++++.++++.",
        "................",
        ".++++.++++.++++.",
        ".++++.++++.++++.",
        "................",
        "                "};
    QPixmap px(Points_Feature_xpm);
    return px;
    // clang-format on
}

bool ViewProviderSheet::setEdit(int ModNum)
{
    if (ModNum == ViewProvider::Default) {
        showSheetMdi();
    }
    return false;
}

bool ViewProviderSheet::doubleClicked()
{
    // assure the SpreadSheet workbench
    if (App::GetApplication()
            .GetUserParameter()
            .GetGroup("BaseApp")
            ->GetGroup("Preferences")
            ->GetGroup("Mod/Spreadsheet")
            ->GetBool("SwitchToWB", true)) {
        Gui::Command::assureWorkbench("SpreadsheetWorkbench");
    }

    showSheetMdi();
    return true;
}

void ViewProviderSheet::showSheetMdi()
{
    if (!this->view) {
        showSpreadsheetView();
        view->viewAll();
    }
    Gui::getMainWindow()->setActiveWindow(this->view);
}

void ViewProviderSheet::exportAsFile()
{
    auto* sheet = static_cast<Spreadsheet::Sheet*>(getObject());
    QString selectedFilter;
    QString formatList = QObject::tr("CSV (*.csv *.CSV);;All (*)");
    QString fileName = Gui::FileDialog::getSaveFileName(Gui::getMainWindow(),
                                                        QObject::tr("Export file"),
                                                        QString(),
                                                        formatList,
                                                        &selectedFilter);
    if (!fileName.isEmpty()) {
        if (sheet) {
            char delim = '\0';
            char quote = '\0';
            char escape = '\0';
            std::string errMsg = "Export";
            bool isValid = sheet->getCharsFromPrefs(delim, quote, escape, errMsg);

            if (isValid) {
                sheet->exportToFile(fileName.toStdString(), delim, quote, escape);
            }
            else {
                Base::Console().Error(errMsg.c_str());
            }
        }
    }
}

void ViewProviderSheet::setupContextMenu(QMenu* menu, QObject* receiver, const char* member)
{
    QAction* act;
    act = menu->addAction(QObject::tr("Show spreadsheet"), receiver, member);
    act->setData(QVariant((int)ViewProvider::Default));
}

Sheet* ViewProviderSheet::getSpreadsheetObject() const
{
    return freecad_dynamic_cast<Sheet>(pcObject);
}

void ViewProviderSheet::beforeDelete()
{
    ViewProviderDocumentObject::beforeDelete();
    if (!view) {
        return;
    }
    if (view == Gui::getMainWindow()->activeWindow()) {
        getDocument()->setActiveView(nullptr, Gui::View3DInventor::getClassTypeId());
    }
    Gui::getMainWindow()->removeWindow(view);
}

SheetView* ViewProviderSheet::showSpreadsheetView()
{
    if (!view) {
        Gui::Document* doc = Gui::Application::Instance->getDocument(this->pcObject->getDocument());
        view = new SheetView(doc, this->pcObject, Gui::getMainWindow());
        view->setWindowIcon(Gui::BitmapFactory().pixmap(":icons/Spreadsheet.svg"));
        view->setWindowTitle(QString::fromUtf8(pcObject->Label.getValue())
                             + QString::fromLatin1("[*]"));
        Gui::getMainWindow()->addWindow(view);
        startEditing();
    }

    return view;
}

Gui::MDIView* ViewProviderSheet::getMDIView() const
{
    return const_cast<ViewProviderSheet*>(this)->showSpreadsheetView();
}

void ViewProviderSheet::updateData(const App::Property* prop)
{
    if (view) {
        view->updateCell(prop);
    }
}

PyObject* ViewProviderSheet::getPyObject()
{
    if (!pyViewObject) {
        pyViewObject = new ViewProviderSpreadsheetPy(this);
    }
    pyViewObject->IncRef();
    return pyViewObject;
}

// Python feature -----------------------------------------------------------------------

namespace Gui
{
/// @cond DOXERR
PROPERTY_SOURCE_TEMPLATE(SpreadsheetGui::ViewProviderSheetPython, SpreadsheetGui::ViewProviderSheet)
/// @endcond

// explicit template instantiation
template class SpreadsheetGuiExport ViewProviderFeaturePythonT<ViewProviderSheet>;
}  // namespace Gui
