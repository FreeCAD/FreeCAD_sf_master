/***************************************************************************
 *                                                                         *
 *   Copyright (c) 2013 Eivind Kvedalen (eivind@kvedalen.name)             *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Library General Public License as       *
 *   published by the Free Software Foundation; either version 2 of the    *
 *   License, or (at your option) any later version.                       *
 *   for detail see the LICENCE text file.                                 *
 *                                                                         *
 ***************************************************************************/

#ifndef SpreadsheetView_H
#define SpreadsheetView_H

#include <Gui/MDIView.h>
#include "SheetModel.h"

class QSlider;
class QAction;
class QActionGroup;
class QPopupMenu;
class QToolBar;

namespace App {
class DocumentObject;
class Property;
}

namespace Spreadsheet {
class Sheet;
}

namespace Ui {
class Sheet;
}

class QTableWidgetItem;

namespace SpreadsheetGui
{

class SpreadsheetDelegate;

class SpreadsheetGuiExport SheetView : public Gui::MDIView
{
    Q_OBJECT

public:
    SheetView(App::DocumentObject* docObj, QWidget* parent);
    ~SheetView();

    const char *getName(void) const {return "SheetView";}

    bool onMsg(const char* pMsg,const char** ppReturn){ return true; }
    bool onHasMsg(const char* pMsg) const { return false; }

    void updateCell(const App::Property * prop);

    Spreadsheet::Sheet * getSheet() { return sheet; }

    QModelIndexList selectedIndexes() const;

    QModelIndex currentIndex() const;

protected Q_SLOTS:
    void editingFinished();
    void currentChanged( const QModelIndex & current, const QModelIndex & previous );
    void columnResized(int col, int oldSize, int newSize);
    void rowResized(int col, int oldSize, int newSize);
protected:
    void updateContentLine();
    void setCurrentCell(QString str);
    void keyPressEvent(QKeyEvent *event);
    void resizeColumn(int col, int newSize);
    void resizeRow(int col, int newSize);

    Ui::Sheet * ui;
    Spreadsheet::Sheet * sheet;
    SpreadsheetDelegate * delegate;
    SheetModel model;
    boost::BOOST_SIGNALS_NAMESPACE::scoped_connection columnWidthChangedConnection;
    boost::BOOST_SIGNALS_NAMESPACE::scoped_connection rowHeightChangedConnection;
};

} // namespace SpreadsheetModGui

#endif // SpreadsheetView_H
