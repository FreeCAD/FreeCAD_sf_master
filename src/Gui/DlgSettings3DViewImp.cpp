/***************************************************************************
 *   Copyright (c) 2002 Jürgen Riegel <juergen.riegel@web.de>              *
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
# include <QApplication>
# include <QRegExp>
# include <QMessageBox>
# include <memory>
#endif

#include "DlgSettings3DViewImp.h"
#include "NavigationStyle.h"
#include "PrefWidgets.h"
#include "View3DInventorViewer.h"
#include "ui_MouseButtons.h"
#include <App/Application.h>
#include <Base/Console.h>
#include <Base/Parameter.h>
#include <Base/Tools.h>

using namespace Gui::Dialog;

/* TRANSLATOR Gui::Dialog::DlgSettings3DViewImp */

bool DlgSettings3DViewImp::showMsg = true;

/**
 *  Constructs a DlgSettings3DViewImp which is a child of 'parent', with the 
 *  name 'name' and widget flags set to 'f' 
 */
DlgSettings3DViewImp::DlgSettings3DViewImp(QWidget* parent)
    : PreferencePage( parent )
{
    this->setupUi(this);
    retranslate();
}

/** 
 *  Destroys the object and frees any allocated resources
 */
DlgSettings3DViewImp::~DlgSettings3DViewImp()
{
    // no need to delete child widgets, Qt does it all for us
}

void DlgSettings3DViewImp::saveSettings()
{
    // must be done as very first because we create a new instance of NavigatorStyle
    // where we set some attributes afterwards
    ParameterGrp::handle hGrp = App::GetApplication().GetParameterGroupByPath
        ("User parameter:BaseApp/Preferences/View");
    QVariant data = comboNavigationStyle->itemData(comboNavigationStyle->currentIndex(), Qt::UserRole);
    hGrp->SetASCII("NavigationStyle", (const char*)data.toByteArray());

    int index = comboOrbitStyle->currentIndex();
    hGrp->SetInt("OrbitStyle", index);
    
    index = this->comboAliasing->currentIndex();
    hGrp->SetInt("AntiAliasing", index);

    checkBoxZoomAtCursor->onSave();
    checkBoxInvertZoom->onSave();
    spinBoxZoomStep->onSave();
    CheckBox_CornerCoordSystem->onSave();
    CheckBox_ShowFPS->onSave();
    CheckBox_UseAutoRotation->onSave();
    FloatSpinBox_EyeDistance->onSave();
    checkBoxBacklight->onSave();
    backlightColor->onSave();
    sliderIntensity->onSave();
    radioPerspective->onSave();
    radioOrthographic->onSave();
}

void DlgSettings3DViewImp::loadSettings()
{
    checkBoxZoomAtCursor->onRestore();
    checkBoxInvertZoom->onRestore();
    spinBoxZoomStep->onRestore();
    CheckBox_CornerCoordSystem->onRestore();
    CheckBox_ShowFPS->onRestore();
    CheckBox_UseAutoRotation->onRestore();
    FloatSpinBox_EyeDistance->onRestore();
    checkBoxBacklight->onRestore();
    backlightColor->onRestore();
    sliderIntensity->onRestore();
    radioPerspective->onRestore();
    radioOrthographic->onRestore();

    ParameterGrp::handle hGrp = App::GetApplication().GetParameterGroupByPath
        ("User parameter:BaseApp/Preferences/View");
    std::string model = hGrp->GetASCII("NavigationStyle",CADNavigationStyle::getClassTypeId().getName());
    int index = comboNavigationStyle->findData(QByteArray(model.c_str()));
    if (index > -1) comboNavigationStyle->setCurrentIndex(index);

    index = hGrp->GetInt("OrbitStyle", int(NavigationStyle::Trackball));
    index = Base::clamp(index, 0, comboOrbitStyle->count()-1);
    comboOrbitStyle->setCurrentIndex(index);
    
    index = hGrp->GetInt("AntiAliasing", int(Gui::View3DInventorViewer::None));
    index = Base::clamp(index, 0, comboAliasing->count()-1);
    comboAliasing->setCurrentIndex(index);
    // connect after setting current item of the combo box
    connect(comboAliasing, SIGNAL(currentIndexChanged(int)),
            this, SLOT(onAliasingChanged(int)));
}

void DlgSettings3DViewImp::on_mouseButton_clicked()
{
    QDialog dlg(this);
    Ui_MouseButtons ui;
    ui.setupUi(&dlg);

    QVariant data = comboNavigationStyle->itemData(comboNavigationStyle->currentIndex(), Qt::UserRole);
    void* instance = Base::Type::createInstanceByName((const char*)data.toByteArray());
    std::unique_ptr<UserNavigationStyle> ns(static_cast<UserNavigationStyle*>(instance));
    ui.groupBox->setTitle(ui.groupBox->title()+QString::fromLatin1(" ")+comboNavigationStyle->currentText());
    QString descr;
    descr = qApp->translate((const char*)data.toByteArray(),ns->mouseButtons(NavigationStyle::SELECTION));
    descr.replace(QLatin1String("\n"), QLatin1String("<p>"));
    ui.selectionLabel->setText(QString::fromLatin1("<b>%1</b>").arg(descr));
    descr = qApp->translate((const char*)data.toByteArray(),ns->mouseButtons(NavigationStyle::PANNING));
    descr.replace(QLatin1String("\n"), QLatin1String("<p>"));
    ui.panningLabel->setText(QString::fromLatin1("<b>%1</b>").arg(descr));
    descr = qApp->translate((const char*)data.toByteArray(),ns->mouseButtons(NavigationStyle::DRAGGING));
    descr.replace(QLatin1String("\n"), QLatin1String("<p>"));
    ui.rotationLabel->setText(QString::fromLatin1("<b>%1</b>").arg(descr));
    descr = qApp->translate((const char*)data.toByteArray(),ns->mouseButtons(NavigationStyle::ZOOMING));
    descr.replace(QLatin1String("\n"), QLatin1String("<p>"));
    ui.zoomingLabel->setText(QString::fromLatin1("<b>%1</b>").arg(descr));
    dlg.exec();
}

/**
 * Sets the strings of the subwidgets using the current language.
 */
void DlgSettings3DViewImp::changeEvent(QEvent *e)
{
    if (e->type() == QEvent::LanguageChange) {
        comboAliasing->blockSignals(true);
        int navigation = comboNavigationStyle->currentIndex();
        int orbit = comboOrbitStyle->currentIndex();
        int aliasing = comboAliasing->currentIndex();
        retranslateUi(this);
        retranslate();
        comboNavigationStyle->setCurrentIndex(navigation);
        comboOrbitStyle->setCurrentIndex(orbit);
        comboAliasing->setCurrentIndex(aliasing);
        comboAliasing->blockSignals(false);
    }
    else {
        QWidget::changeEvent(e);
    }
}

void DlgSettings3DViewImp::retranslate()
{
    comboNavigationStyle->clear();

    // add submenu at the end to select navigation style
    std::map<Base::Type, std::string> styles = UserNavigationStyle::getUserFriendlyNames();
    for (std::map<Base::Type, std::string>::iterator it = styles.begin(); it != styles.end(); ++it) {
        QByteArray data(it->first.getName());
        QString name = QApplication::translate(it->first.getName(), it->second.c_str());

        comboNavigationStyle->addItem(name, data);
    }
}

void DlgSettings3DViewImp::onAliasingChanged(int index)
{
    if (index < 0 || !isVisible())
        return;
    // Show this message only once per application session to reduce
    // annoyance when showing it too often.
    if (showMsg) {
        showMsg = false;
        QMessageBox::information(this, tr("Anti-aliasing"),
            tr("Open a new viewer or restart %1 to apply anti-aliasing changes.").arg(qApp->applicationName()));
    }
}

#include "moc_DlgSettings3DViewImp.cpp"

