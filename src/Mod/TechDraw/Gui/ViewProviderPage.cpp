/***************************************************************************
 *   Copyright (c) 2004 Jürgen Riegel <juergen.riegel@web.de>              *
 *   Copyright (c) 2012 Luke Parry <l.parry@warwick.ac.uk>                 *
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
# include <QAction>
# include <QList>
# include <QMenu>
# include <QMessageBox>
# include <QPointer>
# include <QTextStream>
# include <boost_signals2.hpp>
# include <boost/signals2/connection.hpp>
#endif

#include <App/Document.h>
#include <App/DocumentObject.h>
#include <Base/Console.h>
#include <Gui/Application.h>
#include <Gui/BitmapFactory.h>
#include <Gui/Document.h>
#include <Gui/MainWindow.h>
#include <Gui/ViewProviderDocumentObject.h>

#include <Mod/TechDraw/App/DrawPage.h>
#include <Mod/TechDraw/App/DrawView.h>
#include <Mod/TechDraw/App/DrawProjGroupItem.h>
#include <Mod/TechDraw/App/DrawViewDimension.h>
#include <Mod/TechDraw/App/DrawViewBalloon.h>
#include <Mod/TechDraw/App/DrawLeaderLine.h>
#include <Mod/TechDraw/App/DrawRichAnno.h>
#include <Mod/TechDraw/App/DrawHatch.h>
#include <Mod/TechDraw/App/DrawWeldSymbol.h>

#include "MDIViewPage.h"
#include "PreferencesGui.h"
#include "QGITemplate.h"
#include "QGSPage.h"
#include "QGVPage.h"
#include "ViewProviderTemplate.h"
#include "ViewProviderPage.h"


using namespace TechDrawGui;
using namespace TechDraw;
namespace bp = boost::placeholders;

#define _SHOWDRAWING 10
#define _TOGGLEUPDATE 11

PROPERTY_SOURCE(TechDrawGui::ViewProviderPage, Gui::ViewProviderDocumentObject)


//**************************************************************************
// Construction/Destruction

ViewProviderPage::ViewProviderPage()
  : m_mdiView(nullptr),
    m_docReady(true),
    m_graphicsView(nullptr),
    m_graphicsScene(nullptr)
{
    sPixmap = "TechDraw_TreePage";
    static const char *group = "Grid";

    ADD_PROPERTY_TYPE(ShowFrames ,(true),group,App::Prop_None,"Show or hide View frames and Labels on this Page");
    ADD_PROPERTY_TYPE(ShowGrid ,(PreferencesGui::showGrid()),group,App::Prop_None,"Show or hide a grid on this Page");
    ADD_PROPERTY_TYPE(GridSpacing, (PreferencesGui::gridSpacing()), group, (App::PropertyType)(App::Prop_None),
                     "Grid line spacing in mm");

    ShowFrames.setStatus(App::Property::Hidden,true);
    DisplayMode.setStatus(App::Property::Hidden,true);

    m_graphicsScene = new QGSPage(this);
    m_graphicsScene->setItemIndexMethod(QGraphicsScene::NoIndex); //this prevents crash when deleting dims.
                                                          //scene(view?) indices of dirty regions gets
                                                          //out of sync.  missing prepareGeometryChange
                                                          //somewhere???? QTBUG-18021???
}

ViewProviderPage::~ViewProviderPage()
{
    removeMDIView();                    //if the MDIViewPage is still in MainWindow, remove it.
}

void ViewProviderPage::attach(App::DocumentObject *pcFeat)
{
    ViewProviderDocumentObject::attach(pcFeat);

    auto bnd = boost::bind(&ViewProviderPage::onGuiRepaint, this, bp::_1);
    TechDraw::DrawPage* feature = dynamic_cast<TechDraw::DrawPage*>(pcFeat);
    if (feature) {
        connectGuiRepaint = feature->signalGuiPaint.connect(bnd);
        m_pageName = feature->getNameInDocument();
        m_graphicsScene->setObjectName(QString::fromLocal8Bit(m_pageName.c_str()));
    } else {
        Base::Console().Log("VPP::attach has no Feature!\n");
    }
}

void ViewProviderPage::setDisplayMode(const char* ModeName)
{
    ViewProviderDocumentObject::setDisplayMode(ModeName);
}

std::vector<std::string> ViewProviderPage::getDisplayModes() const
{
    // get the modes of the father
    std::vector<std::string> StrList = ViewProviderDocumentObject::getDisplayModes();
    StrList.emplace_back("Drawing");
    return StrList;
}

void ViewProviderPage::onChanged(const App::Property *prop)
{
    if (prop == &(ShowGrid)) {
        setGrid();
    } else if (prop == &(GridSpacing)) {
        setGrid();
    } else if (prop == &Visibility) {
        //Visibility changes are handled in VPDO::onChanged -> show() or hide()
    }

    Gui::ViewProviderDocumentObject::onChanged(prop);
}

void ViewProviderPage::updateData(const App::Property* prop)
{
    auto page = getDrawPage();
    if (!page) {
        Gui::ViewProviderDocumentObject::updateData(prop);
        return;
    }
    if (prop == &(page->KeepUpdated)) {
       if (getDrawPage()->KeepUpdated.getValue()) {
           sPixmap = "TechDraw_TreePage";
       } else {
           sPixmap = "TechDraw_TreePageUnsync";
       }
       signalChangeIcon();
    //if the template is changed, rebuild the visual
    } else if (prop == &(page->Template)) {
       if (!page->isUnsetting()) {
           //check if a template has been added to scene first?
            m_graphicsScene->matchSceneRectToTemplate();
            m_graphicsScene->updateTemplate();
        }
    } else if (prop == &(page->Label)) {
       if (m_mdiView &&
          !page->isUnsetting()) {
           m_mdiView->setTabText(page->Label.getValue());
       }
    } else if (prop == &page->Views) {
        if (!page->isUnsetting())
            m_graphicsScene->fixOrphans();
    }

    Gui::ViewProviderDocumentObject::updateData(prop);
}

bool ViewProviderPage::onDelete(const std::vector<std::string> &)
{
    // warn the user if the Page is not empty
    // but don't do this if there is just the template

    // check if there are items in the group
    auto objs = claimChildren();

    // check if there is just a template
    // if there are several objects, the template is never the last one
    // the ExportName of a template always begins with "Template"
    bool isTemplate = false;
    for (auto objsIterator : objs) {
        if (objsIterator->getExportName().substr(0, 8).compare(std::string("Template")) == 0)
            isTemplate = true;
        else
            isTemplate = false;
    }

    if (!objs.empty() && !isTemplate)
    {
        // generate dialog
        QString bodyMessage;
        QTextStream bodyMessageStream(&bodyMessage);
        bodyMessageStream << qApp->translate("Std_Delete",
            "The page is not empty, therefore the\nfollowing referencing objects might be lost:");
        bodyMessageStream << '\n';
        for (auto ObjIterator : objs)
            bodyMessageStream << '\n' << QString::fromUtf8(ObjIterator->Label.getValue());
        bodyMessageStream << "\n\n" << QObject::tr("Are you sure you want to continue?");
        // show and evaluate the dialog
        int DialogResult = QMessageBox::warning(Gui::getMainWindow(),
            qApp->translate("Std_Delete", "Object dependencies"), bodyMessage,
            QMessageBox::Yes, QMessageBox::No);
        if (DialogResult == QMessageBox::Yes) {
            removeMDIView();
            return true;
        } else
            return false;
    }
    else {
        removeMDIView();
        return true;
    }
}

void ViewProviderPage::setupContextMenu(QMenu* menu, QObject* receiver, const char* member)
{
    Gui::ViewProviderDocumentObject::setupContextMenu(menu, receiver, member);
    QAction* act = menu->addAction(QObject::tr("Show drawing"), receiver, member);
    act->setData(QVariant((int) _SHOWDRAWING));
    QAction* act2 = menu->addAction(QObject::tr("Toggle KeepUpdated"), receiver, member);
    act2->setData(QVariant((int) _TOGGLEUPDATE));
}

bool ViewProviderPage::setEdit(int ModNum)
{
    if (ModNum == _SHOWDRAWING) {
        showMDIViewPage();   // show the drawing
        return false;  //finished editing
    } else if (ModNum == _TOGGLEUPDATE) {
         auto page = getDrawPage();
         if (page) {
             page->KeepUpdated.setValue(!page->KeepUpdated.getValue());
             page->recomputeFeature();
         }
         return false;
    } else {
        return Gui::ViewProviderDocumentObject::setEdit(ModNum);
    }
}

void ViewProviderPage::unsetEdit(int ModNum)
{
    Q_UNUSED(ModNum);
    return;
}

bool ViewProviderPage::doubleClicked(void)
{
    show();
    if (m_mdiView) {
        Gui::getMainWindow()->setActiveWindow(m_mdiView);
    }
    return true;
}

void ViewProviderPage::show(void)
{
//    Base::Console().Message("VPP::show()\n");
    showMDIViewPage();
    ViewProviderDocumentObject::show();
}

void ViewProviderPage::hide(void)
{
//    Base::Console().Message("VPP::hide()\n");
    if (getMDIView()) {
        getMDIView()->hide();     //this doesn't remove the mdiViewPage from the mainWindow
        removeMDIView();
    }
    ViewProviderDocumentObject::hide();
}

bool ViewProviderPage::showMDIViewPage()
{
    if (m_mdiView.isNull()){
        createMDIViewPage();
        m_graphicsScene->addChildrenToPage();
        m_graphicsScene->updateTemplate(true);
        m_graphicsScene->redrawAllViews();
        m_graphicsScene->fixOrphans(true);
    } else {
        m_graphicsScene->redrawAllViews();
        m_graphicsScene->fixOrphans(true);
    }
    m_graphicsView->centerOnPage();

    m_mdiView->viewAll();
    m_mdiView->showMaximized();

    setGrid();

    return true;
}

void ViewProviderPage::createMDIViewPage()
{
//    Base::Console().Message("VPP::createMDIViewPage()\n");
    Gui::Document* doc = Gui::Application::Instance->getDocument
        (pcObject->getDocument());
    m_mdiView = new MDIViewPage(this, doc, Gui::getMainWindow());
    if (m_graphicsView == nullptr) {
        m_graphicsView = new QGVPage(this, m_graphicsScene, m_mdiView);
        std::string objName = m_pageName + "View";
        m_graphicsView->setObjectName(QString::fromLocal8Bit(objName.c_str()));
    }
    m_mdiView->setScene(m_graphicsScene, m_graphicsView);
    m_graphicsScene->setParent(m_graphicsView);   //for QObjectParent.  required??
    QString tabTitle = QString::fromUtf8(getDrawPage()->Label.getValue());

    m_mdiView->setDocumentObject(getDrawPage()->getNameInDocument());
    m_mdiView->setDocumentName(pcObject->getDocument()->getName());

    m_mdiView->setWindowTitle(tabTitle + QString::fromLatin1("[*]"));
    m_mdiView->setWindowIcon(Gui::BitmapFactory().pixmap("TechDraw_TreePage"));
    Gui::getMainWindow()->addWindow(m_mdiView);
    Gui::getMainWindow()->setActiveWindow(m_mdiView);
}

//NOTE: removing MDIViewPage (parent) destroys QGVPage which will
//delete QGSPage if still parented to QGVPage
void ViewProviderPage::removeMDIView(void)
{
//    Base::Console().Message("VPP::removeMDIViewPage()\n");
    if (!m_mdiView.isNull()) {                                //m_mdiView is a QPointer
        // https://forum.freecadweb.org/viewtopic.php?f=3&t=22797&p=182614#p182614
        //Gui::getMainWindow()->activatePreviousWindow();
        QList<QWidget*> wList= Gui::getMainWindow()->windows();
        bool found = wList.contains(m_mdiView);
        if (found) {
            m_graphicsScene->setParent(nullptr);
            Gui::getMainWindow()->removeWindow(m_mdiView);
            m_graphicsView = nullptr;       //m_graphicsView has been deleted by m_mdiView
            Gui::MDIView* aw = Gui::getMainWindow()->activeWindow();  //WF: this bit should be in the remove window logic, not here.
            if (aw != nullptr) {
                aw->showMaximized();
            }
        }
    }
}

MDIViewPage* ViewProviderPage::getMDIViewPage() const
{
    if (m_mdiView.isNull()) {
        Base::Console().Log("VPP::getMDIViewPage has no m_mdiView!\n");
        return nullptr;
    }
    return m_mdiView;
}

std::vector<App::DocumentObject*> ViewProviderPage::claimChildren(void) const
{
    std::vector<App::DocumentObject*> temp;

    App::DocumentObject *templateFeat = nullptr;
    templateFeat = getDrawPage()->Template.getValue();

    if (templateFeat) {
        temp.push_back(templateFeat);
    }

    // Collect any child views
    // for Page, valid children are any View except: DrawProjGroupItem
    //                                               DrawViewDimension
    //                                               DrawViewBalloon
    //                                               DrawLeaderLine
    //                                               DrawRichAnno
    //                                               any FeatuerView in a DrawViewClip
    //                                               DrawHatch
    //                                               DrawWeldSymbol

    const std::vector<App::DocumentObject *> &views = getDrawPage()->Views.getValues();

    try {
        for (std::vector<App::DocumentObject *>::const_iterator it = views.begin(); it != views.end(); ++it) {
            TechDraw::DrawView* featView = dynamic_cast<TechDraw::DrawView*> (*it);
            App::DocumentObject *docObj = *it;
            //DrawRichAnno with no parent is child of Page
            TechDraw::DrawRichAnno* dra = dynamic_cast<TechDraw::DrawRichAnno*> (*it);
            if (dra) {
                if (!dra->AnnoParent.getValue()) {
                    temp.push_back(*it); //no parent, belongs to page
                }
                continue; //has a parent somewhere else
            }

            // Don't collect if dimension, projection group item, hatch or member of ClipGroup as these should be grouped elsewhere
            if (docObj->isDerivedFrom(TechDraw::DrawProjGroupItem::getClassTypeId())    ||
                docObj->isDerivedFrom(TechDraw::DrawViewDimension::getClassTypeId())    ||
                docObj->isDerivedFrom(TechDraw::DrawHatch::getClassTypeId())            ||
                docObj->isDerivedFrom(TechDraw::DrawViewBalloon::getClassTypeId())      ||
                docObj->isDerivedFrom(TechDraw::DrawRichAnno::getClassTypeId())         ||
                docObj->isDerivedFrom(TechDraw::DrawLeaderLine::getClassTypeId())       ||
                docObj->isDerivedFrom(TechDraw::DrawWeldSymbol::getClassTypeId())       ||
                (featView && featView->isInClip()) )
                continue;
            else
                temp.push_back(*it);
        }
        return temp;
    } catch (...) {
        return std::vector<App::DocumentObject*>();
    }
}

void ViewProviderPage::startRestoring()
{
//    Base::Console().Message("VPP::startRestoring()\n");
    //VPDO::startRestoring hides this VPP
    Gui::ViewProviderDocumentObject::startRestoring();
}

//note this is called when this ViewProvider has been restored, not
//when the whole document has been restored.
void ViewProviderPage::finishRestoring()
{
//    Base::Console().Message("VPP::finishRestoring() - canUpdate: %d viz: %d\n",
//                            getDrawPage()->canUpdate(), Visibility.getValue());
    Gui::ViewProviderDocumentObject::finishRestoring();
}

bool ViewProviderPage::isShow() const
{
    return Visibility.getValue();
}

bool ViewProviderPage::getFrameState()
{
    return ShowFrames.getValue();
}

void ViewProviderPage::setFrameState(bool state)
{
    ShowFrames.setValue(state);
}

void ViewProviderPage::toggleFrameState()
{
//    Base::Console().Message("VPP::toggleFrameState()\n");
    if (m_graphicsScene) {
        setFrameState(!getFrameState());
        m_graphicsScene->refreshViews();
        setTemplateMarkers(getFrameState());
    }
}

void ViewProviderPage::setTemplateMarkers(bool state)
{
//    Base::Console().Message("VPP::setTemplateMarkers(%d)\n",state);
    App::DocumentObject *templateFeat = nullptr;
    templateFeat = getDrawPage()->Template.getValue();
    Gui::Document* guiDoc = Gui::Application::Instance->getDocument(templateFeat->getDocument());
    Gui::ViewProvider* vp = guiDoc->getViewProvider(templateFeat);
    ViewProviderTemplate* vpt = dynamic_cast<ViewProviderTemplate*>(vp);
    if (vpt) {
        vpt->setMarkers(state);
        QGITemplate* t = vpt->getQTemplate();
        if (t) {
            t->updateView(true);
        }
    }
}

bool ViewProviderPage::canDelete(App::DocumentObject *obj) const
{
    // deletions from a page don't necessarily destroy anything
    // thus we can pass this action
    // if an object could break something, like e.g. the template object
    // its ViewProvider handles this in the onDelete() function
    Q_UNUSED(obj)
    return true;
}

//! Redo the whole visual page
void ViewProviderPage::onGuiRepaint(const TechDraw::DrawPage* dp)
{
    if (dp == getDrawPage()) {
        //this signal is for us
        if (!getDrawPage()->isUnsetting()) {
            m_graphicsScene->fixOrphans();
        }
    }
}

TechDraw::DrawPage* ViewProviderPage::getDrawPage() const
{
    //during redo, pcObject can become invalid, but non-zero??
    if (!pcObject) {
        Base::Console().Log("VPP::getDrawPage - no Page Object!\n");
        return nullptr;
    }
    return dynamic_cast<TechDraw::DrawPage*>(pcObject);
}

Gui::MDIView *ViewProviderPage::getMDIView() const
{
    return m_mdiView.data();
}

void  ViewProviderPage::setGrid()
{
    TechDraw::DrawPage* dp = getDrawPage();
    if (!dp) {
        return;
    }
    int pageWidth = 298;
    int pageHeight = 215;
    double gridStep = GridSpacing.getValue() > 0 ? GridSpacing.getValue() : 10.0;
    if (dp) {
        pageWidth = dp->getPageWidth();
        pageHeight = dp->getPageHeight();
    }
    QGVPage* widget = getGraphicsView();
    if (widget) {
        if (ShowGrid.getValue()) {
            widget->showGrid(true);
            widget->makeGrid(pageWidth, pageHeight, gridStep);
        } else {
            widget->showGrid(false);
        }
        widget->updateViewport();
    }
}
