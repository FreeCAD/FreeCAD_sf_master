/***************************************************************************
 *   Copyright (c) 2004 Werner Mayer <wmayer[at]users.sourceforge.net>     *
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
# include <boost/signals2.hpp>
# include <boost_bind_bind.hpp>
# include <QAbstractItemView>
# include <QActionEvent>
# include <QApplication>
# include <QDesktopWidget>
# include <QEvent>
# include <QMessageBox>
# include <QTimer>
# include <QToolBar>
# include <QToolButton>
# include <QElapsedTimer>
#endif

#include <QWidgetAction>

#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
# include <QScreen>
#endif

#include <boost/algorithm/string/predicate.hpp>

#include <Base/Tools.h>
#include <Base/Parameter.h>
#include <App/DocumentObserver.h>
#include "Action.h"
#include "Application.h"
#include "Command.h"
#include "DlgUndoRedo.h"
#include "DlgWorkbenchesImp.h"
#include "FileDialog.h"
#include "MainWindow.h"
#include "WhatsThis.h"
#include "Widgets.h"
#include "Workbench.h"
#include "WorkbenchManager.h"
#include "View3DInventor.h"
#include "Document.h"
#include "SelectionView.h"
#include "ViewParams.h"
#include "BitmapFactory.h"

#include <Base/Exception.h>
#include <App/Application.h>

using namespace Gui;
using namespace Gui::Dialog;
namespace bp = boost::placeholders;

/**
 * Constructs an action called \a name with parent \a parent. It also stores a pointer
 * to the command object.
 */
Action::Action (Command* pcCmd, QObject * parent)
  : QObject(parent), _action(new QAction( this )), _pcCmd(pcCmd)
{
    _action->setObjectName(QString::fromLatin1(_pcCmd->getName()));
    connect(_action, SIGNAL(triggered(bool)), this, SLOT(onActivated()));
}

Action::Action (Command* pcCmd, QAction* action, QObject * parent)
  : QObject(parent), _action(action), _pcCmd(pcCmd)
{
    _action->setParent(this);
    _action->setObjectName(QString::fromLatin1(_pcCmd->getName()));
    connect(_action, SIGNAL(triggered(bool)), this, SLOT(onActivated()));
}

Action::~Action()
{
    delete _action;
}

/**
 * Adds this action to widget \a w.
 */
void Action::addTo(QWidget *w)
{
    w->addAction(_action);
}

/**
 * Activates the command.
 */
void Action::onActivated () 
{
    _pcCmd->invoke(0,Command::TriggerAction);
}

/**
 * Sets whether the command is toggled.
 */
void Action::onToggled(bool b)
{
    _pcCmd->invoke( b ? 1 : 0 , Command::TriggerAction);
} 

void Action::setCheckable(bool b)
{
    if(b == _action->isCheckable())
        return;
    _action->setCheckable(b);
    if (b) {
        disconnect(_action, SIGNAL(triggered(bool)), this, SLOT(onActivated()));
        connect(_action, SIGNAL(toggled(bool)), this, SLOT(onToggled(bool)));
    }
    else {
        connect(_action, SIGNAL(triggered(bool)), this, SLOT(onActivated()));
        disconnect(_action, SIGNAL(toggled(bool)), this, SLOT(onToggled(bool)));
    }
}

void Action::setChecked(bool b, bool no_signal)
{
    bool blocked;
    if(no_signal) 
        blocked = _action->blockSignals(true);
    _action->setChecked(b);
    if(no_signal)
        _action->blockSignals(blocked);
}

bool Action::isChecked() const
{
    return _action->isChecked();
}

/**
 * Sets whether the action is enabled.
 */
void Action::setEnabled(bool b) 
{
    _action->setEnabled(b);
}

void Action::setVisible(bool b) 
{
    _action->setVisible(b);
}

void Action::setShortcut(const QString & key)
{
    _action->setShortcut(key);
}

QKeySequence Action::shortcut() const
{
    return _action->shortcut();
}

void Action::setIcon (const QIcon & icon)
{
    _action->setIcon(icon);
}

QIcon Action::icon () const
{
    return _action->icon();
}

void Action::setStatusTip(const QString & s)
{
    _action->setStatusTip(s);
}

QString Action::statusTip() const
{
    return _action->statusTip();
}

void Action::setText(const QString & s)
{
    _action->setText(s);
}

QString Action::text() const
{
    return _action->text();
}

void Action::setToolTip(const QString & s)
{
    _action->setToolTip(s);
}
  
QString Action::toolTip() const
{
    return _action->toolTip();
}

void Action::setWhatsThis(const QString & s)
{
    _action->setWhatsThis(s);
}

QString Action::whatsThis() const
{
    return _action->whatsThis();
}

void Action::setMenuRole(QAction::MenuRole menuRole)
{
    _action->setMenuRole(menuRole); 
}

// --------------------------------------------------------------------

/**
 * Constructs an action called \a name with parent \a parent. It also stores a pointer
 * to the command object.
 */
ActionGroup::ActionGroup ( Command* pcCmd,QObject * parent)
  : Action(pcCmd, parent), _group(0), _dropDown(false),_external(false),_toggle(false)
{
    _group = new QActionGroup(this);
    connect(_group, SIGNAL(triggered(QAction*)), this, SLOT(onActivated (QAction*)));
    connect(_group, SIGNAL(hovered(QAction*)), this, SLOT(onHovered(QAction*)));
}

ActionGroup::~ActionGroup()
{
    delete _group;
}

/**
 * Adds this action to widget \a w.
 */
void ActionGroup::addTo(QWidget *w)
{
    // When adding an action that has defined a menu then shortcuts
    // of the menu actions don't work. To make this working we must 
    // set the menu explicitly. This means calling QAction::setMenu()
    // and adding this action to the widget doesn't work.
    if (_dropDown) {
        if (w->inherits("QMenu")) {
            QMenu* menu = qobject_cast<QMenu*>(w);
            menu = menu->addMenu(_action->text());
            menu->addActions(_group->actions());
        }
        else if (w->inherits("QToolBar")) {
            w->addAction(_action);
            QToolButton* tb = w->findChildren<QToolButton*>().last();
            tb->setPopupMode(QToolButton::MenuButtonPopup);
            tb->setObjectName(QString::fromLatin1("qt_toolbutton_menubutton"));
            QList<QAction*> acts = _group->actions();
            QMenu* menu = new QMenu(tb);
            menu->addActions(acts);
            tb->setMenu(menu);
            //tb->addActions(_group->actions());
        }
        else {
            w->addActions(_group->actions()); // no drop-down 
        }
    }
    else {
        w->addActions(_group->actions());
    }
}

void ActionGroup::setEnabled( bool b )
{
    Action::setEnabled(b);
    _group->setEnabled(b);
}

void ActionGroup::setDisabled (bool b)
{
    Action::setEnabled(!b);
    _group->setDisabled(b);
}

void ActionGroup::setExclusive (bool b)
{
    _group->setExclusive(b);
}

bool ActionGroup::isExclusive() const
{
    return _group->isExclusive();
}

void ActionGroup::setVisible( bool b )
{
    Action::setVisible(b);
    _group->setVisible(b);
}

QAction* ActionGroup::addAction(QAction* action)
{
    int index = _group->actions().size();
    action = _group->addAction(action);
    action->setData(QVariant(index));
    return action;
}

QAction* ActionGroup::addAction(const QString& text)
{
    int index = _group->actions().size();
    QAction* action = _group->addAction(text);
    action->setData(QVariant(index));
    return action;
}

QList<QAction*> ActionGroup::actions() const
{
    return _group->actions();
}

int ActionGroup::checkedAction() const
{
    QAction* checked = _group->checkedAction();
    return checked ? checked->data().toInt() : -1;
}

void ActionGroup::setCheckedAction(int i)
{
    QAction* a = _group->actions()[i];
    a->setChecked(true);
    this->setIcon(a->icon());
    this->setToolTip(a->toolTip());
    this->setProperty("defaultAction", QVariant(i));
}

/**
 * Activates the command.
 */
void ActionGroup::onActivated () 
{
    _pcCmd->invoke(this->property("defaultAction").toInt(), Command::TriggerAction);
}

void ActionGroup::onToggled(bool checked)
{
    (void)checked;
    onActivated();
} 

/**
 * Activates the command.
 */
void ActionGroup::onActivated (QAction* a) 
{
    int index = _group->actions().indexOf(a);

    // Calling QToolButton::setIcon() etc. has no effect if it has QAction set.
    // We have to change the QAction icon instead
#if 0
    QList<QWidget*> widgets = a->associatedWidgets();
    for (QList<QWidget*>::iterator it = widgets.begin(); it != widgets.end(); ++it) {
        QMenu* menu = qobject_cast<QMenu*>(*it);
        if (menu) {
            QToolButton* button = qobject_cast<QToolButton*>(menu->parent());
            if (button) {
                button->setIcon(a->icon());
                button->setText(a->text());
                button->setToolTip(a->toolTip());
                this->setProperty("defaultAction", QVariant(index));
            }
        }
    }
#endif

    // The following logic is moved to Command::onInvoke()
#if 0
    this->setIcon(a->icon());
    this->setToolTip(a->toolTip());
    this->setProperty("defaultAction", QVariant(index));
#endif

    _pcCmd->invoke(index, Command::TriggerChildAction);
}

void ActionGroup::onHovered (QAction *a) 
{
    Gui::ToolTip::showText(QCursor::pos(), a->toolTip());
}


// --------------------------------------------------------------------

namespace Gui {

/**
 * The WorkbenchActionEvent class is used to send an event of which workbench must be activated.
 * We cannot activate the workbench directly as we will destroy the widget that emits the signal.
 * @author Werner Mayer
 */
class WorkbenchActionEvent : public QEvent
{
public:
    WorkbenchActionEvent(QAction* a)
      : QEvent(QEvent::User), act(a)
    { }
    ~WorkbenchActionEvent()
    { }
    QAction* action() const
    { return act; }
private:
    QAction* act;
};
}

WorkbenchComboBox::WorkbenchComboBox(WorkbenchGroup* wb, QWidget* parent) : QComboBox(parent), group(wb)
{
    connect(this, SIGNAL(activated(int)), this, SLOT(onActivated(int)));
    connect(getMainWindow(), SIGNAL(workbenchActivated(const QString&)), 
            this, SLOT(onWorkbenchActivated(const QString&)));
}

WorkbenchComboBox::~WorkbenchComboBox()
{
}

void WorkbenchComboBox::showPopup()
{
    int rows = count();
    if (rows > 0) {
        int height = view()->sizeHintForRow(0);
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
        int maxHeight = QApplication::primaryScreen()->size().height();
#else
        int maxHeight = QApplication::desktop()->height();
#endif
        view()->setMinimumHeight(qMin(height * rows, maxHeight/2));
    }

    QComboBox::showPopup();
}

void WorkbenchComboBox::actionEvent ( QActionEvent* e )
{
    QAction *action = e->action();
    switch (e->type()) {
    case QEvent::ActionAdded:
        {
            if (action->isVisible()) {
                QIcon icon = action->icon();
                if (icon.isNull())
                    this->addItem(action->text(), action->data());
                else
                    this->addItem(icon, action->text(), action->data());
                if (action->isChecked())
                    this->setCurrentIndex(action->data().toInt());
            }
            break;
        }
    case QEvent::ActionChanged:
        {
            QVariant data = action->data();
            int index = this->findData(data);
            // added a workbench
            if (index < 0 && action->isVisible()) {
                QIcon icon = action->icon();
                if (icon.isNull())
                    this->addItem(action->text(), data);
                else
                    this->addItem(icon, action->text(), data);
            }
            // removed a workbench
            else if (index >=0 && !action->isVisible()) {
                this->removeItem(index);
            }
            break;
        }
    case QEvent::ActionRemoved:
        {
            //Nothing needs to be done
            break;
        }
    default:
        break;
    }
}

void WorkbenchComboBox::onActivated(int i)
{
    // Send the event to the workbench group to delay the destruction of the emitting widget.
    int index = itemData(i).toInt();
    WorkbenchActionEvent* ev = new WorkbenchActionEvent(this->actions()[index]);
    QApplication::postEvent(this->group, ev);
    // TODO: Test if we can use this instead
    //QTimer::singleShot(20, this->actions()[i], SLOT(trigger()));
}

void WorkbenchComboBox::onActivated(QAction* action)
{
    // set the according item to the action
    QVariant data = action->data();
    int index = this->findData(data);
    setCurrentIndex(index);
}

void WorkbenchComboBox::onWorkbenchActivated(const QString& name)
{
    // There might be more than only one instance of WorkbenchComboBox there.
    // However, all of them share the same QAction objects. Thus, if the user
    // has  selected one it also gets checked. Then Application::activateWorkbench
    // also invokes this slot method by calling the signal workbenchActivated in
    // MainWindow. If calling activateWorkbench() from within the Python console
    // the matching action must be set by calling this function.
    // To avoid to recursively (but only one recursion level) call Application::
    // activateWorkbench the method refreshWorkbenchList() shouldn't set the
    // checked item.
    //QVariant item = itemData(currentIndex());
    QList<QAction*> a = actions();
    for (QList<QAction*>::Iterator it = a.begin(); it != a.end(); ++it) {
        if ((*it)->objectName() == name) {
            if (/*(*it)->data() != item*/!(*it)->isChecked())
                (*it)->trigger();
            break;
        }
    }
}

/* TRANSLATOR Gui::WorkbenchGroup */
WorkbenchGroup::WorkbenchGroup (  Command* pcCmd, QObject * parent )
  : ActionGroup( pcCmd, parent )
{
    for (int i=0; i<50; i++) {
        QAction* action = _group->addAction(QLatin1String(""));
        action->setVisible(false);
        action->setCheckable(true);
        action->setData(QVariant(i)); // set the index
    }

    Application::Instance->signalActivateWorkbench.connect(boost::bind(&WorkbenchGroup::slotActivateWorkbench, this, bp::_1));
    Application::Instance->signalAddWorkbench.connect(boost::bind(&WorkbenchGroup::slotAddWorkbench, this, bp::_1));
    Application::Instance->signalRemoveWorkbench.connect(boost::bind(&WorkbenchGroup::slotRemoveWorkbench, this, bp::_1));
}

WorkbenchGroup::~WorkbenchGroup()
{
}

void WorkbenchGroup::addTo(QWidget *w)
{
    refreshWorkbenchList();
    if (w->inherits("QToolBar")) {
        QToolBar* bar = qobject_cast<QToolBar*>(w);
        QComboBox* box = new WorkbenchComboBox(this, w);
        box->setIconSize(QSize(16, 16));
        box->setToolTip(_action->toolTip());
        box->setStatusTip(_action->statusTip());
        box->setWhatsThis(_action->whatsThis());
        box->addActions(_group->actions());
        connect(_group, SIGNAL(triggered(QAction*)), box, SLOT(onActivated (QAction*)));
        bar->addWidget(box);
    }
    else if (w->inherits("QMenu")) {
        QMenu* menu = qobject_cast<QMenu*>(w);
        menu = menu->addMenu(_action->text());
        menu->addActions(_group->actions());
    }
}

void WorkbenchGroup::setWorkbenchData(int i, const QString& wb)
{
    QList<QAction*> workbenches = _group->actions();
    QString name = Application::Instance->workbenchMenuText(wb);
    QPixmap px = Application::Instance->workbenchIcon(wb);
    QString tip = Application::Instance->workbenchToolTip(wb);

    workbenches[i]->setObjectName(wb);
    workbenches[i]->setIcon(px);
    workbenches[i]->setText(name);
    workbenches[i]->setToolTip(tip);
    workbenches[i]->setStatusTip(tr("Select the '%1' workbench").arg(name));
    workbenches[i]->setVisible(true);
    if (i < 9)
        workbenches[i]->setShortcut(QKeySequence(QString::fromUtf8("W,%1").arg(i+1)));
}

void WorkbenchGroup::refreshWorkbenchList()
{
    QStringList items = Application::Instance->workbenches();
    QStringList enabled_wbs_list = DlgWorkbenchesImp::load_enabled_workbenches();
    QStringList disabled_wbs_list = DlgWorkbenchesImp::load_disabled_workbenches();
    int i=0;

    // Go through the list of enabled workbenches and verify that they really exist because
    // it might be possible that a workbench has been removed after setting up the list of
    // enabled workbenches.
    for (QStringList::Iterator it = enabled_wbs_list.begin(); it != enabled_wbs_list.end(); ++it) {
        int index = items.indexOf(*it);
        if (index >= 0) {
            setWorkbenchData(i++, *it);
            items.removeAt(index);
        }
    }

    // Filter out the actively disabled workbenches
    for (QStringList::Iterator it = disabled_wbs_list.begin(); it != disabled_wbs_list.end(); ++it) {
        int index = items.indexOf(*it);
        if (index >= 0) {
            items.removeAt(index);
        }
    }

    // Now add the remaining workbenches of 'items'. They have been added to the application
    // after setting up the list of enabled workbenches.
    for (QStringList::Iterator it = items.begin(); it != items.end(); ++it) {
        setWorkbenchData(i++, *it);
    }
}

void WorkbenchGroup::customEvent( QEvent* e )
{
    if (e->type() == QEvent::User) {
        Gui::WorkbenchActionEvent* ce = (Gui::WorkbenchActionEvent*)e;
        ce->action()->trigger();
    }
}

void WorkbenchGroup::slotActivateWorkbench(const char* /*name*/)
{
}

void WorkbenchGroup::slotAddWorkbench(const char* name)
{
    QList<QAction*> workbenches = _group->actions();
    for (QList<QAction*>::Iterator it = workbenches.begin(); it != workbenches.end(); ++it) {
        if (!(*it)->isVisible()) {
            QString wb = QString::fromLatin1(name);
            QPixmap px = Application::Instance->workbenchIcon(wb);
            QString text = Application::Instance->workbenchMenuText(wb);
            QString tip = Application::Instance->workbenchToolTip(wb);
            (*it)->setIcon(px);
            (*it)->setObjectName(wb);
            (*it)->setText(text);
            (*it)->setToolTip(tip);
            (*it)->setStatusTip(tr("Select the '%1' workbench").arg(wb));
            (*it)->setVisible(true); // do this at last
            break;
        }
    }
}

void WorkbenchGroup::slotRemoveWorkbench(const char* name)
{
    QString workbench = QString::fromLatin1(name);
    QList<QAction*> workbenches = _group->actions();
    for (QList<QAction*>::Iterator it = workbenches.begin(); it != workbenches.end(); ++it) {
        if ((*it)->objectName() == workbench) {
            (*it)->setObjectName(QString());
            (*it)->setIcon(QIcon());
            (*it)->setText(QString());
            (*it)->setToolTip(QString());
            (*it)->setStatusTip(QString());
            (*it)->setVisible(false); // do this at last
            break;
        }
    }
}

// --------------------------------------------------------------------

/* TRANSLATOR Gui::RecentFilesAction */

RecentFilesAction::RecentFilesAction ( Command* pcCmd, QObject * parent )
  : ActionGroup( pcCmd, parent ), visibleItems(4), maximumItems(20)
{
    restore();
}

RecentFilesAction::~RecentFilesAction()
{
}

/** Adds the new item to the recent files. */
void RecentFilesAction::appendFile(const QString& filename)
{
    // restore the list of recent files
    QStringList files = this->files();

    // if already inside remove and prepend it
    files.removeAll(filename);
    files.prepend(filename);
    setFiles(files);
    save();

    // update the XML structure and save the user parameter to disk (#0001989)
    bool saveParameter = App::GetApplication().GetParameterGroupByPath
        ("User parameter:BaseApp/Preferences/General")->GetBool("SaveUserParameter", true);
    if (saveParameter) {
        ParameterManager* parmgr = App::GetApplication().GetParameterSet("User parameter");
        parmgr->SaveDocument(App::Application::Config()["UserParameter"].c_str());
    }
}

/**
 * Set the list of recent files. For each item an action object is
 * created and added to this action group. 
 */
void RecentFilesAction::setFiles(const QStringList& files)
{
    QList<QAction*> recentFiles = _group->actions();

    int numRecentFiles = std::min<int>(recentFiles.count(), files.count());
    for (int index = 0; index < numRecentFiles; index++) {
        QFileInfo fi(files[index]);
        recentFiles[index]->setText(QString::fromLatin1("%1 %2").arg(index+1).arg(fi.fileName()));
        recentFiles[index]->setStatusTip(tr("Open file %1").arg(files[index]));
        recentFiles[index]->setToolTip(files[index]); // set the full name that we need later for saving
        recentFiles[index]->setData(QVariant(index));
        recentFiles[index]->setVisible(true);
    }

    // if less file names than actions
    numRecentFiles = std::min<int>(numRecentFiles, this->visibleItems);
    for (int index = numRecentFiles; index < recentFiles.count(); index++) {
        recentFiles[index]->setVisible(false);
        recentFiles[index]->setText(QString());
        recentFiles[index]->setToolTip(QString());
    }
}

/**
 * Returns the list of defined recent files.
 */
QStringList RecentFilesAction::files() const
{
    QStringList files;
    QList<QAction*> recentFiles = _group->actions();
    for (int index = 0; index < recentFiles.count(); index++) {
        QString file = recentFiles[index]->toolTip();
        if (file.isEmpty())
            break;
        files.append(file);
    }

    return files;
}

void RecentFilesAction::activateFile(int id)
{
    // restore the list of recent files
    QStringList files = this->files();
    if (id < 0 || id >= files.count())
        return; // no valid item

    QString filename = files[id];
    QFileInfo fi(filename);
    if (!fi.exists()) {
        QMessageBox::critical(getMainWindow(), tr("File not found"), tr("The file '%1' cannot be opened.").arg(filename));
        files.removeAll(filename);
        setFiles(files);
    }
    else {
        // invokes appendFile()
        SelectModule::Dict dict = SelectModule::importHandler(filename);
        for (SelectModule::Dict::iterator it = dict.begin(); it != dict.end(); ++it) {
            Application::Instance->open(it.key().toUtf8(), it.value().toLatin1());
            break;
        }
    }
}

void RecentFilesAction::resizeList(int size)
{
    this->visibleItems = size;
    int diff = this->visibleItems - this->maximumItems;
    // create new items if needed
    for (int i=0; i<diff; i++)
        _group->addAction(QLatin1String(""))->setVisible(false);
    setFiles(files());
}

/** Loads all recent files from the preferences. */
void RecentFilesAction::restore()
{
    ParameterGrp::handle hGrp = App::GetApplication().GetUserParameter().GetGroup("BaseApp")->GetGroup("Preferences");
    if (hGrp->HasGroup("RecentFiles")) {
        hGrp = hGrp->GetGroup("RecentFiles");
        // we want at least 20 items but we do only show the number of files
        // that is defined in user parameters
        this->visibleItems = hGrp->GetInt("RecentFiles", this->visibleItems);
    }

    int count = std::max<int>(this->maximumItems, this->visibleItems);
    for (int i=0; i<count; i++)
        _group->addAction(QLatin1String(""))->setVisible(false);
    std::vector<std::string> MRU = hGrp->GetASCIIs("MRU");
    QStringList files;
    for (std::vector<std::string>::iterator it = MRU.begin(); it!=MRU.end();++it)
        files.append(QString::fromUtf8(it->c_str()));
    setFiles(files);
}

/** Saves all recent files to the preferences. */
void RecentFilesAction::save()
{
    ParameterGrp::handle hGrp = App::GetApplication().GetUserParameter().GetGroup("BaseApp")
                                ->GetGroup("Preferences")->GetGroup("RecentFiles");
    int count = hGrp->GetInt("RecentFiles", this->visibleItems); // save number of files
    hGrp->Clear();

    // count all set items
    QList<QAction*> recentFiles = _group->actions();
    int num = std::min<int>(count, recentFiles.count());
    for (int index = 0; index < num; index++) {
        QString key = QString::fromLatin1("MRU%1").arg(index);
        QString value = recentFiles[index]->toolTip();
        if (value.isEmpty())
            break;
        hGrp->SetASCII(key.toLatin1(), value.toUtf8());
    }

    hGrp->SetInt("RecentFiles", count); // restore
}

// --------------------------------------------------------------------

UndoAction::UndoAction (Command* pcCmd,QObject * parent)
  : Action(pcCmd, parent)
{
    _toolAction = new QAction(this);
    _toolAction->setMenu(new UndoDialog());
    connect(_toolAction, SIGNAL(triggered(bool)), this, SLOT(onActivated()));
}

UndoAction::~UndoAction()
{
    QMenu* menu = _toolAction->menu();
    delete menu;
    delete _toolAction;
}

void UndoAction::addTo (QWidget * w)
{
    if (w->inherits("QToolBar")) {
        actionChanged();
        connect(_action, SIGNAL(changed()), this, SLOT(actionChanged()));
        w->addAction(_toolAction);
    }
    else {
        w->addAction(_action);
    }
}

void UndoAction::actionChanged()
{
    // Do NOT set the shortcut again for _toolAction since this is already
    // reserved for _action. Otherwise we get an ambiguity of it with the
    // result that it doesn't work anymore.
    _toolAction->setText(_action->text());
    _toolAction->setToolTip(_action->toolTip());
    _toolAction->setStatusTip(_action->statusTip());
    _toolAction->setWhatsThis(_action->whatsThis());
    _toolAction->setIcon(_action->icon());
}

void UndoAction::setEnabled(bool b)
{
    Action::setEnabled(b);
    _toolAction->setEnabled(b);
}

void UndoAction::setVisible(bool b)
{
    Action::setVisible(b);
    _toolAction->setVisible(b);
}

// --------------------------------------------------------------------

RedoAction::RedoAction ( Command* pcCmd,QObject * parent )
  : Action(pcCmd, parent)
{
    _toolAction = new QAction(this);
    _toolAction->setMenu(new RedoDialog());
    connect(_toolAction, SIGNAL(triggered(bool)), this, SLOT(onActivated()));
}

RedoAction::~RedoAction()
{
    QMenu* menu = _toolAction->menu();
    delete menu;
    delete _toolAction;
}

void RedoAction::addTo ( QWidget * w )
{
    if (w->inherits("QToolBar")) {
        actionChanged();
        connect(_action, SIGNAL(changed()), this, SLOT(actionChanged()));
        w->addAction(_toolAction);
    }
    else {
        w->addAction(_action);
    }
}

void RedoAction::actionChanged()
{
    // Do NOT set the shortcut again for _toolAction since this is already
    // reserved for _action. Otherwise we get an ambiguity of it with the
    // result that it doesn't work anymore.
    _toolAction->setText(_action->text());
    _toolAction->setToolTip(_action->toolTip());
    _toolAction->setStatusTip(_action->statusTip());
    _toolAction->setWhatsThis(_action->whatsThis());
    _toolAction->setIcon(_action->icon());
}

void RedoAction::setEnabled  ( bool b )
{
    Action::setEnabled(b);
    _toolAction->setEnabled(b);
}

void RedoAction::setVisible ( bool b )
{
    Action::setVisible(b);
    _toolAction->setVisible(b);
}

// --------------------------------------------------------------------

DockWidgetAction::DockWidgetAction ( Command* pcCmd, QObject * parent )
  : Action(pcCmd, parent), _menu(0)
{
}

DockWidgetAction::~DockWidgetAction()
{
    delete _menu;
}

void DockWidgetAction::addTo ( QWidget * w )
{
    if (!_menu) {
      _menu = new QMenu();
      _action->setMenu(_menu);
      connect(_menu, SIGNAL(aboutToShow()), getMainWindow(), SLOT(onDockWindowMenuAboutToShow()));
    }
    
    w->addAction(_action);
}

// --------------------------------------------------------------------

ToolBarAction::ToolBarAction ( Command* pcCmd, QObject * parent )
  : Action(pcCmd, parent), _menu(0)
{
}

ToolBarAction::~ToolBarAction()
{
    delete _menu;
}

void ToolBarAction::addTo ( QWidget * w )
{
    if (!_menu) {
      _menu = new QMenu();
      _action->setMenu(_menu);
      connect(_menu, SIGNAL(aboutToShow()), getMainWindow(), SLOT(onToolBarMenuAboutToShow()));
    }
    
    w->addAction(_action);
}

// --------------------------------------------------------------------

WindowAction::WindowAction ( Command* pcCmd, QObject * parent )
  : ActionGroup(pcCmd, parent), _menu(0)
{
}

WindowAction::~WindowAction()
{
}

void WindowAction::addTo ( QWidget * w )
{
    QMenu* menu = qobject_cast<QMenu*>(w);
    if (!menu) {
        if (!_menu) {
            _menu = new QMenu();
            _action->setMenu(_menu);
            _menu->addActions(_group->actions());
            connect(_menu, SIGNAL(aboutToShow()),
                    getMainWindow(), SLOT(onWindowsMenuAboutToShow()));
        }

        w->addAction(_action);
    }
    else {
        menu->addActions(_group->actions());
        connect(menu, SIGNAL(aboutToShow()),
                getMainWindow(), SLOT(onWindowsMenuAboutToShow()));
    }
}

// --------------------------------------------------------------------

ViewCameraBindingAction::ViewCameraBindingAction ( Command* pcCmd, QObject * parent )
  : Action(pcCmd, parent), _menu(0)
{
}

ViewCameraBindingAction::~ViewCameraBindingAction()
{
}

void ViewCameraBindingAction::addTo ( QWidget * w )
{
    if (!_menu) {
        _menu = new QMenu();
        setupMenuStyle(_menu);
        _action->setMenu(_menu);
        connect(_menu, SIGNAL(aboutToShow()), this, SLOT(onShowMenu()));
        connect(_menu, SIGNAL(triggered(QAction*)), this, SLOT(onTriggered(QAction*)));
    }
    w->addAction(_action);
}

void ViewCameraBindingAction::onShowMenu()
{
    _menu->clear();
    setupMenuStyle(_menu);

    auto activeView = Base::freecad_dynamic_cast<View3DInventor>(
            Application::Instance->activeView());
    if(!activeView)
        return;

    auto boundViews = activeView->boundViews();
    if(boundViews.size()) {
        if(boundViews.size() == 1) {
            auto action = _menu->addAction(tr("Sync camera"));
            action->setData(1);
        }
        auto action = _menu->addAction(tr("Unbind"));
        action->setData(2);
        _menu->addSeparator();
    }
    for(auto doc : App::GetApplication().getDocuments()) {
        auto gdoc = Application::Instance->getDocument(doc);
        if(!gdoc)
            continue;
        auto views = gdoc->getMDIViewsOfType(View3DInventor::getClassTypeId());
        for(auto it=views.begin();it!=views.end();) {
            auto view = static_cast<View3DInventor*>(*it);
            if(view == activeView ||
                    (!boundViews.count(view) && view->boundViews(true).count(activeView)))
                it = views.erase(it);
            else
                ++it;
        }
        if(views.empty())
            continue;
        if(views.size() == 1) {
            auto view = static_cast<View3DInventor*>(views.front());
            auto action = _menu->addAction(view->windowTitle());
            action->setCheckable(true);
            if(boundViews.count(view))
                action->setChecked(true);
            continue;
        }
        auto menu = _menu->addMenu(QString::fromUtf8(doc->Label.getValue()));
        for(auto view : views) {
            auto action = menu->addAction(view->windowTitle());
            action->setCheckable(true);
            if(boundViews.count(static_cast<View3DInventor*>(view)))
                action->setChecked(true);
        }
    }
}

void ViewCameraBindingAction::onTriggered(QAction *action)
{
    auto activeView = Base::freecad_dynamic_cast<View3DInventor>(
            Application::Instance->activeView());
    if(!activeView)
        return;

    switch(action->data().toInt()) {
    case 1: {
        auto views = activeView->boundViews();
        if(views.size())
            activeView->syncCamera(*views.begin());
        break;
    }
    case 2:
        activeView->unbindView();
        break;
    default:
        if (action->isChecked())
            activeView->bindView(action->text(), true);
        else
            activeView->unbindView(action->text());
        break;
    }
}

// --------------------------------------------------------------------

SelUpAction::SelUpAction ( Command* pcCmd, QObject * parent )
  : Action(pcCmd, parent), _menu(0)
{
}

SelUpAction::~SelUpAction()
{
    delete _menu;
}

void SelUpAction::addTo ( QWidget * w )
{
    if (!_menu) {
        _menu = new SelUpMenu(nullptr);
        _action->setMenu(_menu);
        connect(_menu, SIGNAL(aboutToShow()), this, SLOT(onShowMenu()));
    }
    w->addAction(_action);
}

void SelUpAction::onShowMenu()
{
    _menu->clear();
    setupMenuStyle(_menu);
    TreeWidget::populateSelUpMenu(_menu);
}

void SelUpAction::popup(const QPoint &pt)
{
    _menu->exec(pt);
}

// --------------------------------------------------------------------

class CommandModel : public QAbstractItemModel
{
public:
    struct CmdInfo {
        QString text;
        Command *cmd;
    };
    std::vector<CmdInfo> cmds;
    int revision;

public:
    CommandModel(QObject* parent)
        : QAbstractItemModel(parent)
    {
        revision = 0;
        update();
    }

    void update()
    {
        auto &manager = Application::Instance->commandManager();
        if (revision == manager.getRevision())
            return;
        beginResetModel();
        revision = manager.getRevision();
        cmds.clear();
        for (auto &v : manager.getCommands()) {
            cmds.emplace_back();
            auto &info = cmds.back();
            info.cmd = v.second;
#if QT_VERSION>=QT_VERSION_CHECK(5,2,0)
            info.text = QString::fromLatin1("%2 (%1)").arg(
                    QString::fromLatin1(info.cmd->getName()),
                    qApp->translate(info.cmd->className(), info.cmd->getMenuText()));
#else
            info.text = qApp->translate(info.cmd->className(), info.cmd->getMenuText());
#endif
        }
        endResetModel();
    }

    virtual QModelIndex parent(const QModelIndex &) const
    {
        return QModelIndex();
    }

    virtual QVariant data(const QModelIndex & index, int role) const
    {
        if (index.row() < 0 || index.row() >= (int)cmds.size())
            return QVariant();

        auto &info = cmds[index.row()];

        switch(role) {
        case Qt::DisplayRole:
        case Qt::EditRole:
            return info.text;
        case Qt::DecorationRole:
            if (info.cmd->getPixmap())
                return BitmapFactory().iconFromTheme(info.cmd->getPixmap());
            return QVariant();
        case Qt::ToolTipRole:
            return qApp->translate(info.cmd->className(), info.cmd->getToolTipText());
        case Qt::UserRole:
            return QByteArray(info.cmd->getName());
        default:
            return QVariant();
        }
    }

    virtual QModelIndex index(int row, int, const QModelIndex &) const
    {
        return this->createIndex(row, 0);
    }

    virtual int rowCount(const QModelIndex &) const
    {
        return (int)(cmds.size());
    }

    virtual int columnCount(const QModelIndex &) const
    {
        return 1;
    }
};

class CmdHistoryMenu: public QMenu
{
public:
    CmdHistoryMenu(QWidget *focus)
        :focusWidget(focus)
    {}

    void keyPressEvent(QKeyEvent *e)
    {
        QMenu::keyPressEvent(e);
        if (e->isAccepted())
            return;
        if (isVisible() && !e->text().isEmpty()) {
            focusWidget->setFocus();
            QKeyEvent ke(e->type(), e->key(), e->modifiers(), e->text(), e->isAutoRepeat(), e->count());
            qApp->sendEvent(focusWidget, &ke);
        }
    }

public:
    QWidget *focusWidget;
};

// --------------------------------------------------------------------
CmdHistoryAction::CmdHistoryAction ( Command* pcCmd, QObject * parent )
  : Action(pcCmd, parent)
{
    qApp->installEventFilter(this);
}

CmdHistoryAction::~CmdHistoryAction()
{
    delete _menu;
}

void CmdHistoryAction::addTo ( QWidget * w )
{
    if (!_menu) {
        _lineedit = new QLineEdit;
        _lineedit->setPlaceholderText(tr("Type to search..."));
        _widgetAction = new QWidgetAction(this);
        _widgetAction->setDefaultWidget(_lineedit);
        _completer = new QCompleter(this);
        _completer->setModel(new CommandModel(_completer));
#if QT_VERSION>=QT_VERSION_CHECK(5,2,0)
        _completer->setFilterMode(Qt::MatchContains);
#endif
        _completer->setCaseSensitivity(Qt::CaseInsensitive);
        _completer->setCompletionMode(QCompleter::PopupCompletion);
        _completer->setWidget(_lineedit);
        connect(_lineedit, SIGNAL(textEdited(QString)), this, SLOT(onTextChanged(QString)));
        connect(_completer, SIGNAL(activated(QModelIndex)), this, SLOT(onCommandActivated(QModelIndex)));
        connect(_completer, SIGNAL(highlighted(QString)), _lineedit, SLOT(setText(QString)));

        _newAction = new QAction(tr("Add toolbar..."), this);
        connect(_newAction, SIGNAL(triggered(bool)), this, SLOT(onNewAction()));

        _menu = new CmdHistoryMenu(_lineedit);
        setupMenuStyle(_menu);
        _action->setMenu(_menu);
        connect(_menu, SIGNAL(aboutToShow()), this, SLOT(onShowMenu()));
    }

    w->addAction(_action);
}

static long _RecentCommandID;
static std::map<long, const char *, std::greater<long> > _RecentCommands;
static std::unordered_map<std::string, long> _RecentCommandMap;
static long _RecentCommandPopulated;
static QElapsedTimer _ButtonTime;

std::vector<Command*> CmdHistoryAction::recentCommands()
{
    auto &manager = Application::Instance->commandManager();
    std::vector<Command*> cmds;
    cmds.reserve(_RecentCommands.size());
    for (auto &v : _RecentCommands) {
        auto cmd = manager.getCommandByName(v.second);
        if (cmd)
            cmds.push_back(cmd);
    }
    return cmds;
}

void CmdHistoryAction::onNewAction()
{
    Application::Instance->commandManager().runCommandByName("Std_DlgCustomize", 1);
}

bool CmdHistoryAction::eventFilter(QObject *o, QEvent *ev)
{
    switch(ev->type()) {
    case QEvent::MouseButtonPress: {
        auto e = static_cast<QMouseEvent*>(ev);
        if (e->button() == Qt::LeftButton)
            _ButtonTime.start();
        break;
    }
    case QEvent::KeyPress: {
        QKeyEvent * ke = static_cast<QKeyEvent*>(ev);
        switch(ke->key()) {
        case Qt::Key_Tab: {
            if (_completer && o == _completer->popup()) {
                QKeyEvent kevent(ke->type(),Qt::Key_Down,0);
                qApp->sendEvent(_completer->popup(), &kevent);
                return true;
            }
            break;
        }
        case Qt::Key_Backtab: {
            if (_completer && o == _completer->popup()) {
                QKeyEvent kevent(ke->type(),Qt::Key_Up,0);
                qApp->sendEvent(_completer->popup(), &kevent);
                return true;
            }
            break;
        }
        default:
            break;
        }
        break;
    }
    default:
        break;
    }
    return false;
}

void CmdHistoryAction::onInvokeCommand(const char *name, bool force)
{
    if(!force && (!_ButtonTime.isValid() || _ButtonTime.elapsed() > 1000))
        return;

    _ButtonTime.invalidate();

    auto &manager = Application::Instance->commandManager();
    Command *cmd = manager.getCommandByName(name);
    if (!cmd || qobject_cast<CmdHistoryAction*>(cmd->getAction())
             || qobject_cast<ToolbarMenuAction*>(cmd->getAction()))
        return;
    
    if (!force && (cmd->getType() & Command::NoHistory) && !_RecentCommandMap.count(name))
        return;

    auto res = _RecentCommandMap.insert(std::make_pair(name, 0));
    if (!res.second)
        _RecentCommands.erase(res.first->second);
    res.first->second = ++_RecentCommandID;
    _RecentCommands[_RecentCommandID] = res.first->first.c_str();
    if (ViewParams::getCommandHistorySize() < (int)_RecentCommandMap.size()) {
        _RecentCommandMap.erase(_RecentCommands.begin()->second);
        _RecentCommands.erase(_RecentCommands.begin());
    }
}

void CmdHistoryAction::onShowMenu()
{
    setupMenuStyle(_menu);

    _menu->setFocus(Qt::PopupFocusReason);
    _lineedit->setText(QString());
    static_cast<CommandModel*>(_completer->model())->update();

    if (_RecentCommandPopulated == _RecentCommandID)
        return;

    _RecentCommandPopulated = _RecentCommandID;
    _menu->clear();
    _menu->addAction(_widgetAction);
    _menu->addAction(_newAction);
    _menu->addSeparator();
    auto &manager = Application::Instance->commandManager();
    for (auto &v : _RecentCommands)
        manager.addTo(v.second, _menu);
}

void CmdHistoryAction::onCommandActivated(const QModelIndex &index)
{
    _menu->hide();

    QByteArray name = _completer->completionModel()->data(index, Qt::UserRole).toByteArray();
    auto &manager = Application::Instance->commandManager();
    if (name.size()) {
        manager.runCommandByName(name.constData());
        onInvokeCommand(name.constData(), true);
    }
}

void CmdHistoryAction::onTextChanged(const QString &txt)
{
    if (txt.size() < 3)
        return;

    _completer->setCompletionPrefix(txt);
    QRect rect = _lineedit->rect();
    if (rect.width() < 300)
        rect.setWidth(300);
    _completer->complete(rect);
}

void CmdHistoryAction::popup(const QPoint &pt)
{
    _menu->exec(pt);
}

// --------------------------------------------------------------------

class ToolbarMenuAction::Private: public ParameterGrp::ObserverType
{
public:
    Private(ToolbarMenuAction *master, const char *path):master(master)
    {
        handle = App::GetApplication().GetParameterGroupByPath(path);
    }

    void OnChange(Base::Subject<const char*> &, const char *)
    {
        master->update();
    }

public:
    ToolbarMenuAction *master;
    ParameterGrp::handle handle;
    ParameterGrp::handle hShortcut;
    std::set<std::string> cmds;
};

// --------------------------------------------------------------------

class GuiExport ToolbarMenuSubAction : public ToolbarMenuAction
{
public:
    ToolbarMenuSubAction (Command* pcCmd, ParameterGrp::handle hGrp, QObject * parent = 0)
        : ToolbarMenuAction(pcCmd, parent)
    {
        _pimpl->handle = hGrp;
        _pimpl->handle->Attach(_pimpl.get());
    }

    ~ToolbarMenuSubAction() {
        _pimpl->handle->Detach(_pimpl.get());
    }

protected:
    virtual void onShowMenu() {
        setupMenuStyle(_menu);

        auto &manager = Application::Instance->commandManager();
        if (revision == manager.getRevision())
            return;
        _menu->clear();
        for (auto &v : _pimpl->handle->GetASCIIMap()) {
            if (v.first == "Name")
                setText(QString::fromUtf8(v.second.c_str()));
            else if (boost::starts_with(v.first, "Separator"))
                _menu->addSeparator();
            else
                manager.addTo(v.first.c_str(), _menu);
        }
        revision = manager.getRevision();
    }

    virtual void update()
    {
        revision = 0;
        std::string text = _pimpl->handle->GetASCII("Name", "");
        this->setText(QString::fromUtf8(text.c_str()));
    }

private:
    int revision = 0;
};

// --------------------------------------------------------------------

class StdCmdToolbarSubMenu : public Gui::Command
{
public:
    StdCmdToolbarSubMenu(const char *name, ParameterGrp::handle hGrp)
        :Command(name)
    {
        menuText      = hGrp->GetASCII("Name", "Custom");
        sGroup        = QT_TR_NOOP("Tools");
        sMenuText     = menuText.c_str();
        sWhatsThis    = "Std_CmdToolbarSubMenu";
        eType         = NoTransaction | NoHistory;

        _pcAction = new ToolbarMenuSubAction(this, hGrp, getMainWindow());
        applyCommandData(this->className(), _pcAction);
    }

    virtual ~StdCmdToolbarSubMenu() {}

    virtual const char* className() const
    { return "StdCmdToolbarSubMenu"; }

protected:
    virtual void activated(int) {
        if (_pcAction)
            static_cast<ToolbarMenuSubAction*>(_pcAction)->popup(QCursor::pos());
    }
    virtual bool isActive(void) { return true;}

    virtual Gui::Action * createAction(void) {
        assert(false);
        return nullptr;
    }

private:
    std::string menuText;
};

// --------------------------------------------------------------------

ToolbarMenuAction::ToolbarMenuAction ( Command* pcCmd, QObject * parent )
  : Action(pcCmd, parent), _menu(0)
  , _pimpl(new Private(this, "User parameter:BaseApp/Workbench/Global/Toolbar"))
{
    _pimpl->hShortcut = WindowParameter::getDefaultParameter()->GetGroup("Shortcut");
    _pimpl->hShortcut->Attach(_pimpl.get());
}

ToolbarMenuAction::~ToolbarMenuAction()
{
    _pimpl->hShortcut->Detach(_pimpl.get());
    delete _menu;
}

void ToolbarMenuAction::addTo ( QWidget * w )
{
    if (!_menu) {
        _menu = new QMenu;
        setupMenuStyle(_menu);
        _action->setMenu(_menu);
        update();
        connect(_menu, SIGNAL(aboutToShow()), this, SLOT(onShowMenu()));
    }
    w->addAction(_action);
}

void ToolbarMenuAction::onShowMenu()
{
    setupMenuStyle(_menu);
}

void ToolbarMenuAction::populate()
{
    auto &manager = Application::Instance->commandManager();
    auto cmd = manager.getCommandByName("Std_CmdToolbarMenus");
    if (!cmd)
        return;
    auto action = qobject_cast<ToolbarMenuAction*>(cmd->getAction());
    if (action)
        action->update();
}

std::string ToolbarMenuAction::commandName(const char *name)
{
    return std::string("Std_ToolbarMenu_") + name;
}

void ToolbarMenuAction::update()
{
    auto &manager = Application::Instance->commandManager();
    _menu->clear();
    std::set<std::string> cmds;
    for (auto &hGrp : _pimpl->handle->GetGroups()) {

        if(hGrp->GetASCII("Name","").empty())
            continue;

        std::string name = commandName(hGrp->GetGroupName());
        QString shortcut = QString::fromLatin1(_pimpl->hShortcut->GetASCII(name.c_str()).c_str());
        if (shortcut.isEmpty())
            continue;

        if (!cmds.insert(name).second)
            continue;

        auto res = _pimpl->cmds.insert(name);
        Command *cmd = manager.getCommandByName(name.c_str());
        if (!cmd) {
            cmd = new StdCmdToolbarSubMenu(res.first->c_str(), hGrp);
            manager.addCommand(cmd);
        }
        if (cmd->getAction() && cmd->getAction()->shortcut() != shortcut)
            cmd->getAction()->setShortcut(shortcut);
        cmd->addTo(_menu);
    }

    for (auto it=_pimpl->cmds.begin(); it!=_pimpl->cmds.end();) {
        if (cmds.count(*it)) {
            ++it;
            continue;
        }
        Command *cmd = manager.getCommandByName(it->c_str());
        if (cmd)
            manager.removeCommand(cmd);
        _pimpl->hShortcut->RemoveASCII(it->c_str());
        it = _pimpl->cmds.erase(it);
    }
}

void ToolbarMenuAction::popup(const QPoint &pt)
{
    _menu->exec(pt);
    _menu->setActiveAction(0);
}

#include "moc_Action.cpp"
