/***************************************************************************
 *   Copyright (c) 2006 Werner Mayer <wmayer[at]users.sourceforge.net>     *
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
# include <sstream>
# include <QApplication>
# include <QEvent>
# include <QFileInfo>
# include <QPixmap>
# include <boost/signals.hpp>
# include <boost/bind.hpp>
# include <Inventor/nodes/SoDrawStyle.h>
# include <Inventor/nodes/SoMaterial.h>
# include <Inventor/nodes/SoSeparator.h>
# include <Inventor/actions/SoSearchAction.h>
# include <Inventor/draggers/SoDragger.h>
# include <Inventor/manips/SoCenterballManip.h>
# include <Inventor/nodes/SoBaseColor.h>
# include <Inventor/nodes/SoCamera.h>
# include <Inventor/nodes/SoDrawStyle.h>
# include <Inventor/nodes/SoMaterial.h>
# include <Inventor/nodes/SoSeparator.h>
# include <Inventor/nodes/SoSwitch.h>
# include <Inventor/nodes/SoDirectionalLight.h>
# include <Inventor/sensors/SoNodeSensor.h> 
# include <Inventor/SoPickedPoint.h>
# include <Inventor/actions/SoRayPickAction.h> 
#endif

#include "ViewProviderFeaturePython.h"
#include "ViewProviderFeaturePythonPyImp.h"
#include "SoFCSelection.h"
#include "SoFCBoundingBox.h"
#include "Tree.h"
#include "Window.h"
#include "Application.h"
#include "BitmapFactory.h"
#include "Document.h"
#include <App/DocumentObjectPy.h>
#include <App/GeoFeature.h>
#include <App/PropertyGeo.h>
#include <Base/Console.h>
#include <Base/Reader.h>
#include <Base/Interpreter.h>
#include <Gui/ViewProviderFeaturePythonPyImp.h>


using namespace Gui;

namespace Gui {

class PropertyEvent : public QEvent
{
public:
    PropertyEvent(App::Property* p1, App::Property* p2)
        : QEvent(QEvent::Type(QEvent::User)), p1(p1), p2(p2)
    {
    }

    App::Property* p1;
    App::Property* p2;
};

class ViewProviderFeaturePythonObserver : public QObject
{
public:
    /// The one and only instance.
    static ViewProviderFeaturePythonObserver* instance();
    /// Destructs the sole instance.
    static void destruct ();
    void slotAppendObject(const Gui::ViewProvider&);
    void slotDeleteObject(const Gui::ViewProvider&);
    void slotDeleteDocument(const Gui::Document&);

private:
    void customEvent(QEvent* e)
    {
        PropertyEvent* pe = static_cast<PropertyEvent*>(e);
        pe->p1->Paste(*pe->p2);
        delete pe->p2;
    }
    static ViewProviderFeaturePythonObserver* _singleton;

    ViewProviderFeaturePythonObserver();
    ~ViewProviderFeaturePythonObserver();
    typedef std::map<
                const App::DocumentObject*,
                App::Property*
            > ObjectProxy;

    std::map<const App::Document*, ObjectProxy> proxyMap;
};

}

ViewProviderFeaturePythonObserver* ViewProviderFeaturePythonObserver::_singleton = 0;

ViewProviderFeaturePythonObserver* ViewProviderFeaturePythonObserver::instance()
{
    if (!_singleton)
        _singleton = new ViewProviderFeaturePythonObserver;
    return _singleton;
}

void ViewProviderFeaturePythonObserver::destruct ()
{
    delete _singleton;
    _singleton = 0;
}

void ViewProviderFeaturePythonObserver::slotDeleteDocument(const Gui::Document& d)
{
    App::Document* doc = d.getDocument();
    std::map<const App::Document*, ObjectProxy>::iterator it = proxyMap.find(doc);
    if (it != proxyMap.end()) {
        proxyMap.erase(it);
    }
}

void ViewProviderFeaturePythonObserver::slotAppendObject(const Gui::ViewProvider& obj)
{
    if (!obj.isDerivedFrom(Gui::ViewProviderDocumentObject::getClassTypeId()))
        return;
    const Gui::ViewProviderDocumentObject& vp = static_cast<const Gui::ViewProviderDocumentObject&>(obj);
    const App::DocumentObject* docobj = vp.getObject();
    App::Document* doc = docobj->getDocument();
    std::map<const App::Document*, ObjectProxy>::iterator it = proxyMap.find(doc);
    if (it != proxyMap.end()) {
        ObjectProxy::iterator jt = it->second.find(docobj);
        if (jt != it->second.end()) {
            Base::PyGILStateLocker lock;
            try {
                App::Property* prop = vp.getPropertyByName("Proxy");
                if (prop && prop->isDerivedFrom(App::PropertyPythonObject::getClassTypeId())) {
                    // make this delayed so that the corresponding item in the tree view is accessible
                    QApplication::postEvent(this, new PropertyEvent(prop, jt->second));
                    it->second.erase(jt);
                }
            }
            catch (Py::Exception& e) {
                e.clear();
            }
        }
        // all cached objects of the documents are already destroyed
        else {
            it->second.clear();
        }
    }
}

void ViewProviderFeaturePythonObserver::slotDeleteObject(const Gui::ViewProvider& obj)
{
    if (!obj.isDerivedFrom(Gui::ViewProviderDocumentObject::getClassTypeId()))
        return;
    const Gui::ViewProviderDocumentObject& vp = static_cast<const Gui::ViewProviderDocumentObject&>(obj);
    const App::DocumentObject* docobj = vp.getObject();
    App::Document* doc = docobj->getDocument();
    if (!doc->getUndoMode())
        return; // object will be deleted immediately, thus we don't need to store anything
    Base::PyGILStateLocker lock;
    try {
        App::Property* prop = vp.getPropertyByName("Proxy");
        if (prop && prop->isDerivedFrom(App::PropertyPythonObject::getClassTypeId())) {
            proxyMap[doc][docobj] = prop->Copy();
        }
    }
    catch (Py::Exception& e) {
        e.clear();
    }
}

ViewProviderFeaturePythonObserver::ViewProviderFeaturePythonObserver()
{
    Gui::Application::Instance->signalDeletedObject.connect(boost::bind
        (&ViewProviderFeaturePythonObserver::slotDeleteObject, this, _1));
    Gui::Application::Instance->signalNewObject.connect(boost::bind
        (&ViewProviderFeaturePythonObserver::slotAppendObject, this, _1));
    Gui::Application::Instance->signalDeleteDocument.connect(boost::bind
        (&ViewProviderFeaturePythonObserver::slotDeleteDocument, this, _1));
}

ViewProviderFeaturePythonObserver::~ViewProviderFeaturePythonObserver()
{
}

// ----------------------------------------------------------------------------

ViewProviderFeaturePythonImp::ViewProviderFeaturePythonImp(ViewProviderDocumentObject* vp)
  : object(vp)
{
    (void)ViewProviderFeaturePythonObserver::instance();
}

ViewProviderFeaturePythonImp::~ViewProviderFeaturePythonImp()
{
}

QIcon ViewProviderFeaturePythonImp::getIcon() const
{
    // default icon
    //static QPixmap px = BitmapFactory().pixmap("Tree_Python");

    // Run the getIcon method of the proxy object.
    Base::PyGILStateLocker lock;
    try {
        App::Property* proxy = object->getPropertyByName("Proxy");
        if (proxy && proxy->getTypeId() == App::PropertyPythonObject::getClassTypeId()) {
            Py::Object vp = static_cast<App::PropertyPythonObject*>(proxy)->getValue();
            if (vp.is(Py::_None()))
                vp = Py::asObject(object->getPyObject());
            if (vp.hasAttr(std::string("getIcon"))) {
                Py::Callable method(vp.getAttr(std::string("getIcon")));
                Py::Tuple args(0);
                Py::String str(method.apply(args));
                std::string content = str.as_std_string();
                QPixmap icon;
                // Check if the passed string is a filename, otherwise treat as xpm data
                QFileInfo fi(QString::fromAscii(content.c_str()));
                if (fi.isFile() && fi.exists()) {
                    icon.load(fi.absoluteFilePath());
                }
                else {
                    QByteArray ary;
                    int strlen = (int)content.size();
                    ary.resize(strlen);
                    for (int j=0; j<strlen; j++)
                        ary[j]=content[j];
                    // Make sure to remove crap around the XPM data
                    QList<QByteArray> lines = ary.split('\n');
                    QByteArray buffer;
                    buffer.reserve(ary.size()+lines.size());
                    for (QList<QByteArray>::iterator it = lines.begin(); it != lines.end(); ++it) {
                        QByteArray trim = it->trimmed();
                        if (!trim.isEmpty()) {
                            buffer.append(trim);
                            buffer.append('\n');
                        }
                    }
                    icon.loadFromData(buffer, "XPM");
                }
                if (!icon.isNull()) {
                    return icon;
                }
            }
        }
    }
    catch (Py::Exception&) {
        Base::PyException e; // extract the Python error text
        Base::Console().Error("ViewProviderFeaturePython::getIcon: %s\n", e.what());
    }

    return QIcon();
}

std::vector<App::DocumentObject*> ViewProviderFeaturePythonImp::claimChildren(const std::vector<App::DocumentObject*>& base) const 
{
    std::vector<App::DocumentObject*> children;
    Base::PyGILStateLocker lock;
    try {
        App::Property* proxy = object->getPropertyByName("Proxy");
        if (proxy && proxy->getTypeId() == App::PropertyPythonObject::getClassTypeId()) {
            Py::Object vp = static_cast<App::PropertyPythonObject*>(proxy)->getValue();
            if (vp.is(Py::_None()))
                vp = Py::asObject(object->getPyObject());
            if (vp.hasAttr(std::string("claimChildren"))) {
                Py::Callable method(vp.getAttr(std::string("claimChildren")));
                Py::Tuple args(0);
                Py::Sequence list(method.apply(args));
                for (Py::Sequence::iterator it = list.begin(); it != list.end(); ++it) {
                    PyObject* item = (*it).ptr();
                    if (PyObject_TypeCheck(item, &(App::DocumentObjectPy::Type))) {
                        App::DocumentObject* obj = static_cast<App::DocumentObjectPy*>(item)->getDocumentObjectPtr();
                        children.push_back(obj);
                    }
                }
            }
            else {
                children = base;
            }
        }
    }
    catch (Py::Exception&) {
        Base::PyException e; // extract the Python error text
        Base::Console().Error("ViewProviderFeaturePython::claimChildren: %s\n", e.what());
    }

    return children;
}

std::string ViewProviderFeaturePythonImp::getElement(const SoDetail *det) const
{
    return "";
}

std::vector<Base::Vector3d> ViewProviderFeaturePythonImp::getSelectionShape(const char* Element) const
{
    return std::vector<Base::Vector3d>();
}

bool ViewProviderFeaturePythonImp::setEdit(int ModNum)
{
    // Run the onChanged method of the proxy object.
    Base::PyGILStateLocker lock;
    try {
        App::Property* proxy = object->getPropertyByName("Proxy");
        if (proxy && proxy->getTypeId() == App::PropertyPythonObject::getClassTypeId()) {
            Py::Object vp = static_cast<App::PropertyPythonObject*>(proxy)->getValue();
            if (vp.is(Py::_None())) {
                vp = Py::asObject(object->getPyObject());
                if (vp.hasAttr(std::string("setEdit"))) {
                    Py::Callable method(vp.getAttr(std::string("setEdit")));
                    Py::Tuple args(1);
                    args.setItem(0, Py::Int(ModNum));
                    Py::Boolean ok(method.apply(args));
                    return (bool)ok;
                }
            }
            else {
                if (vp.hasAttr(std::string("setEdit"))) {
                    Py::Callable method(vp.getAttr(std::string("setEdit")));
                    Py::Tuple args(2);
                    args.setItem(0, Py::Object(object->getPyObject(), true));
                    args.setItem(1, Py::Int(ModNum));
                    Py::Boolean ok(method.apply(args));
                    return (bool)ok;
                }
            }
        }
    }
    catch (Py::Exception&) {
        Base::PyException e; // extract the Python error text
        const char* name = object->getObject()->Label.getValue();
        Base::Console().Error("ViewProviderFeaturePython::setEdit (%s): %s\n", name, e.what());
    }

    return false;
}

bool ViewProviderFeaturePythonImp::unsetEdit(int ModNum)
{
    // Run the onChanged method of the proxy object.
    Base::PyGILStateLocker lock;
    try {
        App::Property* proxy = object->getPropertyByName("Proxy");
        if (proxy && proxy->getTypeId() == App::PropertyPythonObject::getClassTypeId()) {
            Py::Object vp = static_cast<App::PropertyPythonObject*>(proxy)->getValue();
            if (vp.is(Py::_None())) {
                vp = Py::asObject(object->getPyObject());
                if (vp.hasAttr(std::string("unsetEdit"))) {
                    Py::Callable method(vp.getAttr(std::string("unsetEdit")));
                    Py::Tuple args(1);
                    args.setItem(0, Py::Int(ModNum));
                    Py::Boolean ok(method.apply(args));
                    return (bool)ok;
                }
            }
            else {
                if (vp.hasAttr(std::string("unsetEdit"))) {
                    Py::Callable method(vp.getAttr(std::string("unsetEdit")));
                    Py::Tuple args(2);
                    args.setItem(0, Py::Object(object->getPyObject(), true));
                    args.setItem(1, Py::Int(ModNum));
                    Py::Boolean ok(method.apply(args));
                    return (bool)ok;
                }
            }
        }
    }
    catch (Py::Exception&) {
        Base::PyException e; // extract the Python error text
        const char* name = object->getObject()->Label.getValue();
        Base::Console().Error("ViewProviderFeaturePython::unsetEdit (%s): %s\n", name, e.what());
    }

    return false;
}

void ViewProviderFeaturePythonImp::attach(App::DocumentObject *pcObject)
{
    // Run the attach method of the proxy object.
    Base::PyGILStateLocker lock;
    try {
        App::Property* proxy = object->getPropertyByName("Proxy");
        if (proxy && proxy->getTypeId() == App::PropertyPythonObject::getClassTypeId()) {
            Py::Object vp = static_cast<App::PropertyPythonObject*>(proxy)->getValue();
            if (vp.is(Py::_None())) {
                vp = Py::asObject(object->getPyObject());
                if (vp.hasAttr(std::string("attach"))) {
                    Py::Callable method(vp.getAttr(std::string("attach")));
                    Py::Tuple args(0);
                    method.apply(args);
                }

                // #0000415: Now simulate a property change event to call
                // claimChildren if implemented.
                pcObject->Label.touch();
            }
            else {
                if (vp.hasAttr(std::string("attach"))) {
                    Py::Callable method(vp.getAttr(std::string("attach")));
                    Py::Tuple args(1);
                    args.setItem(0, Py::Object(object->getPyObject(), true));
                    method.apply(args);
                }

                // #0000415: Now simulate a property change event to call
                // claimChildren if implemented.
                pcObject->Label.touch();
            }
        }
    }
    catch (Py::Exception&) {
        Base::PyException e; // extract the Python error text
        const char* name = object->getObject()->Label.getValue();
        Base::Console().Error("ViewProviderFeaturePython::attach (%s): %s\n", name, e.what());
    }
}

void ViewProviderFeaturePythonImp::updateData(const App::Property* prop)
{
    // Run the updateData method of the proxy object.
    Base::PyGILStateLocker lock;
    try {
        App::Property* proxy = object->getPropertyByName("Proxy");
        if (proxy && proxy->getTypeId() == App::PropertyPythonObject::getClassTypeId()) {
            Py::Object vp = static_cast<App::PropertyPythonObject*>(proxy)->getValue();
            if (vp.is(Py::_None())) {
                vp = Py::asObject(object->getPyObject());
                if (vp.hasAttr(std::string("updateData"))) {
                    Py::Callable method(vp.getAttr(std::string("updateData")));
                    Py::Tuple args(1);
                    const char* prop_name = object->getObject()->getName(prop);
                    if (prop_name) {
                        args.setItem(0, Py::String(prop_name));
                        method.apply(args);
                    }
                }
            }
            else {
                if (vp.hasAttr(std::string("updateData"))) {
                    Py::Callable method(vp.getAttr(std::string("updateData")));
                    Py::Tuple args(2);
                    args.setItem(0, Py::Object(object->getObject()->getPyObject(), true));
                    const char* prop_name = object->getObject()->getName(prop);
                    if (prop_name) {
                        args.setItem(1, Py::String(prop_name));
                        method.apply(args);
                    }
                }
            }
        }
    }
    catch (Py::Exception&) {
        Base::PyException e; // extract the Python error text
        const char* name = object->getObject()->Label.getValue();
        Base::Console().Error("ViewProviderFeaturePython::updateData (%s): %s\n", name, e.what());
    }
}

void ViewProviderFeaturePythonImp::onChanged(const App::Property* prop)
{
    // Run the onChanged method of the proxy object.
    Base::PyGILStateLocker lock;
    try {
        App::Property* proxy = object->getPropertyByName("Proxy");
        if (proxy && proxy->getTypeId() == App::PropertyPythonObject::getClassTypeId()) {
            Py::Object vp = static_cast<App::PropertyPythonObject*>(proxy)->getValue();
            if (vp.is(Py::_None())) {
                vp = Py::asObject(object->getPyObject());
                if (vp.hasAttr(std::string("onChanged"))) {
                    Py::Callable method(vp.getAttr(std::string("onChanged")));
                    Py::Tuple args(1);
                    std::string prop_name = object->getName(prop);
                    args.setItem(0, Py::String(prop_name));
                    method.apply(args);
                }
            }
            else {
                if (vp.hasAttr(std::string("onChanged"))) {
                    Py::Callable method(vp.getAttr(std::string("onChanged")));
                    Py::Tuple args(2);
                    args.setItem(0, Py::Object(object->getPyObject(), true));
                    std::string prop_name = object->getName(prop);
                    args.setItem(1, Py::String(prop_name));
                    method.apply(args);
                }
            }
        }
    }
    catch (Py::Exception&) {
        Base::PyException e; // extract the Python error text
        const char* name = object->getObject()->Label.getValue();
        Base::Console().Error("ViewProviderFeaturePython::onChanged (%s): %s\n", name, e.what());
    }
}

void ViewProviderFeaturePythonImp::startRestoring()
{
}

void ViewProviderFeaturePythonImp::finishRestoring()
{
    App::Property* proxy = object->getPropertyByName("Proxy");
    if (proxy && proxy->getTypeId() == App::PropertyPythonObject::getClassTypeId()) {
        Py::Object vp = static_cast<App::PropertyPythonObject*>(proxy)->getValue();
        if (vp.ptr() == Py::_None()) {
            object->show();
            static_cast<App::PropertyPythonObject*>(proxy)->setValue(Py::Int(1));
        }
    }
}

const char* ViewProviderFeaturePythonImp::getDefaultDisplayMode() const
{
    // Run the getDefaultDisplayMode method of the proxy object.
    Base::PyGILStateLocker lock;
    static std::string mode;
    try {
        App::Property* proxy = object->getPropertyByName("Proxy");
        if (proxy && proxy->getTypeId() == App::PropertyPythonObject::getClassTypeId()) {
            Py::Object vp = static_cast<App::PropertyPythonObject*>(proxy)->getValue();
            if (vp.is(Py::_None()))
                vp = Py::asObject(object->getPyObject());
            if (vp.hasAttr(std::string("getDefaultDisplayMode"))) {
                Py::Callable method(vp.getAttr(std::string("getDefaultDisplayMode")));
                Py::Tuple args(0);
                Py::String str(method.apply(args));
                if (str.isUnicode())
                    str = str.encode("ascii"); // json converts strings into unicode
                mode = str.as_std_string();
                return mode.c_str();
            }
        }
    }
    catch (Py::Exception&) {
        Base::PyException e; // extract the Python error text
        const char* name = object->getObject()->Label.getValue();
        Base::Console().Error("ViewProviderFeaturePython::getDefaultDisplayMode (%s): %s\n", name, e.what());
    }

    return 0;
}

std::vector<std::string> ViewProviderFeaturePythonImp::getDisplayModes(void) const
{
    // Run the getDisplayModes method of the proxy object.
    Base::PyGILStateLocker lock;
    std::vector<std::string> modes;
    try {
        App::Property* proxy = object->getPropertyByName("Proxy");
        if (proxy && proxy->getTypeId() == App::PropertyPythonObject::getClassTypeId()) {
            Py::Object vp = static_cast<App::PropertyPythonObject*>(proxy)->getValue();
            if (vp.is(Py::_None())) {
                vp = Py::asObject(object->getPyObject());
                if (vp.hasAttr(std::string("getDisplayModes"))) {
                    Py::Callable method(vp.getAttr(std::string("getDisplayModes")));
                    Py::Tuple args(0);
                    Py::List list(method.apply(args));
                    for (Py::List::iterator it = list.begin(); it != list.end(); ++it) {
                        Py::String str(*it);
                        modes.push_back(str.as_std_string());
                    }
                }
            }
            else {
                if (vp.hasAttr(std::string("getDisplayModes"))) {
                    Py::Callable method(vp.getAttr(std::string("getDisplayModes")));
                    Py::Tuple args(1);
                    args.setItem(0, Py::Object(object->getPyObject(), true));
                    Py::List list(method.apply(args));
                    for (Py::List::iterator it = list.begin(); it != list.end(); ++it) {
                        Py::String str(*it);
                        modes.push_back(str.as_std_string());
                    }
                }
            }
        }
    }
    catch (Py::Exception&) {
        Base::PyException e; // extract the Python error text
        const char* name = object->getObject()->Label.getValue();
        Base::Console().Error("ViewProviderFeaturePython::getDisplayModes (%s): %s\n", name, e.what());
    }

    return modes;
}

std::string ViewProviderFeaturePythonImp::setDisplayMode(const char* ModeName)
{
    // Run the setDisplayMode method of the proxy object.
    Base::PyGILStateLocker lock;
    try {
        App::Property* proxy = object->getPropertyByName("Proxy");
        if (proxy && proxy->getTypeId() == App::PropertyPythonObject::getClassTypeId()) {
            Py::Object vp = static_cast<App::PropertyPythonObject*>(proxy)->getValue();
            if (vp.is(Py::_None())) 
                vp = Py::asObject(object->getPyObject());
            if (vp.hasAttr(std::string("setDisplayMode"))) {
                Py::Callable method(vp.getAttr(std::string("setDisplayMode")));
                Py::Tuple args(1);
                args.setItem(0, Py::String(ModeName));
                Py::String str(method.apply(args));
                return str.as_std_string();
            }
        }
    }
    catch (Py::Exception&) {
        Base::PyException e; // extract the Python error text
        const char* name = object->getObject()->Label.getValue();
        Base::Console().Error("ViewProviderFeaturePython::setDisplayMode (%s): %s\n", name, e.what());
    }

    return ModeName;
}

PyObject *ViewProviderFeaturePythonImp::getPyObject(void)
{
    // ref counter is set to 1
    return new ViewProviderFeaturePythonPyT<ViewProviderDocumentObjectPy>(object);
}

// ---------------------------------------------------------

namespace Gui {
PROPERTY_SOURCE_TEMPLATE(Gui::ViewProviderFeaturePython, Gui::ViewProviderDocumentObject)

template<> PyObject* Gui::ViewProviderFeaturePython::getPyObject(void) {
    if (PythonObject.is(Py::_None())) {
        // ref counter is set to 1
        PythonObject = Py::Object(new Gui::ViewProviderFeaturePythonPyT<Gui::ViewProviderDocumentObjectPy>(this),true);
    }
    return Py::new_reference_to(PythonObject);
}
// explicit template instantiation
template class GuiExport ViewProviderFeaturePythonT<ViewProviderDocumentObject>;
}

// ---------------------------------------------------------

namespace Gui {
PROPERTY_SOURCE_TEMPLATE(Gui::ViewProviderPythonGeometry, Gui::ViewProviderGeometryObject)
// explicit template instantiation
template class GuiExport ViewProviderFeaturePythonT<ViewProviderGeometryObject>;
}


