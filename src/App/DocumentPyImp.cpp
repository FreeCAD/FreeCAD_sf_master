/***************************************************************************
 *   Copyright (c) 2007 Jürgen Riegel <juergen.riegel@web.de>              *
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
#endif

#include "Document.h"
#include <Base/FileInfo.h>
#include "DocumentObject.h"
#include "DocumentObjectPy.h"
#include "MergeDocuments.h"
#include "PropertyLinks.h"

// inclusion of the generated files (generated By DocumentPy.xml)
#include "DocumentPy.h"
#include "DocumentPy.cpp"
#include <boost/regex.hpp>

using namespace App;


// returns a string which represent the object e.g. when printed in python
std::string DocumentPy::representation(void) const
{
    std::stringstream str;
    str << "<Document object at " << getDocumentPtr() << ">";

    return str.str();
}

PyObject*  DocumentPy::save(PyObject * args)
{
    if (!PyArg_ParseTuple(args, ""))     // convert args: Python->C
        return NULL;                    // NULL triggers exception

    PY_TRY {
        if (!getDocumentPtr()->save()) {
            PyErr_SetString(PyExc_ValueError, "Object attribute 'FileName' is not set");
            return NULL;
        }
    } PY_CATCH;

    const char* filename = getDocumentPtr()->FileName.getValue();
    Base::FileInfo fi(filename);
    if (!fi.isReadable()) {
        PyErr_Format(PyExc_IOError, "No such file or directory: '%s'", filename);
        return NULL;
    }

    Py_Return;
}

PyObject*  DocumentPy::saveAs(PyObject * args)
{
    char* fn;
    if (!PyArg_ParseTuple(args, "et", "utf-8", &fn))
        return NULL;

    std::string utf8Name = fn;
    PyMem_Free(fn);

    PY_TRY {
        getDocumentPtr()->saveAs(utf8Name.c_str());
        Py_Return;
    }PY_CATCH
}

PyObject*  DocumentPy::saveCopy(PyObject * args)
{
    char* fn;
    if (!PyArg_ParseTuple(args, "s", &fn))     // convert args: Python->C 
        return NULL;                    // NULL triggers exception 

    PY_TRY {
        getDocumentPtr()->saveCopy(fn);
        Py_Return;
    }PY_CATCH
}

PyObject*  DocumentPy::load(PyObject * args)
{
    char* filename=0;
    if (!PyArg_ParseTuple(args, "s", &filename))
        return NULL;
    if (!filename || *filename == '\0') {
        PyErr_Format(PyExc_ValueError, "Path is empty");
        return NULL;
    }

    getDocumentPtr()->FileName.setValue(filename);
    Base::FileInfo fi(filename);
    if (!fi.isReadable()) {
        PyErr_Format(PyExc_IOError, "No such file or directory: '%s'", filename);
        return NULL;
    }
    try {
        getDocumentPtr()->restore();
    } catch (...) {
        PyErr_Format(PyExc_IOError, "Reading from file '%s' failed", filename);
        return NULL;
    }
    Py_Return;
}

PyObject*  DocumentPy::restore(PyObject * args)
{
    if (!PyArg_ParseTuple(args, ""))     // convert args: Python->C 
        return NULL;                    // NULL triggers exception 
    const char* filename = getDocumentPtr()->FileName.getValue();
    if (!filename || *filename == '\0') {
        PyErr_Format(PyExc_ValueError, "Object attribute 'FileName' is not set");
        return NULL;
    }
    Base::FileInfo fi(filename);
    if (!fi.isReadable()) {
        PyErr_Format(PyExc_IOError, "No such file or directory: '%s'", filename);
        return NULL;
    }
    try {
        getDocumentPtr()->restore();
    } catch (...) {
        PyErr_Format(PyExc_IOError, "Reading from file '%s' failed", filename);
        return NULL;
    }
    Py_Return;
}

PyObject*  DocumentPy::mergeProject(PyObject * args)
{
    char* filename;
    if (!PyArg_ParseTuple(args, "s", &filename))     // convert args: Python->C 
        return NULL;                             // NULL triggers exception 

    PY_TRY {
        Base::FileInfo fi(filename);
        Base::ifstream str(fi, std::ios::in | std::ios::binary);
        App::Document* doc = getDocumentPtr();
        MergeDocuments md(doc);
        md.importObjects(str);
        Py_Return;
    } PY_CATCH;
}

PyObject*  DocumentPy::exportGraphviz(PyObject * args)
{
    char* fn=0;
    if (!PyArg_ParseTuple(args, "|s",&fn))     // convert args: Python->C 
        return NULL;                    // NULL triggers exception 
    if (fn) {
        Base::FileInfo fi(fn);
        Base::ofstream str(fi);
        getDocumentPtr()->exportGraphviz(str);
        str.close();
        Py_Return;
    }
    else {
        std::stringstream str;
        getDocumentPtr()->exportGraphviz(str);
#if PY_MAJOR_VERSION >= 3
        return PyUnicode_FromString(str.str().c_str());
#else
        return PyString_FromString(str.str().c_str());
#endif
    }
}

PyObject*  DocumentPy::addObject(PyObject *args, PyObject *kwd)
{
    char *sType,*sName=0,*sViewType=0;
    PyObject* obj=0;
    PyObject* view=0;
    PyObject *attach=Py_False;
    static char *kwlist[] = {"type","name","objProxy","viewProxy","attach","viewType",NULL};
    if (!PyArg_ParseTupleAndKeywords(args,kwd,"s|sOOOs", 
                kwlist, &sType,&sName,&obj,&view,&attach,&sViewType))
        return NULL; 

    DocumentObject *pcFtr = 0;

    if(!obj || !PyObject_IsTrue(attach)) {
        pcFtr = getDocumentPtr()->addObject(sType,sName,true,sViewType);
    }else{
        Base::BaseClass* base = static_cast<Base::BaseClass*>(Base::Type::createInstanceByName(sType,true));
        if(base) {
            if (!base->getTypeId().isDerivedFrom(App::DocumentObject::getClassTypeId())) {
                delete base;
                std::stringstream str;
                str << "'" << sType << "' is not a document object type";
                throw Base::TypeError(str.str());
            }
            pcFtr = static_cast<DocumentObject*>(base);
        }
    }
    if (pcFtr) {
        // Allows to hide the handling with Proxy in client python code
        if (obj) {
            try {
                // the python binding class to the document object
                Py::Object pyftr = Py::asObject(pcFtr->getPyObject());
                // 'pyobj' is the python class with the implementation for DocumentObject
                Py::Object pyobj(obj);
                if (pyobj.hasAttr("__object__")) {
                    pyobj.setAttr("__object__", pyftr);
                }
                pyftr.setAttr("Proxy", pyobj);

                if(PyObject_IsTrue(attach)) {
                    getDocumentPtr()->addObject(pcFtr,sName);

                    try {
                        Py::Callable method(pyobj.getAttr("attach"));
                        if(!method.isNone()) {
                            Py::TupleN arg(pyftr);
                            method.apply(arg);
                        }
                    }catch (Py::Exception&) {
                        Base::PyException e;
                        e.ReportException();
                    }
                }

                // if a document class is set we also need a view provider defined which must be
                // something different to None
                Py::Object pyvp;
                if (view)
                    pyvp = Py::Object(view);
                if (pyvp.isNone())
                    pyvp = Py::Int(1);
                // 'pyvp' is the python class with the implementation for ViewProvider
                if (pyvp.hasAttr("__vobject__")) {
                    pyvp.setAttr("__vobject__", pyftr.getAttr("ViewObject"));
                }
                pyftr.getAttr("ViewObject").setAttr("Proxy", pyvp);
                return Py::new_reference_to(pyftr);
            }
            catch (Py::Exception& e) {
                e.clear();
            }
        }
        return pcFtr->getPyObject();
    }
    else {
        std::stringstream str;
        str << "No document object found of type '" << sType << "'" << std::ends;
        throw Py::Exception(Base::BaseExceptionFreeCADError,str.str());
    }
}

PyObject*  DocumentPy::removeObject(PyObject *args)
{
    char *sName;
    if (!PyArg_ParseTuple(args, "s",&sName))     // convert args: Python->C
        return NULL;                             // NULL triggers exception


    DocumentObject *pcFtr = getDocumentPtr()->getObject(sName);
    if(pcFtr) {
        getDocumentPtr()->removeObject( sName );
        Py_Return;
    } else {
        std::stringstream str;
        str << "No document object found with name '" << sName << "'" << std::ends;
        throw Py::Exception(Base::BaseExceptionFreeCADError,str.str());
    }
}

PyObject*  DocumentPy::copyObject(PyObject *args)
{
    PyObject *obj, *rec=Py_False;
    if (!PyArg_ParseTuple(args, "O|O",&obj,&rec))
        return NULL;    // NULL triggers exception

    std::vector<App::DocumentObject*> objs;
    bool single = false;
    if(PySequence_Check(obj)) {
        Py::Sequence seq(obj);
        for(size_t i=0;i<seq.size();++i) {
            if(!PyObject_TypeCheck(seq[i].ptr(),&DocumentObjectPy::Type)) {
                PyErr_SetString(PyExc_TypeError, "Expect element in sequence to be of type document object");
                return 0;
            }
            objs.push_back(static_cast<DocumentObjectPy*>(seq[i].ptr())->getDocumentObjectPtr());
        }
    }else if(!PyObject_TypeCheck(obj,&DocumentObjectPy::Type)) {
        PyErr_SetString(PyExc_TypeError, 
            "Expect first argument to be either a document object or sequence of document objects");
        return 0;
    }else {
        objs.push_back(static_cast<DocumentObjectPy*>(obj)->getDocumentObjectPtr());
        single = true;
    }

    PY_TRY {
        auto ret = getDocumentPtr()->copyObject(objs,PyObject_IsTrue(rec));
        if(ret.size()==1 && single)
            return ret[0]->getPyObject();

        Py::Tuple tuple(ret.size());
        for(size_t i=0;i<ret.size();++i) 
            tuple.setItem(i,Py::Object(ret[i]->getPyObject(),true));
        return Py::new_reference_to(tuple);
    }PY_CATCH
}

PyObject*  DocumentPy::importLinks(PyObject *args)
{
    PyObject *obj = Py_None;
    if (!PyArg_ParseTuple(args, "|O",&obj))
        return NULL;    // NULL triggers exception

    std::vector<App::DocumentObject*> objs;
    if(PySequence_Check(obj)) {
        Py::Sequence seq(obj);
        for(size_t i=0;i<seq.size();++i) {
            if(!PyObject_TypeCheck(seq[i].ptr(),&DocumentObjectPy::Type)) {
                PyErr_SetString(PyExc_TypeError, "Expect element in sequence to be of type document object");
                return 0;
            }
            objs.push_back(static_cast<DocumentObjectPy*>(seq[i].ptr())->getDocumentObjectPtr());
        }
    }else if(obj == Py_None) {
    }else if(!PyObject_TypeCheck(obj,&DocumentObjectPy::Type)) {
        PyErr_SetString(PyExc_TypeError, 
            "Expect first argument to be either a document object or sequence of document objects");
        return 0;
    }else
        objs.push_back(static_cast<DocumentObjectPy*>(obj)->getDocumentObjectPtr());
    
    if(objs.empty())
        objs = getDocumentPtr()->getObjects();

    PY_TRY {
        auto ret = getDocumentPtr()->importLinks(objs);

        Py::Tuple tuple(ret.size());
        for(size_t i=0;i<ret.size();++i) 
            tuple.setItem(i,Py::Object(ret[i]->getPyObject(),true));
        return Py::new_reference_to(tuple);
    }PY_CATCH
}

PyObject*  DocumentPy::moveObject(PyObject *args)
{
    PyObject *obj, *rec=Py_False;
    if (!PyArg_ParseTuple(args, "O!|O!",&(DocumentObjectPy::Type),&obj,&PyBool_Type,&rec))
        return NULL;    // NULL triggers exception

    DocumentObjectPy* docObj = static_cast<DocumentObjectPy*>(obj);
    DocumentObject* move = getDocumentPtr()->moveObject(docObj->getDocumentObjectPtr(), PyObject_IsTrue(rec) ? true : false);
    if (move) {
        return move->getPyObject();
    }
    else {
        std::string str("Failed to move the object");
        throw Py::Exception(Base::BaseExceptionFreeCADError,str);
    }
}

PyObject*  DocumentPy::openTransaction(PyObject *args)
{
    PyObject *value = 0;
    if (!PyArg_ParseTuple(args, "|O",&value))
        return NULL;    // NULL triggers exception
    std::string cmd;


    if (!value) {
        cmd = "<empty>";
    }
#if PY_MAJOR_VERSION >= 3
    else if (PyUnicode_Check(value)) {
        cmd = PyUnicode_AsUTF8(value);
    }
#else
    else if (PyUnicode_Check(value)) {
        PyObject* unicode = PyUnicode_AsUTF8String(value);
        cmd = PyString_AsString(unicode);
        Py_DECREF(unicode);
    }
    else if (PyString_Check(value)) {
        cmd = PyString_AsString(value);
    }
#endif
    else {
        PyErr_SetString(PyExc_TypeError, "string or unicode expected");
        return NULL;
    }

    getDocumentPtr()->openTransaction(cmd.c_str());
    Py_Return; 
}

PyObject*  DocumentPy::abortTransaction(PyObject * args)
{
    if (!PyArg_ParseTuple(args, ""))     // convert args: Python->C 
        return NULL;                    // NULL triggers exception 
    getDocumentPtr()->abortTransaction();
    Py_Return;
}

PyObject*  DocumentPy::commitTransaction(PyObject * args)
{
    if (!PyArg_ParseTuple(args, ""))     // convert args: Python->C 
        return NULL;                    // NULL triggers exception 
    getDocumentPtr()->commitTransaction();
    Py_Return;
}

Py::Boolean DocumentPy::getHasPendingTransaction() const {
    return Py::Boolean(getDocumentPtr()->hasPendingTransaction());
}

PyObject*  DocumentPy::undo(PyObject * args)
{
    if (!PyArg_ParseTuple(args, ""))     // convert args: Python->C 
        return NULL;                    // NULL triggers exception 
    if (getDocumentPtr()->getAvailableUndos())
        getDocumentPtr()->undo();
    Py_Return;
}

PyObject*  DocumentPy::redo(PyObject * args)
{
    if (!PyArg_ParseTuple(args, ""))     // convert args: Python->C 
        return NULL;                    // NULL triggers exception 
    if (getDocumentPtr()->getAvailableRedos())
        getDocumentPtr()->redo();
    Py_Return;
}

PyObject*  DocumentPy::clearUndos(PyObject * args)
{
    if (!PyArg_ParseTuple(args, ""))     // convert args: Python->C 
        return NULL;                    // NULL triggers exception 
    getDocumentPtr()->clearUndos();
    Py_Return;
}

PyObject*  DocumentPy::recompute(PyObject * args)
{
    PyObject *pyobjs = Py_None;
    PyObject *force = Py_False;
    PyObject *checkCycle = Py_False;
    if (!PyArg_ParseTuple(args, "|OO!O!",&pyobjs,
                &PyBool_Type,&force,&PyBool_Type,&checkCycle))
        return nullptr;

    PY_TRY {
        std::vector<App::DocumentObject *> objs;
        if (pyobjs!=Py_None) {
            if (!PySequence_Check(pyobjs)) {
                PyErr_SetString(PyExc_TypeError, "expect input of sequence of document objects");
                return nullptr;
            }

            Py::Sequence seq(pyobjs);
            for (size_t i=0;i<seq.size();++i) {
                if (!PyObject_TypeCheck(seq[i].ptr(), &DocumentObjectPy::Type)) {
                    PyErr_SetString(PyExc_TypeError, "Expect element in sequence to be of type document object");
                    return nullptr;
                }
                objs.push_back(static_cast<DocumentObjectPy*>(seq[i].ptr())->getDocumentObjectPtr());
            }
        }

        int options = 0;
        if (PyObject_IsTrue(checkCycle))
            options = Document::DepNoCycle;

        int objectCount = getDocumentPtr()->recompute(objs, PyObject_IsTrue(force), 0, options);

        // Document::recompute() hides possibly raised Python exceptions by its features
        // So, check if an error is set and return null if yes
        if (PyErr_Occurred()) {
            return nullptr;
        }

        return Py::new_reference_to(Py::Int(objectCount));
    } PY_CATCH;
}

PyObject*  DocumentPy::getObject(PyObject *args)
{
    long id = -1;
    char *sName = 0;
    if (!PyArg_ParseTuple(args, "s",&sName))  {   // convert args: Python->C 
        if(!PyArg_ParseTuple(args, "l", &id))
            return NULL;                             // NULL triggers exception 
    }

    DocumentObject *pcFtr = sName?getDocumentPtr()->getObject(sName):getDocumentPtr()->getObjectByID(id);
    if (pcFtr)
        return pcFtr->getPyObject();
    else
        Py_Return;
}

PyObject*  DocumentPy::getObjectsByLabel(PyObject *args)
{
    char *sName;
    if (!PyArg_ParseTuple(args, "s",&sName))     // convert args: Python->C 
        return NULL;                             // NULL triggers exception 

    Py::List list;
    std::string name = sName;
    std::vector<DocumentObject*> objs = getDocumentPtr()->getObjects();
    for (std::vector<DocumentObject*>::iterator it = objs.begin(); it != objs.end(); ++it) {
        if (name == (*it)->Label.getValue())
            list.append(Py::asObject((*it)->getPyObject()));
    }

    return Py::new_reference_to(list);
}

PyObject*  DocumentPy::findObjects(PyObject *args, PyObject *kwds)
{
    const char *sType = "App::DocumentObject", *sName = nullptr, *sLabel = nullptr;
    static char *kwlist[] = {"Type", "Name", "Label", nullptr};
    if (!PyArg_ParseTupleAndKeywords(args, kwds, "|sss",
                kwlist, &sType, &sName, &sLabel))
        return nullptr;

    Base::Type type = Base::Type::fromName(sType);
    if (type == Base::Type::badType()) {
        PyErr_Format(PyExc_TypeError, "'%s' is not a valid type", sType);
        return nullptr;
    }

    if (!type.isDerivedFrom(App::DocumentObject::getClassTypeId())) {
        PyErr_Format(PyExc_TypeError, "Type '%s' does not inherit from 'App::DocumentObject'", sType);
        return nullptr;
    }

    std::vector<DocumentObject*> res;

    try {
        res = getDocumentPtr()->findObjects(type, sName, sLabel);
    }
    catch (const boost::regex_error& e) {
        PyErr_SetString(PyExc_RuntimeError, e.what());
        return 0;
    }

    Py_ssize_t index=0;
    PyObject* list = PyList_New((Py_ssize_t)res.size()); 
    for (std::vector<DocumentObject*>::const_iterator It = res.begin();It != res.end();++It, index++)
        PyList_SetItem(list, index, (*It)->getPyObject());
    return list;
}

Py::Object DocumentPy::getActiveObject(void) const
{
    DocumentObject *pcFtr = getDocumentPtr()->getActiveObject();
    if(pcFtr)
        return Py::Object(pcFtr->getPyObject(), true);
    return Py::None();
}

PyObject*  DocumentPy::supportedTypes(PyObject *args)
{
    if (!PyArg_ParseTuple(args, ""))     // convert args: Python->C 
        return NULL;                    // NULL triggers exception
    
    std::vector<Base::Type> ary;
    Base::Type::getAllDerivedFrom(App::DocumentObject::getClassTypeId(), ary);
    Py::List res;
    for (std::vector<Base::Type>::iterator it = ary.begin(); it != ary.end(); ++it)
        res.append(Py::String(it->getName()));
    return Py::new_reference_to(res);
}

Py::List DocumentPy::getObjects(void) const 
{
    std::vector<DocumentObject*> objs = getDocumentPtr()->getObjects();
    Py::List res;

    for (std::vector<DocumentObject*>::const_iterator It = objs.begin();It != objs.end();++It)
        //Note: Here we must force the Py::Object to own this Python object as getPyObject() increments the counter
        res.append(Py::Object((*It)->getPyObject(), true));

    return res;
}

Py::List DocumentPy::getTopologicalSortedObjects(void) const
{
    std::vector<DocumentObject*> objs = getDocumentPtr()->topologicalSort();
    Py::List res;

    for (std::vector<DocumentObject*>::const_iterator It = objs.begin(); It != objs.end(); ++It)
        //Note: Here we must force the Py::Object to own this Python object as getPyObject() increments the counter
        res.append(Py::Object((*It)->getPyObject(), true));

    return res;
}

Py::List DocumentPy::getRootObjects(void) const
{
    std::vector<DocumentObject*> objs = getDocumentPtr()->getRootObjects();
    Py::List res;

    for (std::vector<DocumentObject*>::const_iterator It = objs.begin(); It != objs.end(); ++It)
        //Note: Here we must force the Py::Object to own this Python object as getPyObject() increments the counter
        res.append(Py::Object((*It)->getPyObject(), true));

    return res;
}

Py::Int DocumentPy::getUndoMode(void) const
{
    return Py::Int(getDocumentPtr()->getUndoMode());
}

void  DocumentPy::setUndoMode(Py::Int arg)
{
    getDocumentPtr()->setUndoMode(arg);
}


Py::Int DocumentPy::getUndoRedoMemSize(void) const
{
    return Py::Int((long)getDocumentPtr()->getUndoMemSize());
}

Py::Int DocumentPy::getUndoCount(void) const
{
    return Py::Int((long)getDocumentPtr()->getAvailableUndos());
}

Py::Int DocumentPy::getRedoCount(void) const
{
    return Py::Int((long)getDocumentPtr()->getAvailableRedos());
}

Py::List DocumentPy::getUndoNames(void) const
{
    std::vector<std::string> vList = getDocumentPtr()->getAvailableUndoNames();
    Py::List res;

    for (std::vector<std::string>::const_iterator It = vList.begin();It!=vList.end();++It)
        res.append(Py::String(*It));

    return res;
}

Py::List DocumentPy::getRedoNames(void) const
{
    std::vector<std::string> vList = getDocumentPtr()->getAvailableRedoNames();
    Py::List res;

    for (std::vector<std::string>::const_iterator It = vList.begin();It!=vList.end();++It)
        res.append(Py::String(*It));

    return res;
}

Py::String  DocumentPy::getDependencyGraph(void) const
{
    std::stringstream out;
    getDocumentPtr()->exportGraphviz(out);
    return Py::String(out.str());
}

Py::String DocumentPy::getName(void) const
{
    return Py::String(getDocumentPtr()->getName());
}

Py::Boolean DocumentPy::getRecomputesFrozen(void) const
{
    return Py::Boolean(getDocumentPtr()->testStatus(Document::Status::SkipRecompute));
}

void DocumentPy::setRecomputesFrozen(Py::Boolean arg)
{
    getDocumentPtr()->setStatus(Document::Status::SkipRecompute, arg.isTrue());
}

PyObject* DocumentPy::getTempFileName(PyObject *args)
{
    PyObject *value;
    if (!PyArg_ParseTuple(args, "O",&value))
        return NULL;    // NULL triggers exception

    std::string string;
    if (PyUnicode_Check(value)) {
#if PY_MAJOR_VERSION >= 3
        string = PyUnicode_AsUTF8(value);
#else
        PyObject* unicode = PyUnicode_AsUTF8String(value);
        string = PyString_AsString(unicode);
        Py_DECREF(unicode);
    }
    else if (PyString_Check(value)) {
        string = PyString_AsString(value);
#endif
    }
    else {
        std::string error = std::string("type must be a string!");
        error += value->ob_type->tp_name;
        throw Py::TypeError(error);
    }

    // search for a temp file name in the document transient directory 
    Base::FileInfo fileName(Base::FileInfo::getTempFileName
        (string.c_str(),getDocumentPtr()->TransientDir.getValue()));
    // delete the created file, we need only the name...
    fileName.deleteFile();

    PyObject *p = PyUnicode_DecodeUTF8(fileName.filePath().c_str(),fileName.filePath().size(),0);
    if (!p) {
        throw Base::UnicodeError("UTF8 conversion failure at PropertyString::getPyObject()");
    }
    return p;
}

PyObject *DocumentPy::getCustomAttributes(const char* attr) const
{
    // Note: Here we want to return only a document object if its
    // name matches 'attr'. However, it is possible to have an object
    // with the same name as an attribute. If so, we return 0 as other-
    // wise it wouldn't be possible to address this attribute any more.
    // The object must then be addressed by the getObject() method directly.
    App::Property* prop = getPropertyContainerPtr()->getPropertyByName(attr);
    if (prop) return 0;
    if (this->ob_type->tp_dict == NULL) {
        if (PyType_Ready(this->ob_type) < 0)
            return 0;
    }
    PyObject* item = PyDict_GetItemString(this->ob_type->tp_dict, attr);
    if (item) return 0;
    // search for an object with this name
    DocumentObject* obj = getDocumentPtr()->getObject(attr);
    return (obj ? obj->getPyObject() : 0);
}

int DocumentPy::setCustomAttributes(const char* attr, PyObject *)
{
    // Note: Here we want to return only a document object if its
    // name matches 'attr'. However, it is possible to have an object
    // with the same name as an attribute. If so, we return 0 as other-
    // wise it wouldn't be possible to address this attribute any more.
    // The object must then be addressed by the getObject() method directly.
    App::Property* prop = getPropertyContainerPtr()->getPropertyByName(attr);
    if (prop) return 0;
    if (this->ob_type->tp_dict == NULL) {
        if (PyType_Ready(this->ob_type) < 0)
            return 0;
    }
    PyObject* item = PyDict_GetItemString(this->ob_type->tp_dict, attr);
    if (item) return 0;
    DocumentObject* obj = getDocumentPtr()->getObject(attr);
    if (obj)
    {
        std::stringstream str;
        str << "'Document' object attribute '" << attr 
            << "' must not be set this way" << std::ends;
        PyErr_SetString(PyExc_RuntimeError, str.str().c_str());
        return -1;
    }

    return 0;
}

PyObject* DocumentPy::getLinksTo(PyObject *args)
{
    PyObject *pyobj = Py_None;
    int options = 0;
    short count = 0;
    if (!PyArg_ParseTuple(args, "|Oih", &pyobj,&options, &count))
        return NULL;

    PY_TRY {
        DocumentObject *obj = 0;
        if(pyobj!=Py_None) {
            if(!PyObject_TypeCheck(pyobj,&DocumentObjectPy::Type)) {
                PyErr_SetString(PyExc_TypeError, "Expect the first argument of type document object");
                return 0;
            }
            obj = static_cast<DocumentObjectPy*>(pyobj)->getDocumentObjectPtr();
        }
        std::set<DocumentObject *> links;
        getDocumentPtr()->getLinksTo(links,obj,options,count);
        Py::Tuple ret(links.size());
        int i=0;
        for(auto o : links) 
            ret.setItem(i++,Py::Object(o->getPyObject(),true));
        return Py::new_reference_to(ret);
    }PY_CATCH
}

Py::List DocumentPy::getInList(void) const
{
    Py::List ret;
    auto lists = PropertyXLink::getDocumentInList(getDocumentPtr());
    if(lists.size()==1) {
        for(auto doc : lists.begin()->second)
            ret.append(Py::Object(doc->getPyObject(), true));
    }
    return ret;
}

Py::List DocumentPy::getOutList(void) const
{
    Py::List ret;
    auto lists = PropertyXLink::getDocumentOutList(getDocumentPtr());
    if(lists.size()==1) {
        for(auto doc : lists.begin()->second)
            ret.append(Py::Object(doc->getPyObject(), true));
    }
    return ret;
}

PyObject *DocumentPy::getDependentDocuments(PyObject *args) {
    PyObject *sort = Py_True;
    if (!PyArg_ParseTuple(args, "|O", &sort))
        return 0;
    PY_TRY {
        auto docs = getDocumentPtr()->getDependentDocuments(PyObject_IsTrue(sort));
        Py::List ret;
        for(auto doc : docs)
            ret.append(Py::Object(doc->getPyObject(), true));
        return Py::new_reference_to(ret);
    } PY_CATCH;
}

Py::Boolean DocumentPy::getRestoring(void) const
{
    return Py::Boolean(getDocumentPtr()->testStatus(Document::Status::Restoring));
}

Py::Boolean DocumentPy::getPartial(void) const
{
    return Py::Boolean(getDocumentPtr()->testStatus(Document::Status::PartialDoc));
}

Py::Boolean DocumentPy::getImporting(void) const
{
    return Py::Boolean(getDocumentPtr()->testStatus(Document::Status::Importing));
}

Py::Boolean DocumentPy::getRecomputing(void) const
{
    return Py::Boolean(getDocumentPtr()->testStatus(Document::Status::Recomputing));
}

Py::Boolean DocumentPy::getTransacting() const {
    return Py::Boolean(getDocumentPtr()->isPerformingTransaction());
}

Py::String DocumentPy::getOldLabel() const {
    return Py::String(getDocumentPtr()->getOldLabel());
}

