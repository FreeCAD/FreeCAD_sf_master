/***************************************************************************
 *   Copyright (c) Jürgen Riegel          (juergen.riegel@web.de)          *
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
#endif

#include <Base/Writer.h>

#include "Document.h"
#include "DocumentObject.h"
#include "DocumentObjectGroup.h"
#include "PropertyLinks.h"
#include "PropertyExpressionEngine.h"
#include "DocumentObjectExtension.h"
#include <App/DocumentObjectPy.h>
#include <boost/signals/connection.hpp>
#include <boost/bind.hpp>

using namespace App;


PROPERTY_SOURCE(App::DocumentObject, App::TransactionalObject)

DocumentObjectExecReturn *DocumentObject::StdReturn = 0;

//===========================================================================
// DocumentObject
//===========================================================================

DocumentObject::DocumentObject(void)
    : ExpressionEngine(),_pDoc(0),pcNameInDocument(0)
{
    // define Label of type 'Output' to avoid being marked as touched after relabeling
    ADD_PROPERTY_TYPE(Label,("Unnamed"),"Base",Prop_Output,"User name of the object (UTF8)");
    ADD_PROPERTY_TYPE(ExpressionEngine,(),"Base",Prop_Hidden,"Property expressions");
}

DocumentObject::~DocumentObject(void)
{
    if (!PythonObject.is(Py::_None())){
        // Remark: The API of Py::Object has been changed to set whether the wrapper owns the passed
        // Python object or not. In the constructor we forced the wrapper to own the object so we need
        // not to dec'ref the Python object any more.
        // But we must still invalidate the Python object because it need not to be
        // destructed right now because the interpreter can own several references to it.
        Base::PyObjectBase* obj = (Base::PyObjectBase*)PythonObject.ptr();
        // Call before decrementing the reference counter, otherwise a heap error can occur
        obj->setInvalid();
    }
}

App::DocumentObjectExecReturn *DocumentObject::recompute(void)
{
    // set/unset the execution bit
    ObjectStatusLocker exe(App::Recompute, this);
    return this->execute();
}

DocumentObjectExecReturn *DocumentObject::execute(void)
{
    //call all extensions
    auto vector = getExtensionsDerivedFromType<App::DocumentObjectExtension>();
    for(auto ext : vector)
        ext->extensionExecute();

    return StdReturn;
}

bool DocumentObject::recomputeFeature()
{
    Document* doc = this->getDocument();
    if (doc)
        doc->recomputeFeature(this);
    return isValid();
}

short DocumentObject::mustExecute(void) const
{
    if(isTouched())
        return 1;

    //ask all extensions
    auto vector = getExtensionsDerivedFromType<App::DocumentObjectExtension>();
    for(auto ext : vector) {
        if(ext->extensionMustExecute())
            return 1;
    }
    return 0;
    
}

const char* DocumentObject::getStatusString(void) const
{
    if (isError()) {
        const char* text = getDocument()->getErrorDescription(this);
        return text ? text : "Error";
    }
    else if (isTouched())
        return "Touched";
    else
        return "Valid";
}

const char *DocumentObject::getNameInDocument(void) const
{
    // Note: It can happen that we query the internal name of an object even if it is not
    // part of a document (anymore). This is the case e.g. if we have a reference in Python
    // to an object that has been removed from the document. In this case we should rather
    // return 0.
    //assert(pcNameInDocument);
    if (!pcNameInDocument) return 0;
    return pcNameInDocument->c_str();
}

bool DocumentObject::isAttachedToDocument() const
{
    return (pcNameInDocument != 0);
}

const char* DocumentObject::detachFromDocument()
{
    const std::string* name = pcNameInDocument;
    pcNameInDocument = 0;
    return name ? name->c_str() : 0;
}

std::vector<DocumentObject*> DocumentObject::getOutList(void) const
{
    std::vector<Property*> List;
    std::vector<DocumentObject*> ret;
    getPropertyList(List);
    for (std::vector<Property*>::const_iterator It = List.begin();It != List.end(); ++It) {
        if ((*It)->isDerivedFrom(PropertyLinkList::getClassTypeId())) {
            const std::vector<DocumentObject*> &OutList = static_cast<PropertyLinkList*>(*It)->getValues();
            for (std::vector<DocumentObject*>::const_iterator It2 = OutList.begin();It2 != OutList.end(); ++It2) {
                if (*It2)
                    ret.push_back(*It2);
            }
        }
        else if ((*It)->isDerivedFrom(PropertyLinkSubList::getClassTypeId())) {
            const std::vector<DocumentObject*> &OutList = static_cast<PropertyLinkSubList*>(*It)->getValues();
            for (std::vector<DocumentObject*>::const_iterator It2 = OutList.begin();It2 != OutList.end(); ++It2) {
                if (*It2)
                    ret.push_back(*It2);
            }
        }
        else if ((*It)->isDerivedFrom(PropertyLink::getClassTypeId())) {
            if (static_cast<PropertyLink*>(*It)->getValue())
                ret.push_back(static_cast<PropertyLink*>(*It)->getValue());
        }
        else if ((*It)->isDerivedFrom(PropertyLinkSub::getClassTypeId())) {
            if (static_cast<PropertyLinkSub*>(*It)->getValue())
                ret.push_back(static_cast<PropertyLinkSub*>(*It)->getValue());
        }
    }

    // Get document objects that this document object relies on
    ExpressionEngine.getDocumentObjectDeps(ret);

    return ret;
}

#if USE_OLD_DAG
std::vector<App::DocumentObject*> DocumentObject::getInList(void) const
{
    if (_pDoc)
        return _pDoc->getInList(this);
    else
        return std::vector<App::DocumentObject*>();
}

#else // if USE_OLD_DAG

std::vector<App::DocumentObject*> DocumentObject::getInList(void) const
{
    return _inList;
}

#endif // if USE_OLD_DAG


void _getInListRecursive(std::vector<DocumentObject*>& objSet, const DocumentObject* obj, const DocumentObject* checkObj, int depth)
{
    for (const auto objIt : obj->getInList()){
        // if the check object is in the recursive inList we have a cycle!
        if (objIt == checkObj || depth <= 0){
            std::cerr << "DocumentObject::getInListRecursive(): cyclic dependency detected!"<<std::endl;
            throw Base::Exception("DocumentObject::getInListRecursive(): cyclic dependency detected!");
        }

        objSet.push_back(objIt);
        _getInListRecursive(objSet, objIt, checkObj,depth-1);
    }
}

std::vector<App::DocumentObject*> DocumentObject::getInListRecursive(void) const
{
    // number of objects in document is a good estimate in result size
    int maxDepth = getDocument()->countObjects() +2;
    std::vector<App::DocumentObject*> result;
    result.reserve(maxDepth);

    // using a rcursie helper to collect all InLists
    _getInListRecursive(result, this, this, maxDepth);

    // remove duplicate entries and resize the vector
    std::sort(result.begin(), result.end());
    auto newEnd = std::unique(result.begin(), result.end());
    result.resize(std::distance(result.begin(), newEnd));

    return result;
}

void _getOutListRecursive(std::vector<DocumentObject*>& objSet, const DocumentObject* obj, const DocumentObject* checkObj, int depth)
{
    for (const auto objIt : obj->getOutList()){
        // if the check object is in the recursive inList we have a cycle!
        if (objIt == checkObj || depth <= 0){
            std::cerr << "DocumentObject::getOutListRecursive(): cyclic dependency detected!" << std::endl;
            throw Base::Exception("DocumentObject::getOutListRecursive(): cyclic dependency detected!");
        }
        objSet.push_back(objIt);
        _getOutListRecursive(objSet, objIt, checkObj,depth-1);
    }
}

std::vector<App::DocumentObject*> DocumentObject::getOutListRecursive(void) const
{
    // number of objects in document is a good estimate in result size
    int maxDepth = getDocument()->countObjects() + 2;
    std::vector<App::DocumentObject*> result;
    result.reserve(maxDepth);

    // using a recursive helper to collect all OutLists
    _getOutListRecursive(result, this, this, maxDepth);

    // remove duplicate entries and resize the vector
    std::sort(result.begin(), result.end());
    auto newEnd = std::unique(result.begin(), result.end());
    result.resize(std::distance(result.begin(), newEnd));

    return result;
}

DocumentObjectGroup* DocumentObject::getGroup() const
{
    return dynamic_cast<DocumentObjectGroup*>(GroupExtension::getGroupOfObject(this));
}

bool DocumentObject::testIfLinkDAGCompatible(DocumentObject *linkTo) const
{
    std::vector<App::DocumentObject*> linkTo_in_vector;
    linkTo_in_vector.push_back(linkTo);
    return this->testIfLinkDAGCompatible(linkTo_in_vector);
}

bool DocumentObject::testIfLinkDAGCompatible(const std::vector<DocumentObject *> &linksTo) const
{
    Document* doc = this->getDocument();
    if (!doc)
        throw Base::Exception("DocumentObject::testIfLinkIsDAG: object is not in any document.");
    std::vector<App::DocumentObject*> deplist = doc->getDependencyList(linksTo);
    if( std::find(deplist.begin(),deplist.end(),this) != deplist.end() )
        //found this in dependency list
        return false;
    else
        return true;
}

bool DocumentObject::testIfLinkDAGCompatible(PropertyLinkSubList &linksTo) const
{
    const std::vector<App::DocumentObject*> &linksTo_in_vector = linksTo.getValues();
    return this->testIfLinkDAGCompatible(linksTo_in_vector);
}

bool DocumentObject::testIfLinkDAGCompatible(PropertyLinkSub &linkTo) const
{
    std::vector<App::DocumentObject*> linkTo_in_vector;
    linkTo_in_vector.reserve(1);
    linkTo_in_vector.push_back(linkTo.getValue());
    return this->testIfLinkDAGCompatible(linkTo_in_vector);
}

#if USE_OLD_DAG
#else
bool DocumentObject::_isInInListRecursive(const DocumentObject *act, const DocumentObject* test, const DocumentObject* checkObj, int depth) const
{
    if (std::find(_inList.begin(), _inList.end(), test) != _inList.end())
        return true;

    for (auto obj : _inList){
        // if the check object is in the recursive inList we have a cycle!
        if (obj == checkObj || depth <= 0){
            std::cerr << "DocumentObject::getOutListRecursive(): cyclic dependency detected!" << std::endl;
            throw Base::Exception("DocumentObject::getOutListRecursive(): cyclic dependency detected!");
        }

        if (_isInInListRecursive(obj, test, checkObj, depth - 1))
            return true;
    }

    return false;
}

bool DocumentObject::isInInListRecursive(DocumentObject *linkTo) const
{
    return _isInInListRecursive(this, linkTo, this, getDocument()->countObjects());
}

bool DocumentObject::isInInList(DocumentObject *linkTo) const
{
    if (std::find(_inList.begin(), _inList.end(), linkTo) != _inList.end())
        return true;
    else
        return false;
}

bool DocumentObject::_isInOutListRecursive(const DocumentObject *act, const DocumentObject* test, const DocumentObject* checkObj, int depth) const
{
    std::vector <DocumentObject*> outList = act->getOutList();

    if (std::find(outList.begin(), outList.end(), test) != outList.end())
        return true;

    for (auto obj : outList){
        // if the check object is in the recursive inList we have a cycle!
        if (obj == checkObj || depth <= 0){
            std::cerr << "DocumentObject::isInOutListRecursive(): cyclic dependency detected!" << std::endl;
            throw Base::Exception("DocumentObject::isInOutListRecursive(): cyclic dependency detected!");
        }

        if (_isInOutListRecursive(obj, test, checkObj, depth - 1))
            return true;
    }

    return false;
}

bool DocumentObject::isInOutListRecursive(DocumentObject *linkTo) const
{
    return _isInOutListRecursive(this, linkTo, this, getDocument()->countObjects());
}
#endif //USE_OLD_DAG

void DocumentObject::onLostLinkToObject(DocumentObject*)
{

}

App::Document *DocumentObject::getDocument(void) const
{
    return _pDoc;
}

void DocumentObject::setDocument(App::Document* doc)
{
    _pDoc=doc;
    onSettingDocument();
}

void DocumentObject::onBeforeChange(const Property* prop)
{

    // Store current name in oldLabel, to be able to easily retrieve old name of document object later
    // when renaming expressions.
    if (prop == &Label)
        oldLabel = Label.getStrValue();

    if (_pDoc)
        onBeforeChangeProperty(_pDoc, prop);
}

/// get called by the container when a Property was changed
void DocumentObject::onChanged(const Property* prop)
{
    if (_pDoc)
        _pDoc->onChangedProperty(this,prop);

    if (prop == &Label && _pDoc && oldLabel != Label.getStrValue())
        _pDoc->signalRelabelObject(*this);

    if (prop->getType() & Prop_Output)
        return;
    // set object touched
    StatusBits.set(0);
}

PyObject *DocumentObject::getPyObject(void)
{
    if (PythonObject.is(Py::_None())) {
        // ref counter is set to 1
        PythonObject = Py::Object(new DocumentObjectPy(this),true);
    }
    return Py::new_reference_to(PythonObject);
}

std::vector<PyObject *> DocumentObject::getPySubObjects(const std::vector<std::string>&) const
{
    // default implementation returns nothing
    return std::vector<PyObject *>();
}

void DocumentObject::touch(void)
{
    StatusBits.set(0);
}

/**
 * @brief Check whether the document object is touched or not.
 * @return true if document object is touched, false if not.
 */

bool DocumentObject::isTouched() const
{
    return ExpressionEngine.isTouched() || StatusBits.test(0);
}

void DocumentObject::Save (Base::Writer &writer) const
{
    writer.ObjectName = this->getNameInDocument();
    App::ExtensionContainer::Save(writer);
}

/**
 * @brief Associate the expression \expr with the object identifier \a path in this document object.
 * @param path Target object identifier for the result of the expression
 * @param expr Expression tree
 * @param comment Optional comment describing the expression
 */

void DocumentObject::setExpression(const ObjectIdentifier &path, boost::shared_ptr<Expression> expr, const char * comment)
{
    ExpressionEngine.setValue(path, expr, comment);
    connectRelabelSignals();
}

/**
 * @brief Get expression information associated with \a path.
 * @param path Object identifier
 * @return Expression info, containing expression and optional comment.
 */

const PropertyExpressionEngine::ExpressionInfo DocumentObject::getExpression(const ObjectIdentifier &path) const
{
    boost::any value = ExpressionEngine.getPathValue(path);

    if (value.type() == typeid(PropertyExpressionEngine::ExpressionInfo))
        return boost::any_cast<PropertyExpressionEngine::ExpressionInfo>(value);
    else
        return PropertyExpressionEngine::ExpressionInfo();
}

/**
 * @brief Invoke ExpressionEngine's renameObjectIdentifier, to possibly rewrite expressions using
 * the \a paths map with current and new identifiers.
 *
 * @param paths
 */

void DocumentObject::renameObjectIdentifiers(const std::map<ObjectIdentifier, ObjectIdentifier> &paths)
{
    ExpressionEngine.renameObjectIdentifiers(paths);
}

/**
 * @brief Helper function that sets up a signal to track document object renames.
 */

void DocumentObject::connectRelabelSignals()
{
    // Only keep signal if the ExpressionEngine has at least one expression
    if (ExpressionEngine.numExpressions() > 0) {

        // Not already connected?
        if (!onRelabledObjectConnection.connected())
            onRelabledObjectConnection = getDocument()->signalRelabelObject.connect(boost::bind(&PropertyExpressionEngine::slotObjectRenamed, &ExpressionEngine, _1));

        // Connect to signalDeletedObject, to properly track deletion of other objects that might be referenced in an expression
        if (!onDeletedObjectConnection.connected())
            onDeletedObjectConnection = getDocument()->signalDeletedObject.connect(boost::bind(&PropertyExpressionEngine::slotObjectDeleted, &ExpressionEngine, _1));

        try {
            // Crude method to resolve all expression dependencies
            ExpressionEngine.execute();
        }
        catch (...) {
            // Ignore any error
        }
    }
    else {
        // Disconnect signals; nothing to track now
        onRelabledObjectConnection.disconnect();
        onRelabledDocumentConnection.disconnect();
        onDeletedObjectConnection.disconnect();
    }
}

void DocumentObject::onSettingDocument()
{
    //call all extensions
    auto vector = getExtensionsDerivedFromType<App::DocumentObjectExtension>();
    for(auto ext : vector)
        ext->onExtendedSettingDocument();
}

void DocumentObject::setupObject()
{
    //call all extensions
    auto vector = getExtensionsDerivedFromType<App::DocumentObjectExtension>();
    for(auto ext : vector)
        ext->onExtendedSetupObject();
}

void DocumentObject::unsetupObject()
{
    //call all extensions
    auto vector = getExtensionsDerivedFromType<App::DocumentObjectExtension>();
    for(auto ext : vector)
        ext->onExtendedUnsetupObject();
}

#if USE_OLD_DAG
#else
void App::DocumentObject::_removeBackLink(DocumentObject* rmfObj)
{
       _inList.erase(std::remove(_inList.begin(), _inList.end(), rmfObj), _inList.end());
}

void App::DocumentObject::_addBackLink(DocumentObject* newObje)
{
    if ( std::find(_inList.begin(), _inList.end(), newObje) == _inList.end() )
       _inList.push_back(newObje);
}
#endif //USE_OLD_DAG