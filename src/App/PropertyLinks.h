/***************************************************************************
 *   Copyright (c) Jürgen Riegel          (juergen.riegel@web.de) 2002     *
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


#ifndef APP_PROPERTYLINKS_H
#define APP_PROPERTYLINKS_H

// Std. configurations


#include <vector>
#include <map>
#include <list>
#include <string>
#include <memory>
#include <cinttypes>
#include "Property.h"

namespace Base {
class Writer;
}

namespace App
{
class DocumentObject;
class Document;

/**
 * @brief Defines different scopes for which a link can be valid
 * The scopes defined in this enum describe the different possibilities of where a link can point to.
 * Local:    links are valid only within the same GeoFeatureGroup as the linkowner is in or in none. 
 * Child:    links are valid within the same or any sub GeoFeatureGroup
 * Global:   all possible links are valid
 * Hidden:   links are not included in dependency calculation
 */
enum class LinkScope {
    Local,
    Child,
    Global,
    Hidden,
};

/**
 * @brief Enables scope handling for links
 * This class is a base for all link properties and enables them to handle scopes of the linked objects.
 * The possible scopes are defined by LinkScope enum class. The default value is Local. 
 * The scope of a property is not saved in the document. It is a value that needs to be fixed when
 * the object holding the property is loaded. That is possible with two methods: 
 * 1. Set the scope value in the constructor of the link property
 * 2. Use setScope to change the scope in the constructor of the link property
 * 
 * The second option is only available in c++, not in python, as setscope is not exposed. It would 
 * not make sense to expose it there, as restoring python objects does not call the constructor again.
 * Hence in python the only way to create a LinkProperty with different scope than local is to use a 
 * specialized property for that. In c++ existing properties can simply be changed via setScope in the 
 * objects constructor. 
 */
class AppExport ScopedLink {
  
public:
    /**
     * @brief Set the links scope
     * Allows to define what kind of links are allowed. Only in the Local GeoFeatureGroup, in this and 
     * all Childs or to all objects within the Glocal scope.
     */
    void setScope(LinkScope scope) {_pcScope = scope;};    
    /**
     * @brief Get the links scope
     * Retrieve what kind of links are allowed. Only in the Local GeoFeatureGroup, in this and 
     * all Childs or to all objects within the Glocal scope.
     */
    LinkScope getScope() {return _pcScope;};
    
protected:
    LinkScope _pcScope = LinkScope::Local;
};

/// Parent class of all link type properties
class AppExport PropertyLinkBase : public Property, public ScopedLink
{
    TYPESYSTEM_HEADER();
public:
    typedef std::pair<std::string,std::string> ShadowSub;

    PropertyLinkBase();
    virtual ~PropertyLinkBase();

    /** Link type property interface APIs
     * These APIs are moved here so that any type of property can have the
     * property link behavior, e.g. the PropertyExpressionEngine
     */
    //@{

    /** Called to update the element reference of this link property
     *
     * @sa _updateElementReference()
     */
    virtual void updateElementReference(App::DocumentObject *feature,bool reverse=false) {
        (void)feature;
        (void)reverse;
    }

    /// Clear internal element reference registration
    void unregisterElementReference();

    /** Register label reference for future object relabel update
     *
     *  @param subs: subname reference to check for label references
     *  @param reset: if ture, then calls unregisterLabelReference() before
     *  registering new references
     */
    void registerLabelReferences(const std::vector<std::string> &subs, bool reset=true);

    /// Clear internal label references registration
    void unregisterLabelReferences();

    /// Test if the element reference has changed after restore
    virtual bool referenceChanged() const {
        return false;
    }

    /** Obtain the linked objects
     *
     * @param objs: hold the returned linked objects on output
     * @param all: if true, then return all the linked object regardless of
     *             this LinkScope. If false, then return only if the LinkScope
     *             is not hidden.
     * @param sub: if given, then return subname references.
     * @param newStyle: whether to return new or old style subname reference 
     */
    virtual void getLinks(std::vector<App::DocumentObject *> &objs, 
            bool all=false, std::vector<std::string> *subs=0, bool newStyle=true) const = 0;

    /** Called to reset this link property
     *
     * @param obj: reset link property if it is linked to this object
     * @param clear: if true, then also reset property if the owner of this proeprty is \a obj
     *
     * @sa breakLinks()
     */
    virtual void breakLink(App::DocumentObject *obj, bool clear) = 0;

    /** Called to adjust the link to avoid potential cyclic dependency
     *
     * @param inList: recursive in-list of the would-be parent
     *
     * @return Return whether the link has been adjusted
     *
     * This function tries to correct the link to avoid any (sub)object inside
     * in-list. If the adjustment is impossible, exception will be raised
     */
    virtual bool adjustLink(const std::set<App::DocumentObject *> &inList) = 0;

    /** Return a copy of the property if any changes caused by importing external linked object 
     *
     * @param nameMap: a map from the original external object name to the
     * imported new object name
     *
     * @return Returns a copy of the property with the updated link reference if
     * affected. The copy will later be assgiend to this property by calling its
     * Paste().
     */
    virtual Property *CopyOnImportExternal(const std::map<std::string,std::string> &nameMap) const {
        (void)nameMap;
        return 0;
    }

    /** Update object label reference in this property
     *
     * @param obj: the object owner of the changing label
     * @param ref: subname reference to old label
     * @param newLabel: the future new label
     *
     * @return Returns a copy of the property if its link reference is affected.
     * The copy will later be assgiend to this property by calling its Paste().
     */
    virtual Property *CopyOnLabelChange(App::DocumentObject *obj, 
                        const std::string &ref, const char *newLabel) const
    {
        (void)obj;
        (void)ref;
        (void)newLabel;
        return 0;
    }

    /// Helper function to return all linked objects of this property
    std::vector<App::DocumentObject *> linkedObjects(bool all=false) const {
        std::vector<App::DocumentObject*> ret;
        getLinks(ret,all);
        return ret;
    }

    /// Helper function to return linked objects using an std::inserter
    template<class T>
    void getLinkedObjects(T &inserter, bool all=false) const {
        std::vector<App::DocumentObject*> ret;
        getLinks(ret,all);
        std::copy(ret.begin(),ret.end(),inserter);
    }

    /// Helper function to return a map of linked object and its subname references
    void getLinkedElements(std::map<App::DocumentObject*, std::vector<std::string> > &elements, 
            bool newStyle=true, bool all=true) const 
    {
        std::vector<App::DocumentObject*> ret;
        std::vector<std::string> subs;
        getLinks(ret,all,&subs,newStyle);
        assert(ret.size()==subs.size());
        int i=0;
        for(auto obj : ret)
            elements[obj].push_back(subs[i++]);
    }

    /// Helper function to return a map of linked object and its subname references
    std::map<App::DocumentObject*, std::vector<std::string> > 
        linkedElements(bool newStyle=true, bool all=true) const 
    {
        std::map<App::DocumentObject*, std::vector<std::string> > ret;
        getLinkedElements(ret,newStyle,all);
        return ret;
    }
    //@}

    void setAllowExternalLink(bool allow) { _allowExternal = allow; }
    bool allowExternalLink() const {return _allowExternal;}

    //@{

    /// Update all element references in all link properties of \a feature
    static void updateElementReferences(DocumentObject *feature, bool reverse=false);


    /** Helper function for update individual element reference
     *
     * @param owner: the owner property of the subname referece
     * @param feature: if given, than only update element reference belonging
     *                 to this feature. If not, then update geometry element
     *                 references.
     * @param sub: the subname reference to be updated.
     * @param shadow: a pair of new and old style element references to be updated.
     * @param reverse: if true, then use the old style, i.e. non-mapped element
     *                 reference to query for the new style, i.e. mapped
     *                 element reference when update. If false, then the other
     *                 way around.
     *
     * This helper function is to be called by each link property in the event of
     * geometry element reference change due to geometry model changes.
     */
    static bool _updateElementReference(PropertyLinkBase *owner, App::DocumentObject *feature,
        App::DocumentObject *obj, std::string &sub, ShadowSub &shadow, bool reverse);

    /** Helper function to register geometry element reference
     * 
     * @param owner: the link property to be registered
     * @param obj: the linked object
     * @param sub: the subname reference
     * @param shadow: a pair of new and old style element references to be updated.
     *
     * Search for any geometry element reference inside the subname, and
     * register for future update in case of geometry model update.
     */
    static void _registerElementReference(PropertyLinkBase *owner, 
                    App::DocumentObject *obj, std::string &sub, ShadowSub &shadow);

    /** Helper function for breaking link properties
     *
     * @param link: reset link property if it is linked to this object
     * @param objs: the objects to check for the link properties
     * @param clear: if ture, then also reset property if the owner of the link property is \a link
     *
     * App::Document::breakDependency() calls this function to break the link property
     */
    static void breakLinks(App::DocumentObject *link, const std::vector<App::DocumentObject*> &objs, bool clear);

    static std::string tryImportSubName(const std::map<std::string,std::string> &nameMap, 
                                        const App::DocumentObject *obj, const char *sub);
    static std::string exportSubName(const App::DocumentObject *obj, const char *sub);
    static std::string importSubName(Base::XMLReader &reader, const char *sub);

    static void getLabelReferences(std::vector<std::string> &labels, const char *subname);

    static std::vector<std::pair<Property*, std::unique_ptr<Property> > > updateLabelReferences(
            App::DocumentObject *obj, const char *newLabel);

    static std::string updateLabelReference(App::DocumentObject *obj, const std::string &ref, 
            const char *newLabel, App::DocumentObject *parent, const char *subname);
    //@}

protected:
    bool _allowExternal = false;

private:
    std::set<std::string> _LabelRefs;
    std::set<App::DocumentObject*> _ElementRefs;
};

/** The general Link Property
 *  Main Purpose of this property is to Link Objects and Features in a document. Like all links this 
 *  property is scope aware, meaning it does define which objects are allowed to be linked depending 
 *  of the GeoFeatureGroup where it is in. Default is Local.
 * 
 *  @note Links that are invalid in respect to the scope of this property is set to are not rejected. 
 *        They are only detected to be invalid and prevent the feature from recomputing.
 */
class AppExport PropertyLink : public PropertyLinkBase
{
    TYPESYSTEM_HEADER();

public:
    /**
     * A constructor.
     * A more elaborate description of the constructor.
     */
    PropertyLink();

    /**
     * A destructor.
     * A more elaborate description of the destructor.
     */
    virtual ~PropertyLink();

    void resetLink();

    /** Sets the property
     */
    virtual void setValue(App::DocumentObject *);

    /** This method returns the linked DocumentObject
     */
    App::DocumentObject * getValue(void) const;

    /** Returns the link type checked
     */
    App::DocumentObject * getValue(Base::Type t) const;

   /** Returns the link type checked
     */
    template <typename _type>
    inline _type getValue(void) const {
        return _pcLink ? dynamic_cast<_type>(_pcLink) : 0;
    }

    virtual PyObject *getPyObject(void);
    virtual void setPyObject(PyObject *);

    virtual void Save (Base::Writer &writer) const;
    virtual void Restore(Base::XMLReader &reader);

    virtual Property *Copy(void) const;
    virtual void Paste(const Property &from);

    virtual unsigned int getMemSize (void) const{
        return sizeof(App::DocumentObject *);
    }
    virtual const char* getEditorName(void) const
    { return "Gui::PropertyEditor::PropertyLinkItem"; }

    virtual void getLinks(std::vector<App::DocumentObject *> &objs, 
            bool all=false, std::vector<std::string> *subs=0, bool newStyle=true) const override;

    virtual void breakLink(App::DocumentObject *obj, bool clear) override;

    virtual bool adjustLink(const std::set<App::DocumentObject *> &inList) override;

protected:
    App::DocumentObject *_pcLink;
};

/** The general Link Property with Child scope
 */
class AppExport PropertyLinkChild : public PropertyLink
{
    TYPESYSTEM_HEADER();
public:
    PropertyLinkChild() {_pcScope = LinkScope::Child;};
};

/** The general Link Property with Global scope
 */
class AppExport PropertyLinkGlobal : public PropertyLink
{
    TYPESYSTEM_HEADER();
public:
    PropertyLinkGlobal() {_pcScope = LinkScope::Global;};
};

/** The general Link Property that are hidden from dependency checking
 */
class AppExport PropertyLinkHidden : public PropertyLink
{
    TYPESYSTEM_HEADER();
public:
    PropertyLinkHidden() {_pcScope = LinkScope::Hidden;};
};


class AppExport PropertyLinkListBase: public PropertyLinkBase, public PropertyListsBase
{
    TYPESYSTEM_HEADER();
public:
    virtual void setPyObject(PyObject *obj) override {
        _setPyObject(obj);
    }
};

class AppExport PropertyLinkList : 
    public PropertyListsT<DocumentObject*,std::vector<DocumentObject*>, PropertyLinkListBase>
{
    TYPESYSTEM_HEADER();
    typedef PropertyListsT<DocumentObject*,std::vector<DocumentObject*>,PropertyLinkListBase> inherited;

public:
    /**
    * A constructor.
    * A more elaborate description of the constructor.
    */
    PropertyLinkList();

    /**
    * A destructor.
    * A more elaborate description of the destructor.
    */
    virtual ~PropertyLinkList();

    virtual void setSize(int newSize);
    virtual void setSize(int newSize, const_reference def);

    /** Sets the property
    */
    void setValues(const std::vector<DocumentObject*>&) override;

    void set1Value(int idx, DocumentObject * const &value, bool touch=false) override;

    virtual PyObject *getPyObject(void);

    virtual void Save(Base::Writer &writer) const;
    virtual void Restore(Base::XMLReader &reader);

    virtual Property *Copy(void) const;
    virtual void Paste(const Property &from);

    virtual unsigned int getMemSize(void) const;
    virtual const char* getEditorName(void) const
    { return "Gui::PropertyEditor::PropertyLinkListItem"; }

    virtual void getLinks(std::vector<App::DocumentObject *> &objs, 
            bool all=false, std::vector<std::string> *subs=0, bool newStyle=true) const override;

    virtual void breakLink(App::DocumentObject *obj, bool clear) override;

    virtual bool adjustLink(const std::set<App::DocumentObject *> &inList) override;

    DocumentObject *find(const char *, int *pindex=0) const;

protected:
    DocumentObject *getPyValue(PyObject *item) const override;

protected:
    mutable std::map<std::string, int> _nameMap;
};

/** The general Link Property with Child scope
 */
class AppExport PropertyLinkListChild : public PropertyLinkList
{
    TYPESYSTEM_HEADER();
public:
    PropertyLinkListChild() {_pcScope = LinkScope::Child;};
};

/** The general Link Property with Global scope
 */
class AppExport PropertyLinkListGlobal : public PropertyLinkList
{
    TYPESYSTEM_HEADER();
public:
    PropertyLinkListGlobal() {_pcScope = LinkScope::Global;};
};

/** The general Link Property that are hidden from dependency checking
 */
class AppExport PropertyLinkListHidden : public PropertyLinkList
{
    TYPESYSTEM_HEADER();
public:
    PropertyLinkListHidden() {_pcScope = LinkScope::Hidden;};
};

class PropertyXLinkSub;

/** the Link Property with sub elements
 *  This property links an object and a defined sequence of
 *  sub elements. These subelements (like Edges of a Shape)
 *  are stored as names, which can be resolved by the 
 *  ComplexGeoDataType interface to concrete sub objects.
 */
class AppExport PropertyLinkSub: public PropertyLinkBase
{
    TYPESYSTEM_HEADER();

public:
    /**
     * A constructor.
     * A more elaborate description of the constructor.
     */
    PropertyLinkSub();

    /**
     * A destructor.
     * A more elaborate description of the destructor.
     */
    virtual ~PropertyLinkSub();

    virtual void afterRestore() override;

    /** Sets the property
     */
    void setValue(App::DocumentObject *,const std::vector<std::string> &SubList,
                    std::vector<ShadowSub> &&ShadowSubList={});
    void setValue(App::DocumentObject *,std::vector<std::string> &&SubList={},
                    std::vector<ShadowSub> &&ShadowSubList={});

    /** This method returns the linked DocumentObject
     */
    App::DocumentObject * getValue(void) const;

    /// return the list of sub elements 
    const std::vector<std::string>& getSubValues(void) const;

    /// return the list of sub elements with mapped names
    const std::vector<ShadowSub> &getShadowSubs() const {
        return _ShadowSubList;
    }

    std::vector<std::string> getSubValues(bool newStyle) const;

    /// return the list of sub elements starts with a special string 
    std::vector<std::string> getSubValuesStartsWith(const char*, bool newStyle=false) const;

    /** Returns the link type checked
     */
    App::DocumentObject * getValue(Base::Type t) const;

   /** Returns the link type checked
     */
    template <typename _type>
    inline _type getValue(void) const {
        return _pcLinkSub ? dynamic_cast<_type>(_pcLinkSub) : 0;
    }

    virtual PyObject *getPyObject(void);
    virtual void setPyObject(PyObject *);

    virtual void Save (Base::Writer &writer) const;
    virtual void Restore(Base::XMLReader &reader);

    virtual Property *Copy(void) const;
    virtual void Paste(const Property &from);

    /// Return a copy of the property if any changes caused by importing external object 
    virtual Property *CopyOnImportExternal(const std::map<std::string,std::string> &nameMap) const override;

    virtual Property *CopyOnLabelChange(App::DocumentObject *obj, 
            const std::string &ref, const char *newLabel) const override;

    virtual unsigned int getMemSize (void) const{
        return sizeof(App::DocumentObject *);
    }

    virtual void updateElementReference(DocumentObject *feature,bool reverse=false) override;

    virtual bool referenceChanged() const override;

    virtual void getLinks(std::vector<App::DocumentObject *> &objs, 
            bool all=false, std::vector<std::string> *subs=0, bool newStyle=true) const override;

    virtual void breakLink(App::DocumentObject *obj, bool clear) override;

    virtual bool adjustLink(const std::set<App::DocumentObject *> &inList) override;

protected:
    App::DocumentObject*     _pcLinkSub;
    std::vector<std::string> _cSubList;
    std::vector<ShadowSub> _ShadowSubList;
    std::vector<int> _mapped;
};

/** The general Link Property with Child scope
 */
class AppExport PropertyLinkSubChild : public PropertyLinkSub
{
    TYPESYSTEM_HEADER();
public:
    PropertyLinkSubChild() {_pcScope = LinkScope::Child;};
};

/** The general Link Property with Global scope
 */
class AppExport PropertyLinkSubGlobal : public PropertyLinkSub
{
    TYPESYSTEM_HEADER();
public:
    PropertyLinkSubGlobal() {_pcScope = LinkScope::Global;};
};

/** The general Link Property that are hidden from dependency checking
 */
class AppExport PropertyLinkSubHidden : public PropertyLinkSub
{
    TYPESYSTEM_HEADER();
public:
    PropertyLinkSubHidden() {_pcScope = LinkScope::Hidden;};
};

class AppExport PropertyLinkSubList: public PropertyLinkBase
{
    TYPESYSTEM_HEADER();

public:
    typedef std::pair<DocumentObject*, std::vector<std::string> > SubSet;
    /**
     * A constructor.
     * A more elaborate description of the constructor.
     */
    PropertyLinkSubList();

    /**
     * A destructor.
     * A more elaborate description of the destructor.
     */
    virtual ~PropertyLinkSubList();

    virtual void afterRestore() override;

    int getSize(void) const;
    void setSize(int newSize);

    /** Sets the property.
     * setValue(0, whatever) clears the property
     */
    void setValue(DocumentObject*,const char*);
    void setValues(const std::vector<DocumentObject*>&,const std::vector<const char*>&);
    void setValues(const std::vector<DocumentObject*>&,const std::vector<std::string>&,
                    std::vector<ShadowSub> &&ShadowSubList={});
    void setValues(std::vector<DocumentObject*>&&, std::vector<std::string> &&subs,
                    std::vector<ShadowSub> &&ShadowSubList={});

    /**
     * @brief setValue: PropertyLinkSub-compatible overload
     * @param SubList
     */
    void setValue(App::DocumentObject *lValue, const std::vector<std::string> &SubList=std::vector<std::string>());

    const std::vector<DocumentObject*> &getValues(void) const {
        return _lValueList;
    }

    const std::string getPyReprString() const;

    /**
     * @brief getValue emulates the action of a single-object link.
     * @return reference to object, if the link is to only one object. NULL if
     * the link is empty, or links to subelements of more than one document
     * object.
     */
    DocumentObject* getValue() const;

    const std::vector<std::string> &getSubValues(void) const {
        return _lSubList;
    }

    std::vector<std::string> getSubValues(bool newStyle) const;

    const std::vector<ShadowSub> &getShadowSubs() const {
        return _ShadowSubList;
    }

    /**
     * @brief Removes all occurrences of \a lValue in the property
     * together with its sub-elements and returns the number of entries removed.
     */
    int removeValue(App::DocumentObject *lValue);

    void setSubListValues(const std::vector<SubSet>&);
    std::vector<SubSet> getSubListValues() const;

    virtual PyObject *getPyObject(void);
    virtual void setPyObject(PyObject *);

    virtual void Save (Base::Writer &writer) const;
    virtual void Restore(Base::XMLReader &reader);

    virtual Property *Copy(void) const;
    virtual void Paste(const Property &from);

    /// Return a copy of the property if any changes caused by importing external object 
    virtual Property *CopyOnImportExternal(const std::map<std::string,std::string> &nameMap) const override;

    virtual Property *CopyOnLabelChange(App::DocumentObject *obj, 
            const std::string &ref, const char *newLabel) const override;

    virtual unsigned int getMemSize (void) const;

    virtual void updateElementReference(DocumentObject *feature,bool reverse=false) override;

    virtual bool referenceChanged() const override;

    virtual void getLinks(std::vector<App::DocumentObject *> &objs, 
            bool all=false, std::vector<std::string> *subs=0, bool newStyle=true) const override;

    virtual void breakLink(App::DocumentObject *obj, bool clear) override;

    virtual bool adjustLink(const std::set<App::DocumentObject *> &inList) override;

private:
    //FIXME: Do not make two independent lists because this will lead to some inconsistencies!
    std::vector<DocumentObject*> _lValueList;
    std::vector<std::string>     _lSubList;
    std::vector<ShadowSub> _ShadowSubList;
    std::vector<int> _mapped;
};

/** The general Link Property with Child scope
 */
class AppExport PropertyLinkSubListChild : public PropertyLinkSubList
{
    TYPESYSTEM_HEADER();
public:
    PropertyLinkSubListChild() {_pcScope = LinkScope::Child;};
};

/** The general Link Property with Global scope
 */
class AppExport PropertyLinkSubListGlobal : public PropertyLinkSubList
{
    TYPESYSTEM_HEADER();
public:
    PropertyLinkSubListGlobal() {_pcScope = LinkScope::Global;};
};

/** The general Link Property that are hidden from dependency checking
 */
class AppExport PropertyLinkSubListHidden : public PropertyLinkSubList
{
    TYPESYSTEM_HEADER();
public:
    PropertyLinkSubListHidden() {_pcScope = LinkScope::Hidden;};
};

class PropertyXLinkSubList;

/** Link to an (sub)object in the same or different document
 */
class AppExport PropertyXLink : public PropertyLinkGlobal
{
    TYPESYSTEM_HEADER();

public:
    PropertyXLink(bool allowPartial=false, Property *parent=0);

    virtual ~PropertyXLink();

    virtual void afterRestore() override;

    void setValue(App::DocumentObject *) override;
    void setValue(App::DocumentObject *, const char *subname);

    const char *getSubName(bool newStyle=true) const;
    void setSubName(const char *subname);
    
    bool hasSubName() const {return !_SubList.empty();}

    App::Document *getDocument() const;
    const char *getDocumentPath() const;
    const char *getObjectName() const;

    static int checkRestore(const App::Property *prop);
    int checkRestore() const;

    virtual void Save (Base::Writer &writer) const;
    virtual void Restore(Base::XMLReader &reader);

    virtual Property *Copy(void) const;
    virtual void Paste(const Property &from);

    /// Return a copy of the property if any changes caused by importing external object 
    virtual Property *CopyOnImportExternal(const std::map<std::string,std::string> &nameMap) const override;

    virtual Property *CopyOnLabelChange(App::DocumentObject *obj, 
            const std::string &ref, const char *newLabel) const override;

    virtual PyObject *getPyObject(void);
    virtual void setPyObject(PyObject *);

    class DocInfo;
    friend class DocInfo;
    typedef std::shared_ptr<DocInfo> DocInfoPtr;

    static bool hasXLink(const App::Document *doc);
    static bool hasXLink(const std::vector<App::DocumentObject*> &objs, std::vector<App::Document*> *unsaved=0);
    static std::map<App::Document*,std::set<App::Document*> > getDocumentOutList(App::Document *doc=0);
    static std::map<App::Document*,std::set<App::Document*> > getDocumentInList(App::Document *doc=0);

    virtual void updateElementReference(DocumentObject *feature,bool reverse=false) override;
    virtual bool referenceChanged() const override;

    virtual void getLinks(std::vector<App::DocumentObject *> &objs, 
            bool all=false, std::vector<std::string> *subs=0, bool newStyle=true) const override;

    virtual bool adjustLink(const std::set<App::DocumentObject *> &inList) override;

    // The following APIs are provided to be compatible with PropertyLinkSub.
    // Note that although PropertyXLink is capable of holding multiple subnames,
    // there no public APIs allowing user to set more that one subname. Multiple
    // subname adding API is published in PropertyXLinkSub.

    const std::vector<std::string>& getSubValues(void) const {
        return _SubList;
    }
    const std::vector<ShadowSub > &getShadowSubs() const {
        return _ShadowSubList;
    }
    std::vector<std::string> getSubValues(bool newStyle) const;
    std::vector<std::string> getSubValuesStartsWith(const char*, bool newStyle=false) const;

    void setAllowPartial(bool enable);
    bool allowPartial() const { return _allowPartial; }

protected:
    void unlink();
    void detach();

    void restoreLink(App::DocumentObject *);

    void _setSubValues(std::vector<std::string> &&SubList,
            std::vector<ShadowSub> &&ShadowSubList = {});

    void _setValue(std::string &&filePath, std::string &&objectName, std::vector<std::string> &&SubList,
            std::vector<ShadowSub> &&ShadowSubList = {});

    void _setValue(App::DocumentObject *,std::vector<std::string> &&SubList,
            std::vector<ShadowSub> &&ShadowSubList = {});

    virtual PropertyXLink *createInstance() const;

    virtual bool upgrade(Base::XMLReader &reader, const char *typeName);

    void copyTo(PropertyXLink &other) const;

    virtual void aboutToSetValue() override;

    virtual void hasSetValue() override;

    friend class PropertyXLinkSubList;

protected:
    DocInfoPtr docInfo;
    std::string filePath;
    std::string objectName;
    std::string stamp;
    std::vector<std::string> _SubList;
    std::vector<ShadowSub> _ShadowSubList;
    std::vector<int> _mapped;
    Property *parentProp;
    bool _allowPartial;
};


class AppExport PropertyXLinkSub: public PropertyXLink {
    TYPESYSTEM_HEADER();

public:
    PropertyXLinkSub(bool allowPartial=false, Property *parent=0);

    virtual ~PropertyXLinkSub();

    void setValue(App::DocumentObject *,const std::vector<std::string> &SubList, 
            std::vector<ShadowSub > &&ShadowSubList={});

    void setValue(App::DocumentObject *,std::vector<std::string> &&SubList={},
            std::vector<ShadowSub > &&ShadowSubList={});

    void setSubValues(std::vector<std::string> &&SubList,
            std::vector<ShadowSub> &&ShadowSubList={});

    virtual bool upgrade(Base::XMLReader &reader, const char *typeName) override;

    virtual PyObject *getPyObject(void);
    virtual void setPyObject(PyObject *);

protected:
    virtual PropertyXLink *createInstance() const;
};


class AppExport PropertyXLinkSubList: public PropertyLinkBase {
    TYPESYSTEM_HEADER();

public:
    PropertyXLinkSubList();
    virtual ~PropertyXLinkSubList();

    virtual void afterRestore() override;

    int getSize(void) const;

    /** Sets the property.
     * setValue(0, whatever) clears the property
     */
    void setValue(DocumentObject*,const char*);
    void setValues(const std::vector<DocumentObject*>&,const std::vector<const char*>&);
    void setValues(const std::vector<DocumentObject*>&,const std::vector<std::string>&);
    void setValues(std::map<App::DocumentObject*,std::vector<std::string> > &&);
    void setValues(const std::map<App::DocumentObject*,std::vector<std::string> > &);

    void addValue(App::DocumentObject *obj, const std::vector<std::string> &SubList={}, bool reset=false);
    void addValue(App::DocumentObject *obj, std::vector<std::string> &&SubList={}, bool reset=false);

    /**
     * @brief setValue: PropertyLinkSub-compatible overload
     * @param SubList
     */
    void setValue(App::DocumentObject *lValue, const std::vector<std::string> &SubList=std::vector<std::string>());

    std::vector<DocumentObject*> getValues(void);

    const std::string getPyReprString() const;

    DocumentObject* getValue() const;

    const std::vector<std::string> &getSubValues(App::DocumentObject *obj) const;

    std::vector<std::string> getSubValues(App::DocumentObject *obj, bool newStyle) const;

    const std::vector<ShadowSub> &getShadowSubs(App::DocumentObject *obj) const;

    /**
     * @brief Removes all occurrences of \a lValue in the property
     * together with its sub-elements and returns the number of entries removed.
     */
    int removeValue(App::DocumentObject *lValue);

    void setSubListValues(const std::vector<PropertyLinkSubList::SubSet>&);

    const std::list<PropertyXLinkSub> &getSubListValues() const {
        return _Links;
    }

    virtual PyObject *getPyObject(void);
    virtual void setPyObject(PyObject *);

    virtual void Save (Base::Writer &writer) const;
    virtual void Restore(Base::XMLReader &reader);

    virtual Property *Copy(void) const;
    virtual void Paste(const Property &from);

    virtual Property *CopyOnImportExternal(const std::map<std::string,std::string> &nameMap) const override;

    virtual Property *CopyOnLabelChange(App::DocumentObject *obj, 
            const std::string &ref, const char *newLabel) const override;

    virtual unsigned int getMemSize (void) const;

    virtual void updateElementReference(DocumentObject *feature,bool reverse=false) override;

    virtual bool referenceChanged() const override;

    virtual void getLinks(std::vector<App::DocumentObject *> &objs, 
            bool all=false, std::vector<std::string> *subs=0, bool newStyle=true) const override;

    virtual void breakLink(App::DocumentObject *obj, bool clear) override;

    virtual bool adjustLink(const std::set<App::DocumentObject *> &inList) override;

    bool upgrade(Base::XMLReader &reader, const char *typeName);
    int checkRestore() const;

    void setAllowPartial(bool enable);

protected:
    std::list<PropertyXLinkSub> _Links;
    bool _allowPartial;
};

} // namespace App


#endif // APP_PROPERTYLINKS_H
