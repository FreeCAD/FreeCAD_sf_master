
// This file is generated by src/Tools/generateTemaplates/templateClassPyExport.py out of the XML file
// Every change you make here get lost at the next full rebuild!
#ifndef GUI_VIEWPROVIDERPYTHONFEATUREPY_H
#define GUI_VIEWPROVIDERPYTHONFEATUREPY_H

#include <Gui/ViewProviderDocumentObjectPy.h>
#include <Gui/ViewProviderPythonFeature.h>
#include <string>

namespace Gui
{

//===========================================================================
// ViewProviderPythonFeaturePy - Python wrapper
//===========================================================================

/** The python export class for ViewProviderPythonFeature
 */
class GuiExport ViewProviderPythonFeaturePy : public Gui::ViewProviderDocumentObjectPy
{
public:
    static PyTypeObject   Type;
    static PyMethodDef    Methods[];
    static PyGetSetDef    GetterSetter[];
    virtual PyTypeObject *GetType(void) {return &Type;}

public:
    ViewProviderPythonFeaturePy(ViewProviderDocumentObject *pcObject, PyTypeObject *T = &Type);
    static PyObject *PyMake(struct _typeobject *, PyObject *, PyObject *);
    virtual int PyInit(PyObject* args, PyObject*k);
    ~ViewProviderPythonFeaturePy();
    

    typedef ViewProviderDocumentObject* PointerType ;

    virtual PyObject *_repr(void);        // the representation
    std::string representation(void) const;

    /** @name callbacks and implementers for the python object methods */
    //@{
    /// callback for the addDisplayMode() method
    static PyObject * staticCallback_addDisplayMode (PyObject *self, PyObject *args);
    /// implementer for the addDisplayMode() method
    PyObject*  addDisplayMode(PyObject *args);
    /// callback for the addProperty() method
    static PyObject * staticCallback_addProperty (PyObject *self, PyObject *args);
    /// implementer for the addProperty() method
    PyObject*  addProperty(PyObject *args);
    /// callback for the removeProperty() method
    static PyObject * staticCallback_removeProperty (PyObject *self, PyObject *args);
    /// implementer for the removeProperty() method
    PyObject*  removeProperty(PyObject *args);
    /// callback for the supportedProperties() method
    static PyObject * staticCallback_supportedProperties (PyObject *self, PyObject *args);
    /// implementer for the supportedProperties() method
    PyObject*  supportedProperties(PyObject *args);
    //@}


    /** @name callbacks and implementers for the python object attributes */
    //@{
    //@}

    /// getter method for special attributes (e.g. dynamic ones)
    PyObject *getCustomAttributes(const char* attr) const;
    /// setter for special attributes (e.g. dynamic ones)
    /// Output: Success=1, Failure=-1, Ignore=0
    int setCustomAttributes(const char* attr, PyObject *obj);
    PyObject *_getattr(char *attr);              // __getattr__ function
    int _setattr(char *attr, PyObject *value);        // __setattr__ function

    /// getter for the object handled by this class
    ViewProviderDocumentObject *getViewProviderPythonFeaturePtr(void) const;

    /** @name additional declarations and methods for the wrapper class */
    //@{

    //@}
};

}  //namespace Gui

#endif  // GUI_VIEWPROVIDERPYTHONFEATUREPY_H


