
// This file is generated by src/Tools/generateTemaplates/templateClassPyExport.py out of the XML file
// Every change you make here get lost at the next full rebuild!
#ifndef GUI_VIEWPROVIDERDOCUMENTOBJECTPY_H
#define GUI_VIEWPROVIDERDOCUMENTOBJECTPY_H

#include <Gui/ViewProviderPy.h>
#include <Gui/ViewProviderDocumentObject.h>
#include <string>

namespace Gui
{

//===========================================================================
// ViewProviderDocumentObjectPy - Python wrapper
//===========================================================================

/** The python export class for ViewProviderDocumentObject
 */
class GuiExport ViewProviderDocumentObjectPy : public Gui::ViewProviderPy
{
public:
    static PyTypeObject   Type;
    static PyMethodDef    Methods[];
    static PyGetSetDef    GetterSetter[];
    virtual PyTypeObject *GetType(void) {return &Type;};

public:
    ViewProviderDocumentObjectPy(ViewProviderDocumentObject *pcObject, PyTypeObject *T = &Type);
    static PyObject *PyMake(struct _typeobject *, PyObject *, PyObject *);
    virtual int PyInit(PyObject* args, PyObject*k);
    ~ViewProviderDocumentObjectPy();

    typedef ViewProviderDocumentObject* PointerType ;

    virtual PyObject *_repr(void);        // the representation
    std::string representation(void) const;

    /** @name callbacks and implementers for the python object methods */
    //@{
    /// callback for the update() method
    static PyObject * staticCallback_update (PyObject *self, PyObject *args);
    /// implementer for the update() method
    PyObject*  update(PyObject *args);
    //@}


    /** @name callbacks and implementers for the python object attributes */
    //@{
    ///getter callback for the Object attribute
    static PyObject * staticCallback_getObject (PyObject *self, void *closure);
    /// getter for the Object attribute
    Py::Object getObject(void) const;
    /// setter callback for the Object attribute
    static int staticCallback_setObject (PyObject *self, PyObject *value, void *closure);
    // no setter method,  Object is read only!
    //@}

    /// getter method for special attributes (e.g. dynamic ones)
    PyObject *getCustomAttributes(const char* attr) const;
    /// setter for special attributes (e.g. dynamic ones)
    /// Output: Success=1, Failure=-1, Ignore=0
    int setCustomAttributes(const char* attr, PyObject *obj);
    PyObject *_getattr(char *attr);              // __getattr__ function
    int _setattr(char *attr, PyObject *value);        // __setattr__ function

    /// getter for the object handled by this class
    ViewProviderDocumentObject *getViewProviderDocumentObjectPtr(void) const;

    /** @name additional declarations and methods for the wrapper class */
    //@{

    //@}
};

}  //namespace Gui

#endif  // GUI_VIEWPROVIDERDOCUMENTOBJECTPY_H


