
// This file is generated by src/Tools/generateTemaplates/templateClassPyExport.py out of the XML file
// Every change you make here get lost at the next full rebuild!
#ifndef BASE_PERSISTENCEPY_H
#define BASE_PERSISTENCEPY_H

#include <Base/BaseClassPy.h>
#include <Base/Persistence.h>
#include <string>

namespace Base
{

//===========================================================================
// PersistencePy - Python wrapper
//===========================================================================

/** The python export class for Persistence
 */
class BaseExport PersistencePy : public Base::BaseClassPy
{
public:
    static PyTypeObject   Type;
    static PyMethodDef    Methods[];
    static PyGetSetDef    GetterSetter[];
    virtual PyTypeObject *GetType(void) {return &Type;};

public:
    PersistencePy(Persistence *pcObject, PyTypeObject *T = &Type);
    static PyObject *PyMake(struct _typeobject *, PyObject *, PyObject *);
    virtual int PyInit(PyObject* args, PyObject*k);
    ~PersistencePy();

    typedef Persistence* PointerType ;

    virtual PyObject *_repr(void);        // the representation
    std::string representation(void) const;

    /** @name callbacks and implementers for the python object methods */
    //@{
    //@}


    /** @name callbacks and implementers for the python object attributes */
    //@{
    ///getter callback for the Content attribute
    static PyObject * staticCallback_getContent (PyObject *self, void *closure);
    /// getter for the Content attribute
    Py::String getContent(void) const;
    /// setter callback for the Content attribute
    static int staticCallback_setContent (PyObject *self, PyObject *value, void *closure);
    /// setter for the Content attribute
    void setContent(Py::String arg);
    ///getter callback for the MemSize attribute
    static PyObject * staticCallback_getMemSize (PyObject *self, void *closure);
    /// getter for the MemSize attribute
    Py::Int getMemSize(void) const;
    /// setter callback for the MemSize attribute
    static int staticCallback_setMemSize (PyObject *self, PyObject *value, void *closure);
    // no setter method,  MemSize is read only!
    //@}

    /// getter method for special attributes (e.g. dynamic ones)
    PyObject *getCustomAttributes(const char* attr) const;
    /// setter for special attributes (e.g. dynamic ones)
    /// Output: Success=1, Failure=-1, Ignore=0
    int setCustomAttributes(const char* attr, PyObject *obj);
    PyObject *_getattr(char *attr);              // __getattr__ function
    int _setattr(char *attr, PyObject *value);        // __setattr__ function

    /// getter for the object handled by this class
    Persistence *getPersistencePtr(void) const;

    /** @name additional declarations and methods for the wrapper class */
    //@{

    //@}
};

}  //namespace Base

#endif  // BASE_PERSISTENCEPY_H


