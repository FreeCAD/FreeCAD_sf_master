
// This file is generated by src/Tools/generateTemaplates/templateClassPyExport.py out of the XML file
// Every change you make here get lost at the next full rebuild!
#ifndef APP_EXTENSIONCONTAINERPY_H
#define APP_EXTENSIONCONTAINERPY_H

#include <App/PropertyContainerPy.h>
#include <App/ExtensionContainer.h>
#include <string>

namespace App
{

//===========================================================================
// ExtensionContainerPy - Python wrapper
//===========================================================================

/** The python export class for ExtensionContainer
 */
class AppExport ExtensionContainerPy : public App::PropertyContainerPy
{
public:
    static PyTypeObject   Type;
    static PyMethodDef    Methods[];
    static PyGetSetDef    GetterSetter[];
    virtual PyTypeObject *GetType(void) {return &Type;}

public:
    ExtensionContainerPy(ExtensionContainer *pcObject, PyTypeObject *T = &Type);
    static PyObject *PyMake(struct _typeobject *, PyObject *, PyObject *);
    virtual int PyInit(PyObject* args, PyObject*k);
    ~ExtensionContainerPy();
    
    int initialization();
    int finalization();

    typedef ExtensionContainer* PointerType ;

    virtual PyObject *_repr(void);        // the representation
    std::string representation(void) const;

    /** @name callbacks and implementers for the python object methods */
    //@{
    /// callback for the addExtension() method
    static PyObject * staticCallback_addExtension (PyObject *self, PyObject *args);
    /// implementer for the addExtension() method
    PyObject*  addExtension(PyObject *args);
    /// callback for the hasExtension() method
    static PyObject * staticCallback_hasExtension (PyObject *self, PyObject *args);
    /// implementer for the hasExtension() method
    PyObject*  hasExtension(PyObject *args);
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
    ExtensionContainer *getExtensionContainerPtr(void) const;

    /** @name additional declarations and methods for the wrapper class */
    //@{

    //@}
};

}  //namespace App

#endif  // APP_EXTENSIONCONTAINERPY_H


