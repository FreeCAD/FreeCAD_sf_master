
// This file is generated by src/Tools/generateTemaplates/templateClassPyExport.py out of the XML file
// Every change you make here get lost at the next full rebuild!
#ifndef PATH_COMMANDPY_H
#define PATH_COMMANDPY_H

#include <Base/PersistencePy.h>
#include <Mod/Path/App/Command.h>
#include <string>

namespace Path
{

//===========================================================================
// CommandPy - Python wrapper
//===========================================================================

/** The python export class for Command
 */
class PathExport CommandPy : public Base::PersistencePy
{
public:
    static PyTypeObject   Type;
    static PyMethodDef    Methods[];
    static PyGetSetDef    GetterSetter[];
    virtual PyTypeObject *GetType(void) {return &Type;};

public:
    CommandPy(Command *pcObject, PyTypeObject *T = &Type);
    static PyObject *PyMake(struct _typeobject *, PyObject *, PyObject *);
    virtual int PyInit(PyObject* args, PyObject*k);
    ~CommandPy();

    typedef Command* PointerType ;

    virtual PyObject *_repr(void);        // the representation
    std::string representation(void) const;

    /** @name callbacks and implementers for the python object methods */
    //@{
    /// callback for the toGCode() method
    static PyObject * staticCallback_toGCode (PyObject *self, PyObject *args);
    /// implementer for the toGCode() method
    PyObject*  toGCode(PyObject *args);
    /// callback for the setFromGCode() method
    static PyObject * staticCallback_setFromGCode (PyObject *self, PyObject *args);
    /// implementer for the setFromGCode() method
    PyObject*  setFromGCode(PyObject *args);
    /// callback for the transform() method
    static PyObject * staticCallback_transform (PyObject *self, PyObject *args);
    /// implementer for the transform() method
    PyObject*  transform(PyObject *args);
    //@}


    /** @name callbacks and implementers for the python object attributes */
    //@{
    ///getter callback for the Name attribute
    static PyObject * staticCallback_getName (PyObject *self, void *closure);
    /// getter for the Name attribute
    Py::String getName(void) const;
    /// setter callback for the Name attribute
    static int staticCallback_setName (PyObject *self, PyObject *value, void *closure);
    /// setter for the Name attribute
    void setName(Py::String arg);
    ///getter callback for the Parameters attribute
    static PyObject * staticCallback_getParameters (PyObject *self, void *closure);
    /// getter for the Parameters attribute
    Py::Dict getParameters(void) const;
    /// setter callback for the Parameters attribute
    static int staticCallback_setParameters (PyObject *self, PyObject *value, void *closure);
    /// setter for the Parameters attribute
    void setParameters(Py::Dict arg);
    ///getter callback for the Placement attribute
    static PyObject * staticCallback_getPlacement (PyObject *self, void *closure);
    /// getter for the Placement attribute
    Py::Object getPlacement(void) const;
    /// setter callback for the Placement attribute
    static int staticCallback_setPlacement (PyObject *self, PyObject *value, void *closure);
    /// setter for the Placement attribute
    void setPlacement(Py::Object arg);
    //@}

    /// getter method for special attributes (e.g. dynamic ones)
    PyObject *getCustomAttributes(const char* attr) const;
    /// setter for special attributes (e.g. dynamic ones)
    /// Output: Success=1, Failure=-1, Ignore=0
    int setCustomAttributes(const char* attr, PyObject *obj);
    PyObject *_getattr(char *attr);              // __getattr__ function
    int _setattr(char *attr, PyObject *value);        // __setattr__ function

    /// getter for the object handled by this class
    Command *getCommandPtr(void) const;

    /** @name additional declarations and methods for the wrapper class */
    //@{

    //@}
};

}  //namespace Path

#endif  // PATH_COMMANDPY_H


