
// This file is generated by src/Tools/generateTemaplates/templateClassPyExport.py out of the XML file
// Every change you make here get lost at the next full rebuild!
#ifndef PART_PARABOLAPY_H
#define PART_PARABOLAPY_H

#include <Mod/Part/App/ConicPy.h>
#include <Mod/Part/App/Geometry.h>
#include <string>

namespace Part
{

//===========================================================================
// ParabolaPy - Python wrapper
//===========================================================================

/** The python export class for GeomParabola
 */
class PartExport ParabolaPy : public Part::ConicPy
{
public:
    static PyTypeObject   Type;
    static PyMethodDef    Methods[];
    static PyGetSetDef    GetterSetter[];
    virtual PyTypeObject *GetType(void) {return &Type;}

public:
    ParabolaPy(GeomParabola *pcObject, PyTypeObject *T = &Type);
    static PyObject *PyMake(struct _typeobject *, PyObject *, PyObject *);
    virtual int PyInit(PyObject* args, PyObject*k);
    ~ParabolaPy();
    

    typedef GeomParabola* PointerType ;

    virtual PyObject *_repr(void);        // the representation
    std::string representation(void) const;

    /** @name callbacks and implementers for the python object methods */
    //@{
    /// callback for the compute() method
    static PyObject * staticCallback_compute (PyObject *self, PyObject *args);
    /// implementer for the compute() method
    PyObject*  compute(PyObject *args);
    //@}


    /** @name callbacks and implementers for the python object attributes */
    //@{
    ///getter callback for the Focal attribute
    static PyObject * staticCallback_getFocal (PyObject *self, void *closure);
    /// getter for the Focal attribute
    Py::Float getFocal(void) const;
    /// setter callback for the Focal attribute
    static int staticCallback_setFocal (PyObject *self, PyObject *value, void *closure);
    /// setter for the Focal attribute
    void setFocal(Py::Float arg);
    ///getter callback for the Focus attribute
    static PyObject * staticCallback_getFocus (PyObject *self, void *closure);
    /// getter for the Focus attribute
    Py::Object getFocus(void) const;
    /// setter callback for the Focus attribute
    static int staticCallback_setFocus (PyObject *self, PyObject *value, void *closure);
    // no setter method,  Focus is read only!
    ///getter callback for the Parameter attribute
    static PyObject * staticCallback_getParameter (PyObject *self, void *closure);
    /// getter for the Parameter attribute
    Py::Float getParameter(void) const;
    /// setter callback for the Parameter attribute
    static int staticCallback_setParameter (PyObject *self, PyObject *value, void *closure);
    // no setter method,  Parameter is read only!
    //@}

    /// getter method for special attributes (e.g. dynamic ones)
    PyObject *getCustomAttributes(const char* attr) const;
    /// setter for special attributes (e.g. dynamic ones)
    /// Output: Success=1, Failure=-1, Ignore=0
    int setCustomAttributes(const char* attr, PyObject *obj);
    PyObject *_getattr(char *attr);              // __getattr__ function
    int _setattr(char *attr, PyObject *value);        // __setattr__ function

    /// getter for the object handled by this class
    GeomParabola *getGeomParabolaPtr(void) const;

    /** @name additional declarations and methods for the wrapper class */
    //@{

    //@}
};

}  //namespace Part

#endif  // PART_PARABOLAPY_H


