
// This file is generated by src/Tools/generateTemaplates/templateClassPyExport.py out of the XML file
// Every change you make here get lost at the next full rebuild!
#ifndef PART_CIRCLEPY_H
#define PART_CIRCLEPY_H

#include <Mod/Part/App/ConicPy.h>
#include <Mod/Part/App/Geometry.h>
#include <string>

namespace Part
{

//===========================================================================
// CirclePy - Python wrapper
//===========================================================================

/** The python export class for GeomCircle
 */
class PartExport CirclePy : public Part::ConicPy
{
public:
    static PyTypeObject   Type;
    static PyMethodDef    Methods[];
    static PyGetSetDef    GetterSetter[];
    virtual PyTypeObject *GetType(void) {return &Type;}

public:
    CirclePy(GeomCircle *pcObject, PyTypeObject *T = &Type);
    static PyObject *PyMake(struct _typeobject *, PyObject *, PyObject *);
    virtual int PyInit(PyObject* args, PyObject*k);
    ~CirclePy();
    

    typedef GeomCircle* PointerType ;

    virtual PyObject *_repr(void);        // the representation
    std::string representation(void) const;

    /** @name callbacks and implementers for the python object methods */
    //@{
    //@}


    /** @name callbacks and implementers for the python object attributes */
    //@{
    ///getter callback for the Radius attribute
    static PyObject * staticCallback_getRadius (PyObject *self, void *closure);
    /// getter for the Radius attribute
    Py::Float getRadius(void) const;
    /// setter callback for the Radius attribute
    static int staticCallback_setRadius (PyObject *self, PyObject *value, void *closure);
    /// setter for the Radius attribute
    void setRadius(Py::Float arg);
    //@}

    /// getter method for special attributes (e.g. dynamic ones)
    PyObject *getCustomAttributes(const char* attr) const;
    /// setter for special attributes (e.g. dynamic ones)
    /// Output: Success=1, Failure=-1, Ignore=0
    int setCustomAttributes(const char* attr, PyObject *obj);
    PyObject *_getattr(char *attr);              // __getattr__ function
    int _setattr(char *attr, PyObject *value);        // __setattr__ function

    /// getter for the object handled by this class
    GeomCircle *getGeomCirclePtr(void) const;

    /** @name additional declarations and methods for the wrapper class */
    //@{

    //@}
};

}  //namespace Part

#endif  // PART_CIRCLEPY_H


