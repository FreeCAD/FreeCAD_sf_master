
// This file is generated by src/Tools/generateTemaplates/templateClassPyExport.py out of the XML file
// Every change you make here get lost at the next full rebuild!
#ifndef PART_CIRCLE2DPY_H
#define PART_CIRCLE2DPY_H

#include <Mod/Part/App/Geom2d/Conic2dPy.h>
#include <Mod/Part/App/Geometry2d.h>
#include <string>

namespace Part
{

//===========================================================================
// Circle2dPy - Python wrapper
//===========================================================================

/** The python export class for Geom2dCircle
 */
class PartExport Circle2dPy : public Part::Conic2dPy
{
public:
    static PyTypeObject   Type;
    static PyMethodDef    Methods[];
    static PyGetSetDef    GetterSetter[];
    virtual PyTypeObject *GetType(void) {return &Type;}

public:
    Circle2dPy(Geom2dCircle *pcObject, PyTypeObject *T = &Type);
    static PyObject *PyMake(struct _typeobject *, PyObject *, PyObject *);
    virtual int PyInit(PyObject* args, PyObject*k);
    ~Circle2dPy();
    

    typedef Geom2dCircle* PointerType ;

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
    Geom2dCircle *getGeom2dCirclePtr(void) const;

    /** @name additional declarations and methods for the wrapper class */
    //@{

    //@}
};

}  //namespace Part

#endif  // PART_CIRCLE2DPY_H


