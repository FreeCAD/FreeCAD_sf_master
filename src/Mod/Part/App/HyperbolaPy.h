
// This file is generated by src/Tools/generateTemaplates/templateClassPyExport.py out of the XML file
// Every change you make here get lost at the next full rebuild!
#ifndef PART_HYPERBOLAPY_H
#define PART_HYPERBOLAPY_H

#include <Mod/Part/App/ConicPy.h>
#include <Mod/Part/App/Geometry.h>
#include <string>

namespace Part
{

//===========================================================================
// HyperbolaPy - Python wrapper
//===========================================================================

/** The python export class for GeomHyperbola
 */
class PartExport HyperbolaPy : public Part::ConicPy
{
public:
    static PyTypeObject   Type;
    static PyMethodDef    Methods[];
    static PyGetSetDef    GetterSetter[];
    virtual PyTypeObject *GetType(void) {return &Type;}

public:
    HyperbolaPy(GeomHyperbola *pcObject, PyTypeObject *T = &Type);
    static PyObject *PyMake(struct _typeobject *, PyObject *, PyObject *);
    virtual int PyInit(PyObject* args, PyObject*k);
    ~HyperbolaPy();
    

    typedef GeomHyperbola* PointerType ;

    virtual PyObject *_repr(void);        // the representation
    std::string representation(void) const;

    /** @name callbacks and implementers for the python object methods */
    //@{
    //@}


    /** @name callbacks and implementers for the python object attributes */
    //@{
    ///getter callback for the MajorRadius attribute
    static PyObject * staticCallback_getMajorRadius (PyObject *self, void *closure);
    /// getter for the MajorRadius attribute
    Py::Float getMajorRadius(void) const;
    /// setter callback for the MajorRadius attribute
    static int staticCallback_setMajorRadius (PyObject *self, PyObject *value, void *closure);
    /// setter for the MajorRadius attribute
    void setMajorRadius(Py::Float arg);
    ///getter callback for the MinorRadius attribute
    static PyObject * staticCallback_getMinorRadius (PyObject *self, void *closure);
    /// getter for the MinorRadius attribute
    Py::Float getMinorRadius(void) const;
    /// setter callback for the MinorRadius attribute
    static int staticCallback_setMinorRadius (PyObject *self, PyObject *value, void *closure);
    /// setter for the MinorRadius attribute
    void setMinorRadius(Py::Float arg);
    ///getter callback for the Focal attribute
    static PyObject * staticCallback_getFocal (PyObject *self, void *closure);
    /// getter for the Focal attribute
    Py::Float getFocal(void) const;
    /// setter callback for the Focal attribute
    static int staticCallback_setFocal (PyObject *self, PyObject *value, void *closure);
    // no setter method,  Focal is read only!
    ///getter callback for the Focus1 attribute
    static PyObject * staticCallback_getFocus1 (PyObject *self, void *closure);
    /// getter for the Focus1 attribute
    Py::Object getFocus1(void) const;
    /// setter callback for the Focus1 attribute
    static int staticCallback_setFocus1 (PyObject *self, PyObject *value, void *closure);
    // no setter method,  Focus1 is read only!
    ///getter callback for the Focus2 attribute
    static PyObject * staticCallback_getFocus2 (PyObject *self, void *closure);
    /// getter for the Focus2 attribute
    Py::Object getFocus2(void) const;
    /// setter callback for the Focus2 attribute
    static int staticCallback_setFocus2 (PyObject *self, PyObject *value, void *closure);
    // no setter method,  Focus2 is read only!
    //@}

    /// getter method for special attributes (e.g. dynamic ones)
    PyObject *getCustomAttributes(const char* attr) const;
    /// setter for special attributes (e.g. dynamic ones)
    /// Output: Success=1, Failure=-1, Ignore=0
    int setCustomAttributes(const char* attr, PyObject *obj);
    PyObject *_getattr(char *attr);              // __getattr__ function
    int _setattr(char *attr, PyObject *value);        // __setattr__ function

    /// getter for the object handled by this class
    GeomHyperbola *getGeomHyperbolaPtr(void) const;

    /** @name additional declarations and methods for the wrapper class */
    //@{

    //@}
};

}  //namespace Part

#endif  // PART_HYPERBOLAPY_H


