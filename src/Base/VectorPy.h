
// This file is generated by src/Tools/generateTemaplates/templateClassPyExport.py out of the XML file
// Every change you make here get lost at the next full rebuild!
#ifndef BASE_VECTORPY_H
#define BASE_VECTORPY_H

#include <Base/PyObjectBase.h>
#include <Base/Vector3D.h>
#include <string>

namespace Base
{

//===========================================================================
// VectorPy - Python wrapper
//===========================================================================

/** The python export class for Vector
 */
class BaseExport VectorPy : public Base::PyObjectBase
{
public:
    static PyTypeObject   Type;
    static PyMethodDef    Methods[];
    static PyNumberMethods Number[];
    static PySequenceMethods Sequence[];
    static PyObject * richCompare(PyObject *v, PyObject *w, int op);
    static PyGetSetDef    GetterSetter[];
    virtual PyTypeObject *GetType(void) {return &Type;}

public:
    VectorPy(Vector3d *pcObject, PyTypeObject *T = &Type);
    static PyObject *PyMake(struct _typeobject *, PyObject *, PyObject *);
    virtual int PyInit(PyObject* args, PyObject*k);
    ~VectorPy();
    

    typedef Vector3d* PointerType ;

    virtual PyObject *_repr(void);        // the representation
    std::string representation(void) const;

    /** @name callbacks and implementers for the python object methods */
    //@{
    /// callback for the add() method
    static PyObject * staticCallback_add (PyObject *self, PyObject *args);
    /// implementer for the add() method
    PyObject*  add(PyObject *args);
    /// callback for the sub() method
    static PyObject * staticCallback_sub (PyObject *self, PyObject *args);
    /// implementer for the sub() method
    PyObject*  sub(PyObject *args);
    /// callback for the negative() method
    static PyObject * staticCallback_negative (PyObject *self, PyObject *args);
    /// implementer for the negative() method
    PyObject*  negative(PyObject *args);
    /// callback for the scale() method
    static PyObject * staticCallback_scale (PyObject *self, PyObject *args);
    /// implementer for the scale() method
    PyObject*  scale(PyObject *args);
    /// callback for the multiply() method
    static PyObject * staticCallback_multiply (PyObject *self, PyObject *args);
    /// implementer for the multiply() method
    PyObject*  multiply(PyObject *args);
    /// callback for the dot() method
    static PyObject * staticCallback_dot (PyObject *self, PyObject *args);
    /// implementer for the dot() method
    PyObject*  dot(PyObject *args);
    /// callback for the cross() method
    static PyObject * staticCallback_cross (PyObject *self, PyObject *args);
    /// implementer for the cross() method
    PyObject*  cross(PyObject *args);
    /// callback for the getAngle() method
    static PyObject * staticCallback_getAngle (PyObject *self, PyObject *args);
    /// implementer for the getAngle() method
    PyObject*  getAngle(PyObject *args);
    /// callback for the normalize() method
    static PyObject * staticCallback_normalize (PyObject *self, PyObject *args);
    /// implementer for the normalize() method
    PyObject*  normalize(PyObject *args);
    /// callback for the isEqual() method
    static PyObject * staticCallback_isEqual (PyObject *self, PyObject *args);
    /// implementer for the isEqual() method
    PyObject*  isEqual(PyObject *args);
    /// callback for the projectToLine() method
    static PyObject * staticCallback_projectToLine (PyObject *self, PyObject *args);
    /// implementer for the projectToLine() method
    PyObject*  projectToLine(PyObject *args);
    /// callback for the projectToPlane() method
    static PyObject * staticCallback_projectToPlane (PyObject *self, PyObject *args);
    /// implementer for the projectToPlane() method
    PyObject*  projectToPlane(PyObject *args);
    /// callback for the distanceToPoint() method
    static PyObject * staticCallback_distanceToPoint (PyObject *self, PyObject *args);
    /// implementer for the distanceToPoint() method
    PyObject*  distanceToPoint(PyObject *args);
    /// callback for the distanceToLine() method
    static PyObject * staticCallback_distanceToLine (PyObject *self, PyObject *args);
    /// implementer for the distanceToLine() method
    PyObject*  distanceToLine(PyObject *args);
    /// callback for the distanceToLineSegment() method
    static PyObject * staticCallback_distanceToLineSegment (PyObject *self, PyObject *args);
    /// implementer for the distanceToLineSegment() method
    PyObject*  distanceToLineSegment(PyObject *args);
    /// callback for the distanceToPlane() method
    static PyObject * staticCallback_distanceToPlane (PyObject *self, PyObject *args);
    /// implementer for the distanceToPlane() method
    PyObject*  distanceToPlane(PyObject *args);
    //@}

    /** @name callbacks and implementers for the python object number protocol */
    //@{
    /// callback for the number_add_handler
    static PyObject * number_add_handler (PyObject *self, PyObject *other);
    /// callback for the number_subtract_handler
    static PyObject * number_subtract_handler (PyObject *self, PyObject *other);
    /// callback for the number_multiply_handler
    static PyObject * number_multiply_handler (PyObject *self, PyObject *other);
    /// callback for the number_divide_handler
    static PyObject * number_divide_handler (PyObject *self, PyObject *other);
    /// callback for the number_remainder_handler
    static PyObject * number_remainder_handler (PyObject *self, PyObject *other);
    /// callback for the number_divmod_handler
    static PyObject * number_divmod_handler (PyObject *self, PyObject *other);
    /// callback for the number_power_handler
    static PyObject * number_power_handler (PyObject *self, PyObject *other, PyObject *modulo);
    /// callback for the number_negative_handler
    static PyObject * number_negative_handler (PyObject *self);
    /// callback for the number_positive_handler
    static PyObject * number_positive_handler (PyObject *self);
    /// callback for the number_absolute_handler
    static PyObject * number_absolute_handler (PyObject *self);
    /// callback for the number_nonzero_handler
    static int number_nonzero_handler (PyObject *self);
    /// callback for the number_invert_handler
    static PyObject * number_invert_handler (PyObject *self);
    /// callback for the number_lshift_handler
    static PyObject * number_lshift_handler (PyObject *self, PyObject *other);
    /// callback for the number_rshift_handler
    static PyObject * number_rshift_handler (PyObject *self, PyObject *other);
    /// callback for the number_and_handler
    static PyObject * number_and_handler (PyObject *self, PyObject *other);
    /// callback for the number_xor_handler
    static PyObject * number_xor_handler (PyObject *self, PyObject *other);
    /// callback for the number_or_handler
    static PyObject * number_or_handler (PyObject *self, PyObject *other);
    /// callback for the number_coerce_handler
    static int number_coerce_handler (PyObject **self, PyObject **other);
    /// callback for the number_int_handler
    static PyObject * number_int_handler (PyObject *self);
    /// callback for the number_long_handler
    static PyObject * number_long_handler (PyObject *self);
    /// callback for the number_float_handler
    static PyObject * number_float_handler (PyObject *self);
    /// callback for the number_oct_handler
    static PyObject * number_oct_handler (PyObject *self);
    /// callback for the number_hex_handler
    static PyObject * number_hex_handler (PyObject *self);
    //@}
    /** @name callbacks and implementers for the python object sequence protocol */
    //@{
    static Py_ssize_t sequence_length(PyObject *);
    static PyObject * sequence_item(PyObject *, Py_ssize_t);
    static int sequence_ass_item(PyObject *, Py_ssize_t, PyObject *);
    //@}

    /** @name callbacks and implementers for the python object attributes */
    //@{
    ///getter callback for the Length attribute
    static PyObject * staticCallback_getLength (PyObject *self, void *closure);
    /// getter for the Length attribute
    Py::Float getLength(void) const;
    /// setter callback for the Length attribute
    static int staticCallback_setLength (PyObject *self, PyObject *value, void *closure);
    /// setter for the Length attribute
    void setLength(Py::Float arg);
    ///getter callback for the x attribute
    static PyObject * staticCallback_getx (PyObject *self, void *closure);
    /// getter for the x attribute
    Py::Float getx(void) const;
    /// setter callback for the x attribute
    static int staticCallback_setx (PyObject *self, PyObject *value, void *closure);
    /// setter for the x attribute
    void setx(Py::Float arg);
    ///getter callback for the y attribute
    static PyObject * staticCallback_gety (PyObject *self, void *closure);
    /// getter for the y attribute
    Py::Float gety(void) const;
    /// setter callback for the y attribute
    static int staticCallback_sety (PyObject *self, PyObject *value, void *closure);
    /// setter for the y attribute
    void sety(Py::Float arg);
    ///getter callback for the z attribute
    static PyObject * staticCallback_getz (PyObject *self, void *closure);
    /// getter for the z attribute
    Py::Float getz(void) const;
    /// setter callback for the z attribute
    static int staticCallback_setz (PyObject *self, PyObject *value, void *closure);
    /// setter for the z attribute
    void setz(Py::Float arg);
    //@}

    /// getter method for special attributes (e.g. dynamic ones)
    PyObject *getCustomAttributes(const char* attr) const;
    /// setter for special attributes (e.g. dynamic ones)
    /// Output: Success=1, Failure=-1, Ignore=0
    int setCustomAttributes(const char* attr, PyObject *obj);
    PyObject *_getattr(char *attr);              // __getattr__ function
    int _setattr(char *attr, PyObject *value);        // __setattr__ function

    /// getter for the object handled by this class
    Vector3d *getVectorPtr(void) const;

    /** @name additional declarations and methods for the wrapper class */
    //@{
public:
    VectorPy(const Vector3d & vec, PyTypeObject *T = &Type)
    :PyObjectBase(new Vector3d(vec),T){}
    VectorPy(const Vector3f & vec, PyTypeObject *T = &Type)
    :PyObjectBase(new Vector3d(vec.x,vec.y,vec.z),T){}
    Vector3d value() const
    { return *(getVectorPtr()); }
		
    //@}
};

}  //namespace Base

#endif  // BASE_VECTORPY_H


