
// This file is generated by src/Tools/generateTemaplates/templateClassPyExport.py out of the XML file
// Every change you make here get lost at the next full rebuild!
#ifndef MESH_FACETPY_H
#define MESH_FACETPY_H

#include <Base/PyObjectBase.h>
#include <Mod/Mesh/App/Facet.h>
#include <string>

namespace Mesh
{

//===========================================================================
// FacetPy - Python wrapper
//===========================================================================

/** The python export class for Facet
 */
class MeshExport FacetPy : public Base::PyObjectBase
{
public:
    static PyTypeObject   Type;
    static PyMethodDef    Methods[];
    static PyGetSetDef    GetterSetter[];
    virtual PyTypeObject *GetType(void) {return &Type;}

public:
    FacetPy(Facet *pcObject, PyTypeObject *T = &Type);
    static PyObject *PyMake(struct _typeobject *, PyObject *, PyObject *);
    virtual int PyInit(PyObject* args, PyObject*k);
    ~FacetPy();
    

    typedef Facet* PointerType ;

    virtual PyObject *_repr(void);        // the representation
    std::string representation(void) const;

    /** @name callbacks and implementers for the python object methods */
    //@{
    /// callback for the unbound() method
    static PyObject * staticCallback_unbound (PyObject *self, PyObject *args);
    /// implementer for the unbound() method
    PyObject*  unbound(PyObject *args);
    /// callback for the intersect() method
    static PyObject * staticCallback_intersect (PyObject *self, PyObject *args);
    /// implementer for the intersect() method
    PyObject*  intersect(PyObject *args);
    /// callback for the isDegenerated() method
    static PyObject * staticCallback_isDegenerated (PyObject *self, PyObject *args);
    /// implementer for the isDegenerated() method
    PyObject*  isDegenerated(PyObject *args);
    //@}


    /** @name callbacks and implementers for the python object attributes */
    //@{
    ///getter callback for the Index attribute
    static PyObject * staticCallback_getIndex (PyObject *self, void *closure);
    /// getter for the Index attribute
    Py::Int getIndex(void) const;
    /// setter callback for the Index attribute
    static int staticCallback_setIndex (PyObject *self, PyObject *value, void *closure);
    // no setter method,  Index is read only!
    ///getter callback for the Bound attribute
    static PyObject * staticCallback_getBound (PyObject *self, void *closure);
    /// getter for the Bound attribute
    Py::Boolean getBound(void) const;
    /// setter callback for the Bound attribute
    static int staticCallback_setBound (PyObject *self, PyObject *value, void *closure);
    // no setter method,  Bound is read only!
    ///getter callback for the Normal attribute
    static PyObject * staticCallback_getNormal (PyObject *self, void *closure);
    /// getter for the Normal attribute
    Py::Object getNormal(void) const;
    /// setter callback for the Normal attribute
    static int staticCallback_setNormal (PyObject *self, PyObject *value, void *closure);
    // no setter method,  Normal is read only!
    ///getter callback for the Points attribute
    static PyObject * staticCallback_getPoints (PyObject *self, void *closure);
    /// getter for the Points attribute
    Py::List getPoints(void) const;
    /// setter callback for the Points attribute
    static int staticCallback_setPoints (PyObject *self, PyObject *value, void *closure);
    // no setter method,  Points is read only!
    ///getter callback for the PointIndices attribute
    static PyObject * staticCallback_getPointIndices (PyObject *self, void *closure);
    /// getter for the PointIndices attribute
    Py::Tuple getPointIndices(void) const;
    /// setter callback for the PointIndices attribute
    static int staticCallback_setPointIndices (PyObject *self, PyObject *value, void *closure);
    // no setter method,  PointIndices is read only!
    ///getter callback for the NeighbourIndices attribute
    static PyObject * staticCallback_getNeighbourIndices (PyObject *self, void *closure);
    /// getter for the NeighbourIndices attribute
    Py::Tuple getNeighbourIndices(void) const;
    /// setter callback for the NeighbourIndices attribute
    static int staticCallback_setNeighbourIndices (PyObject *self, PyObject *value, void *closure);
    // no setter method,  NeighbourIndices is read only!
    ///getter callback for the Area attribute
    static PyObject * staticCallback_getArea (PyObject *self, void *closure);
    /// getter for the Area attribute
    Py::Float getArea(void) const;
    /// setter callback for the Area attribute
    static int staticCallback_setArea (PyObject *self, PyObject *value, void *closure);
    // no setter method,  Area is read only!
    //@}

    /// getter method for special attributes (e.g. dynamic ones)
    PyObject *getCustomAttributes(const char* attr) const;
    /// setter for special attributes (e.g. dynamic ones)
    /// Output: Success=1, Failure=-1, Ignore=0
    int setCustomAttributes(const char* attr, PyObject *obj);
    PyObject *_getattr(char *attr);              // __getattr__ function
    int _setattr(char *attr, PyObject *value);        // __setattr__ function

    /// getter for the object handled by this class
    Facet *getFacetPtr(void) const;

    /** @name additional declarations and methods for the wrapper class */
    //@{

    //@}
};

}  //namespace Mesh

#endif  // MESH_FACETPY_H


