
// This file is generated by src/Tools/generateTemaplates/templateClassPyExport.py out of the XML file
// Every change you make here get lost at the next full rebuild!
#ifndef MESH_MESHFEATUREPY_H
#define MESH_MESHFEATUREPY_H

#include <App/DocumentObjectPy.h>
#include <Mod/Mesh/App/MeshFeature.h>
#include <string>

namespace Mesh
{

//===========================================================================
// MeshFeaturePy - Python wrapper
//===========================================================================

/** The python export class for Feature
 */
class MeshExport MeshFeaturePy : public App::DocumentObjectPy
{
public:
    static PyTypeObject   Type;
    static PyMethodDef    Methods[];
    static PyGetSetDef    GetterSetter[];
    virtual PyTypeObject *GetType(void) {return &Type;};

public:
    MeshFeaturePy(Feature *pcObject, PyTypeObject *T = &Type);
    static PyObject *PyMake(struct _typeobject *, PyObject *, PyObject *);
    virtual int PyInit(PyObject* args, PyObject*k);
    ~MeshFeaturePy();

    typedef Feature* PointerType ;

    virtual PyObject *_repr(void);        // the representation
    std::string representation(void) const;

    /** @name callbacks and implementers for the python object methods */
    //@{
    /// callback for the countPoints() method
    static PyObject * staticCallback_countPoints (PyObject *self, PyObject *args);
    /// implementer for the countPoints() method
    PyObject*  countPoints(PyObject *args);
    /// callback for the countFacets() method
    static PyObject * staticCallback_countFacets (PyObject *self, PyObject *args);
    /// implementer for the countFacets() method
    PyObject*  countFacets(PyObject *args);
    /// callback for the harmonizeNormals() method
    static PyObject * staticCallback_harmonizeNormals (PyObject *self, PyObject *args);
    /// implementer for the harmonizeNormals() method
    PyObject*  harmonizeNormals(PyObject *args);
    /// callback for the smooth() method
    static PyObject * staticCallback_smooth (PyObject *self, PyObject *args);
    /// implementer for the smooth() method
    PyObject*  smooth(PyObject *args);
    /// callback for the removeNonManifolds() method
    static PyObject * staticCallback_removeNonManifolds (PyObject *self, PyObject *args);
    /// implementer for the removeNonManifolds() method
    PyObject*  removeNonManifolds(PyObject *args);
    /// callback for the fixIndices() method
    static PyObject * staticCallback_fixIndices (PyObject *self, PyObject *args);
    /// implementer for the fixIndices() method
    PyObject*  fixIndices(PyObject *args);
    /// callback for the fixDegenerations() method
    static PyObject * staticCallback_fixDegenerations (PyObject *self, PyObject *args);
    /// implementer for the fixDegenerations() method
    PyObject*  fixDegenerations(PyObject *args);
    /// callback for the removeDuplicatedFacets() method
    static PyObject * staticCallback_removeDuplicatedFacets (PyObject *self, PyObject *args);
    /// implementer for the removeDuplicatedFacets() method
    PyObject*  removeDuplicatedFacets(PyObject *args);
    /// callback for the removeDuplicatedPoints() method
    static PyObject * staticCallback_removeDuplicatedPoints (PyObject *self, PyObject *args);
    /// implementer for the removeDuplicatedPoints() method
    PyObject*  removeDuplicatedPoints(PyObject *args);
    /// callback for the fixSelfIntersections() method
    static PyObject * staticCallback_fixSelfIntersections (PyObject *self, PyObject *args);
    /// implementer for the fixSelfIntersections() method
    PyObject*  fixSelfIntersections(PyObject *args);
    /// callback for the removeFoldsOnSurface() method
    static PyObject * staticCallback_removeFoldsOnSurface (PyObject *self, PyObject *args);
    /// implementer for the removeFoldsOnSurface() method
    PyObject*  removeFoldsOnSurface(PyObject *args);
    /// callback for the removeInvalidPoints() method
    static PyObject * staticCallback_removeInvalidPoints (PyObject *self, PyObject *args);
    /// implementer for the removeInvalidPoints() method
    PyObject*  removeInvalidPoints(PyObject *args);
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
    Feature *getFeaturePtr(void) const;

    /** @name additional declarations and methods for the wrapper class */
    //@{

    //@}
};

}  //namespace Mesh

#endif  // MESH_MESHFEATUREPY_H


