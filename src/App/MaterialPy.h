
// This file is generated by src/Tools/generateTemaplates/templateClassPyExport.py out of the XML file
// Every change you make here get lost at the next full rebuild!
#ifndef APP_MATERIALPY_H
#define APP_MATERIALPY_H

#include <Base/PyObjectBase.h>
#include <App/Material.h>
#include <string>

namespace App
{

//===========================================================================
// MaterialPy - Python wrapper
//===========================================================================

/** The python export class for Material
 */
class AppExport MaterialPy : public Base::PyObjectBase
{
public:
    static PyTypeObject   Type;
    static PyMethodDef    Methods[];
    static PyGetSetDef    GetterSetter[];
    virtual PyTypeObject *GetType(void) {return &Type;}

public:
    MaterialPy(Material *pcObject, PyTypeObject *T = &Type);
    static PyObject *PyMake(struct _typeobject *, PyObject *, PyObject *);
    virtual int PyInit(PyObject* args, PyObject*k);
    ~MaterialPy();
    

    typedef Material* PointerType ;

    virtual PyObject *_repr(void);        // the representation
    std::string representation(void) const;

    /** @name callbacks and implementers for the python object methods */
    //@{
    /// callback for the set() method
    static PyObject * staticCallback_set (PyObject *self, PyObject *args);
    /// implementer for the set() method
    PyObject*  set(PyObject *args);
    //@}


    /** @name callbacks and implementers for the python object attributes */
    //@{
    ///getter callback for the AmbientColor attribute
    static PyObject * staticCallback_getAmbientColor (PyObject *self, void *closure);
    /// getter for the AmbientColor attribute
    Py::Tuple getAmbientColor(void) const;
    /// setter callback for the AmbientColor attribute
    static int staticCallback_setAmbientColor (PyObject *self, PyObject *value, void *closure);
    /// setter for the AmbientColor attribute
    void setAmbientColor(Py::Tuple arg);
    ///getter callback for the DiffuseColor attribute
    static PyObject * staticCallback_getDiffuseColor (PyObject *self, void *closure);
    /// getter for the DiffuseColor attribute
    Py::Tuple getDiffuseColor(void) const;
    /// setter callback for the DiffuseColor attribute
    static int staticCallback_setDiffuseColor (PyObject *self, PyObject *value, void *closure);
    /// setter for the DiffuseColor attribute
    void setDiffuseColor(Py::Tuple arg);
    ///getter callback for the EmissiveColor attribute
    static PyObject * staticCallback_getEmissiveColor (PyObject *self, void *closure);
    /// getter for the EmissiveColor attribute
    Py::Tuple getEmissiveColor(void) const;
    /// setter callback for the EmissiveColor attribute
    static int staticCallback_setEmissiveColor (PyObject *self, PyObject *value, void *closure);
    /// setter for the EmissiveColor attribute
    void setEmissiveColor(Py::Tuple arg);
    ///getter callback for the SpecularColor attribute
    static PyObject * staticCallback_getSpecularColor (PyObject *self, void *closure);
    /// getter for the SpecularColor attribute
    Py::Tuple getSpecularColor(void) const;
    /// setter callback for the SpecularColor attribute
    static int staticCallback_setSpecularColor (PyObject *self, PyObject *value, void *closure);
    /// setter for the SpecularColor attribute
    void setSpecularColor(Py::Tuple arg);
    ///getter callback for the Shininess attribute
    static PyObject * staticCallback_getShininess (PyObject *self, void *closure);
    /// getter for the Shininess attribute
    Py::Float getShininess(void) const;
    /// setter callback for the Shininess attribute
    static int staticCallback_setShininess (PyObject *self, PyObject *value, void *closure);
    /// setter for the Shininess attribute
    void setShininess(Py::Float arg);
    ///getter callback for the Transparency attribute
    static PyObject * staticCallback_getTransparency (PyObject *self, void *closure);
    /// getter for the Transparency attribute
    Py::Float getTransparency(void) const;
    /// setter callback for the Transparency attribute
    static int staticCallback_setTransparency (PyObject *self, PyObject *value, void *closure);
    /// setter for the Transparency attribute
    void setTransparency(Py::Float arg);
    //@}

    /// getter method for special attributes (e.g. dynamic ones)
    PyObject *getCustomAttributes(const char* attr) const;
    /// setter for special attributes (e.g. dynamic ones)
    /// Output: Success=1, Failure=-1, Ignore=0
    int setCustomAttributes(const char* attr, PyObject *obj);
    PyObject *_getattr(char *attr);              // __getattr__ function
    int _setattr(char *attr, PyObject *value);        // __setattr__ function

    /// getter for the object handled by this class
    Material *getMaterialPtr(void) const;

    /** @name additional declarations and methods for the wrapper class */
    //@{

    //@}
};

}  //namespace App

#endif  // APP_MATERIALPY_H


