
// This file is generated by src/Tools/generateTemaplates/templateClassPyExport.py out of the XML file
// Every change you make here get lost at the next full rebuild!
#ifndef SPREADSHEET_PROPERTYSHEETPY_H
#define SPREADSHEET_PROPERTYSHEETPY_H

#include <Base/PersistencePy.h>
#include <Mod/Spreadsheet/App/PropertySheet.h>
#include <string>

namespace Spreadsheet
{

//===========================================================================
// PropertySheetPy - Python wrapper
//===========================================================================

/** The python export class for PropertySheet
 */
class SpreadsheetExport PropertySheetPy : public Base::PersistencePy
{
public:
    static PyTypeObject   Type;
    static PyMethodDef    Methods[];
    static PyGetSetDef    GetterSetter[];
    virtual PyTypeObject *GetType(void) {return &Type;}

public:
    PropertySheetPy(PropertySheet *pcObject, PyTypeObject *T = &Type);
    static PyObject *PyMake(struct _typeobject *, PyObject *, PyObject *);
    virtual int PyInit(PyObject* args, PyObject*k);
    ~PropertySheetPy();
    

    typedef PropertySheet* PointerType ;

    virtual PyObject *_repr(void);        // the representation
    std::string representation(void) const;

    /** @name callbacks and implementers for the python object methods */
    //@{
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
    PropertySheet *getPropertySheetPtr(void) const;

    /** @name additional declarations and methods for the wrapper class */
    //@{

    //@}
};

}  //namespace Spreadsheet

#endif  // SPREADSHEET_PROPERTYSHEETPY_H


