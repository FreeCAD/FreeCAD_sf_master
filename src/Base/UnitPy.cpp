
// This file is generated by src/Tools/generateTemaplates/templateClassPyExport.py out of the .XML file
// Every change you make here get lost at the next full rebuild!
// This File is normaly build as an include in UnitPyImp.cpp! Its not intended to be in a project!

#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/exception.hpp>
#include <Base/PyObjectBase.h>
#include <Base/Console.h>
#include <Base/Exception.h>
#include <CXX/Objects.hxx>

using Base::streq;
using namespace Base;

/// Type structure of UnitPy
PyTypeObject UnitPy::Type = {
    PyObject_HEAD_INIT(&PyType_Type)
    0,                                                /*ob_size*/
    "Base.Unit",     /*tp_name*/
    sizeof(UnitPy),                       /*tp_basicsize*/
    0,                                                /*tp_itemsize*/
    /* methods */
    PyDestructor,                                     /*tp_dealloc*/
    0,                                                /*tp_print*/
    __getattr,                                        /*tp_getattr*/
    __setattr,                                        /*tp_setattr*/
    0,                                                /*tp_compare*/
    __repr,                                           /*tp_repr*/
    Base::UnitPy::Number,      /*tp_as_number*/
    0,                                                /*tp_as_sequence*/
    0,                                                /*tp_as_mapping*/
    0,                                                /*tp_hash*/
    0,                                                /*tp_call */
    0,                                                /*tp_str  */
    0,                                                /*tp_getattro*/
    0,                                                /*tp_setattro*/
    /* --- Functions to access object as input/output buffer ---------*/
    0,                                                /* tp_as_buffer */
    /* --- Flags to define presence of optional/expanded features */
    Py_TPFLAGS_HAVE_CLASS|Py_TPFLAGS_HAVE_RICHCOMPARE|Py_TPFLAGS_CHECKTYPES,        /*tp_flags */
    "\n"
    " Unit\n"
    " defines a unit type, calculate and compare.\n"
    "\n"
    " The following constructors are supported:\n"
    " Unit()                        -- empty constructor\n"
    " Unit(i1,i2,i3,i4,i5,i6,i7,i8) -- unit signature\n"
    " Unit(Quantity)                -- copy unit from Quantity\n"
    " Unit(Unit)                    -- copy constructor\n"
    " Unit(string)                  -- parse the string for units\n"
    "        ",           /*tp_doc */
    0,                                                /*tp_traverse */
    0,                                                /*tp_clear */
    Base::UnitPy::richCompare,      /*tp_richcompare*/
    0,                                                /*tp_weaklistoffset */
    0,                                                /*tp_iter */
    0,                                                /*tp_iternext */
    Base::UnitPy::Methods,                     /*tp_methods */
    0,                                                /*tp_members */
    Base::UnitPy::GetterSetter,                     /*tp_getset */
    &Base::PyObjectBase::Type,                        /*tp_base */
    0,                                                /*tp_dict */
    0,                                                /*tp_descr_get */
    0,                                                /*tp_descr_set */
    0,                                                /*tp_dictoffset */
    __PyInit,                                         /*tp_init */
    0,                                                /*tp_alloc */
    Base::UnitPy::PyMake,/*tp_new */
    0,                                                /*tp_free   Low-level free-memory routine */
    0,                                                /*tp_is_gc  For PyObject_IS_GC */
    0,                                                /*tp_bases */
    0,                                                /*tp_mro    method resolution order */
    0,                                                /*tp_cache */
    0,                                                /*tp_subclasses */
    0,                                                /*tp_weaklist */
    0,                                                /*tp_del */
    0                                                 /*tp_version_tag */
};

/// Methods structure of UnitPy
PyMethodDef UnitPy::Methods[] = {
    {NULL, NULL, 0, NULL}		/* Sentinel */
};

PyNumberMethods UnitPy::Number[] = { {
    number_add_handler,
    number_subtract_handler,
    number_multiply_handler,
    number_divide_handler,
    number_remainder_handler,
    number_divmod_handler,
    number_power_handler,
    number_negative_handler,
    number_positive_handler,
    number_absolute_handler,
    number_nonzero_handler,
    number_invert_handler,
    number_lshift_handler,
    number_rshift_handler,
    number_and_handler,
    number_xor_handler,
    number_or_handler,
    number_coerce_handler,
    number_int_handler,
    number_long_handler,
    number_float_handler,
    number_oct_handler,
    number_hex_handler,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL
} };


/// Attribute structure of UnitPy
PyGetSetDef UnitPy::GetterSetter[] = {
    {"Type",
        (getter) staticCallback_getType,
        (setter) staticCallback_setType, 
        "holds the unit type as a string, e.g. 'Area'.",
        NULL
    },
    {NULL, NULL, NULL, NULL, NULL}		/* Sentinel */
};

// Type() callback and implementer
// PyObject*  UnitPy::Type(PyObject *args){};
// has to be implemented in UnitPyImp.cpp
PyObject * UnitPy::staticCallback_getType (PyObject *self, void * /*closure*/)
{
    if (!static_cast<PyObjectBase*>(self)->isValid()){
        PyErr_SetString(PyExc_ReferenceError, "This object is already deleted most likely through closing a document. This reference is no longer valid!");
        return NULL;
    }

    try {
        return Py::new_reference_to(static_cast<UnitPy*>(self)->getType());
    } catch (const Py::Exception&) {
        // The exception text is already set
        return NULL;
    } catch (...) {
        PyErr_SetString(Base::BaseExceptionFreeCADError, "Unknown exception while reading attribute 'Type' of object 'Unit'");
        return NULL;
    }
}

int UnitPy::staticCallback_setType (PyObject *self, PyObject * /*value*/, void * /*closure*/)
{
    if (!static_cast<PyObjectBase*>(self)->isValid()){
        PyErr_SetString(PyExc_ReferenceError, "This object is already deleted most likely through closing a document. This reference is no longer valid!");
        return -1;
    }

    PyErr_SetString(PyExc_AttributeError, "Attribute 'Type' of object 'Unit' is read-only");
    return -1;
}




//--------------------------------------------------------------------------
// Constructor
//--------------------------------------------------------------------------
UnitPy::UnitPy(Unit *pcObject, PyTypeObject *T)
    : PyObjectBase(static_cast<PyObjectBase::PointerType>(pcObject), T)
{
    
}


//--------------------------------------------------------------------------
// destructor
//--------------------------------------------------------------------------
UnitPy::~UnitPy()                                // Everything handled in parent
{
    // delete the handled object when the PyObject dies
    UnitPy::PointerType ptr = static_cast<UnitPy::PointerType>(_pcTwinPointer);
    delete ptr;
}

//--------------------------------------------------------------------------
// UnitPy representation
//--------------------------------------------------------------------------
PyObject *UnitPy::_repr(void)
{
    return Py_BuildValue("s", representation().c_str());
}

//--------------------------------------------------------------------------
// UnitPy Attributes
//--------------------------------------------------------------------------
PyObject *UnitPy::_getattr(char *attr)				// __getattr__ function: note only need to handle new state
{
    try {
        // getter method for special Attributes (e.g. dynamic ones)
        PyObject *r = getCustomAttributes(attr);
        if(r) return r;
    }
#ifndef DONT_CATCH_CXX_EXCEPTIONS 
    catch(const Base::Exception& e) // catch the FreeCAD exceptions
    {
        std::string str;
        str += "FreeCAD exception thrown (";
        str += e.what();
        str += ")";
        e.ReportException();
        PyErr_SetString(Base::BaseExceptionFreeCADError,str.c_str());
        return NULL;
    }
    catch(const std::exception& e) // catch other c++ exceptions
    {
        std::string str;
        str += "FC++ exception thrown (";
        str += e.what();
        str += ")";
        Base::Console().Error(str.c_str());
        PyErr_SetString(Base::BaseExceptionFreeCADError,str.c_str());
        return NULL;
    }
    catch(const Py::Exception&)
    {
        // The exception text is already set
        return NULL;
    }
    catch(...)  // catch the rest!
    {
        PyErr_SetString(Base::BaseExceptionFreeCADError,"Unknown C++ exception");
        return NULL;
    }
#else  // DONT_CATCH_CXX_EXCEPTIONS  
    catch(const Base::Exception& e) // catch the FreeCAD exceptions
    {
        std::string str;
        str += "FreeCAD exception thrown (";
        str += e.what();
        str += ")";
        e.ReportException();
        PyErr_SetString(Base::BaseExceptionFreeCADError,str.c_str());
        return NULL;
    }
    catch(const Py::Exception&)
    {
        // The exception text is already set
        return NULL;
    }
#endif  // DONT_CATCH_CXX_EXCEPTIONS

    PyObject *rvalue = Py_FindMethod(Methods, this, attr);
    if (rvalue == NULL)
    {
        PyErr_Clear();
        return PyObjectBase::_getattr(attr);
    }
    else
    {
        return rvalue;
    }
}

int UnitPy::_setattr(char *attr, PyObject *value) // __setattr__ function: note only need to handle new state
{
    try {
        // setter for  special Attributes (e.g. dynamic ones)
        int r = setCustomAttributes(attr, value);
        // r = 1: handled
        // r = -1: error
        // r = 0: ignore
        if (r == 1)
            return 0;
        else if (r == -1)
            return -1;
    }
#ifndef DONT_CATCH_CXX_EXCEPTIONS 
    catch(const Base::Exception& e) // catch the FreeCAD exceptions
    {
        std::string str;
        str += "FreeCAD exception thrown (";
        str += e.what();
        str += ")";
        e.ReportException();
        PyErr_SetString(Base::BaseExceptionFreeCADError,str.c_str());
        return -1;
    }
    catch(const std::exception& e) // catch other c++ exceptions
    {
        std::string str;
        str += "FC++ exception thrown (";
        str += e.what();
        str += ")";
        Base::Console().Error(str.c_str());
        PyErr_SetString(Base::BaseExceptionFreeCADError,str.c_str());
        return -1;
    }
    catch(const Py::Exception&)
    {
        // The exception text is already set
        return -1;
    }
    catch(...)  // catch the rest!
    {
        PyErr_SetString(Base::BaseExceptionFreeCADError,"Unknown C++ exception");
        return -1;
    }
#else  // DONT_CATCH_CXX_EXCEPTIONS  
    catch(const Base::Exception& e) // catch the FreeCAD exceptions
    {
        std::string str;
        str += "FreeCAD exception thrown (";
        str += e.what();
        str += ")";
        e.ReportException();
        PyErr_SetString(Base::BaseExceptionFreeCADError,str.c_str());
        return -1;
    }
    catch(const Py::Exception&)
    {
        // The exception text is already set
        return -1;
    }
#endif  // DONT_CATCH_CXX_EXCEPTIONS

    return PyObjectBase::_setattr(attr, value);
}

Unit *UnitPy::getUnitPtr(void) const
{
    return static_cast<Unit *>(_pcTwinPointer);
}

#if 0
/* From here on come the methods you have to implement, but NOT in this module. Implement in UnitPyImp.cpp! This prototypes 
 * are just for convenience when you add a new method.
 */

PyObject *UnitPy::PyMake(struct _typeobject *, PyObject *, PyObject *)  // Python wrapper
{
    // create a new instance of UnitPy and the Twin object 
    return new UnitPy(new Unit);
}

// constructor method
int UnitPy::PyInit(PyObject* /*args*/, PyObject* /*kwd*/)
{
    return 0;
}


// returns a string which represents the object e.g. when printed in python
std::string UnitPy::representation(void) const
{
    return std::string("<Unit object>");
}

PyObject* UnitPy::number_add_handler(PyObject* /*self*/, PyObject* /*other*/)
{
    PyErr_SetString(PyExc_NotImplementedError, "Not implemented");
    return 0;
}

PyObject* UnitPy::number_subtract_handler(PyObject* /*self*/, PyObject* /*other*/)
{
    PyErr_SetString(PyExc_NotImplementedError, "Not implemented");
    return 0;
}

PyObject* UnitPy::number_multiply_handler(PyObject* /*self*/, PyObject* /*other*/)
{
    PyErr_SetString(PyExc_NotImplementedError, "Not implemented");
    return 0;
}

PyObject * UnitPy::number_divide_handler (PyObject* /*self*/, PyObject* /*other*/)
{
    PyErr_SetString(PyExc_NotImplementedError, "Not implemented");
    return 0;
}

PyObject * UnitPy::number_remainder_handler (PyObject* /*self*/, PyObject* /*other*/)
{
    PyErr_SetString(PyExc_NotImplementedError, "Not implemented");
    return 0;
}

PyObject * UnitPy::number_divmod_handler (PyObject* /*self*/, PyObject* /*other*/)
{
    PyErr_SetString(PyExc_NotImplementedError, "Not implemented");
    return 0;
}

PyObject * UnitPy::number_power_handler (PyObject* /*self*/, PyObject* /*other*/, PyObject* /*modulo*/)
{
    PyErr_SetString(PyExc_NotImplementedError, "Not implemented");
    return 0;
}

PyObject * UnitPy::number_negative_handler (PyObject* /*self*/)
{
    PyErr_SetString(PyExc_NotImplementedError, "Not implemented");
    return 0;
}

PyObject * UnitPy::number_positive_handler (PyObject* /*self*/)
{
    PyErr_SetString(PyExc_NotImplementedError, "Not implemented");
    return 0;
}

PyObject * UnitPy::number_absolute_handler (PyObject* /*self*/)
{
    PyErr_SetString(PyExc_NotImplementedError, "Not implemented");
    return 0;
}

int UnitPy::number_nonzero_handler (PyObject* /*self*/)
{
    return 1;
}

PyObject * UnitPy::number_invert_handler (PyObject* /*self*/)
{
    PyErr_SetString(PyExc_NotImplementedError, "Not implemented");
    return 0;
}

PyObject * UnitPy::number_lshift_handler (PyObject* /*self*/, PyObject* /*other*/)
{
    PyErr_SetString(PyExc_NotImplementedError, "Not implemented");
    return 0;
}

PyObject * UnitPy::number_rshift_handler (PyObject* /*self*/, PyObject* /*other*/)
{
    PyErr_SetString(PyExc_NotImplementedError, "Not implemented");
    return 0;
}

PyObject * UnitPy::number_and_handler (PyObject* /*self*/, PyObject* /*other*/)
{
    PyErr_SetString(PyExc_NotImplementedError, "Not implemented");
    return 0;
}

PyObject * UnitPy::number_xor_handler (PyObject* /*self*/, PyObject* /*other*/)
{
    PyErr_SetString(PyExc_NotImplementedError, "Not implemented");
    return 0;
}

PyObject * UnitPy::number_or_handler (PyObject* /*self*/, PyObject* /*other*/)
{
    PyErr_SetString(PyExc_NotImplementedError, "Not implemented");
    return 0;
}

int UnitPy::number_coerce_handler (PyObject** /*self*/, PyObject** /*other*/)
{
    return 1;
}

PyObject * UnitPy::number_int_handler (PyObject* /*self*/)
{
    PyErr_SetString(PyExc_NotImplementedError, "Not implemented");
    return 0;
}

PyObject * UnitPy::number_long_handler (PyObject* /*self*/)
{
    PyErr_SetString(PyExc_NotImplementedError, "Not implemented");
    return 0;
}

PyObject * UnitPy::number_float_handler (PyObject* /*self*/)
{
    PyErr_SetString(PyExc_NotImplementedError, "Not implemented");
    return 0;
}

PyObject * UnitPy::number_oct_handler (PyObject* /*self*/)
{
    PyErr_SetString(PyExc_NotImplementedError, "Not implemented");
    return 0;
}

PyObject * UnitPy::number_hex_handler (PyObject* /*self*/)
{
    PyErr_SetString(PyExc_NotImplementedError, "Not implemented");
    return 0;
}

PyObject* UnitPy::richCompare(PyObject *v, PyObject *w, int op)
{
    PyErr_SetString(PyExc_NotImplementedError, "Not yet implemented");
    return 0;
}

Py::String UnitPy::getType(void) const
{
    //return Py::String();
    throw Py::AttributeError("Not yet implemented");
}

PyObject *UnitPy::getCustomAttributes(const char* /*attr*/) const
{
    return 0;
}

int UnitPy::setCustomAttributes(const char* /*attr*/, PyObject* /*obj*/)
{
    return 0; 
}
#endif



