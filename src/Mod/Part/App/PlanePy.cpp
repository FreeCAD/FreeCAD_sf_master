
// This file is generated by src/Tools/generateTemaplates/templateClassPyExport.py out of the .XML file
// Every change you make here get lost at the next full rebuild!
// This File is normaly build as an include in PlanePyImp.cpp! Its not intended to be in a project!

#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/exception.hpp>
#include <Base/PyObjectBase.h>
#include <Base/Console.h>
#include <Base/Exception.h>
#include <CXX/Objects.hxx>

using Base::streq;
using namespace Part;

/// Type structure of PlanePy
PyTypeObject PlanePy::Type = {
    PyObject_HEAD_INIT(&PyType_Type)
    0,                                                /*ob_size*/
    "Part.Plane",     /*tp_name*/
    sizeof(PlanePy),                       /*tp_basicsize*/
    0,                                                /*tp_itemsize*/
    /* methods */
    PyDestructor,                                     /*tp_dealloc*/
    0,                                                /*tp_print*/
    __getattr,                                        /*tp_getattr*/
    __setattr,                                        /*tp_setattr*/
    0,                                                /*tp_compare*/
    __repr,                                           /*tp_repr*/
    0,                                                /*tp_as_number*/
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
    Py_TPFLAGS_HAVE_CLASS,        /*tp_flags */
    "Describes an infinite plane\n"
    "To create a plane there are several ways:\n"
    "Part.Plane()\n"
    "    Creates a default plane with base (0,0,0) and normal (0,0,1)\n"
    "\n"
    "Part.Plane(Plane)\n"
    "    Creates a copy of the given plane\n"
    "\n"
    "Part.Plane(Plane, Distance)\n"
    "    Creates a plane parallel to given plane at a certain distance\n"
    "\n"
    "Part.Plane(Location,Normal)\n"
    "    Creates a plane with a given location and normal\n"
    "\n"
    "Part.Plane(Point1,Point2,Point3)\n"
    "    Creates a plane defined by three non-linear points\n"
    "\n"
    "Part.Plane(A,B,C,D)\n"
    "    Creates a plane from its cartesian equation\n"
    "    Ax+By+Cz+D=0\n"
    "",           /*tp_doc */
    0,                                                /*tp_traverse */
    0,                                                /*tp_clear */
    0,                                                /*tp_richcompare */
    0,                                                /*tp_weaklistoffset */
    0,                                                /*tp_iter */
    0,                                                /*tp_iternext */
    Part::PlanePy::Methods,                     /*tp_methods */
    0,                                                /*tp_members */
    Part::PlanePy::GetterSetter,                     /*tp_getset */
    &Part::GeometrySurfacePy::Type,                        /*tp_base */
    0,                                                /*tp_dict */
    0,                                                /*tp_descr_get */
    0,                                                /*tp_descr_set */
    0,                                                /*tp_dictoffset */
    __PyInit,                                         /*tp_init */
    0,                                                /*tp_alloc */
    Part::PlanePy::PyMake,/*tp_new */
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

/// Methods structure of PlanePy
PyMethodDef PlanePy::Methods[] = {
    {"uIso",
        (PyCFunction) staticCallback_uIso,
        METH_VARARGS,
        "Builds the U isoparametric line of this plane"
    },
    {"vIso",
        (PyCFunction) staticCallback_vIso,
        METH_VARARGS,
        "Builds the V isoparametric line of this plane"
    },
    {NULL, NULL, 0, NULL}		/* Sentinel */
};



/// Attribute structure of PlanePy
PyGetSetDef PlanePy::GetterSetter[] = {
    {"Position",
        (getter) staticCallback_getPosition,
        (setter) staticCallback_setPosition, 
        "Returns the position point of this plane.",
        NULL
    },
    {"Axis",
        (getter) staticCallback_getAxis,
        (setter) staticCallback_setAxis, 
        "Returns the axis of this plane.",
        NULL
    },
    {NULL, NULL, NULL, NULL, NULL}		/* Sentinel */
};

// uIso() callback and implementer
// PyObject*  PlanePy::uIso(PyObject *args){};
// has to be implemented in PlanePyImp.cpp
PyObject * PlanePy::staticCallback_uIso (PyObject *self, PyObject *args)
{
    // test if twin object not allready deleted
    if (!static_cast<PyObjectBase*>(self)->isValid()) {
        PyErr_SetString(PyExc_ReferenceError, "This object is already deleted most likely through closing a document. This reference is no longer valid!");
        return NULL;
    }

    // test if object is set Const
    if (static_cast<PyObjectBase*>(self)->isConst()) {
        PyErr_SetString(PyExc_ReferenceError, "This object is immutable, you can not set any attribute or call a non const method");
        return NULL;
    }

    try { // catches all exceptions coming up from c++ and generate a python exception
        PyObject* ret = static_cast<PlanePy*>(self)->uIso(args);
        if (ret != 0)
            static_cast<PlanePy*>(self)->startNotify();
        return ret;
    }
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
    catch(const boost::filesystem::filesystem_error& e) // catch boost filesystem exception
    {
        std::string str;
        str += "File system exception thrown (";
        //str += e.who();
        //str += ", ";
        str += e.what();
        str += ")\n";
        Base::Console().Error(str.c_str());
        PyErr_SetString(Base::BaseExceptionFreeCADError,str.c_str());
        return NULL;
    }
    catch(const Py::Exception&)
    {
        // The exception text is already set
        return NULL;
    }
    catch(const char* e) // catch simple string exceptions
    {
        Base::Console().Error(e);
        PyErr_SetString(Base::BaseExceptionFreeCADError,e);
        return NULL;
    }
    // in debug not all exceptions will be catched to get the attention of the developer!
#ifndef DONT_CATCH_CXX_EXCEPTIONS 
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
    catch(...)  // catch the rest!
    {
        PyErr_SetString(Base::BaseExceptionFreeCADError,"Unknown C++ exception");
        return NULL;
    }
#endif
}

// vIso() callback and implementer
// PyObject*  PlanePy::vIso(PyObject *args){};
// has to be implemented in PlanePyImp.cpp
PyObject * PlanePy::staticCallback_vIso (PyObject *self, PyObject *args)
{
    // test if twin object not allready deleted
    if (!static_cast<PyObjectBase*>(self)->isValid()) {
        PyErr_SetString(PyExc_ReferenceError, "This object is already deleted most likely through closing a document. This reference is no longer valid!");
        return NULL;
    }

    // test if object is set Const
    if (static_cast<PyObjectBase*>(self)->isConst()) {
        PyErr_SetString(PyExc_ReferenceError, "This object is immutable, you can not set any attribute or call a non const method");
        return NULL;
    }

    try { // catches all exceptions coming up from c++ and generate a python exception
        PyObject* ret = static_cast<PlanePy*>(self)->vIso(args);
        if (ret != 0)
            static_cast<PlanePy*>(self)->startNotify();
        return ret;
    }
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
    catch(const boost::filesystem::filesystem_error& e) // catch boost filesystem exception
    {
        std::string str;
        str += "File system exception thrown (";
        //str += e.who();
        //str += ", ";
        str += e.what();
        str += ")\n";
        Base::Console().Error(str.c_str());
        PyErr_SetString(Base::BaseExceptionFreeCADError,str.c_str());
        return NULL;
    }
    catch(const Py::Exception&)
    {
        // The exception text is already set
        return NULL;
    }
    catch(const char* e) // catch simple string exceptions
    {
        Base::Console().Error(e);
        PyErr_SetString(Base::BaseExceptionFreeCADError,e);
        return NULL;
    }
    // in debug not all exceptions will be catched to get the attention of the developer!
#ifndef DONT_CATCH_CXX_EXCEPTIONS 
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
    catch(...)  // catch the rest!
    {
        PyErr_SetString(Base::BaseExceptionFreeCADError,"Unknown C++ exception");
        return NULL;
    }
#endif
}

// Position() callback and implementer
// PyObject*  PlanePy::Position(PyObject *args){};
// has to be implemented in PlanePyImp.cpp
PyObject * PlanePy::staticCallback_getPosition (PyObject *self, void * /*closure*/)
{
    if (!static_cast<PyObjectBase*>(self)->isValid()){
        PyErr_SetString(PyExc_ReferenceError, "This object is already deleted most likely through closing a document. This reference is no longer valid!");
        return NULL;
    }

    try {
        return Py::new_reference_to(static_cast<PlanePy*>(self)->getPosition());
    } catch (const Py::Exception&) {
        // The exception text is already set
        return NULL;
    } catch (...) {
        PyErr_SetString(Base::BaseExceptionFreeCADError, "Unknown exception while reading attribute 'Position' of object 'GeomPlane'");
        return NULL;
    }
}

int PlanePy::staticCallback_setPosition (PyObject *self, PyObject *value, void * /*closure*/)
{
    if (!static_cast<PyObjectBase*>(self)->isValid()){
        PyErr_SetString(PyExc_ReferenceError, "This object is already deleted most likely through closing a document. This reference is no longer valid!");
        return -1;
    }
    if (static_cast<PyObjectBase*>(self)->isConst()){
        PyErr_SetString(PyExc_ReferenceError, "This object is immutable, you can not set any attribute or call a method");
        return -1;
    }

    try {
        static_cast<PlanePy*>(self)->setPosition(Py::Object(value,false));
        return 0;
    } catch (const Py::Exception&) {
        // The exception text is already set
        return -1;
    } catch (...) {
        PyErr_SetString(Base::BaseExceptionFreeCADError, "Unknown exception while writing attribute 'Position' of object 'GeomPlane'");
        return -1;
    }
}

// Axis() callback and implementer
// PyObject*  PlanePy::Axis(PyObject *args){};
// has to be implemented in PlanePyImp.cpp
PyObject * PlanePy::staticCallback_getAxis (PyObject *self, void * /*closure*/)
{
    if (!static_cast<PyObjectBase*>(self)->isValid()){
        PyErr_SetString(PyExc_ReferenceError, "This object is already deleted most likely through closing a document. This reference is no longer valid!");
        return NULL;
    }

    try {
        return Py::new_reference_to(static_cast<PlanePy*>(self)->getAxis());
    } catch (const Py::Exception&) {
        // The exception text is already set
        return NULL;
    } catch (...) {
        PyErr_SetString(Base::BaseExceptionFreeCADError, "Unknown exception while reading attribute 'Axis' of object 'GeomPlane'");
        return NULL;
    }
}

int PlanePy::staticCallback_setAxis (PyObject *self, PyObject *value, void * /*closure*/)
{
    if (!static_cast<PyObjectBase*>(self)->isValid()){
        PyErr_SetString(PyExc_ReferenceError, "This object is already deleted most likely through closing a document. This reference is no longer valid!");
        return -1;
    }
    if (static_cast<PyObjectBase*>(self)->isConst()){
        PyErr_SetString(PyExc_ReferenceError, "This object is immutable, you can not set any attribute or call a method");
        return -1;
    }

    try {
        static_cast<PlanePy*>(self)->setAxis(Py::Object(value,false));
        return 0;
    } catch (const Py::Exception&) {
        // The exception text is already set
        return -1;
    } catch (...) {
        PyErr_SetString(Base::BaseExceptionFreeCADError, "Unknown exception while writing attribute 'Axis' of object 'GeomPlane'");
        return -1;
    }
}




//--------------------------------------------------------------------------
// Constructor
//--------------------------------------------------------------------------
PlanePy::PlanePy(GeomPlane *pcObject, PyTypeObject *T)
    : GeometrySurfacePy(static_cast<GeometrySurfacePy::PointerType>(pcObject), T)
{
    
}


//--------------------------------------------------------------------------
// destructor
//--------------------------------------------------------------------------
PlanePy::~PlanePy()                                // Everything handled in parent
{
}

//--------------------------------------------------------------------------
// PlanePy representation
//--------------------------------------------------------------------------
PyObject *PlanePy::_repr(void)
{
    return Py_BuildValue("s", representation().c_str());
}

//--------------------------------------------------------------------------
// PlanePy Attributes
//--------------------------------------------------------------------------
PyObject *PlanePy::_getattr(char *attr)				// __getattr__ function: note only need to handle new state
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
        return GeometrySurfacePy::_getattr(attr);
    }
    else
    {
        return rvalue;
    }
}

int PlanePy::_setattr(char *attr, PyObject *value) // __setattr__ function: note only need to handle new state
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

    return GeometrySurfacePy::_setattr(attr, value);
}

GeomPlane *PlanePy::getGeomPlanePtr(void) const
{
    return static_cast<GeomPlane *>(_pcTwinPointer);
}

#if 0
/* From here on come the methods you have to implement, but NOT in this module. Implement in PlanePyImp.cpp! This prototypes 
 * are just for convenience when you add a new method.
 */

PyObject *PlanePy::PyMake(struct _typeobject *, PyObject *, PyObject *)  // Python wrapper
{
    // create a new instance of PlanePy and the Twin object 
    return new PlanePy(new GeomPlane);
}

// constructor method
int PlanePy::PyInit(PyObject* /*args*/, PyObject* /*kwd*/)
{
    return 0;
}


// returns a string which represents the object e.g. when printed in python
std::string PlanePy::representation(void) const
{
    return std::string("<GeomPlane object>");
}

PyObject* PlanePy::uIso(PyObject *args)
{
    PyErr_SetString(PyExc_NotImplementedError, "Not yet implemented");
    return 0;
}

PyObject* PlanePy::vIso(PyObject *args)
{
    PyErr_SetString(PyExc_NotImplementedError, "Not yet implemented");
    return 0;
}



Py::Object PlanePy::getPosition(void) const
{
    //return Py::Object();
    throw Py::AttributeError("Not yet implemented");
}

void  PlanePy::setPosition(Py::Object arg)
{
    throw Py::AttributeError("Not yet implemented");
}

Py::Object PlanePy::getAxis(void) const
{
    //return Py::Object();
    throw Py::AttributeError("Not yet implemented");
}

void  PlanePy::setAxis(Py::Object arg)
{
    throw Py::AttributeError("Not yet implemented");
}

PyObject *PlanePy::getCustomAttributes(const char* /*attr*/) const
{
    return 0;
}

int PlanePy::setCustomAttributes(const char* /*attr*/, PyObject* /*obj*/)
{
    return 0; 
}
#endif



