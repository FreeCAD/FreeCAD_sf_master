
// This file is generated by src/Tools/generateTemaplates/templateClassPyExport.py out of the .XML file
// Every change you make here get lost at the next full rebuild!
// This File is normaly build as an include in WaypointPyImp.cpp! Its not intended to be in a project!

#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/exception.hpp>
#include <Base/PyObjectBase.h>
#include <Base/Console.h>
#include <Base/Exception.h>
#include <CXX/Objects.hxx>

using Base::streq;
using namespace Robot;

/// Type structure of WaypointPy
PyTypeObject WaypointPy::Type = {
    PyObject_HEAD_INIT(&PyType_Type)
    0,                                                /*ob_size*/
    "Robot.Waypoint",     /*tp_name*/
    sizeof(WaypointPy),                       /*tp_basicsize*/
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
    "Waypoint class",           /*tp_doc */
    0,                                                /*tp_traverse */
    0,                                                /*tp_clear */
    0,                                                /*tp_richcompare */
    0,                                                /*tp_weaklistoffset */
    0,                                                /*tp_iter */
    0,                                                /*tp_iternext */
    Robot::WaypointPy::Methods,                     /*tp_methods */
    0,                                                /*tp_members */
    Robot::WaypointPy::GetterSetter,                     /*tp_getset */
    &Base::PersistencePy::Type,                        /*tp_base */
    0,                                                /*tp_dict */
    0,                                                /*tp_descr_get */
    0,                                                /*tp_descr_set */
    0,                                                /*tp_dictoffset */
    __PyInit,                                         /*tp_init */
    0,                                                /*tp_alloc */
    Robot::WaypointPy::PyMake,/*tp_new */
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

/// Methods structure of WaypointPy
PyMethodDef WaypointPy::Methods[] = {
    {NULL, NULL, 0, NULL}		/* Sentinel */
};



/// Attribute structure of WaypointPy
PyGetSetDef WaypointPy::GetterSetter[] = {
    {"Name",
        (getter) staticCallback_getName,
        (setter) staticCallback_setName, 
        "Name of the waypoint",
        NULL
    },
    {"Type",
        (getter) staticCallback_getType,
        (setter) staticCallback_setType, 
        "Type of the waypoint[PTP|LIN|CIRC|WAIT]",
        NULL
    },
    {"Pos",
        (getter) staticCallback_getPos,
        (setter) staticCallback_setPos, 
        "End position (destination) of the the waypoint",
        NULL
    },
    {"Cont",
        (getter) staticCallback_getCont,
        (setter) staticCallback_setCont, 
        "Control the continuity to the next waypoint in the trajectory",
        NULL
    },
    {"Velocity",
        (getter) staticCallback_getVelocity,
        (setter) staticCallback_setVelocity, 
        "Control the velocity to the next waypoint in the trajectory\nIn Case of PTP 0-100% Axis speed\nIn Case of LIN m/s\nIn Case of WAIT s wait time\n",
        NULL
    },
    {"Tool",
        (getter) staticCallback_getTool,
        (setter) staticCallback_setTool, 
        "descripe which tool frame to use for that point",
        NULL
    },
    {"Base",
        (getter) staticCallback_getBase,
        (setter) staticCallback_setBase, 
        "descripe which Base frame to use for that point",
        NULL
    },
    {NULL, NULL, NULL, NULL, NULL}		/* Sentinel */
};

// Name() callback and implementer
// PyObject*  WaypointPy::Name(PyObject *args){};
// has to be implemented in WaypointPyImp.cpp
PyObject * WaypointPy::staticCallback_getName (PyObject *self, void * /*closure*/)
{
    if (!static_cast<PyObjectBase*>(self)->isValid()){
        PyErr_SetString(PyExc_ReferenceError, "This object is already deleted most likely through closing a document. This reference is no longer valid!");
        return NULL;
    }

    try {
        return Py::new_reference_to(static_cast<WaypointPy*>(self)->getName());
    } catch (const Py::Exception&) {
        // The exception text is already set
        return NULL;
    } catch (...) {
        PyErr_SetString(Base::BaseExceptionFreeCADError, "Unknown exception while reading attribute 'Name' of object 'Waypoint'");
        return NULL;
    }
}

int WaypointPy::staticCallback_setName (PyObject *self, PyObject *value, void * /*closure*/)
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
        static_cast<WaypointPy*>(self)->setName(Py::String(value,false));
        return 0;
    } catch (const Py::Exception&) {
        // The exception text is already set
        return -1;
    } catch (...) {
        PyErr_SetString(Base::BaseExceptionFreeCADError, "Unknown exception while writing attribute 'Name' of object 'Waypoint'");
        return -1;
    }
}

// Type() callback and implementer
// PyObject*  WaypointPy::Type(PyObject *args){};
// has to be implemented in WaypointPyImp.cpp
PyObject * WaypointPy::staticCallback_getType (PyObject *self, void * /*closure*/)
{
    if (!static_cast<PyObjectBase*>(self)->isValid()){
        PyErr_SetString(PyExc_ReferenceError, "This object is already deleted most likely through closing a document. This reference is no longer valid!");
        return NULL;
    }

    try {
        return Py::new_reference_to(static_cast<WaypointPy*>(self)->getType());
    } catch (const Py::Exception&) {
        // The exception text is already set
        return NULL;
    } catch (...) {
        PyErr_SetString(Base::BaseExceptionFreeCADError, "Unknown exception while reading attribute 'Type' of object 'Waypoint'");
        return NULL;
    }
}

int WaypointPy::staticCallback_setType (PyObject *self, PyObject *value, void * /*closure*/)
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
        static_cast<WaypointPy*>(self)->setType(Py::String(value,false));
        return 0;
    } catch (const Py::Exception&) {
        // The exception text is already set
        return -1;
    } catch (...) {
        PyErr_SetString(Base::BaseExceptionFreeCADError, "Unknown exception while writing attribute 'Type' of object 'Waypoint'");
        return -1;
    }
}

// Pos() callback and implementer
// PyObject*  WaypointPy::Pos(PyObject *args){};
// has to be implemented in WaypointPyImp.cpp
PyObject * WaypointPy::staticCallback_getPos (PyObject *self, void * /*closure*/)
{
    if (!static_cast<PyObjectBase*>(self)->isValid()){
        PyErr_SetString(PyExc_ReferenceError, "This object is already deleted most likely through closing a document. This reference is no longer valid!");
        return NULL;
    }

    try {
        return Py::new_reference_to(static_cast<WaypointPy*>(self)->getPos());
    } catch (const Py::Exception&) {
        // The exception text is already set
        return NULL;
    } catch (...) {
        PyErr_SetString(Base::BaseExceptionFreeCADError, "Unknown exception while reading attribute 'Pos' of object 'Waypoint'");
        return NULL;
    }
}

int WaypointPy::staticCallback_setPos (PyObject *self, PyObject *value, void * /*closure*/)
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
        static_cast<WaypointPy*>(self)->setPos(Py::Object(value,false));
        return 0;
    } catch (const Py::Exception&) {
        // The exception text is already set
        return -1;
    } catch (...) {
        PyErr_SetString(Base::BaseExceptionFreeCADError, "Unknown exception while writing attribute 'Pos' of object 'Waypoint'");
        return -1;
    }
}

// Cont() callback and implementer
// PyObject*  WaypointPy::Cont(PyObject *args){};
// has to be implemented in WaypointPyImp.cpp
PyObject * WaypointPy::staticCallback_getCont (PyObject *self, void * /*closure*/)
{
    if (!static_cast<PyObjectBase*>(self)->isValid()){
        PyErr_SetString(PyExc_ReferenceError, "This object is already deleted most likely through closing a document. This reference is no longer valid!");
        return NULL;
    }

    try {
        return Py::new_reference_to(static_cast<WaypointPy*>(self)->getCont());
    } catch (const Py::Exception&) {
        // The exception text is already set
        return NULL;
    } catch (...) {
        PyErr_SetString(Base::BaseExceptionFreeCADError, "Unknown exception while reading attribute 'Cont' of object 'Waypoint'");
        return NULL;
    }
}

int WaypointPy::staticCallback_setCont (PyObject *self, PyObject *value, void * /*closure*/)
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
        static_cast<WaypointPy*>(self)->setCont(Py::Boolean(value,false));
        return 0;
    } catch (const Py::Exception&) {
        // The exception text is already set
        return -1;
    } catch (...) {
        PyErr_SetString(Base::BaseExceptionFreeCADError, "Unknown exception while writing attribute 'Cont' of object 'Waypoint'");
        return -1;
    }
}

// Velocity() callback and implementer
// PyObject*  WaypointPy::Velocity(PyObject *args){};
// has to be implemented in WaypointPyImp.cpp
PyObject * WaypointPy::staticCallback_getVelocity (PyObject *self, void * /*closure*/)
{
    if (!static_cast<PyObjectBase*>(self)->isValid()){
        PyErr_SetString(PyExc_ReferenceError, "This object is already deleted most likely through closing a document. This reference is no longer valid!");
        return NULL;
    }

    try {
        return Py::new_reference_to(static_cast<WaypointPy*>(self)->getVelocity());
    } catch (const Py::Exception&) {
        // The exception text is already set
        return NULL;
    } catch (...) {
        PyErr_SetString(Base::BaseExceptionFreeCADError, "Unknown exception while reading attribute 'Velocity' of object 'Waypoint'");
        return NULL;
    }
}

int WaypointPy::staticCallback_setVelocity (PyObject *self, PyObject *value, void * /*closure*/)
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
        static_cast<WaypointPy*>(self)->setVelocity(Py::Float(PyNumber_Float(value),true));
        return 0;
    } catch (const Py::Exception&) {
        // The exception text is already set
        return -1;
    } catch (...) {
        PyErr_SetString(Base::BaseExceptionFreeCADError, "Unknown exception while writing attribute 'Velocity' of object 'Waypoint'");
        return -1;
    }
}

// Tool() callback and implementer
// PyObject*  WaypointPy::Tool(PyObject *args){};
// has to be implemented in WaypointPyImp.cpp
PyObject * WaypointPy::staticCallback_getTool (PyObject *self, void * /*closure*/)
{
    if (!static_cast<PyObjectBase*>(self)->isValid()){
        PyErr_SetString(PyExc_ReferenceError, "This object is already deleted most likely through closing a document. This reference is no longer valid!");
        return NULL;
    }

    try {
        return Py::new_reference_to(static_cast<WaypointPy*>(self)->getTool());
    } catch (const Py::Exception&) {
        // The exception text is already set
        return NULL;
    } catch (...) {
        PyErr_SetString(Base::BaseExceptionFreeCADError, "Unknown exception while reading attribute 'Tool' of object 'Waypoint'");
        return NULL;
    }
}

int WaypointPy::staticCallback_setTool (PyObject *self, PyObject *value, void * /*closure*/)
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
        static_cast<WaypointPy*>(self)->setTool(Py::Int(value,false));
        return 0;
    } catch (const Py::Exception&) {
        // The exception text is already set
        return -1;
    } catch (...) {
        PyErr_SetString(Base::BaseExceptionFreeCADError, "Unknown exception while writing attribute 'Tool' of object 'Waypoint'");
        return -1;
    }
}

// Base() callback and implementer
// PyObject*  WaypointPy::Base(PyObject *args){};
// has to be implemented in WaypointPyImp.cpp
PyObject * WaypointPy::staticCallback_getBase (PyObject *self, void * /*closure*/)
{
    if (!static_cast<PyObjectBase*>(self)->isValid()){
        PyErr_SetString(PyExc_ReferenceError, "This object is already deleted most likely through closing a document. This reference is no longer valid!");
        return NULL;
    }

    try {
        return Py::new_reference_to(static_cast<WaypointPy*>(self)->getBase());
    } catch (const Py::Exception&) {
        // The exception text is already set
        return NULL;
    } catch (...) {
        PyErr_SetString(Base::BaseExceptionFreeCADError, "Unknown exception while reading attribute 'Base' of object 'Waypoint'");
        return NULL;
    }
}

int WaypointPy::staticCallback_setBase (PyObject *self, PyObject *value, void * /*closure*/)
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
        static_cast<WaypointPy*>(self)->setBase(Py::Int(value,false));
        return 0;
    } catch (const Py::Exception&) {
        // The exception text is already set
        return -1;
    } catch (...) {
        PyErr_SetString(Base::BaseExceptionFreeCADError, "Unknown exception while writing attribute 'Base' of object 'Waypoint'");
        return -1;
    }
}




//--------------------------------------------------------------------------
// Constructor
//--------------------------------------------------------------------------
WaypointPy::WaypointPy(Waypoint *pcObject, PyTypeObject *T)
    : PersistencePy(static_cast<PersistencePy::PointerType>(pcObject), T)
{
    
}


//--------------------------------------------------------------------------
// destructor
//--------------------------------------------------------------------------
WaypointPy::~WaypointPy()                                // Everything handled in parent
{
    // delete the handled object when the PyObject dies
    WaypointPy::PointerType ptr = static_cast<WaypointPy::PointerType>(_pcTwinPointer);
    delete ptr;
}

//--------------------------------------------------------------------------
// WaypointPy representation
//--------------------------------------------------------------------------
PyObject *WaypointPy::_repr(void)
{
    return Py_BuildValue("s", representation().c_str());
}

//--------------------------------------------------------------------------
// WaypointPy Attributes
//--------------------------------------------------------------------------
PyObject *WaypointPy::_getattr(char *attr)				// __getattr__ function: note only need to handle new state
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
        return PersistencePy::_getattr(attr);
    }
    else
    {
        return rvalue;
    }
}

int WaypointPy::_setattr(char *attr, PyObject *value) // __setattr__ function: note only need to handle new state
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

    return PersistencePy::_setattr(attr, value);
}

Waypoint *WaypointPy::getWaypointPtr(void) const
{
    return static_cast<Waypoint *>(_pcTwinPointer);
}

#if 0
/* From here on come the methods you have to implement, but NOT in this module. Implement in WaypointPyImp.cpp! This prototypes 
 * are just for convenience when you add a new method.
 */

PyObject *WaypointPy::PyMake(struct _typeobject *, PyObject *, PyObject *)  // Python wrapper
{
    // create a new instance of WaypointPy and the Twin object 
    return new WaypointPy(new Waypoint);
}

// constructor method
int WaypointPy::PyInit(PyObject* /*args*/, PyObject* /*kwd*/)
{
    return 0;
}


// returns a string which represents the object e.g. when printed in python
std::string WaypointPy::representation(void) const
{
    return std::string("<Waypoint object>");
}



Py::String WaypointPy::getName(void) const
{
    //return Py::String();
    throw Py::AttributeError("Not yet implemented");
}

void  WaypointPy::setName(Py::String arg)
{
    throw Py::AttributeError("Not yet implemented");
}

Py::String WaypointPy::getType(void) const
{
    //return Py::String();
    throw Py::AttributeError("Not yet implemented");
}

void  WaypointPy::setType(Py::String arg)
{
    throw Py::AttributeError("Not yet implemented");
}

Py::Object WaypointPy::getPos(void) const
{
    //return Py::Object();
    throw Py::AttributeError("Not yet implemented");
}

void  WaypointPy::setPos(Py::Object arg)
{
    throw Py::AttributeError("Not yet implemented");
}

Py::Boolean WaypointPy::getCont(void) const
{
    //return Py::Boolean();
    throw Py::AttributeError("Not yet implemented");
}

void  WaypointPy::setCont(Py::Boolean arg)
{
    throw Py::AttributeError("Not yet implemented");
}

Py::Float WaypointPy::getVelocity(void) const
{
    //return Py::Float();
    throw Py::AttributeError("Not yet implemented");
}

void  WaypointPy::setVelocity(Py::Float arg)
{
    throw Py::AttributeError("Not yet implemented");
}

Py::Int WaypointPy::getTool(void) const
{
    //return Py::Int();
    throw Py::AttributeError("Not yet implemented");
}

void  WaypointPy::setTool(Py::Int arg)
{
    throw Py::AttributeError("Not yet implemented");
}

Py::Int WaypointPy::getBase(void) const
{
    //return Py::Int();
    throw Py::AttributeError("Not yet implemented");
}

void  WaypointPy::setBase(Py::Int arg)
{
    throw Py::AttributeError("Not yet implemented");
}

PyObject *WaypointPy::getCustomAttributes(const char* /*attr*/) const
{
    return 0;
}

int WaypointPy::setCustomAttributes(const char* /*attr*/, PyObject* /*obj*/)
{
    return 0; 
}
#endif



