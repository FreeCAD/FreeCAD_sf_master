
// This file is generated by src/Tools/generateTemaplates/templateClassPyExport.py out of the .XML file
// Every change you make here get lost at the next full rebuild!
// This File is normaly build as an include in TopoShapeShellPyImp.cpp! Its not intended to be in a project!

#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/exception.hpp>
#include <Base/PyObjectBase.h>
#include <Base/Console.h>
#include <Base/Exception.h>
#include <CXX/Objects.hxx>

using Base::streq;
using namespace Part;

/// Type structure of TopoShapeShellPy
PyTypeObject TopoShapeShellPy::Type = {
    PyObject_HEAD_INIT(&PyType_Type)
    0,                                                /*ob_size*/
    "Part.TopoShape",     /*tp_name*/
    sizeof(TopoShapeShellPy),                       /*tp_basicsize*/
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
    "Create a shell out of a list of faces",           /*tp_doc */
    0,                                                /*tp_traverse */
    0,                                                /*tp_clear */
    0,                                                /*tp_richcompare */
    0,                                                /*tp_weaklistoffset */
    0,                                                /*tp_iter */
    0,                                                /*tp_iternext */
    Part::TopoShapeShellPy::Methods,                     /*tp_methods */
    0,                                                /*tp_members */
    Part::TopoShapeShellPy::GetterSetter,                     /*tp_getset */
    &Part::TopoShapePy::Type,                        /*tp_base */
    0,                                                /*tp_dict */
    0,                                                /*tp_descr_get */
    0,                                                /*tp_descr_set */
    0,                                                /*tp_dictoffset */
    __PyInit,                                         /*tp_init */
    0,                                                /*tp_alloc */
    Part::TopoShapeShellPy::PyMake,/*tp_new */
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

/// Methods structure of TopoShapeShellPy
PyMethodDef TopoShapeShellPy::Methods[] = {
    {"add",
        (PyCFunction) staticCallback_add,
        METH_VARARGS,
        "Add a face to the shell."
    },
    {"getFreeEdges",
        (PyCFunction) staticCallback_getFreeEdges,
        METH_VARARGS,
        "Get free edges as compound."
    },
    {"getBadEdges",
        (PyCFunction) staticCallback_getBadEdges,
        METH_VARARGS,
        "Get bad edges as compound."
    },
    {"makeHalfSpace",
        (PyCFunction) staticCallback_makeHalfSpace,
        METH_VARARGS,
        "Make a half-space solid by this shell and a reference point."
    },
    {NULL, NULL, 0, NULL}		/* Sentinel */
};



/// Attribute structure of TopoShapeShellPy
PyGetSetDef TopoShapeShellPy::GetterSetter[] = {
    {"Mass",
        (getter) staticCallback_getMass,
        (setter) staticCallback_setMass, 
        "Returns the mass of the current system.",
        NULL
    },
    {"CenterOfMass",
        (getter) staticCallback_getCenterOfMass,
        (setter) staticCallback_setCenterOfMass, 
        "Returns the center of mass of the current system.\nIf the gravitational field is uniform, it is the center of gravity.\nThe coordinates returned for the center of mass are expressed in the\nabsolute Cartesian coordinate system.",
        NULL
    },
    {"MatrixOfInertia",
        (getter) staticCallback_getMatrixOfInertia,
        (setter) staticCallback_setMatrixOfInertia, 
        "Returns the matrix of inertia. It is a symmetrical matrix. \nThe coefficients of the matrix are the quadratic moments of \ninertia. \n\n | Ixx Ixy Ixz 0 | \n | Ixy Iyy Iyz 0 | \n | Ixz Iyz Izz 0 | \n | 0   0   0   1 | \n\nThe moments of inertia are denoted by Ixx, Iyy, Izz. \nThe products of inertia are denoted by Ixy, Ixz, Iyz. \nThe matrix of inertia is returned in the central coordinate \nsystem (G, Gx, Gy, Gz) where G is the centre of mass of the \nsystem and Gx, Gy, Gz the directions parallel to the X(1,0,0) \nY(0,1,0) Z(0,0,1) directions of the absolute cartesian \ncoordinate system.",
        NULL
    },
    {"StaticMoments",
        (getter) staticCallback_getStaticMoments,
        (setter) staticCallback_setStaticMoments, 
        "Returns Ix, Iy, Iz, the static moments of inertia of the \n current system; i.e. the moments of inertia about the \n three axes of the Cartesian coordinate system.",
        NULL
    },
    {"PrincipalProperties",
        (getter) staticCallback_getPrincipalProperties,
        (setter) staticCallback_setPrincipalProperties, 
        "Computes the principal properties of inertia of the current system. \n There is always a set of axes for which the products \n of inertia of a geometric system are equal to 0; i.e. the \n matrix of inertia of the system is diagonal. These axes \n are the principal axes of inertia. Their origin is \n coincident with the center of mass of the system. The \n associated moments are called the principal moments of inertia. \n This function computes the eigen values and the \n eigen vectors of the matrix of inertia of the system.",
        NULL
    },
    {NULL, NULL, NULL, NULL, NULL}		/* Sentinel */
};

// add() callback and implementer
// PyObject*  TopoShapeShellPy::add(PyObject *args){};
// has to be implemented in TopoShapeShellPyImp.cpp
PyObject * TopoShapeShellPy::staticCallback_add (PyObject *self, PyObject *args)
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
        PyObject* ret = static_cast<TopoShapeShellPy*>(self)->add(args);
        if (ret != 0)
            static_cast<TopoShapeShellPy*>(self)->startNotify();
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

// getFreeEdges() callback and implementer
// PyObject*  TopoShapeShellPy::getFreeEdges(PyObject *args){};
// has to be implemented in TopoShapeShellPyImp.cpp
PyObject * TopoShapeShellPy::staticCallback_getFreeEdges (PyObject *self, PyObject *args)
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
        PyObject* ret = static_cast<TopoShapeShellPy*>(self)->getFreeEdges(args);
        if (ret != 0)
            static_cast<TopoShapeShellPy*>(self)->startNotify();
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

// getBadEdges() callback and implementer
// PyObject*  TopoShapeShellPy::getBadEdges(PyObject *args){};
// has to be implemented in TopoShapeShellPyImp.cpp
PyObject * TopoShapeShellPy::staticCallback_getBadEdges (PyObject *self, PyObject *args)
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
        PyObject* ret = static_cast<TopoShapeShellPy*>(self)->getBadEdges(args);
        if (ret != 0)
            static_cast<TopoShapeShellPy*>(self)->startNotify();
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

// makeHalfSpace() callback and implementer
// PyObject*  TopoShapeShellPy::makeHalfSpace(PyObject *args){};
// has to be implemented in TopoShapeShellPyImp.cpp
PyObject * TopoShapeShellPy::staticCallback_makeHalfSpace (PyObject *self, PyObject *args)
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
        PyObject* ret = static_cast<TopoShapeShellPy*>(self)->makeHalfSpace(args);
        if (ret != 0)
            static_cast<TopoShapeShellPy*>(self)->startNotify();
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

// Mass() callback and implementer
// PyObject*  TopoShapeShellPy::Mass(PyObject *args){};
// has to be implemented in TopoShapeShellPyImp.cpp
PyObject * TopoShapeShellPy::staticCallback_getMass (PyObject *self, void * /*closure*/)
{
    if (!static_cast<PyObjectBase*>(self)->isValid()){
        PyErr_SetString(PyExc_ReferenceError, "This object is already deleted most likely through closing a document. This reference is no longer valid!");
        return NULL;
    }

    try {
        return Py::new_reference_to(static_cast<TopoShapeShellPy*>(self)->getMass());
    } catch (const Py::Exception&) {
        // The exception text is already set
        return NULL;
    } catch (...) {
        PyErr_SetString(Base::BaseExceptionFreeCADError, "Unknown exception while reading attribute 'Mass' of object 'TopoShape'");
        return NULL;
    }
}

int TopoShapeShellPy::staticCallback_setMass (PyObject *self, PyObject * /*value*/, void * /*closure*/)
{
    if (!static_cast<PyObjectBase*>(self)->isValid()){
        PyErr_SetString(PyExc_ReferenceError, "This object is already deleted most likely through closing a document. This reference is no longer valid!");
        return -1;
    }

    PyErr_SetString(PyExc_AttributeError, "Attribute 'Mass' of object 'TopoShape' is read-only");
    return -1;
}

// CenterOfMass() callback and implementer
// PyObject*  TopoShapeShellPy::CenterOfMass(PyObject *args){};
// has to be implemented in TopoShapeShellPyImp.cpp
PyObject * TopoShapeShellPy::staticCallback_getCenterOfMass (PyObject *self, void * /*closure*/)
{
    if (!static_cast<PyObjectBase*>(self)->isValid()){
        PyErr_SetString(PyExc_ReferenceError, "This object is already deleted most likely through closing a document. This reference is no longer valid!");
        return NULL;
    }

    try {
        return Py::new_reference_to(static_cast<TopoShapeShellPy*>(self)->getCenterOfMass());
    } catch (const Py::Exception&) {
        // The exception text is already set
        return NULL;
    } catch (...) {
        PyErr_SetString(Base::BaseExceptionFreeCADError, "Unknown exception while reading attribute 'CenterOfMass' of object 'TopoShape'");
        return NULL;
    }
}

int TopoShapeShellPy::staticCallback_setCenterOfMass (PyObject *self, PyObject * /*value*/, void * /*closure*/)
{
    if (!static_cast<PyObjectBase*>(self)->isValid()){
        PyErr_SetString(PyExc_ReferenceError, "This object is already deleted most likely through closing a document. This reference is no longer valid!");
        return -1;
    }

    PyErr_SetString(PyExc_AttributeError, "Attribute 'CenterOfMass' of object 'TopoShape' is read-only");
    return -1;
}

// MatrixOfInertia() callback and implementer
// PyObject*  TopoShapeShellPy::MatrixOfInertia(PyObject *args){};
// has to be implemented in TopoShapeShellPyImp.cpp
PyObject * TopoShapeShellPy::staticCallback_getMatrixOfInertia (PyObject *self, void * /*closure*/)
{
    if (!static_cast<PyObjectBase*>(self)->isValid()){
        PyErr_SetString(PyExc_ReferenceError, "This object is already deleted most likely through closing a document. This reference is no longer valid!");
        return NULL;
    }

    try {
        return Py::new_reference_to(static_cast<TopoShapeShellPy*>(self)->getMatrixOfInertia());
    } catch (const Py::Exception&) {
        // The exception text is already set
        return NULL;
    } catch (...) {
        PyErr_SetString(Base::BaseExceptionFreeCADError, "Unknown exception while reading attribute 'MatrixOfInertia' of object 'TopoShape'");
        return NULL;
    }
}

int TopoShapeShellPy::staticCallback_setMatrixOfInertia (PyObject *self, PyObject * /*value*/, void * /*closure*/)
{
    if (!static_cast<PyObjectBase*>(self)->isValid()){
        PyErr_SetString(PyExc_ReferenceError, "This object is already deleted most likely through closing a document. This reference is no longer valid!");
        return -1;
    }

    PyErr_SetString(PyExc_AttributeError, "Attribute 'MatrixOfInertia' of object 'TopoShape' is read-only");
    return -1;
}

// StaticMoments() callback and implementer
// PyObject*  TopoShapeShellPy::StaticMoments(PyObject *args){};
// has to be implemented in TopoShapeShellPyImp.cpp
PyObject * TopoShapeShellPy::staticCallback_getStaticMoments (PyObject *self, void * /*closure*/)
{
    if (!static_cast<PyObjectBase*>(self)->isValid()){
        PyErr_SetString(PyExc_ReferenceError, "This object is already deleted most likely through closing a document. This reference is no longer valid!");
        return NULL;
    }

    try {
        return Py::new_reference_to(static_cast<TopoShapeShellPy*>(self)->getStaticMoments());
    } catch (const Py::Exception&) {
        // The exception text is already set
        return NULL;
    } catch (...) {
        PyErr_SetString(Base::BaseExceptionFreeCADError, "Unknown exception while reading attribute 'StaticMoments' of object 'TopoShape'");
        return NULL;
    }
}

int TopoShapeShellPy::staticCallback_setStaticMoments (PyObject *self, PyObject * /*value*/, void * /*closure*/)
{
    if (!static_cast<PyObjectBase*>(self)->isValid()){
        PyErr_SetString(PyExc_ReferenceError, "This object is already deleted most likely through closing a document. This reference is no longer valid!");
        return -1;
    }

    PyErr_SetString(PyExc_AttributeError, "Attribute 'StaticMoments' of object 'TopoShape' is read-only");
    return -1;
}

// PrincipalProperties() callback and implementer
// PyObject*  TopoShapeShellPy::PrincipalProperties(PyObject *args){};
// has to be implemented in TopoShapeShellPyImp.cpp
PyObject * TopoShapeShellPy::staticCallback_getPrincipalProperties (PyObject *self, void * /*closure*/)
{
    if (!static_cast<PyObjectBase*>(self)->isValid()){
        PyErr_SetString(PyExc_ReferenceError, "This object is already deleted most likely through closing a document. This reference is no longer valid!");
        return NULL;
    }

    try {
        return Py::new_reference_to(static_cast<TopoShapeShellPy*>(self)->getPrincipalProperties());
    } catch (const Py::Exception&) {
        // The exception text is already set
        return NULL;
    } catch (...) {
        PyErr_SetString(Base::BaseExceptionFreeCADError, "Unknown exception while reading attribute 'PrincipalProperties' of object 'TopoShape'");
        return NULL;
    }
}

int TopoShapeShellPy::staticCallback_setPrincipalProperties (PyObject *self, PyObject * /*value*/, void * /*closure*/)
{
    if (!static_cast<PyObjectBase*>(self)->isValid()){
        PyErr_SetString(PyExc_ReferenceError, "This object is already deleted most likely through closing a document. This reference is no longer valid!");
        return -1;
    }

    PyErr_SetString(PyExc_AttributeError, "Attribute 'PrincipalProperties' of object 'TopoShape' is read-only");
    return -1;
}




//--------------------------------------------------------------------------
// Constructor
//--------------------------------------------------------------------------
TopoShapeShellPy::TopoShapeShellPy(TopoShape *pcObject, PyTypeObject *T)
    : TopoShapePy(static_cast<TopoShapePy::PointerType>(pcObject), T)
{
    
}


//--------------------------------------------------------------------------
// destructor
//--------------------------------------------------------------------------
TopoShapeShellPy::~TopoShapeShellPy()                                // Everything handled in parent
{
}

//--------------------------------------------------------------------------
// TopoShapeShellPy representation
//--------------------------------------------------------------------------
PyObject *TopoShapeShellPy::_repr(void)
{
    return Py_BuildValue("s", representation().c_str());
}

//--------------------------------------------------------------------------
// TopoShapeShellPy Attributes
//--------------------------------------------------------------------------
PyObject *TopoShapeShellPy::_getattr(char *attr)				// __getattr__ function: note only need to handle new state
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
        return TopoShapePy::_getattr(attr);
    }
    else
    {
        return rvalue;
    }
}

int TopoShapeShellPy::_setattr(char *attr, PyObject *value) // __setattr__ function: note only need to handle new state
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

    return TopoShapePy::_setattr(attr, value);
}

TopoShape *TopoShapeShellPy::getTopoShapePtr(void) const
{
    return static_cast<TopoShape *>(_pcTwinPointer);
}

#if 0
/* From here on come the methods you have to implement, but NOT in this module. Implement in TopoShapeShellPyImp.cpp! This prototypes 
 * are just for convenience when you add a new method.
 */

PyObject *TopoShapeShellPy::PyMake(struct _typeobject *, PyObject *, PyObject *)  // Python wrapper
{
    // create a new instance of TopoShapeShellPy and the Twin object 
    return new TopoShapeShellPy(new TopoShape);
}

// constructor method
int TopoShapeShellPy::PyInit(PyObject* /*args*/, PyObject* /*kwd*/)
{
    return 0;
}


// returns a string which represents the object e.g. when printed in python
std::string TopoShapeShellPy::representation(void) const
{
    return std::string("<TopoShape object>");
}

PyObject* TopoShapeShellPy::add(PyObject *args)
{
    PyErr_SetString(PyExc_NotImplementedError, "Not yet implemented");
    return 0;
}

PyObject* TopoShapeShellPy::getFreeEdges(PyObject *args)
{
    PyErr_SetString(PyExc_NotImplementedError, "Not yet implemented");
    return 0;
}

PyObject* TopoShapeShellPy::getBadEdges(PyObject *args)
{
    PyErr_SetString(PyExc_NotImplementedError, "Not yet implemented");
    return 0;
}

PyObject* TopoShapeShellPy::makeHalfSpace(PyObject *args)
{
    PyErr_SetString(PyExc_NotImplementedError, "Not yet implemented");
    return 0;
}



Py::Object TopoShapeShellPy::getMass(void) const
{
    //return Py::Object();
    throw Py::AttributeError("Not yet implemented");
}

Py::Object TopoShapeShellPy::getCenterOfMass(void) const
{
    //return Py::Object();
    throw Py::AttributeError("Not yet implemented");
}

Py::Object TopoShapeShellPy::getMatrixOfInertia(void) const
{
    //return Py::Object();
    throw Py::AttributeError("Not yet implemented");
}

Py::Object TopoShapeShellPy::getStaticMoments(void) const
{
    //return Py::Object();
    throw Py::AttributeError("Not yet implemented");
}

Py::Dict TopoShapeShellPy::getPrincipalProperties(void) const
{
    //return Py::Dict();
    throw Py::AttributeError("Not yet implemented");
}

PyObject *TopoShapeShellPy::getCustomAttributes(const char* /*attr*/) const
{
    return 0;
}

int TopoShapeShellPy::setCustomAttributes(const char* /*attr*/, PyObject* /*obj*/)
{
    return 0; 
}
#endif



