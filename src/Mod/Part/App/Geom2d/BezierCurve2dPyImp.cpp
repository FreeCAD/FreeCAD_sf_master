/***************************************************************************
 *   Copyright (c) 2008 Werner Mayer <wmayer[at]users.sourceforge.net>     *
 *                                                                         *
 *   This file is part of the FreeCAD CAx development system.              *
 *                                                                         *
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Library General Public           *
 *   License as published by the Free Software Foundation; either          *
 *   version 2 of the License, or (at your option) any later version.      *
 *                                                                         *
 *   This library  is distributed in the hope that it will be useful,      *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Library General Public License for more details.                  *
 *                                                                         *
 *   You should have received a copy of the GNU Library General Public     *
 *   License along with this library; see the file COPYING.LIB. If not,    *
 *   write to the Free Software Foundation, Inc., 59 Temple Place,         *
 *   Suite 330, Boston, MA  02111-1307, USA                                *
 *                                                                         *
 ***************************************************************************/


#include "PreCompiled.h"
#ifndef _PreComp_
# include <Geom_BezierCurve.hxx>
# include <gp_Pnt.hxx>
# include <TColStd_Array1OfReal.hxx>
# include <TColgp_Array1OfPnt.hxx>
#endif

#include <Base/VectorPy.h>
#include <Base/GeometryPyCXX.h>

#include <Mod/Part/App/OCCError.h>
#include <Mod/Part/App/Geometry2d.h>
#include <Mod/Part/App/Geom2d/BezierCurve2dPy.h>
#include <Mod/Part/App/Geom2d/BezierCurve2dPy.cpp>

using namespace Part;

// returns a string which represents the object e.g. when printed in python
std::string BezierCurve2dPy::representation(void) const
{
    return "<BezierCurve object>";
}

PyObject *BezierCurve2dPy::PyMake(struct _typeobject *, PyObject *, PyObject *)  // Python wrapper
{
    // create a new instance of BezierCurve2dPy and the Twin object
    return new BezierCurve2dPy(new Geom2dBezierCurve);
}

// constructor method
int BezierCurve2dPy::PyInit(PyObject* /*args*/, PyObject* /*kwd*/)
{
    return 0;
}
#if 0
PyObject* BezierCurve2dPy::isRational(PyObject *args)
{
    if (!PyArg_ParseTuple(args, ""))
        return 0;
    Handle_Geom_BezierCurve curve = Handle_Geom_BezierCurve::DownCast
        (getGeometryPtr()->handle());
    Standard_Boolean val = curve->IsRational();
    return PyBool_FromLong(val ? 1 : 0);
}

PyObject* BezierCurve2dPy::isPeriodic(PyObject *args)
{
    if (!PyArg_ParseTuple(args, ""))
        return 0;
    Handle_Geom_BezierCurve curve = Handle_Geom_BezierCurve::DownCast
        (getGeometryPtr()->handle());
    Standard_Boolean val = curve->IsPeriodic();
    return PyBool_FromLong(val ? 1 : 0);
}

PyObject* BezierCurve2dPy::isClosed(PyObject *args)
{
    if (!PyArg_ParseTuple(args, ""))
        return 0;
    Handle_Geom_BezierCurve curve = Handle_Geom_BezierCurve::DownCast
        (getGeometryPtr()->handle());
    Standard_Boolean val = curve->IsClosed();
    return PyBool_FromLong(val ? 1 : 0);
}

PyObject* BezierCurve2dPy::increase(PyObject * args)
{
    int degree;
    if (!PyArg_ParseTuple(args, "i", &degree))
        return 0;
    Handle_Geom_BezierCurve curve = Handle_Geom_BezierCurve::DownCast
        (getGeometryPtr()->handle());
    curve->Increase(degree);
    Py_Return;
}

PyObject* BezierCurve2dPy::insertPoleAfter(PyObject * args)
{
    int index;
    double weight=1.0;
    PyObject* p;
    if (!PyArg_ParseTuple(args, "iO!|d", &index, &(Base::VectorPy::Type), &p, &weight))
        return 0;
    Base::Vector3d vec = static_cast<Base::VectorPy*>(p)->value();
    gp_Pnt pnt(vec.x, vec.y, vec.z);
    try {
        Handle_Geom_BezierCurve curve = Handle_Geom_BezierCurve::DownCast
            (getGeometryPtr()->handle());
        curve->InsertPoleAfter(index,pnt,weight);
        Py_Return;
    }
    catch (Standard_Failure) {
        Handle_Standard_Failure e = Standard_Failure::Caught();
        PyErr_SetString(PartExceptionOCCError, e->GetMessageString());
        return 0;
    }
}

PyObject* BezierCurve2dPy::insertPoleBefore(PyObject * args)
{
    int index;
    double weight=1.0;
    PyObject* p;
    if (!PyArg_ParseTuple(args, "iO!|d", &index, &(Base::VectorPy::Type), &p, &weight))
        return 0;
    Base::Vector3d vec = static_cast<Base::VectorPy*>(p)->value();
    gp_Pnt pnt(vec.x, vec.y, vec.z);
    try {
        Handle_Geom_BezierCurve curve = Handle_Geom_BezierCurve::DownCast
            (getGeometryPtr()->handle());
        curve->InsertPoleBefore(index,pnt,weight);
        Py_Return;
    }
    catch (Standard_Failure) {
        Handle_Standard_Failure e = Standard_Failure::Caught();
        PyErr_SetString(PartExceptionOCCError, e->GetMessageString());
        return 0;
    }
}

PyObject* BezierCurve2dPy::removePole(PyObject * args)
{
    int index;
    if (!PyArg_ParseTuple(args, "i", &index))
        return 0;
    try {
        Handle_Geom_BezierCurve curve = Handle_Geom_BezierCurve::DownCast
            (getGeometryPtr()->handle());
        curve->RemovePole(index);
        Py_Return;
    }
    catch (Standard_Failure) {
        Handle_Standard_Failure e = Standard_Failure::Caught();
        PyErr_SetString(PartExceptionOCCError, e->GetMessageString());
        return 0;
    }
}

PyObject* BezierCurve2dPy::segment(PyObject * args)
{
    double u1,u2;
    if (!PyArg_ParseTuple(args, "dd", &u1,&u2))
        return 0;
    try {
        Handle_Geom_BezierCurve curve = Handle_Geom_BezierCurve::DownCast
            (getGeometryPtr()->handle());
        curve->Segment(u1,u2);
        Py_Return;
    }
    catch (Standard_Failure) {
        Handle_Standard_Failure e = Standard_Failure::Caught();
        PyErr_SetString(PartExceptionOCCError, e->GetMessageString());
        return 0;
    }
}

PyObject* BezierCurve2dPy::setPole(PyObject * args)
{
    int index;
    double weight=-1.0;
    PyObject* p;
    if (!PyArg_ParseTuple(args, "iO!|d", &index, &(Base::VectorPy::Type), &p, &weight))
        return 0;
    Base::Vector3d vec = static_cast<Base::VectorPy*>(p)->value();
    gp_Pnt pnt(vec.x, vec.y, vec.z);
    try {
        Handle_Geom_BezierCurve curve = Handle_Geom_BezierCurve::DownCast
            (getGeometryPtr()->handle());
        if (weight < 0.0)
            curve->SetPole(index,pnt);
        else
            curve->SetPole(index,pnt,weight);
        Py_Return;
    }
    catch (Standard_Failure) {
        Handle_Standard_Failure e = Standard_Failure::Caught();
        PyErr_SetString(PartExceptionOCCError, e->GetMessageString());
        return 0;
    }
}

PyObject* BezierCurve2dPy::getPole(PyObject * args)
{
    int index;
    if (!PyArg_ParseTuple(args, "i", &index))
        return 0;
    try {
        Handle_Geom_BezierCurve curve = Handle_Geom_BezierCurve::DownCast
            (getGeometryPtr()->handle());
        Standard_OutOfRange_Raise_if
            (index < 1 || index > curve->NbPoles(), "Pole index out of range");
        gp_Pnt pnt = curve->Pole(index);
        Base::VectorPy* vec = new Base::VectorPy(Base::Vector3d(
            pnt.X(), pnt.Y(), pnt.Z()));
        return vec;
    }
    catch (Standard_Failure) {
        Handle_Standard_Failure e = Standard_Failure::Caught();
        PyErr_SetString(PartExceptionOCCError, e->GetMessageString());
        return 0;
    }
}

PyObject* BezierCurve2dPy::getPoles(PyObject * args)
{
    if (!PyArg_ParseTuple(args, ""))
        return 0;
    try {
        Handle_Geom_BezierCurve curve = Handle_Geom_BezierCurve::DownCast
            (getGeometryPtr()->handle());
        TColgp_Array1OfPnt p(1,curve->NbPoles());
        curve->Poles(p);
        Py::List poles;
        for (Standard_Integer i=p.Lower(); i<=p.Upper(); i++) {
            gp_Pnt pnt = p(i);
            Base::VectorPy* vec = new Base::VectorPy(Base::Vector3d(
                pnt.X(), pnt.Y(), pnt.Z()));
            poles.append(Py::Object(vec));
        }
        return Py::new_reference_to(poles);
    }
    catch (Standard_Failure) {
        Handle_Standard_Failure e = Standard_Failure::Caught();
        PyErr_SetString(PartExceptionOCCError, e->GetMessageString());
        return 0;
    }
}

PyObject* BezierCurve2dPy::setPoles(PyObject * args)
{
    PyObject* plist;
    if (!PyArg_ParseTuple(args, "O", &plist))
        return 0;
    try {
        Py::Sequence list(plist);
        TColgp_Array1OfPnt poles(1,list.size());
        int index = poles.Lower();
        for (Py::Sequence::iterator it = list.begin(); it != list.end(); ++it) {
            Py::Vector v(*it);
            Base::Vector3d pole = v.toVector();
            poles.SetValue(index++, gp_Pnt(pole.x,pole.y,pole.z));
        }

        Handle_Geom_BezierCurve bezier = new Geom_BezierCurve(poles);
        this->getGeomBezierCurvePtr()->setHandle(bezier);
        Py_Return;
    }
    catch (Standard_Failure) {
        Handle_Standard_Failure e = Standard_Failure::Caught();
        PyErr_SetString(PartExceptionOCCError, e->GetMessageString());
        return 0;
    }
}

PyObject* BezierCurve2dPy::setWeight(PyObject * args)
{
    int index;
    double weight;
    if (!PyArg_ParseTuple(args, "id", &index,&weight))
        return 0;
    try {
        Handle_Geom_BezierCurve curve = Handle_Geom_BezierCurve::DownCast
            (getGeometryPtr()->handle());
        curve->SetWeight(index,weight);
        Py_Return;
    }
    catch (Standard_Failure) {
        Handle_Standard_Failure e = Standard_Failure::Caught();
        PyErr_SetString(PartExceptionOCCError, e->GetMessageString());
        return 0;
    }
}

PyObject* BezierCurve2dPy::getWeight(PyObject * args)
{
    int index;
    if (!PyArg_ParseTuple(args, "i", &index))
        return 0;
    try {
        Handle_Geom_BezierCurve curve = Handle_Geom_BezierCurve::DownCast
            (getGeometryPtr()->handle());
        Standard_OutOfRange_Raise_if
            (index < 1 || index > curve->NbPoles() , "Weight index out of range");
        double weight = curve->Weight(index);
        return Py_BuildValue("d", weight);
    }
    catch (Standard_Failure) {
        Handle_Standard_Failure e = Standard_Failure::Caught();
        PyErr_SetString(PartExceptionOCCError, e->GetMessageString());
        return 0;
    }
}

PyObject* BezierCurve2dPy::getWeights(PyObject * args)
{
    if (!PyArg_ParseTuple(args, ""))
        return 0;
    try {
        Handle_Geom_BezierCurve curve = Handle_Geom_BezierCurve::DownCast
            (getGeometryPtr()->handle());
        TColStd_Array1OfReal w(1,curve->NbPoles());
        curve->Weights(w);
        Py::List weights;
        for (Standard_Integer i=w.Lower(); i<=w.Upper(); i++) {
            weights.append(Py::Float(w(i)));
        }
        return Py::new_reference_to(weights);
    }
    catch (Standard_Failure) {
        Handle_Standard_Failure e = Standard_Failure::Caught();
        PyErr_SetString(PartExceptionOCCError, e->GetMessageString());
        return 0;
    }
}

PyObject* BezierCurve2dPy::getResolution(PyObject* args)
{
    double tol;
    if (!PyArg_ParseTuple(args, "d", &tol))
        return 0;
    try {
        Handle_Geom_BezierCurve curve = Handle_Geom_BezierCurve::DownCast
            (getGeometryPtr()->handle());
        double utol;
        curve->Resolution(tol,utol);
        return Py_BuildValue("d",utol);
    }
    catch (Standard_Failure) {
        Handle_Standard_Failure e = Standard_Failure::Caught();
        PyErr_SetString(PartExceptionOCCError, e->GetMessageString());
        return 0;
    }
}

Py::Int BezierCurve2dPy::getDegree(void) const
{
    Handle_Geom_BezierCurve curve = Handle_Geom_BezierCurve::DownCast
        (getGeometryPtr()->handle());
    return Py::Int(curve->Degree()); 
}

Py::Int BezierCurve2dPy::getMaxDegree(void) const
{
    Handle_Geom_BezierCurve curve = Handle_Geom_BezierCurve::DownCast
        (getGeometryPtr()->handle());
    return Py::Int(curve->MaxDegree()); 
}

Py::Int BezierCurve2dPy::getNbPoles(void) const
{
    Handle_Geom_BezierCurve curve = Handle_Geom_BezierCurve::DownCast
        (getGeometryPtr()->handle());
    return Py::Int(curve->NbPoles()); 
}

Py::Object BezierCurve2dPy::getStartPoint(void) const
{
    Handle_Geom_BezierCurve c = Handle_Geom_BezierCurve::DownCast
        (getGeometryPtr()->handle());
    gp_Pnt pnt = c->StartPoint();
    return Py::Vector(Base::Vector3d(pnt.X(), pnt.Y(), pnt.Z()));
}

Py::Object BezierCurve2dPy::getEndPoint(void) const
{
    Handle_Geom_BezierCurve c = Handle_Geom_BezierCurve::DownCast
        (getGeometryPtr()->handle());
    gp_Pnt pnt = c->EndPoint();
    return Py::Vector(Base::Vector3d(pnt.X(), pnt.Y(), pnt.Z()));
}
#endif
PyObject *BezierCurve2dPy::getCustomAttributes(const char* /*attr*/) const
{
    return 0;
}

int BezierCurve2dPy::setCustomAttributes(const char* /*attr*/, PyObject* /*obj*/)
{
    return 0; 
}
