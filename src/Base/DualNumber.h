/***************************************************************************
 *   Copyright (c) 2019 Viktor Titov (DeepSOIC) <vv.titov@gmail.com>       *
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

#ifndef FREECAD_BASE_DUAL_NUMBER_H
#define FREECAD_BASE_DUAL_NUMBER_H

#include <cmath>
#include <string>

//forward declare PyObject, to avoid include python
struct _object;
typedef _object PyObject;

namespace Base {

/**
 * @brief Dual Numbers are 2-part numbers like complex numbers, but different
 * algebra. They are denoted as a + b*eps, where eps^2 = 0. eps, the nilpotent,
 * is like imaginary unit of complex numbers. The neat utility of dual numbers
 * is that if you use them instead of normal numbers in a function like sin(),
 * derivative is implicitly calculated as a multiplier to the dual part.
 */
class BaseExport DualNumber
{
public:
    double re = 0.0;
    double du = 0.0;
public:
    DualNumber(){}
    DualNumber(double re, double du = 0.0)
        : re(re), du(du)
    {}
    DualNumber operator-() const {return DualNumber(-re,-du);}

    PyObject* getPyObject() const;
    std::string repr() const;
};

inline DualNumber operator+(DualNumber a, DualNumber b){
    return DualNumber(a.re + b.re, a.du + b.du);
}
inline DualNumber operator+(DualNumber a, double b){
    return DualNumber(a.re + b, a.du);
}
inline DualNumber operator+(double a, DualNumber b){
    return DualNumber(a + b.re, b.du);
}

inline DualNumber operator-(DualNumber a, DualNumber b){
    return DualNumber(a.re - b.re, a.du - b.du);
}
inline DualNumber operator-(DualNumber a, double b){
    return DualNumber(a.re - b, a.du);
}
inline DualNumber operator-(double a, DualNumber b){
    return DualNumber(a - b.re, -b.du);
}

inline DualNumber operator*(DualNumber a, DualNumber b){
    return DualNumber(a.re * b.re, a.re * b.du + a.du * b.re);
}
inline DualNumber operator*(double a, DualNumber b){
    return DualNumber(a * b.re, a * b.du);
}
inline DualNumber operator*(DualNumber a, double b){
    return DualNumber(a.re * b, a.du * b);
}

inline DualNumber operator/(DualNumber a, DualNumber b){
    return DualNumber(a.re / b.re, (a.du * b.re - a.re * b.du) / (b.re * b.re));
}
inline DualNumber operator/(DualNumber a, double b){
    return DualNumber(a.re / b, a.du / b);
}

#define IMPLEMENT_DUALNUMBER_OPERATOR(op)              \
inline bool operator op (DualNumber a, DualNumber b){  \
    return a.re op b.re;                               \
}                                                      \
inline bool operator op (double a, DualNumber b){      \
    return a op b.re;                                  \
}                                                      \
inline bool operator op (DualNumber a, double b){      \
    return a.re op b;                                  \
}                                                      \

IMPLEMENT_DUALNUMBER_OPERATOR(<);
IMPLEMENT_DUALNUMBER_OPERATOR(<=);
IMPLEMENT_DUALNUMBER_OPERATOR(>);
IMPLEMENT_DUALNUMBER_OPERATOR(>=);

inline DualNumber pow(DualNumber a, double pw){
    return Base::DualNumber(std::pow(a.re, pw), pw * std::pow(a.re, pw - 1.0) * a.du);
}

inline DualNumber abs(DualNumber a){
    return a < 0 ? -a : a;
}
} //namespace


#endif
