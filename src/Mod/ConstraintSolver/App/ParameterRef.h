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
#pragma once //to make qt creator happy, see QTCREATORBUG-20883

#ifndef FREECAD_CONSTRAINTSOLVER_PARAMETERREF_H
#define FREECAD_CONSTRAINTSOLVER_PARAMETERREF_H

#include "Utils.h"
#include "Parameter.h"
#include "ParameterStore.h"

namespace GCS {

class ParameterStore;
typedef UnsafePyHandle<ParameterStore> HParameterStore;

/**
 * @brief ParameterRef class: refers to a parameter in store. Also is used as a key into value vectors, for constraint code.
 * Memory management: regular. getPyObject returns a copy.
 */
class GCSExport ParameterRef
{
protected://data
    HParameterStore _store;
    int _ownIndex;
public://methods
    ParameterRef(HParameterStore st, int index);
    ~ParameterRef();

    const HParameterStore& host() const {return _store;}
    int ownIndex() const {return _ownIndex;}

    int masterIndex() const;

    ///value of the parameter, bypassing redirection
    double& ownValue() const;
    ///returns value, obeying redirection
    double& value() const;

    ///scale of the parameter, bypassing redirection
    double& ownScale() const;
    ///scale of the parameter, obeying redirection
    double& masterScale() const;

    Parameter& param() const;
    ///the parameter object this parameter is redirected to
    ParameterRef masterParam() const;

    bool isSameRef(const ParameterRef &other) const;
    bool isSameValue(const ParameterRef &other) const;

    UnsafePyHandle<ParameterRef> getPyObject() const;
};

} //namespace

#endif
