/***************************************************************************
 *   Copyright (c) 2017 Shai Seger         <shaise at gmail>               *
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


#ifndef APP_PRECOMPILED_H
#define APP_PRECOMPILED_H

#include <FCConfig.h>

// Exporting of App classes
#ifdef FC_OS_WIN32
# define PathSimulatorExport __declspec(dllexport)
# define PathExport  __declspec(dllimport)
# define PartExport __declspec(dllimport)
# define MeshExport __declspec(dllimport)
#else // for Linux
# define PathSimulatorExport
# define PathExport
# define PartExport
# define MeshExport
#endif

#ifdef _PreComp_

// standard
#include <cstdio>
#include <cassert>
#include <iostream>

// STL
#include <algorithm>
#include <iostream>
#include <list>
#include <map>
#include <queue>
#include <set>
#include <sstream>
#include <stack>
#include <string>
#include <vector>

// Boost
#include <boost/regex.hpp>

// Xerces
#include <xercesc/util/XercesDefs.hpp>

#endif //_PreComp_

#endif

