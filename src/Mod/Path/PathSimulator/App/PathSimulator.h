/***************************************************************************
 *   Copyright (c) Shsi Seger (shaise at gmail) 2017                       *
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


#ifndef PATH_PathSimulator_H
#define PATH_PathSimulator_H

#include <Base/Persistence.h>
#include <Base/Vector3D.h>

namespace Path
{

    /** The representation of a CNC Toolpath Simulator */
    
	class PathSimulatorAppExport PathSimulator : public Base::Persistence
    {
        TYPESYSTEM_HEADER();
    
        public:
			PathSimulator() {};
			~PathSimulator() {};
            
			virtual unsigned int getMemSize(void) const {
				return 0;
			};
			virtual void Save(Base::Writer &/*writer*/) const {};
			virtual void Restore(Base::XMLReader &/*reader*/) {};
	};

} //namespace Path


#endif // PATH_PathSimulator_H
