const char FreeCAD[] ="// FreeCAD Povray standard file\n"
"/***************************************************************************\n"
" *   Copyright (c) 2005 Juergen Riegel         <juergen.riegel@web.de>     *\n"
" *                                                                         *\n"
" *   This file is part of the FreeCAD CAx development system.              *\n"
" *                                                                         *\n"
" *   This library is free software; you can redistribute it and/or         *\n"
" *   modify it under the terms of the GNU Library General Public           *\n"
" *   License as published by the Free Software Foundation; either          *\n"
" *   version 2 of the License, or (at your option) any later version.      *\n"
" *                                                                         *\n"
" *   This library  is distributed in the hope that it will be useful,      *\n"
" *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *\n"
" *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *\n"
" *   GNU Library General Public License for more details.                  *\n"
" *                                                                         *\n"
" *   You should have received a copy of the GNU Library General Public     *\n"
" *   License along with this library; see the file COPYING.LIB. If not,    *\n"
" *   write to the Free Software Foundation, Inc., 59 Temple Place,         *\n"
" *   Suite 330, Boston, MA  02111-1307, USA                                *\n"
" *                                                                         *\n"
" ***************************************************************************/\n"
"\n"
"// -w320 -h240\n"
"// -w8000 -h6000 +a0.3\n"
"// Use povray -iLehreW221animation.pov LehreW221animation.ini to trace.\n"
"// Use povray -w1280 -h720 +a0.3 -iLehreW221animation.pov LehreW221animation.ini to trace.\n"
"\n"
"// Include Standard-Colors provided by povray\n"
"#include \"colors.inc\"\n"
"// Include Standard-Textures and Finishes provided by povray\n"
"#include \"textures.inc\"\n"
"#include \"woods.inc\"\n"
"\n"
"// default texture\n"
"//default {\n"
"//  texture { pigment {rgb 1} finish {ambient 0.2 reflection 0.2 specular 0.7} }\n"
"//}\n"
"\n"
"\n"
"sky_sphere {\n"
"    pigment { rgb <0.8,0.8,0.8> }\n"
"    pigment {\n"
"    gradient x\n"
"      color_map {\n"
"        [0.00 color rgbt <1,1,1,0>]\n"
"        [0.08 color rgbt <1,1,1,0>]\n"
"        [0.09 color rgbt <0.1,0.1,0.1,1>]\n"
"        [1.00 color rgbt <0.1,0.1,0.1,1>]\n"
"      }\n"
"      scale 0.05\n"
"    }\n"
"    pigment {\n"
"    gradient y\n"
"      color_map {\n"
"        [0.00 color rgbt <1,1,1,0>]\n"
"        [0.08 color rgbt <1,1,1,0>]\n"
"        [0.09 color rgbt <0.1,0.1,0.1,1>]\n"
"        [1.00 color rgbt <0.1,0.1,0.1,1>]\n"
"      }\n"
"      scale 0.05\n"
"    }\n"
"}\n"
"\n"
"\n"
"// Fussboden\n"
"plane {               // checkered floor\n"
"  y, -1\n"
"  texture\n"
"  {\n"
"    pigment {\n"
"      checker\n"
"      color rgb 1\n"
"      color rgb 0.5\n"
"      scale 0.5\n"
"    }\n"
"    finish{\n"
"      diffuse 0.2\n"
"      ambient 0.4\n"
"    }\n"
"  }\n"
"  scale 1000\n"
"}\n"
"\n"
"\n"
"// includes the Part mesh writen from FreeCAD\n"
"#include \"TempPart.inc\"\n"
"object {Part\n"
"   texture { pigment {rgb <0.3,0.8,0.3>} finish {ambient 0.2 reflection 0.2 specular 0.7} }\n"
" }\n"
"\n"
"// includes the camera from FreeCAD\n"
"#include \"TempCamera.inc\"\n"
"camera {\n"
"  location  CamPos\n"
"  look_at   LookAt\n"
"  sky       Up\n"
"  angle     50\n"
"}\n"
"\n"
"\n"
"\n"
"// Lightsource\n"
"light_source {\n"
"<-1573.9813500000005,1310.07165000000003,-2000.1032>, color White\n"
"}\n"
;


