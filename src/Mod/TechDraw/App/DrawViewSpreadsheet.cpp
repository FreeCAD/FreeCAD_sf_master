/***************************************************************************
 *   Copyright (c) Yorik van Havre          (yorik@uncreated.net) 2015     *
 *   Copyright (c) 2016 WandererFan    (wandererfan@gmail.com)             *
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
# include <sstream>
#endif

#include <iomanip>

#include <App/Application.h>
#include <App/Property.h>
#include <App/PropertyStandard.h>
#include <App/PropertyUnits.h>
#include <Base/Console.h>
#include <Base/Exception.h>
#include <Base/FileInfo.h>
#include <Base/Parameter.h>

#include "DrawViewSpreadsheet.h"

#include <Mod/Spreadsheet/App/Cell.h>
#include <Mod/Spreadsheet/App/Sheet.h>

using namespace TechDraw;
using namespace std;


//===========================================================================
// DrawViewSpreadsheet
//===========================================================================

PROPERTY_SOURCE(TechDraw::DrawViewSpreadsheet, TechDraw::DrawViewSymbol)

DrawViewSpreadsheet::DrawViewSpreadsheet(void)
{
    static const char *vgroup = "Spreadsheet";

    Base::Reference<ParameterGrp> hGrp = App::GetApplication().GetUserParameter()
        .GetGroup("BaseApp")->GetGroup("Preferences")->GetGroup("Mod/TechDraw/Labels");
    std::string fontName = hGrp->GetASCII("LabelFont", "Sans");

    ADD_PROPERTY_TYPE(Source ,(0),vgroup,App::Prop_None,"Spreadsheet to view");
    ADD_PROPERTY_TYPE(CellStart ,("A1"),vgroup,App::Prop_None,"The top left cell of the range to display");
    ADD_PROPERTY_TYPE(CellEnd ,("B2"),vgroup,App::Prop_None,"The bottom right cell of the range to display");
    ADD_PROPERTY_TYPE(Font ,((fontName.c_str())),vgroup,App::Prop_None,"The name of the font to use");
    ADD_PROPERTY_TYPE(TextColor,(0.0f,0.0f,0.0f),vgroup,App::Prop_None,"The default color of the text and lines");
    ADD_PROPERTY_TYPE(TextSize,(12.0),vgroup,App::Prop_None,"The size of the text");
    ADD_PROPERTY_TYPE(LineWidth,(0.35),vgroup,App::Prop_None,"The thickness of the cell lines");
    //ADD_PROPERTY_TYPE(Symbol,(""),vgroup,App::Prop_Hidden,"The SVG image of this spreadsheet");

    EditableTexts.setStatus(App::Property::Hidden,true);

}

DrawViewSpreadsheet::~DrawViewSpreadsheet()
{
}

void DrawViewSpreadsheet::onChanged(const App::Property* prop)
{
    if (!isRestoring()) {
        if (prop == &Source ||
            prop == &CellStart ||
            prop == &CellEnd ||
            prop == &Font ||
            prop == &TextSize ||
            prop == &TextColor ||
            prop == &LineWidth) {
            try {
                App::DocumentObjectExecReturn *ret = recompute();
                delete ret;
            }
            catch (...) {
            }
        }
    }
    TechDraw::DrawView::onChanged(prop);
}

App::DocumentObjectExecReturn *DrawViewSpreadsheet::execute(void)
{
    App::DocumentObject* link = Source.getValue();
    std::string scellstart = CellStart.getValue();
    std::string scellend = CellEnd.getValue();
    if (!link)
        return new App::DocumentObjectExecReturn("No spreadsheet linked");
    if (!link->getTypeId().isDerivedFrom(Spreadsheet::Sheet::getClassTypeId()))
        return new App::DocumentObjectExecReturn("The linked object is not a spreadsheet");
    if ( (scellstart.empty()) || (scellend.empty()) )
        return new App::DocumentObjectExecReturn("Empty cell value");

    Symbol.setValue(getSheetImage());

    return TechDraw::DrawView::execute();
}

std::vector<std::string> DrawViewSpreadsheet::getAvailColumns(void)
{
    // build a list of available colums: A, B, C, ... AA, AB, ... ZY, ZZ.
    std::string alphabet = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";
    std::vector<std::string> availcolumns;
    for (int i=0; i<26; ++i) {
        std::stringstream s;
        s << alphabet[i];
        availcolumns.push_back(s.str());
    }
    for (int i=0; i<26; ++i) {
        for (int j=0; i<26; ++i) {
            std::stringstream s;
            s << alphabet[i] << alphabet[j];
            availcolumns.push_back(s.str());
        }
    }
    return availcolumns;
}

//note: newlines need to be double escaped for python, but single for C++
std::string DrawViewSpreadsheet::getSVGHead(void)
{
    std::string head = std::string("<svg\n") +
                       std::string("	xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\"\n") +
                       std::string("	xmlns:freecad=\"http://www.freecadweb.org/wiki/index.php?title=Svg_Namespace\">\n");
    return head;
}

std::string DrawViewSpreadsheet::getSVGTail(void)
{
    std::string tail = "\n</svg>";
    return tail;
}

std::string DrawViewSpreadsheet::getSheetImage(void)
{
    std::stringstream result;

    App::DocumentObject* link = Source.getValue();
    std::string scellstart = CellStart.getValue();
    std::string scellend = CellEnd.getValue();

    std::vector<std::string> availcolumns = getAvailColumns();

    // build rows range and columns range
    std::vector<std::string> columns;
    std::vector<int> rows;
    try {
        for (unsigned int i=0; i<scellstart.length(); ++i) {
            if (isdigit(scellstart[i])) {
                columns.push_back(scellstart.substr(0,i));
                rows.push_back(std::atoi(scellstart.substr(i,scellstart.length()-1).c_str()));
            }
        }
        for (unsigned int i=0; i<scellend.length(); ++i) {
            if (isdigit(scellend[i])) {
                std::string startcol = columns.back();
                std::string endcol = scellend.substr(0,i);
                bool valid = false;
                for (std::vector<std::string>::const_iterator j = availcolumns.begin(); j != availcolumns.end(); ++j) {
                    if ( (*j) == startcol) {
                        if ( (*j) != endcol) {
                            valid = true;
                        }
                    } else {
                        if (valid) {
                            if ( (*j) == endcol) {
                                columns.push_back((*j));
                                valid = false;
                            } else {
                                columns.push_back((*j));
                            }
                        }
                    }
                }
                int endrow = std::atoi(scellend.substr(i,scellend.length()-1).c_str());
                for (int j=rows.back()+1; j<=endrow; ++j) {
                    rows.push_back(j);
                }
            }
        }
    } catch (std::exception) {
        Base::Console().Error("Invalid cell range for %s\n",getNameInDocument());
        return result.str();
    }

    // create the containing group
    std::string ViewName = Label.getValue();

    result << getSVGHead();

    App::Color c = TextColor.getValue();
    result  << "<g id=\"" << ViewName << "\">" << endl;

    // fill the cells
    float rowoffset = 0.0;
    float coloffset = 0.0;
    float cellheight = 100;
    float cellwidth = 100;
    std::string celltext;
    Spreadsheet::Sheet* sheet = static_cast<Spreadsheet::Sheet*>(link);
    std::vector<std::string> skiplist;
    for (std::vector<std::string>::const_iterator col = columns.begin(); col != columns.end(); ++col) {
        // create a group for each column
        result << "  <g id=\"" << ViewName << "_col" << (*col) << "\">" << endl;
        for (std::vector<int>::const_iterator row = rows.begin(); row != rows.end(); ++row) {
            // get cell size
            std::stringstream srow;
            srow << (*row);
            App::CellAddress address((*col) + srow.str());
            cellwidth = sheet->getColumnWidth(address.col());
            cellheight = sheet->getRowHeight(address.row());
            celltext = "";
            // get the text
            App::Property* prop = sheet->getPropertyByName(address.toString().c_str());
            std::stringstream field;
            if (prop != 0) {
                if (prop->isDerivedFrom((App::PropertyQuantity::getClassTypeId())))
                    field << static_cast<App::PropertyQuantity*>(prop)->getValue();
                else if (prop->isDerivedFrom((App::PropertyFloat::getClassTypeId())))
                    field << static_cast<App::PropertyFloat*>(prop)->getValue();
                else if (prop->isDerivedFrom((App::PropertyString::getClassTypeId())))
                    field << static_cast<App::PropertyString*>(prop)->getValue();
                else
                    assert(0);
                celltext = field.str();
            }
            // get colors, style, alignment and span
            int alignment;
            std::string bcolor = "none";
            std::string fcolor = c.asCSSString();
            std::string textstyle = "";
            Spreadsheet::Cell* cell = sheet->getCell(address);
            if (cell) {
                App::Color f,b;
                std::set<std::string> st;
                int colspan, rowspan;
                if (cell->getBackground(b)) {
                    bcolor = b.asCSSString();
                }
                if (cell->getForeground(f)) {
                    fcolor = f.asCSSString();
                }
                if (cell->getStyle(st)) {
                    for (std::set<std::string>::const_iterator i = st.begin(); i != st.end(); ++i) {
                         if ((*i) == "bold")
                            textstyle = textstyle + "font-weight: bold; ";
                        else if ((*i) == "italic")
                            textstyle = textstyle + "font-style: italic; ";
                        else if ((*i) == "underline")
                            textstyle = textstyle + "text-decoration: underline; ";
                    }
                }
                if (cell->getSpans(rowspan,colspan)) {
                    for (int i=0; i<colspan; ++i) {
                        for (int j=0; j<rowspan; ++j) {
                            App::CellAddress nextcell(address.row()+j,address.col()+i);
                            if (i > 0)
                                cellwidth = cellwidth + sheet->getColumnWidth(nextcell.col());
                            if (j > 0)
                                cellheight = cellheight + sheet->getRowHeight(nextcell.row());
                            if ( (i > 0) || (j > 0) )
                                skiplist.push_back(nextcell.toString());
                        }
                    }
                }
                cell->getAlignment(alignment);
            }
            // skip cell if found in skiplist
            if (std::find(skiplist.begin(), skiplist.end(), address.toString()) == skiplist.end()) {
                result << "    <rect x=\"" << coloffset << "\" y=\"" << rowoffset << "\" width=\"" << cellwidth
                       << "\" height=\"" << cellheight << "\" style=\"fill:" << bcolor << ";stroke-width:"
                       << LineWidth.getValue()/Scale.getValue() << ";stroke:" << c.asCSSString() << ";\" />" << endl;
                if (alignment & Spreadsheet::Cell::ALIGNMENT_LEFT)
                    result << "    <text style=\"" << textstyle << "\" x=\"" << coloffset + TextSize.getValue()/2 << "\" y=\"" << rowoffset + 0.75 * cellheight << "\" font-family=\"" ;
                if (alignment & Spreadsheet::Cell::ALIGNMENT_HCENTER)
                    result << "    <text text-anchor=\"middle\" style=\"" << textstyle << "\" x=\"" << coloffset + cellwidth/2 << "\" y=\"" << rowoffset + 0.75 * cellheight << "\" font-family=\"" ;
                if (alignment & Spreadsheet::Cell::ALIGNMENT_RIGHT)
                    result << "    <text text-anchor=\"end\" style=\"" << textstyle << "\" x=\"" << coloffset + (cellwidth - TextSize.getValue()/2) << "\" y=\"" << rowoffset + 0.75 * cellheight << "\" font-family=\"" ;
                result << Font.getValue() << "\"" << " font-size=\"" << TextSize.getValue() << "\""
                       << " fill=\"" << fcolor << "\">" << celltext << "</text>" << endl;
            }
            rowoffset = rowoffset + cellheight;
        }
        result << "  </g>" << endl;
        rowoffset = 0.0;
        coloffset = coloffset + cellwidth;
    }

    // close the containing group
    result << "</g>" << endl;

    result << getSVGTail();

    return result.str();
}

// Python Drawing feature ---------------------------------------------------------

namespace App {
/// @cond DOXERR
PROPERTY_SOURCE_TEMPLATE(TechDraw::DrawViewSpreadsheetPython, TechDraw::DrawViewSpreadsheet)
template<> const char* TechDraw::DrawViewSpreadsheetPython::getViewProviderName(void) const {
    return "TechDrawGui::ViewProviderSpreadsheet";
}
/// @endcond

// explicit template instantiation
template class TechDrawExport FeaturePythonT<TechDraw::DrawViewSpreadsheet>;
}
