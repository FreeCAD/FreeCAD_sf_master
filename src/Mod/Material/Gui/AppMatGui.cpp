/***************************************************************************
*   Copyright (c) 2002 Juergen Riegel <juergen.riegel@web.de>             *
*                                                                         *
*   This file is part of the FreeCAD CAx development system.              *
*                                                                         *
*   This program is free software; you can redistribute it and/or modify  *
*   it under the terms of the GNU Lesser General Public License (LGPL)    *
*   as published by the Free Software Foundation; either version 2 of     *
*   the License, or (at your option) any later version.                   *
*   for detail see the LICENCE text file.                                 *
*                                                                         *
*   FreeCAD is distributed in the hope that it will be useful,            *
*   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
*   GNU Lesser General Public License for more details.                   *
*                                                                         *
*   You should have received a copy of the GNU Library General Public     *
*   License along with FreeCAD; if not, write to the Free Software        *
*   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  *
*   USA                                                                   *
*                                                                         *
***************************************************************************/


#include "PreCompiled.h"
#ifndef _PreComp_
# include <Standard_math.hxx>
#endif

#include <Base/Console.h>
#include <Base/Interpreter.h>
#include <Base/PyObjectBase.h>
#include <Gui/Application.h>
// #include <Gui/BitmapFactory.h>
// #include <Gui/DlgPreferencesImp.h>
// #include <Gui/WidgetFactory.h>
#include <Gui/Language/Translator.h>

// use a different name to CreateCommand()
void CreateMaterialCommands();

void loadPartResource()
{
    // add resources and reloads the translators
    Q_INIT_RESOURCE(Material);
    Q_INIT_RESOURCE(Material_translation);
    Gui::Translator::instance()->refresh();
}

namespace MatGui {
class Module : public Py::ExtensionModule<Module>
{
public:
    Module() : Py::ExtensionModule<Module>("MatGui")
    {
        initialize("This module is the MatGui module."); // register with Python
    }

    ~Module() override {}

private:
};

PyObject* initModule()
{
    return Base::Interpreter().addModule(new Module);
}

} // namespace MatGui

PyMOD_INIT_FUNC(MatGui)
{
    if (!Gui::Application::Instance) {
        PyErr_SetString(PyExc_ImportError, "Cannot load Gui module in console application.");
        PyMOD_Return(nullptr);
    }

    // load needed modules
    try {
        Base::Interpreter().runString("import Material");
    }
    catch(const Base::Exception& e) {
        PyErr_SetString(PyExc_ImportError, e.what());
        PyMOD_Return(nullptr);
    }

    PyObject* matGuiModule = MatGui::initModule();

    Base::Console().Log("Loading GUI of Material module... done\n");

    // instantiating the commands
    CreateMaterialCommands();
    try{
        Py::Object ae = Base::Interpreter().runStringObject("__import__('AttachmentEditor.Commands').Commands");
        Py::Module(matGuiModule).setAttr(std::string("AttachmentEditor"),ae);
    } catch (Base::PyException &err){
        err.ReportException();
    }

    // add resources and reloads the translators
    loadPartResource();

    PyMOD_Return(matGuiModule);
}
