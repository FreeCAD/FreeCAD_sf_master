/***************************************************************************
 *   Copyright (c) 2010 Jürgen Riegel <juergen.riegel@web.de>              *
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
# include <Inventor/nodes/SoPickStyle.h>
# include <QApplication>
# include <QMessageBox>
# include <stdlib.h>
# include <qdebug.h>
# include <QString>
# include <GC_MakeEllipse.hxx>
# include <boost/math/special_functions/fpclassify.hpp>
# include <memory>
#endif

#include <Base/Console.h>
#include <Base/Exception.h>
#include <Base/Tools.h>

#include <App/OriginFeature.h>
#include <Gui/Action.h>
#include <Gui/Application.h>
#include <Gui/BitmapFactory.h>
#include <Gui/Document.h>
#include <Gui/CommandT.h>
#include <Gui/MainWindow.h>
#include <Gui/DlgEditFileIncludePropertyExternal.h>
#include <Gui/Selection.h>
#include <Gui/SelectionFilter.h>
#include <Mod/Sketcher/App/SketchObject.h>
#include <Mod/Part/App/DatumFeature.h>
#include <Mod/Part/App/BodyBase.h>
#include <Mod/Part/App/Geometry2d.h>
#include <Mod/Sketcher/App/Constraint.h>

#include "ViewProviderSketch.h"
#include "DrawSketchHandler.h"
#include "Utils.h"

#include <Gui/View3DInventor.h>
#include <Gui/View3DInventorViewer.h>
#include <Gui/SoFCUnifiedSelection.h>

#include <Gui/ToolBarManager.h>

#include "GeometryCreationMode.h"

#include "SketcherRegularPolygonDialog.h"

#include "TaskSketcherTool.h"

using namespace std;
using namespace SketcherGui;

namespace SketcherGui {
GeometryCreationMode geometryCreationMode=Normal;
}

/* helper functions ======================================================*/

// Return counter-clockwise angle from horizontal out of p1 to p2 in radians.
double GetPointAngle (const Base::Vector2d &p1, const Base::Vector2d &p2)
{
  double dX = p2.x - p1.x;
  double dY = p2.y - p1.y;
  return dY >= 0 ? atan2(dY, dX) : atan2(dY, dX) + 2*M_PI;
}

void ActivateHandler(Gui::Document *doc, DrawSketchHandler *handler)
{
    std::unique_ptr<DrawSketchHandler> ptr(handler);
    if (doc) {
        if (doc->getInEdit() && doc->getInEdit()->isDerivedFrom(SketcherGui::ViewProviderSketch::getClassTypeId())) {
            SketcherGui::ViewProviderSketch* vp = static_cast<SketcherGui::ViewProviderSketch*> (doc->getInEdit());
            vp->purgeHandler();
            vp->activateHandler(ptr.release());
        }
    }
}

bool isCreateGeoActive(Gui::Document *doc)
{
    if (doc) {
        // checks if a Sketch Viewprovider is in Edit and is in no special mode
        if (doc->getInEdit() && doc->getInEdit()->isDerivedFrom
            (SketcherGui::ViewProviderSketch::getClassTypeId())) {
            /*if (dynamic_cast<SketcherGui::ViewProviderSketch*>(doc->getInEdit())->
                getSketchMode() == ViewProviderSketch::STATUS_NONE)*/
                return true;
        }
    }
    return false;
}

SketcherGui::ViewProviderSketch* getSketchViewprovider(Gui::Document *doc)
{
    if (doc) {
        if (doc->getInEdit() && doc->getInEdit()->isDerivedFrom
            (SketcherGui::ViewProviderSketch::getClassTypeId()) )
            return dynamic_cast<SketcherGui::ViewProviderSketch*>(doc->getInEdit());
    }
    return 0;
}

void removeRedundantHorizontalVertical(Sketcher::SketchObject* psketch,
                                       std::vector<AutoConstraint> &sug1,
                                       std::vector<AutoConstraint> &sug2)
{
    if(!sug1.empty() && !sug2.empty()) {

        bool rmvhorvert = false;

        // we look for:
        // 1. Coincident to external on both endpoints
        // 2. Coincident in one endpoint to origin and pointonobject/tangent to an axis on the other
        auto detectredundant = [psketch](std::vector<AutoConstraint> &sug, bool &ext, bool &orig, bool &axis) {

            ext = false;
            orig = false;
            axis = false;

            for(std::vector<AutoConstraint>::const_iterator it = sug.begin(); it!=sug.end(); ++it) {
                if( (*it).Type == Sketcher::Coincident && ext == false) {
                    const std::map<int, Sketcher::PointPos> coincidents = psketch->getAllCoincidentPoints((*it).GeoId, (*it).PosId);

                    if(!coincidents.empty()) {
                        // the keys are ordered, so if the first is negative, it is coincident with external
                        ext = coincidents.begin()->first < 0;

                        std::map<int, Sketcher::PointPos>::const_iterator geoId1iterator;

                        geoId1iterator = coincidents.find(-1);

                        if( geoId1iterator != coincidents.end()) {
                            if( (*geoId1iterator).second == Sketcher::PointPos::start )
                                orig = true;
                        }
                    }
                    else { // it may be that there is no constraint at all, but there is external geometry
                        ext = (*it).GeoId < 0;
                        orig = ((*it).GeoId == -1 && (*it).PosId == Sketcher::PointPos::start);
                    }
                }
                else if( (*it).Type == Sketcher::PointOnObject && axis == false) {
                    axis = (((*it).GeoId == -1 && (*it).PosId == Sketcher::PointPos::none) || ((*it).GeoId == -2 && (*it).PosId == Sketcher::PointPos::none));
                }

            }
        };

        bool firstext = false, secondext = false, firstorig = false, secondorig = false, firstaxis = false, secondaxis = false;

        detectredundant(sug1, firstext, firstorig, firstaxis);
        detectredundant(sug2, secondext, secondorig, secondaxis);


        rmvhorvert = ((firstext && secondext)   ||  // coincident with external on both endpoints
                      (firstorig && secondaxis) ||  // coincident origin and point on object on other
                      (secondorig && firstaxis));

        if(rmvhorvert) {
            for(std::vector<AutoConstraint>::reverse_iterator it = sug2.rbegin(); it!=sug2.rend(); ++it) {
                if( (*it).Type == Sketcher::Horizontal || (*it).Type == Sketcher::Vertical) {
                    sug2.erase(std::next(it).base());
                    it = sug2.rbegin(); // erase invalidates the iterator
                }
            }
        }
    }
}

/* Sketch commands =======================================================*/

static const char cursor_crosshair_color_fmt[] = "+ c #%06lX";
char cursor_crosshair_color[12];

void DrawSketchHandler::setCrosshairColor()
{
    unsigned long color = getCrosshairColor();
    sprintf(cursor_crosshair_color, cursor_crosshair_color_fmt, color);
}

unsigned long DrawSketchHandler::getCrosshairColor()
{
    unsigned long color = 0xFFFFFFFF; // white
    ParameterGrp::handle hGrp = App::GetApplication().GetParameterGroupByPath
        ("User parameter:BaseApp/Preferences/View");
    color = hGrp->GetUnsigned("CursorCrosshairColor", color);
    // from rgba to rgb
    color = (color >> 8) & 0xFFFFFF;
    return color;
}

bool DrawSketchHandler::distanceXYorPointOnObject(bool distanceXZeroYOne, int geoId, Sketcher::PointPos posId, double distance) {
    if (distance == 0) {
        if (distanceXZeroYOne) {
            Gui::cmdAppObjectArgs(sketchgui->getObject(), "addConstraint(Sketcher.Constraint('PointOnObject',%d,%d,%d)) ",
                geoId, static_cast<int>(posId), Sketcher::GeoEnum::HAxis);
        }
        else {
            Gui::cmdAppObjectArgs(sketchgui->getObject(), "addConstraint(Sketcher.Constraint('PointOnObject',%d,%d,%d)) ",
                geoId, static_cast<int>(posId), Sketcher::GeoEnum::VAxis);
        }
        return 0;
    }
    else {
        if (!distanceXZeroYOne) {
            Gui::cmdAppObjectArgs(sketchgui->getObject(), "addConstraint(Sketcher.Constraint('DistanceX',%d,%d,%f)) ",
                geoId, static_cast<int>(posId), distance);
        }
        else {
            Gui::cmdAppObjectArgs(sketchgui->getObject(), "addConstraint(Sketcher.Constraint('DistanceY',%d,%d,%f)) ",
                geoId, static_cast<int>(posId), distance);
        }
        return 1;
    }

}
/* Create  Line =====================================================*/

class DrawSketchHandlerLine: public DrawSketchHandler
{
public:
    DrawSketchHandlerLine():Mode(STATUS_SEEK_First),EditCurve(2){}
    virtual ~DrawSketchHandlerLine(){
        sketchgui->toolSettings->widget->setSettings(0);
    }
    /// mode table
    enum SelectMode {
        STATUS_SEEK_First,      /**< enum value ----. */
        STATUS_SEEK_Second,     /**< enum value ----. */
        STATUS_End
    };

    virtual void activated(ViewProviderSketch *)
    {
        setCrosshairCursor("Sketcher_Pointer_Create_Line");
        sketchgui->toolSettings->widget->setSettings(5);
    }

    virtual void mouseMove(Base::Vector2d onSketchPos)
    {
        if (Mode==STATUS_SEEK_First) {
            setPositionText(onSketchPos);
            if (seekAutoConstraint(sugConstr1, onSketchPos, Base::Vector2d(0.f,0.f))) {
                renderSuggestConstraintsCursor(sugConstr1);
                return;
            }
            if (sketchgui->toolSettings->widget->isSettingSet[0] + sketchgui->toolSettings->widget->isSettingSet[1] == 2) {
                pressButton(onSketchPos);
                releaseButton(onSketchPos);
            }
        }
        else if (Mode==STATUS_SEEK_Second){
            float length = (onSketchPos - EditCurve[0]).Length();
            float angle = (onSketchPos - EditCurve[0]).GetAngle(Base::Vector2d(1.f,0.f));
            SbString text;
            text.sprintf(" (%.1f,%.1fdeg)", length, angle * 180 / M_PI);
            setPositionText(onSketchPos, text);

            EditCurve[1] = onSketchPos;
            if (sketchgui->toolSettings->widget->isSettingSet[2] == 1) {
                EditCurve[1].x = sketchgui->toolSettings->widget->toolParameters[2];
            }
            if (sketchgui->toolSettings->widget->isSettingSet[3] == 1) {
                EditCurve[1].y = sketchgui->toolSettings->widget->toolParameters[3];
            }

            drawEdit(EditCurve);
            if (seekAutoConstraint(sugConstr2, onSketchPos, onSketchPos - EditCurve[0])) {
                renderSuggestConstraintsCursor(sugConstr2);
                return;
            }
            if (sketchgui->toolSettings->widget->isSettingSet[2] + sketchgui->toolSettings->widget->isSettingSet[3] == 2) {
                pressButton(onSketchPos);
                releaseButton(onSketchPos);
            }
        }
        applyCursor();
    }

    virtual bool pressButton(Base::Vector2d onSketchPos)
    {
        if (Mode==STATUS_SEEK_First){
            if (sketchgui->toolSettings->widget->isSettingSet[0] == 1) {
                EditCurve[0].x = sketchgui->toolSettings->widget->toolParameters[0];
            }
            else {
                EditCurve[0].x = onSketchPos.x;
            }
            if (sketchgui->toolSettings->widget->isSettingSet[1] == 1) {
                EditCurve[0].y = sketchgui->toolSettings->widget->toolParameters[1];
            }
            else {
                EditCurve[0].y = onSketchPos.y;
            }

            sketchgui->toolSettings->widget->setParameterActive(0, 0);
            sketchgui->toolSettings->widget->setParameterActive(0, 1);
            sketchgui->toolSettings->widget->setParameterActive(1, 2);
            sketchgui->toolSettings->widget->setParameterActive(1, 3);
            sketchgui->toolSettings->widget->setParameterFocus(2);
            Mode = STATUS_SEEK_Second;
        }
        else {
            EditCurve[1] = onSketchPos;
            if (sketchgui->toolSettings->widget->isSettingSet[2] == 1) {
                EditCurve[1].x = sketchgui->toolSettings->widget->toolParameters[2];
            }
            if (sketchgui->toolSettings->widget->isSettingSet[3] == 1) {
                EditCurve[1].y = sketchgui->toolSettings->widget->toolParameters[3];
            }
            drawEdit(EditCurve);
            Mode = STATUS_End;
        }
        return true;
    }

    virtual bool releaseButton(Base::Vector2d onSketchPos)
    {
        Q_UNUSED(onSketchPos);
        if (Mode==STATUS_End){
            unsetCursor();
            resetPositionText();

            try {
                Gui::Command::openCommand(QT_TRANSLATE_NOOP("Command", "Add sketch line"));
                Gui::cmdAppObjectArgs(sketchgui->getObject(), "addGeometry(Part.LineSegment(App.Vector(%f,%f,0),App.Vector(%f,%f,0)),%s)",
                          EditCurve[0].x,EditCurve[0].y,EditCurve[1].x,EditCurve[1].y,
                          geometryCreationMode==Construction?"True":"False");

                Gui::Command::commitCommand();
            }
            catch (const Base::Exception& e) {
                Base::Console().Error("Failed to add line: %s\n", e.what());
                Gui::Command::abortCommand();
            }

            ParameterGrp::handle hGrp = App::GetApplication().GetParameterGroupByPath("User parameter:BaseApp/Preferences/Mod/Sketcher");
            bool avoidredundant = sketchgui->AvoidRedundant.getValue()  && sketchgui->Autoconstraints.getValue();

            if(avoidredundant)
                removeRedundantHorizontalVertical(static_cast<Sketcher::SketchObject *>(sketchgui->getObject()),sugConstr1,sugConstr2);
            
            int firstCurve = getHighestCurveIndex();
            //add constraint if user typed in some dimensions in tool widget
            if (sketchgui->toolSettings->widget->isSettingSet[0] + sketchgui->toolSettings->widget->isSettingSet[1] + sketchgui->toolSettings->widget->isSettingSet[2] + sketchgui->toolSettings->widget->isSettingSet[3] != 0) {
                if (sketchgui->toolSettings->widget->isSettingSet[0] == 1) {
                    distanceXYorPointOnObject(0, firstCurve, Sketcher::PointPos::start, sketchgui->toolSettings->widget->toolParameters[0]);
                }
                if (sketchgui->toolSettings->widget->isSettingSet[1] == 1) {
                    distanceXYorPointOnObject(1, firstCurve, Sketcher::PointPos::start, sketchgui->toolSettings->widget->toolParameters[1]);
                }
                if (sketchgui->toolSettings->widget->isSettingSet[2] == 1) {
                    distanceXYorPointOnObject(0, firstCurve, Sketcher::PointPos::end, sketchgui->toolSettings->widget->toolParameters[2]);
                }
                if (sketchgui->toolSettings->widget->isSettingSet[3] == 1) {
                    distanceXYorPointOnObject(1, firstCurve, Sketcher::PointPos::end, sketchgui->toolSettings->widget->toolParameters[3]);
                }
            }

            // add auto constraints for the line segment start
            if (!sugConstr1.empty()) {
                createAutoConstraints(sugConstr1, getHighestCurveIndex(), Sketcher::PointPos::start);
                sugConstr1.clear();
            }

            // add auto constraints for the line segment end
            if (!sugConstr2.empty()) {
                createAutoConstraints(sugConstr2, getHighestCurveIndex(), Sketcher::PointPos::end);
                sugConstr2.clear();
            }

            tryAutoRecomputeIfNotSolve(static_cast<Sketcher::SketchObject *>(sketchgui->getObject()));

            EditCurve.clear();
            drawEdit(EditCurve);

            bool continuousMode = hGrp->GetBool("ContinuousCreationMode",true);
            if(continuousMode){
                // This code enables the continuous creation mode.
                sketchgui->toolSettings->widget->setSettings(5);
                Mode=STATUS_SEEK_First;
                EditCurve.resize(2);
                applyCursor();
                /* It is ok not to call to purgeHandler
                * in continuous creation mode because the
                * handler is destroyed by the quit() method on pressing the
                * right button of the mouse */
            }
            else{
                sketchgui->purgeHandler(); // no code after this line, Handler get deleted in ViewProvider
            }
        }
        return true;
    }
protected:
    SelectMode Mode;
    std::vector<Base::Vector2d> EditCurve;
    std::vector<AutoConstraint> sugConstr1, sugConstr2;
};

DEF_STD_CMD_AU(CmdSketcherCreateLine)

CmdSketcherCreateLine::CmdSketcherCreateLine()
  : Command("Sketcher_CreateLine")
{
    sAppModule      = "Sketcher";
    sGroup          = "Sketcher";
    sMenuText       = QT_TR_NOOP("Create line");
    sToolTipText    = QT_TR_NOOP("Create a line in the sketch");
    sWhatsThis      = "Sketcher_CreateLine";
    sStatusTip      = sToolTipText;
    sPixmap         = "Sketcher_CreateLine";
    sAccel          = "G, L";
    eType           = ForEdit;
}

void CmdSketcherCreateLine::activated(int iMsg)
{
    Q_UNUSED(iMsg);
    ActivateHandler(getActiveGuiDocument(),new DrawSketchHandlerLine() );
}

void CmdSketcherCreateLine::updateAction(int mode)
{
    switch (mode) {
    case Normal:
        if (getAction())
            getAction()->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateLine"));
        break;
    case Construction:
        if (getAction())
            getAction()->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateLine_Constr"));
        break;
    }
}

bool CmdSketcherCreateLine::isActive(void)
{
    return isCreateGeoActive(getActiveGuiDocument());
}


/* Create Box =======================================================*/

class DrawSketchHandlerBox: public DrawSketchHandler
{
public:
    enum ConstructionMethod {
        Diagonal,
        CenterAndCorner
    };

    DrawSketchHandlerBox(ConstructionMethod constrMethod = Diagonal):   Mode(STATUS_SEEK_First),
                                                                        EditCurve(5),
                                                                        constructionMethod(constrMethod){}
    virtual ~DrawSketchHandlerBox(){
        sketchgui->toolSettings->widget->setSettings(0);
    }

    /// mode table
    enum BoxMode {
        STATUS_SEEK_First,      /**< enum value ----. */
        STATUS_SEEK_Second,     /**< enum value ----. */
        STATUS_End
    };

    virtual void activated(ViewProviderSketch* sketchgui)
    {
        setCrosshairCursor("Sketcher_Pointer_Create_Box");
        sketchgui->toolSettings->widget->setSettings(1);

    }

    virtual void mouseMove(Base::Vector2d onSketchPos)
    {

        if (Mode==STATUS_SEEK_First) {
            setPositionText(onSketchPos);
            /*//If mouse move and settings haven't been typed in, then update the values in the widget
            //issue is that this triggers onSetParameters. And it doesn't feel very useful?
            if (sketchgui->toolSettings->widget->isSettingSet[0] == 0){
                sketchgui->toolSettings->widget->setparameter(onSketchPos.x, 0);
            }
            if (sketchgui->toolSettings->widget->isSettingSet[1] == 0) {
                sketchgui->toolSettings->widget->setparameter(onSketchPos.y, 1);
            }*/
            if (sketchgui->toolSettings->widget->isSettingSet[0] + sketchgui->toolSettings->widget->isSettingSet[1] == 2) {
                pressButton(onSketchPos);
                releaseButton(onSketchPos);
            }

            if (seekAutoConstraint(sugConstr1, onSketchPos, Base::Vector2d(0.f,0.f))) {
                renderSuggestConstraintsCursor(sugConstr1);
                return;
            }
        }
        else if (Mode==STATUS_SEEK_Second) {
            if(constructionMethod == Diagonal) {
                //recalculate secondpoint in case mouse moved too fast for mousemove to catch it.
                secondPoint = onSketchPos;
                if (sketchgui->toolSettings->widget->isSettingSet[2] == 1) {
                    float dx = sketchgui->toolSettings->widget->toolParameters[2];
                    if (onSketchPos.x - firstPoint.x < 0) {
                        dx = -dx;
                    }
                    secondPoint.x = firstPoint.x + dx;
                }
                if (sketchgui->toolSettings->widget->isSettingSet[3] == 1) {
                    float dy = sketchgui->toolSettings->widget->toolParameters[3];
                    if (onSketchPos.y - firstPoint.y < 0) {
                        dy = -dy;
                    }
                    secondPoint.y = firstPoint.y + dy;
                }

                EditCurve[2] = secondPoint;
                EditCurve[1] = Base::Vector2d(secondPoint.x, firstPoint.y);
                EditCurve[3] = Base::Vector2d(firstPoint.x, secondPoint.y);
            }
            else if (constructionMethod == CenterAndCorner) {
                float dx, dy;
                if (sketchgui->toolSettings->widget->isSettingSet[2] == 1) {
                    dx = sketchgui->toolSettings->widget->toolParameters[2]/2;
                    secondPoint.x = center.x + dx;
                }
                else {
                    dx = onSketchPos.x - center.x;
                    secondPoint.x = onSketchPos.x;
                }
                if (sketchgui->toolSettings->widget->isSettingSet[3] == 1) {
                    dy = sketchgui->toolSettings->widget->toolParameters[3]/2;
                    secondPoint.y = center.y + dy;
                }
                else {
                    dy = onSketchPos.y - center.y;
                    secondPoint.y = onSketchPos.y;
                }
                SbString text;
                text.sprintf(" (%.1f x %.1f)", dx, dy);
                setPositionText(onSketchPos, text);

                EditCurve[0] = center - (secondPoint - center);
                EditCurve[1] = Base::Vector2d(EditCurve[0].x, secondPoint.y);
                EditCurve[2] = secondPoint;
                EditCurve[3] = Base::Vector2d(secondPoint.x,EditCurve[0].y);
                EditCurve[4] = EditCurve[0];
                // is user input length and width then validate the square.
                if (sketchgui->toolSettings->widget->isSettingSet[2] == 1 && sketchgui->toolSettings->widget->isSettingSet[3] == 1) {
                    pressButton(onSketchPos);
                    releaseButton(onSketchPos);
                }
            }

            drawEdit(EditCurve);
            if (seekAutoConstraint(sugConstr2, onSketchPos, Base::Vector2d(0.0,0.0))) {
                renderSuggestConstraintsCursor(sugConstr2);
                return;
            }

        }
        applyCursor();
    }

    virtual bool pressButton(Base::Vector2d onSketchPos)
    {
        if (Mode==STATUS_SEEK_First){
            if(constructionMethod == Diagonal) {
                if (sketchgui->toolSettings->widget->isSettingSet[0] == 1) {
                    firstPoint.x = sketchgui->toolSettings->widget->toolParameters[0];
                }
                else {
                    firstPoint.x = onSketchPos.x;
                }
                if (sketchgui->toolSettings->widget->isSettingSet[1] == 1) {
                    firstPoint.y = sketchgui->toolSettings->widget->toolParameters[1];
                }
                else {
                    firstPoint.y = onSketchPos.y;
                }
                EditCurve[0] = firstPoint;
                EditCurve[4] = firstPoint;
            }
            else if (constructionMethod == CenterAndCorner) {
                if (sketchgui->toolSettings->widget->isSettingSet[0] == 1) {
                    center.x = sketchgui->toolSettings->widget->toolParameters[0];
                }
                else {
                    center.x = onSketchPos.x;
                }
                if (sketchgui->toolSettings->widget->isSettingSet[1] == 1) {
                    center.y = sketchgui->toolSettings->widget->toolParameters[1];
                }
                else {
                    center.y = onSketchPos.y;
                }
            }

            sketchgui->toolSettings->widget->setParameterActive(0, 0);
            sketchgui->toolSettings->widget->setParameterActive(0, 1);
            sketchgui->toolSettings->widget->setParameterActive(1, 2);
            sketchgui->toolSettings->widget->setParameterActive(1, 3);
            sketchgui->toolSettings->widget->setParameterFocus(2);
            Mode = STATUS_SEEK_Second;
        }
        else { //Mode==STATUS_SEEK_Second
            if(constructionMethod == Diagonal) {
                //recalculate secondpoint in case mouse moved too fast for mousemove to catch it.
                float dx, dy;
                if (sketchgui->toolSettings->widget->isSettingSet[2] == 1) {
                    dx = sketchgui->toolSettings->widget->toolParameters[2];
                    if (onSketchPos.x - firstPoint.x < 0) {
                        dx = -dx;
                    }
                    secondPoint.x = firstPoint.x + dx;
                }
                else {
                    secondPoint.x = onSketchPos.x;
                }
                if (sketchgui->toolSettings->widget->isSettingSet[3] == 1) {
                    dy = sketchgui->toolSettings->widget->toolParameters[3];
                    if (onSketchPos.y - firstPoint.y < 0) {
                        dy = -dy;
                    }
                    secondPoint.y = firstPoint.y + dy;
                }
                else {
                    secondPoint.y = onSketchPos.y;
                }

                EditCurve[2] = secondPoint;
                EditCurve[1] = Base::Vector2d(secondPoint.x , firstPoint.y);
                EditCurve[3] = Base::Vector2d(firstPoint.x, secondPoint.y);
                drawEdit(EditCurve);
                Mode = STATUS_End;
            }
            else if (constructionMethod == CenterAndCorner) {
                if (sketchgui->toolSettings->widget->isSettingSet[2] == 1) {
                    secondPoint.x = center.x + sketchgui->toolSettings->widget->toolParameters[2] / 2;
                }
                else {
                    secondPoint.x = onSketchPos.x;
                }
                if (sketchgui->toolSettings->widget->isSettingSet[3] == 1) {
                    secondPoint.y = center.y + sketchgui->toolSettings->widget->toolParameters[3] / 2;
                }
                else {
                    secondPoint.y = onSketchPos.y;
                }

                EditCurve[0] = center - (secondPoint - center);
                EditCurve[1] = Base::Vector2d(EditCurve[0].x, secondPoint.y);
                EditCurve[2] = secondPoint;
                EditCurve[3] = Base::Vector2d(secondPoint.x, EditCurve[0].y);
                EditCurve[4] = EditCurve[0];
                drawEdit(EditCurve);
                Mode = STATUS_End;
            }
        }
        return true;
    }

    virtual bool releaseButton(Base::Vector2d onSketchPos)
    {
        Q_UNUSED(onSketchPos);
        if (Mode==STATUS_End){
            unsetCursor();
            resetPositionText();
            int firstCurve = getHighestCurveIndex() + 1;

            try {
                if(constructionMethod == Diagonal) {
                    Gui::Command::openCommand(QT_TRANSLATE_NOOP("Command", "Add sketch box"));
                    Gui::Command::doCommand(Gui::Command::Doc,
                        "geoList = []\n"
                        "geoList.append(Part.LineSegment(App.Vector(%f,%f,0),App.Vector(%f,%f,0)))\n"
                        "geoList.append(Part.LineSegment(App.Vector(%f,%f,0),App.Vector(%f,%f,0)))\n"
                        "geoList.append(Part.LineSegment(App.Vector(%f,%f,0),App.Vector(%f,%f,0)))\n"
                        "geoList.append(Part.LineSegment(App.Vector(%f,%f,0),App.Vector(%f,%f,0)))\n"
                        "%s.addGeometry(geoList,%s)\n"
                        "conList = []\n"
                        "conList.append(Sketcher.Constraint('Coincident',%i,2,%i,1))\n"
                        "conList.append(Sketcher.Constraint('Coincident',%i,2,%i,1))\n"
                        "conList.append(Sketcher.Constraint('Coincident',%i,2,%i,1))\n"
                        "conList.append(Sketcher.Constraint('Coincident',%i,2,%i,1))\n"
                        "conList.append(Sketcher.Constraint('Horizontal',%i))\n"
                        "conList.append(Sketcher.Constraint('Horizontal',%i))\n"
                        "conList.append(Sketcher.Constraint('Vertical',%i))\n"
                        "conList.append(Sketcher.Constraint('Vertical',%i))\n"
                        "%s.addConstraint(conList)\n"
                        "del geoList, conList\n",
                        EditCurve[0].x,EditCurve[0].y,EditCurve[1].x,EditCurve[1].y, // line 1
                        EditCurve[1].x,EditCurve[1].y,EditCurve[2].x,EditCurve[2].y, // line 2
                        EditCurve[2].x,EditCurve[2].y,EditCurve[3].x,EditCurve[3].y, // line 3
                        EditCurve[3].x,EditCurve[3].y,EditCurve[0].x,EditCurve[0].y, // line 4
                        Gui::Command::getObjectCmd(sketchgui->getObject()).c_str(), // the sketch
                        geometryCreationMode==Construction?"True":"False", // geometry as construction or not
                        firstCurve,firstCurve+1, // coincident1
                        firstCurve+1,firstCurve+2, // coincident2
                        firstCurve+2,firstCurve+3, // coincident3
                        firstCurve+3,firstCurve, // coincident4
                        firstCurve, // horizontal1
                        firstCurve+2, // horizontal2
                        firstCurve+1, // vertical1
                        firstCurve+3, // vertical2
                        Gui::Command::getObjectCmd(sketchgui->getObject()).c_str()); // the sketch

                //add constraint if user typed in some dimensions in tool widget
                    if (sketchgui->toolSettings->widget->isSettingSet[0] + sketchgui->toolSettings->widget->isSettingSet[1] + sketchgui->toolSettings->widget->isSettingSet[2] + sketchgui->toolSettings->widget->isSettingSet[3] != 0) {
                        if (sketchgui->toolSettings->widget->isSettingSet[0] == 1) {
                            distanceXYorPointOnObject(0, firstCurve, Sketcher::PointPos::start, sketchgui->toolSettings->widget->toolParameters[0]);
                        }
                        if (sketchgui->toolSettings->widget->isSettingSet[1] == 1) {
                            distanceXYorPointOnObject(1, firstCurve, Sketcher::PointPos::start, sketchgui->toolSettings->widget->toolParameters[1]);
                        }
                        if (sketchgui->toolSettings->widget->isSettingSet[2] == 1) {
                            Gui::cmdAppObjectArgs(sketchgui->getObject(), "addConstraint(Sketcher.Constraint('Distance',%d,%d,%d,%d,%f)) ",
                                firstCurve, 1, firstCurve, 2, sketchgui->toolSettings->widget->toolParameters[2]);
                        }
                        if (sketchgui->toolSettings->widget->isSettingSet[3] == 1) {
                            Gui::cmdAppObjectArgs(sketchgui->getObject(), "addConstraint(Sketcher.Constraint('Distance',%d,%d,%d,%d,%f)) ",
                                firstCurve + 3, 1, firstCurve + 3, 2, sketchgui->toolSettings->widget->toolParameters[3]);
                        }
                    }

                    Gui::Command::commitCommand();
                }
                else if (constructionMethod == CenterAndCorner) {
                    Gui::Command::openCommand(QT_TRANSLATE_NOOP("Command", "Add centered sketch box"));
                    Gui::Command::doCommand(Gui::Command::Doc,
                        "geoList = []\n"
                        "geoList.append(Part.LineSegment(App.Vector(%f,%f,0),App.Vector(%f,%f,0)))\n"
                        "geoList.append(Part.LineSegment(App.Vector(%f,%f,0),App.Vector(%f,%f,0)))\n"
                        "geoList.append(Part.LineSegment(App.Vector(%f,%f,0),App.Vector(%f,%f,0)))\n"
                        "geoList.append(Part.LineSegment(App.Vector(%f,%f,0),App.Vector(%f,%f,0)))\n"
                        "geoList.append(Part.Point(App.Vector(%f,%f,0)))\n"
                        "%s.addGeometry(geoList,%s)\n"
                        "conList = []\n"
                        "conList.append(Sketcher.Constraint('Coincident',%i,2,%i,1))\n"
                        "conList.append(Sketcher.Constraint('Coincident',%i,2,%i,1))\n"
                        "conList.append(Sketcher.Constraint('Coincident',%i,2,%i,1))\n"
                        "conList.append(Sketcher.Constraint('Coincident',%i,2,%i,1))\n"
                        "conList.append(Sketcher.Constraint('Horizontal',%i))\n"
                        "conList.append(Sketcher.Constraint('Horizontal',%i))\n"
                        "conList.append(Sketcher.Constraint('Vertical',%i))\n"
                        "conList.append(Sketcher.Constraint('Vertical',%i))\n"
                        "conList.append(Sketcher.Constraint('Symmetric',%i,2,%i,1,%i,1))\n"
                        "%s.addConstraint(conList)\n"
                        "del geoList, conList\n",
                        EditCurve[0].x,EditCurve[0].y,EditCurve[1].x,EditCurve[1].y, // line 1
                        EditCurve[1].x,EditCurve[1].y,EditCurve[2].x,EditCurve[2].y, // line 2
                        EditCurve[2].x,EditCurve[2].y,EditCurve[3].x,EditCurve[3].y, // line 3
                        EditCurve[3].x,EditCurve[3].y,EditCurve[0].x,EditCurve[0].y, // line 4
                        center.x,center.y,                                           // center point
                        Gui::Command::getObjectCmd(sketchgui->getObject()).c_str(), // the sketch
                        geometryCreationMode==Construction?"True":"False", // geometry as construction or not
                        firstCurve,firstCurve+1, // coincident1
                        firstCurve+1,firstCurve+2, // coincident2
                        firstCurve+2,firstCurve+3, // coincident3
                        firstCurve+3,firstCurve, // coincident4
                        firstCurve+1, // horizontal1
                        firstCurve+3, // horizontal2
                        firstCurve, // vertical1
                        firstCurve+2, // vertical2
                        firstCurve+1, firstCurve, firstCurve + 4, // Symmetric
                        Gui::Command::getObjectCmd(sketchgui->getObject()).c_str()); // the sketch

                //add constraint if user typed in some dimensions in tool widget
                    if (sketchgui->toolSettings->widget->isSettingSet[0] + sketchgui->toolSettings->widget->isSettingSet[1] + sketchgui->toolSettings->widget->isSettingSet[2] + sketchgui->toolSettings->widget->isSettingSet[3] != 0) {
                        if (sketchgui->toolSettings->widget->isSettingSet[0] == 1) {
                            distanceXYorPointOnObject(0, firstCurve + 4, Sketcher::PointPos::start, sketchgui->toolSettings->widget->toolParameters[0]);
                        }
                        if (sketchgui->toolSettings->widget->isSettingSet[1] == 1) {
                            distanceXYorPointOnObject(1, firstCurve + 4, Sketcher::PointPos::start, sketchgui->toolSettings->widget->toolParameters[1]);
                        }
                        if (sketchgui->toolSettings->widget->isSettingSet[2] == 1) {
                            Gui::cmdAppObjectArgs(sketchgui->getObject(), "addConstraint(Sketcher.Constraint('Distance',%d,%d,%d,%d,%f)) ",
                                firstCurve + 3, 1, firstCurve + 3, 2, sketchgui->toolSettings->widget->toolParameters[2]);
                        }
                        if (sketchgui->toolSettings->widget->isSettingSet[3] == 1) {
                            Gui::cmdAppObjectArgs(sketchgui->getObject(), "addConstraint(Sketcher.Constraint('Distance',%d,%d,%d,%d,%f)) ",
                                firstCurve, 1, firstCurve, 2, sketchgui->toolSettings->widget->toolParameters[3]);
                        }
                    }

                    Gui::Command::commitCommand();
                }
            }
            catch (const Base::Exception& e) {
                Base::Console().Error("Failed to add box: %s\n", e.what());
                Gui::Command::abortCommand();
            }

            if(constructionMethod == Diagonal) {
                // add auto constraints at the start of the first side
                if (sugConstr1.size() > 0) {
                    createAutoConstraints(sugConstr1, getHighestCurveIndex() - 3 , Sketcher::PointPos::start);
                    sugConstr1.clear();
                }

                // add auto constraints at the end of the second side
                if (sugConstr2.size() > 0) {
                    createAutoConstraints(sugConstr2, getHighestCurveIndex() - 2, Sketcher::PointPos::end);
                    sugConstr2.clear();
                }

            }
            else if (constructionMethod == CenterAndCorner) {
                // add auto constraints at the start of the first side
                if (sugConstr1.size() > 0) {
                    createAutoConstraints(sugConstr1, getHighestCurveIndex(), Sketcher::PointPos::start);
                    sugConstr1.clear();
                }

                // add auto constraints at the end of the second side
                if (sugConstr2.size() > 0) {
                    createAutoConstraints(sugConstr2, getHighestCurveIndex() - 3, Sketcher::PointPos::end);
                    sugConstr2.clear();
                }
            }

            tryAutoRecomputeIfNotSolve(static_cast<Sketcher::SketchObject *>(sketchgui->getObject()));

            ParameterGrp::handle hGrp = App::GetApplication().GetParameterGroupByPath("User parameter:BaseApp/Preferences/Mod/Sketcher");
            bool continuousMode = hGrp->GetBool("ContinuousCreationMode",true);
            if(continuousMode){
            // This code enables the continuous creation mode.

                sketchgui->toolSettings->widget->setSettings(1);
                Mode=STATUS_SEEK_First;
                EditCurve.clear();
                drawEdit(EditCurve);
                EditCurve.resize(5);
                applyCursor();
                /* this is ok not to call to purgeHandler
                * in continuous creation mode because the
                * handler is destroyed by the quit() method on pressing the
                * right button of the mouse */
            }
            else{
                sketchgui->purgeHandler(); // no code after this line, Handler get deleted in ViewProvider
            }


        }
        return true;
    }
protected:
    BoxMode Mode;
    std::vector<Base::Vector2d> EditCurve;
    std::vector<AutoConstraint> sugConstr1, sugConstr2;
    ConstructionMethod constructionMethod;
    Base::Vector2d center, firstPoint, secondPoint;
};

DEF_STD_CMD_AU(CmdSketcherCreateRectangle)

CmdSketcherCreateRectangle::CmdSketcherCreateRectangle()
  : Command("Sketcher_CreateRectangle")
{
    sAppModule      = "Sketcher";
    sGroup          = "Sketcher";
    sMenuText       = QT_TR_NOOP("Create rectangle");
    sToolTipText    = QT_TR_NOOP("Create a rectangle in the sketch");
    sWhatsThis      = "Sketcher_CreateRectangle";
    sStatusTip      = sToolTipText;
    sPixmap         = "Sketcher_CreateRectangle";
    sAccel          = "G, R";
    eType           = ForEdit;
}

void CmdSketcherCreateRectangle::activated(int iMsg)
{
    Q_UNUSED(iMsg);
    ActivateHandler(getActiveGuiDocument(),new DrawSketchHandlerBox(DrawSketchHandlerBox::Diagonal) );
}

void CmdSketcherCreateRectangle::updateAction(int mode)
{
    switch (mode) {
    case Normal:
        if (getAction())
            getAction()->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateRectangle"));
        break;
    case Construction:
        if (getAction())
            getAction()->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateRectangle_Constr"));
        break;
    }
}

bool CmdSketcherCreateRectangle::isActive(void)
{
    return isCreateGeoActive(getActiveGuiDocument());
}

DEF_STD_CMD_AU(CmdSketcherCreateRectangleCenter)

CmdSketcherCreateRectangleCenter::CmdSketcherCreateRectangleCenter()
  : Command("Sketcher_CreateRectangle_Center")
{
    sAppModule      = "Sketcher";
    sGroup          = "Sketcher";
    sMenuText       = QT_TR_NOOP("Create centered rectangle");
    sToolTipText    = QT_TR_NOOP("Create a centered rectangle in the sketch");
    sWhatsThis      = "Sketcher_CreateRectangle_Center";
    sStatusTip      = sToolTipText;
    sPixmap         = "Sketcher_CreateRectangle_Center";
    sAccel          = "G, V";
    eType           = ForEdit;
}

void CmdSketcherCreateRectangleCenter::activated(int iMsg)
{
    Q_UNUSED(iMsg);
    ActivateHandler(getActiveGuiDocument(),new DrawSketchHandlerBox(DrawSketchHandlerBox::CenterAndCorner) );
}

void CmdSketcherCreateRectangleCenter::updateAction(int mode)
{
    switch (mode) {
    case Normal:
        if (getAction())
            getAction()->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateRectangle_Center"));
        break;
    case Construction:
        if (getAction())
            getAction()->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateRectangle_Center_Constr"));
        break;
    }
}

bool CmdSketcherCreateRectangleCenter::isActive(void)
{
    return isCreateGeoActive(getActiveGuiDocument());
}


/* Create rounded oblong =======================================================*/

class DrawSketchHandlerOblong : public DrawSketchHandler
{
public:
    DrawSketchHandlerOblong()
        : Mode(STATUS_SEEK_First)
        , lengthX(0), lengthY(0), radius(0), signX(1), signY(1)
        , EditCurve(37)
    {
    }
    virtual ~DrawSketchHandlerOblong() {}
    /// mode table
    enum BoxMode {
        STATUS_SEEK_First,      /**< enum value ----. */
        STATUS_SEEK_Second,     /**< enum value ----. */
        STATUS_SEEK_Third,     /**< enum value ----. */
        STATUS_End
    };

    virtual void activated(ViewProviderSketch*)
    {
        setCrosshairCursor("Sketcher_Pointer_Oblong");
        sketchgui->toolSettings->widget->setSettings(2);
    }

    virtual void mouseMove(Base::Vector2d onSketchPos)
    {
        if (Mode == STATUS_SEEK_First) {
            setPositionText(onSketchPos);
            if (seekAutoConstraint(sugConstr1, onSketchPos, Base::Vector2d(0.f, 0.f))) {
                renderSuggestConstraintsCursor(sugConstr1);
                return;
            }
            if (sketchgui->toolSettings->widget->isSettingSet[0] + sketchgui->toolSettings->widget->isSettingSet[1] == 2) {
                pressButton(onSketchPos);
                releaseButton(onSketchPos);
            }
        }
        else if (Mode == STATUS_SEEK_Second) {
            if (sketchgui->toolSettings->widget->isSettingSet[2] == 1) {
                lengthX = sketchgui->toolSettings->widget->toolParameters[2];
                if (onSketchPos.x - StartPos.x < 0) {
                    lengthX = -lengthX;
                }
                EndPos.x = StartPos.x + lengthX;
            }
            else {
                lengthX = onSketchPos.x - StartPos.x;
                EndPos.x = onSketchPos.x;
            }
            if (sketchgui->toolSettings->widget->isSettingSet[3] == 1) {
                lengthY = sketchgui->toolSettings->widget->toolParameters[3];
                if (onSketchPos.y - StartPos.y < 0) {
                    lengthY = -lengthY;
                }
                EndPos.y = StartPos.y + lengthY;
            }
            else {
                lengthY = onSketchPos.y - StartPos.y;
                EndPos.y = onSketchPos.y;
            }

            signX = Base::sgn(lengthX);
            signY = Base::sgn(lengthY);
            if (fabs(lengthX) > fabs(lengthY)) {
                radius = fabs(lengthY) / 5; // we use a fifth of the smaller distance as default radius
            }
            else {
                radius = fabs(lengthX) / 5;
            }

            // we draw the lines with 36 segments, 8 for each arc and 4 lines
            // draw the arcs
            for (int i = 0; i < 8; i++) {
                // calculate the x,y positions forming the the arc
                double angle = i * M_PI / 16.0;
                double x_i = -radius * sin(angle);
                double y_i = -radius * cos(angle);
                // we are drawing clockwise starting with the arc that is besides StartPos
                if (signX == signY) {
                    EditCurve[i] = Base::Vector2d(StartPos.x + signX * (radius + x_i), StartPos.y + signY * (radius + y_i));
                    EditCurve[9 + i] = Base::Vector2d(StartPos.x + signY * (radius + y_i), StartPos.y + lengthY - signX * (radius + x_i));
                    EditCurve[18 + i] = Base::Vector2d(StartPos.x + lengthX - signX * (radius + x_i), StartPos.y + lengthY - signY * (radius + y_i));
                    EditCurve[27 + i] = Base::Vector2d(StartPos.x + lengthX - signY * (radius + y_i), StartPos.y + signX * (radius + x_i));
                }
                else {
                    EditCurve[i] = Base::Vector2d(StartPos.x - signY * (radius + y_i), StartPos.y - signX * (radius + x_i));
                    EditCurve[9 + i] = Base::Vector2d(StartPos.x + lengthX - signX * (radius + x_i), StartPos.y + signY * (radius + y_i));
                    EditCurve[18 + i] = Base::Vector2d(StartPos.x + lengthX + signY * (radius + y_i), StartPos.y + lengthY + signX * (radius + x_i));
                    EditCurve[27 + i] = Base::Vector2d(StartPos.x + signX * (radius + x_i), StartPos.y + lengthY - signY * (radius + y_i));
                }
            }
            // draw the lines
            if (signX == signY) {
                EditCurve[8] = Base::Vector2d(StartPos.x, StartPos.y + (signY * radius));
                EditCurve[17] = Base::Vector2d(StartPos.x + (signX * radius), StartPos.y + lengthY);
                EditCurve[26] = Base::Vector2d(StartPos.x + lengthX, StartPos.y + lengthY - (signY * radius));
                EditCurve[35] = Base::Vector2d(StartPos.x + lengthX - (signX * radius), StartPos.y);
            }
            else {
                EditCurve[8] = Base::Vector2d(StartPos.x + (signX * radius), StartPos.y);
                EditCurve[17] = Base::Vector2d(StartPos.x + lengthX, StartPos.y + (signY * radius));
                EditCurve[26] = Base::Vector2d(StartPos.x + lengthX - (signX * radius), StartPos.y + lengthY);
                EditCurve[35] = Base::Vector2d(StartPos.x, StartPos.y + lengthY - (signY * radius));
            }
            // close the curve
            EditCurve[36] = EditCurve[0];

            SbString text;
            text.sprintf(" (%.1fR %.1fX %.1fY)", radius, lengthX, lengthY);
            setPositionText(onSketchPos, text);

            drawEdit(EditCurve);
            if (seekAutoConstraint(sugConstr2, onSketchPos, Base::Vector2d(0.f, 0.f))) {
                renderSuggestConstraintsCursor(sugConstr2);
                return;
            }
        }
        else if (Mode == STATUS_SEEK_Third) {
            if (sketchgui->toolSettings->widget->isSettingSet[4] == 1) {
                radius = min(sketchgui->toolSettings->widget->toolParameters[4] , min(abs(lengthX / 2), abs(lengthY / 2)));
            }
            else {
                double dx, dy, minX, minY, maxX, maxY;
                minX = min(StartPos.x, EndPos.x);
                maxX = max(StartPos.x, EndPos.x);
                minY = min(StartPos.y, EndPos.y);
                maxY = max(StartPos.y, EndPos.y);
                if (onSketchPos.x < minX || onSketchPos.y < minY || onSketchPos.x > maxX || onSketchPos.y > maxY) {
                    radius = 0.001;
                }
                else {
                    dx = onSketchPos.x - minX;
                    dy = onSketchPos.y - minY;
                    if (dx < abs(lengthX / 2)) {
                        dx = (onSketchPos.x - minX);
                    }
                    else {
                        dx = -(onSketchPos.x - maxX);
                    }
                    dy = onSketchPos.y - minY;
                    if (dy < abs(lengthY / 2)) {
                        dy = (onSketchPos.y - minY);
                    }
                    else {
                        dy = -(onSketchPos.y - maxY);
                    }
                    radius = min((dx + dy +sqrt(2*dx*dy)), min(abs(lengthX / 2), abs(lengthY / 2)) * 0.99);
                }
            }

            // we draw the lines with 36 segments, 8 for each arc and 4 lines
            // draw the arcs
            for (int i = 0; i < 8; i++) {
                // calculate the x,y positions forming the the arc
                double angle = i * M_PI / 16.0;
                double x_i = -radius * sin(angle);
                double y_i = -radius * cos(angle);
                // we are drawing clockwise starting with the arc that is besides StartPos
                if (signX == signY) {
                    EditCurve[i] = Base::Vector2d(StartPos.x + signX * (radius + x_i), StartPos.y + signY * (radius + y_i));
                    EditCurve[9 + i] = Base::Vector2d(StartPos.x + signY * (radius + y_i), StartPos.y + lengthY - signX * (radius + x_i));
                    EditCurve[18 + i] = Base::Vector2d(StartPos.x + lengthX - signX * (radius + x_i), StartPos.y + lengthY - signY * (radius + y_i));
                    EditCurve[27 + i] = Base::Vector2d(StartPos.x + lengthX - signY * (radius + y_i), StartPos.y + signX * (radius + x_i));
                }
                else {
                    EditCurve[i] = Base::Vector2d(StartPos.x - signY * (radius + y_i), StartPos.y - signX * (radius + x_i));
                    EditCurve[9 + i] = Base::Vector2d(StartPos.x + lengthX - signX * (radius + x_i), StartPos.y + signY * (radius + y_i));
                    EditCurve[18 + i] = Base::Vector2d(StartPos.x + lengthX + signY * (radius + y_i), StartPos.y + lengthY + signX * (radius + x_i));
                    EditCurve[27 + i] = Base::Vector2d(StartPos.x + signX * (radius + x_i), StartPos.y + lengthY - signY * (radius + y_i));
                }
            }
            // draw the lines
            if (signX == signY) {
                EditCurve[8] = Base::Vector2d(StartPos.x, StartPos.y + (signY * radius));
                EditCurve[17] = Base::Vector2d(StartPos.x + (signX * radius), StartPos.y + lengthY);
                EditCurve[26] = Base::Vector2d(StartPos.x + lengthX, StartPos.y + lengthY - (signY * radius));
                EditCurve[35] = Base::Vector2d(StartPos.x + lengthX - (signX * radius), StartPos.y);
            }
            else {
                EditCurve[8] = Base::Vector2d(StartPos.x + (signX * radius), StartPos.y);
                EditCurve[17] = Base::Vector2d(StartPos.x + lengthX, StartPos.y + (signY * radius));
                EditCurve[26] = Base::Vector2d(StartPos.x + lengthX - (signX * radius), StartPos.y + lengthY);
                EditCurve[35] = Base::Vector2d(StartPos.x, StartPos.y + lengthY - (signY * radius));
            }
            // close the curve
            EditCurve[36] = EditCurve[0];

            SbString text;
            text.sprintf(" (%.1fR %.1fX %.1fY)", radius, lengthX, lengthY);
            setPositionText(onSketchPos, text);

            drawEdit(EditCurve);
            if (sketchgui->toolSettings->widget->isSettingSet[4] == 1) {
                pressButton(onSketchPos);
                releaseButton(onSketchPos);
            }
        }
        applyCursor();
    }

    virtual bool pressButton(Base::Vector2d onSketchPos)
    {
        if (Mode == STATUS_SEEK_First) {
            if (sketchgui->toolSettings->widget->isSettingSet[0] == 1) {
                StartPos.x = sketchgui->toolSettings->widget->toolParameters[0];
            }
            else {
                StartPos.x = onSketchPos.x;
            }
            if (sketchgui->toolSettings->widget->isSettingSet[1] == 1) {
                StartPos.y = sketchgui->toolSettings->widget->toolParameters[1];
            }
            else {
                StartPos.y = onSketchPos.y;
            }

            sketchgui->toolSettings->widget->setParameterActive(0, 0);
            sketchgui->toolSettings->widget->setParameterActive(0, 1);
            sketchgui->toolSettings->widget->setParameterActive(1, 2);
            sketchgui->toolSettings->widget->setParameterActive(1, 3);
            sketchgui->toolSettings->widget->setParameterFocus(2);
            Mode = STATUS_SEEK_Second;
        }
        else if (Mode == STATUS_SEEK_Second) {
            if (sketchgui->toolSettings->widget->isSettingSet[2] == 1) {
                lengthX = sketchgui->toolSettings->widget->toolParameters[2];
                if (onSketchPos.x - StartPos.x < 0) {
                    lengthX = -lengthX;
                }
                EndPos.x = StartPos.x + lengthX;
            }
            else {
                lengthX = onSketchPos.x - StartPos.x;
                EndPos.x = onSketchPos.x;
            }
            if (sketchgui->toolSettings->widget->isSettingSet[3] == 1) {
                lengthY = sketchgui->toolSettings->widget->toolParameters[3];
                if (onSketchPos.y - StartPos.y < 0) {
                    lengthY = -lengthY;
                }
                EndPos.y = StartPos.y + lengthY;
            }
            else {
                lengthY = onSketchPos.y - StartPos.y;
                EndPos.y = onSketchPos.y;
            }

            sketchgui->toolSettings->widget->setParameterActive(0, 2);
            sketchgui->toolSettings->widget->setParameterActive(0, 3);
            sketchgui->toolSettings->widget->setParameterActive(1, 4);
            sketchgui->toolSettings->widget->setParameterFocus(4);
            Mode = STATUS_SEEK_Third;
        }
        else {
            if (sketchgui->toolSettings->widget->isSettingSet[4] == 1) {
                radius = min(sketchgui->toolSettings->widget->toolParameters[4], min(abs(lengthX / 2), abs(lengthY / 2)));
            }
            else {
                double dx, dy, minX, minY, maxX, maxY;
                minX = min(StartPos.x, EndPos.x);
                maxX = max(StartPos.x, EndPos.x);
                minY = min(StartPos.y, EndPos.y);
                maxY = max(StartPos.y, EndPos.y);
                if (onSketchPos.x < minX || onSketchPos.y < minY || onSketchPos.x > maxX || onSketchPos.y > maxY) {
                    radius = 0.001;
                }
                else {
                    dx = onSketchPos.x - minX;
                    dy = onSketchPos.y - minY;
                    if (dx < abs(lengthX / 2)) {
                        dx = (onSketchPos.x - minX);
                    }
                    else {
                        dx = -(onSketchPos.x - maxX);
                    }
                    dy = onSketchPos.y - minY;
                    if (dy < abs(lengthY / 2)) {
                        dy = (onSketchPos.y - minY);
                    }
                    else {
                        dy = -(onSketchPos.y - maxY);
                    }
                    radius = min((dx + dy +sqrt(2*dx*dy)), min(abs(lengthX / 2), abs(lengthY / 2)) * 0.99);
                }
            }
            Mode = STATUS_End;
        }
        return true;
    }

    virtual bool releaseButton(Base::Vector2d onSketchPos)
    {
        Q_UNUSED(onSketchPos);
        if (Mode == STATUS_End) {
            unsetCursor();
            resetPositionText();

            int firstCurve = getHighestCurveIndex() + 1;
            // add the geometry to the sketch
            // first determine the angles for the first arc
            double start = 0;
            double end = M_PI / 2;
            if (signX > 0 && signY > 0) {
                start = -2 * end;
                end = -1 * end;
            }
            else if (signX > 0 && signY < 0) {
                start = end;
                end = 2 * end;
            }
            else if (signX < 0 && signY > 0) {
                start = -1 * end;
                end = 0;
            }

            try {
                Gui::Command::openCommand(QT_TRANSLATE_NOOP("Command", "Add rounded rectangle"));
                Gui::Command::doCommand(Gui::Command::Doc,
                    // syntax for arcs: Part.ArcOfCircle(Part.Circle(center, axis, radius), startangle, endangle)
                    "geoList = []\n"
                    "geoList.append(Part.ArcOfCircle(Part.Circle(App.Vector(%f, %f, 0), App.Vector(0, 0, 1), %f), %f, %f))\n"
                    "geoList.append(Part.LineSegment(App.Vector(%f, %f, 0), App.Vector(%f, %f, 0)))\n"
                    "geoList.append(Part.ArcOfCircle(Part.Circle(App.Vector(%f, %f, 0), App.Vector(0, 0, 1), %f), %f, %f))\n"
                    "geoList.append(Part.LineSegment(App.Vector(%f, %f, 0), App.Vector(%f, %f, 0)))\n"
                    "geoList.append(Part.ArcOfCircle(Part.Circle(App.Vector(%f, %f, 0), App.Vector(0, 0, 1), %f), %f, %f))\n"
                    "geoList.append(Part.LineSegment(App.Vector(%f, %f, 0), App.Vector(%f, %f, 0)))\n"
                    "geoList.append(Part.ArcOfCircle(Part.Circle(App.Vector(%f, %f, 0), App.Vector(0, 0, 1), %f), %f, %f))\n"
                    "geoList.append(Part.LineSegment(App.Vector(%f, %f, 0), App.Vector(%f, %f, 0)))\n"
                    "%s.addGeometry(geoList, %s)\n"
                    "conList = []\n"
                    "conList.append(Sketcher.Constraint('Tangent', %i, 1, %i, 1))\n"
                    "conList.append(Sketcher.Constraint('Tangent', %i, 2, %i, 2))\n"
                    "conList.append(Sketcher.Constraint('Tangent', %i, 1, %i, 1))\n"
                    "conList.append(Sketcher.Constraint('Tangent', %i, 2, %i, 2))\n"
                    "conList.append(Sketcher.Constraint('Tangent', %i, 1, %i, 1))\n"
                    "conList.append(Sketcher.Constraint('Tangent', %i, 2, %i, 2))\n"
                    "conList.append(Sketcher.Constraint('Tangent', %i, 1, %i, 1))\n"
                    "conList.append(Sketcher.Constraint('Tangent', %i, 2, %i, 2))\n"
                    "conList.append(Sketcher.Constraint('Horizontal', %i))\n"
                    "conList.append(Sketcher.Constraint('Horizontal', %i))\n"
                    "conList.append(Sketcher.Constraint('Vertical', %i))\n"
                    "conList.append(Sketcher.Constraint('Vertical', %i))\n"
                    "conList.append(Sketcher.Constraint('Equal', %i, %i))\n"
                    "conList.append(Sketcher.Constraint('Equal', %i, %i))\n"
                    "conList.append(Sketcher.Constraint('Equal', %i, %i))\n"
                    "%s.addConstraint(conList)\n"
                    "del geoList, conList\n",
                    StartPos.x + (signX * radius), StartPos.y + (signY * radius), // center of the  arc 1
                    radius,
                    start, end,                 // start and end angle of arc1
                    EditCurve[8].x, EditCurve[8].y, EditCurve[9].x, EditCurve[9].y, // line 1
                    signX == signY ? StartPos.x + (signX * radius) : StartPos.x + lengthX - (signX * radius), // center of the arc 2
                    signX == signY ? StartPos.y + lengthY - (signY * radius) : StartPos.y + (signY * radius),
                    radius,
                    // start and end angle of arc 2
                    // the logic is that end is start + M_PI / 2 and start is the previous end - M_PI
                    end - M_PI,
                    end - 0.5 * M_PI,
                    EditCurve[17].x, EditCurve[17].y, EditCurve[18].x, EditCurve[18].y, // line 2
                    StartPos.x + lengthX - (signX * radius), StartPos.y + lengthY - (signY * radius),  // center of the arc 3
                    radius,
                    end - 1.5 * M_PI,
                    end - M_PI,
                    EditCurve[26].x, EditCurve[26].y, EditCurve[27].x, EditCurve[27].y, // line 3
                    signX == signY ? StartPos.x + lengthX - (signX * radius) : StartPos.x + (signX * radius), // center of the arc 4
                    signX == signY ? StartPos.y + (signY * radius) : StartPos.y + lengthY - (signY * radius),
                    radius,
                    end - 2 * M_PI,
                    end - 1.5 * M_PI,
                    EditCurve[35].x, EditCurve[35].y, EditCurve[36].x, EditCurve[36].y, // line 4
                    Gui::Command::getObjectCmd(sketchgui->getObject()).c_str(), // the sketch
                    geometryCreationMode == Construction ? "True" : "False", // geometry as construction or not
                    firstCurve, firstCurve + 1,     // tangent 1
                    firstCurve + 1, firstCurve + 2, // tangent 2
                    firstCurve + 2, firstCurve + 3, // tangent 3
                    firstCurve + 3, firstCurve + 4, // tangent 4
                    firstCurve + 4, firstCurve + 5, // tangent 5
                    firstCurve + 5, firstCurve + 6, // tangent 6
                    firstCurve + 6, firstCurve + 7, // tangent 7
                    firstCurve + 7, firstCurve,     // tangent 8
                    signX == signY ? firstCurve + 3 : firstCurve + 1, // horizontal constraint
                    signX == signY ? firstCurve + 7 : firstCurve + 5, // horizontal constraint
                    signX == signY ? firstCurve + 1 : firstCurve + 3, // vertical constraint
                    signX == signY ? firstCurve + 5 : firstCurve + 7, // vertical constraint
                    firstCurve, firstCurve + 2,     // equal  1
                    firstCurve + 2, firstCurve + 4, // equal  2
                    firstCurve + 4, firstCurve + 6, // equal  3
                    Gui::Command::getObjectCmd(sketchgui->getObject()).c_str()); // the sketch

                // now add construction geometry - two points used to take suggested constraints
                Gui::Command::doCommand(Gui::Command::Doc,
                    "geoList = []\n"
                    "geoList.append(Part.Point(App.Vector(%f, %f, 0)))\n"
                    "geoList.append(Part.Point(App.Vector(%f, %f, 0)))\n"
                    "%s.addGeometry(geoList, True)\n" // geometry as construction
                    "conList = []\n"
                    "conList.append(Sketcher.Constraint('PointOnObject', %i, 1, %i, ))\n"
                    "conList.append(Sketcher.Constraint('PointOnObject', %i, 1, %i, ))\n"
                    "conList.append(Sketcher.Constraint('PointOnObject', %i, 1, %i, ))\n"
                    "conList.append(Sketcher.Constraint('PointOnObject', %i, 1, %i, ))\n"
                    "%s.addConstraint(conList)\n"
                    "del geoList, conList\n",
                    StartPos.x, StartPos.y, // point at StartPos
                    EndPos.x, EndPos.y,     // point at EndPos
                    Gui::Command::getObjectCmd(sketchgui->getObject()).c_str(), // the sketch
                    firstCurve + 8, firstCurve + 1, // point on object constraint
                    firstCurve + 8, firstCurve + 7, // point on object constraint
                    firstCurve + 9, firstCurve + 3, // point on object constraint
                    firstCurve + 9, firstCurve + 5, // point on object constraint
                    Gui::Command::getObjectCmd(sketchgui->getObject()).c_str()); // the sketch

                //add constraint if user typed in some dimensions in tool widget
                if (sketchgui->toolSettings->widget->isSettingSet[0] + sketchgui->toolSettings->widget->isSettingSet[1] + sketchgui->toolSettings->widget->isSettingSet[2] + sketchgui->toolSettings->widget->isSettingSet[3] != 0) {
                    if (sketchgui->toolSettings->widget->isSettingSet[0] == 1) {
                        distanceXYorPointOnObject(0, signX == signY ? firstCurve + 1 : firstCurve + 7, signX == signY ? Sketcher::PointPos::start : Sketcher::PointPos::end, sketchgui->toolSettings->widget->toolParameters[0]);
                    }
                    if (sketchgui->toolSettings->widget->isSettingSet[1] == 1) {
                        distanceXYorPointOnObject(1, signX == signY ? firstCurve + 7 : firstCurve + 1, signX == signY ? Sketcher::PointPos::end : Sketcher::PointPos::start, sketchgui->toolSettings->widget->toolParameters[1]);
                    }
                    if (sketchgui->toolSettings->widget->isSettingSet[2] == 1) {
                        Gui::cmdAppObjectArgs(sketchgui->getObject(), "addConstraint(Sketcher.Constraint('Distance',%d,%d,%d,%d,%f)) ",
                            signX == signY ? firstCurve + 1 : firstCurve + 3, 1, signX == signY ? firstCurve + 5 : firstCurve + 7, 2, sketchgui->toolSettings->widget->toolParameters[2]);
                    }
                    if (sketchgui->toolSettings->widget->isSettingSet[3] == 1) {
                        Gui::cmdAppObjectArgs(sketchgui->getObject(), "addConstraint(Sketcher.Constraint('Distance',%d,%d,%d,%d,%f)) ",
                            signX == signY ? firstCurve + 3 : firstCurve + 1, 1, signX == signY ? firstCurve + 7 : firstCurve + 5, 2, sketchgui->toolSettings->widget->toolParameters[3]);
                    }
                    if (sketchgui->toolSettings->widget->isSettingSet[4] == 1) {
                        Gui::cmdAppObjectArgs(sketchgui->getObject(), "addConstraint(Sketcher.Constraint('Radius',%d,%f)) ",
                            firstCurve, radius);
                    }
                }

                Gui::Command::commitCommand();

                // add auto constraints at the StartPos auxiliary point
                if (sugConstr1.size() > 0) {
                    createAutoConstraints(sugConstr1, getHighestCurveIndex() - 1, Sketcher::PointPos::start);
                    sugConstr1.clear();
                }

                // add auto constraints at the EndPos auxiliary point
                if (sugConstr2.size() > 0) {
                    createAutoConstraints(sugConstr2, getHighestCurveIndex(), Sketcher::PointPos::start);
                    sugConstr2.clear();
                }

                tryAutoRecomputeIfNotSolve(static_cast<Sketcher::SketchObject*>(sketchgui->getObject()));
            }
            catch (const Base::Exception& e) {
                Base::Console().Error("Failed to add rounded rectangle: %s\n", e.what());
                Gui::Command::abortCommand();

                tryAutoRecompute(static_cast<Sketcher::SketchObject*>(sketchgui->getObject()));
            }
            
            ParameterGrp::handle hGrp = App::GetApplication().GetParameterGroupByPath("User parameter:BaseApp/Preferences/Mod/Sketcher");
            bool continuousMode = hGrp->GetBool("ContinuousCreationMode", true);

            if (continuousMode) {
                // This code enables the continuous creation mode.
                sketchgui->toolSettings->widget->setSettings(2);
                Mode = STATUS_SEEK_First;
                EditCurve.clear();
                drawEdit(EditCurve);
                EditCurve.resize(37);
                applyCursor();
                /* this is ok not to call to purgeHandler
                * in continuous creation mode because the
                * handler is destroyed by the quit() method on pressing the
                * right button of the mouse */
            }
            else {
                sketchgui->purgeHandler(); // no code after this line, Handler get deleted in ViewProvider
            }
        }
        return true;
    }
protected:
    BoxMode Mode;
    Base::Vector2d StartPos, EndPos;
    double lengthX, lengthY, radius;
    float signX, signY;
    std::vector<Base::Vector2d> EditCurve;
    std::vector<AutoConstraint> sugConstr1, sugConstr2;
};

DEF_STD_CMD_AU(CmdSketcherCreateOblong)

CmdSketcherCreateOblong::CmdSketcherCreateOblong()
    : Command("Sketcher_CreateOblong")
{
    sAppModule = "Sketcher";
    sGroup = "Sketcher";
    sMenuText = QT_TR_NOOP("Create rounded rectangle");
    sToolTipText = QT_TR_NOOP("Create a rounded rectangle in the sketch");
    sWhatsThis = "Sketcher_CreateOblong";
    sStatusTip = sToolTipText;
    sPixmap = "Sketcher_CreateOblong";
    sAccel = "G, O";
    eType = ForEdit;
}

void CmdSketcherCreateOblong::activated(int iMsg)
{
    Q_UNUSED(iMsg);
    ActivateHandler(getActiveGuiDocument(), new DrawSketchHandlerOblong());
}

void CmdSketcherCreateOblong::updateAction(int mode)
{
    switch (mode) {
    case Normal:
        if (getAction())
            getAction()->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateOblong"));
        break;
    case Construction:
        if (getAction())
            getAction()->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateOblong_Constr"));
        break;
    }
}

bool CmdSketcherCreateOblong::isActive(void)
{
    return isCreateGeoActive(getActiveGuiDocument());
}

/* Create frame =======================================================*/

class DrawSketchHandlerFrame : public DrawSketchHandler
{
public:
    DrawSketchHandlerFrame(): Mode(STATUS_SEEK_First), thickness(0), EditCurve(5) {}
    virtual ~DrawSketchHandlerFrame() {}

    enum BoxMode {
        STATUS_SEEK_First,      /**< enum value ----. */
        STATUS_SEEK_Second,     /**< enum value ----. */
        STATUS_SEEK_Third,     /**< enum value ----. */
        STATUS_End
    };

    virtual void activated(ViewProviderSketch*)
    {
        setCrosshairCursor("Sketcher_Pointer_Frame");
        sketchgui->toolSettings->widget->setSettings(15);

        // Constrain icon size in px
        qreal pixelRatio = devicePixelRatio();
        const unsigned long defaultCrosshairColor = 0xFFFFFF;
        unsigned long color = getCrosshairColor();
        auto colorMapping = std::map<unsigned long, unsigned long>();
        colorMapping[defaultCrosshairColor] = color;

        qreal fullIconWidth = 32 * pixelRatio;
        qreal iconWidth = 16 * pixelRatio;
        QPixmap cursorPixmap = Gui::BitmapFactory().pixmapFromSvg("Sketcher_Crosshair", QSizeF(fullIconWidth, fullIconWidth), colorMapping),
            icon = Gui::BitmapFactory().pixmapFromSvg("Sketcher_CreateFrame", QSizeF(iconWidth, iconWidth));
        QPainter cursorPainter;
        cursorPainter.begin(&cursorPixmap);
        cursorPainter.drawPixmap(16 * pixelRatio, 16 * pixelRatio, icon);
        cursorPainter.end();
        int hotX = 8;
        int hotY = 8;
        cursorPixmap.setDevicePixelRatio(pixelRatio);
        // only X11 needs hot point coordinates to be scaled
        if (qGuiApp->platformName() == QLatin1String("xcb")) {
            hotX *= pixelRatio;
            hotY *= pixelRatio;
        }
        setCursor(cursorPixmap, hotX, hotY, false);
    }

    virtual void mouseMove(Base::Vector2d onSketchPos)
    {
        if (Mode == STATUS_SEEK_First) {
            setPositionText(onSketchPos);
            if (seekAutoConstraint(sugConstr1, onSketchPos, Base::Vector2d(0.f, 0.f))) {
                renderSuggestConstraintsCursor(sugConstr1);
                return;
            }
            if (sketchgui->toolSettings->widget->isSettingSet[0] + sketchgui->toolSettings->widget->isSettingSet[1] == 2) {
                pressButton(onSketchPos);
                releaseButton(onSketchPos);
            }
        }
        else if (Mode == STATUS_SEEK_Second) {
            float dx = onSketchPos.x - firstPoint.x;
            float dy = onSketchPos.y - firstPoint.y;
            secondPoint = onSketchPos;
            if (sketchgui->toolSettings->widget->isSettingSet[2] == 1) {
                dx = sketchgui->toolSettings->widget->toolParameters[2];
                if (onSketchPos.x - firstPoint.x < 0) {
                    dx = -dx;
                }
                secondPoint.x = firstPoint.x + dx;
            }
            if (sketchgui->toolSettings->widget->isSettingSet[3] == 1) {
                dy = sketchgui->toolSettings->widget->toolParameters[3];
                if (onSketchPos.y - firstPoint.y < 0) {
                    dy = -dy;
                }
                secondPoint.y = firstPoint.y + dy;
            }

            EditCurve[0] = firstPoint;
            EditCurve[2] = secondPoint;
            EditCurve[1] = Base::Vector2d(secondPoint.x, firstPoint.y);
            EditCurve[3] = Base::Vector2d(firstPoint.x, secondPoint.y);
            EditCurve[4] = firstPoint;

            SbString text;
            text.sprintf(" (%.1f x %.1f)", dx, dy);
            setPositionText(onSketchPos, text);
            drawEdit(EditCurve);
        }
        else if (Mode == STATUS_SEEK_Third) {
            double dx, dy, minX, minY, maxX, maxY;
            minX = min(firstPoint.x, secondPoint.x);
            maxX = max(firstPoint.x, secondPoint.x);
            minY = min(firstPoint.y, secondPoint.y);
            maxY = max(firstPoint.y, secondPoint.y);

            if (sketchgui->toolSettings->widget->isSettingSet[4] == 1) {
                thickness = -sketchgui->toolSettings->widget->toolParameters[4];
            }
            else {
                dx = min(abs(onSketchPos.x - minX), abs(onSketchPos.x - maxX));
                dy = min(abs(onSketchPos.y - minY), abs(onSketchPos.y - maxY));
                if (onSketchPos.x - minX > 0 && onSketchPos.x - maxX < 0 && !(onSketchPos.y - minY > 0 && onSketchPos.y - maxY < 0) ) {
                    thickness = dy;
                }
                else if (onSketchPos.y - minY > 0 && onSketchPos.y - maxY < 0 && !(onSketchPos.x - minX > 0 && onSketchPos.x - maxX < 0)) {
                    thickness = dx;
                }
                else if (onSketchPos.y - minY > 0 && onSketchPos.y - maxY < 0 && onSketchPos.x - minX > 0 && onSketchPos.x - maxX < 0) {
                    thickness = -min(dx, dy);
                }
                else {
                    thickness = max(dx, dy);
                }
            }

            thirdPoint.x = firstPoint.x == minX ? minX - thickness : maxX + thickness;
            thirdPoint.y = firstPoint.y == minY ? minY - thickness : maxY + thickness;
            fourthPoint.x = secondPoint.x == minX ? minX - thickness : maxX + thickness;
            fourthPoint.y = secondPoint.y == minY ? minY - thickness : maxY + thickness;

            EditCurve.resize(10);
            EditCurve[0] = firstPoint;
            EditCurve[1] = Base::Vector2d(secondPoint.x, firstPoint.y);
            EditCurve[2] = secondPoint;
            EditCurve[3] = Base::Vector2d(firstPoint.x, secondPoint.y);
            EditCurve[4] = firstPoint;
            EditCurve[5] = thirdPoint;
            EditCurve[6] = Base::Vector2d(fourthPoint.x, thirdPoint.y);
            EditCurve[7] = fourthPoint;
            EditCurve[8] = Base::Vector2d(thirdPoint.x, fourthPoint.y);
            EditCurve[9] = thirdPoint;

            SbString text;
            text.sprintf(" (%.1fT)", -thickness);
            setPositionText(onSketchPos, text);

            drawEdit(EditCurve);
            if (sketchgui->toolSettings->widget->isSettingSet[4] == 1) {
                pressButton(onSketchPos);
                releaseButton(onSketchPos);
            }
        }
        applyCursor();
    }

    virtual bool pressButton(Base::Vector2d onSketchPos)
    {
        if (Mode == STATUS_SEEK_First) {
            firstPoint = onSketchPos;
            if (sketchgui->toolSettings->widget->isSettingSet[0] == 1) {
                firstPoint.x = sketchgui->toolSettings->widget->toolParameters[0];
            }
            if (sketchgui->toolSettings->widget->isSettingSet[1] == 1) {
                firstPoint.y = sketchgui->toolSettings->widget->toolParameters[1];
            }

            sketchgui->toolSettings->widget->setParameterActive(0, 0);
            sketchgui->toolSettings->widget->setParameterActive(0, 1);
            sketchgui->toolSettings->widget->setParameterActive(1, 2);
            sketchgui->toolSettings->widget->setParameterActive(1, 3);
            sketchgui->toolSettings->widget->setParameterFocus(2);
            Mode = STATUS_SEEK_Second;
        }
        else if (Mode == STATUS_SEEK_Second) {
            //recalculate secondpoint in case mouse moved too fast for mousemove to catch it.
            secondPoint = onSketchPos;
            if (sketchgui->toolSettings->widget->isSettingSet[2] == 1) {
                float dx = sketchgui->toolSettings->widget->toolParameters[2];
                if (onSketchPos.x - firstPoint.x < 0) {
                    dx = -dx;
                }
                secondPoint.x = firstPoint.x + dx;
            }
            if (sketchgui->toolSettings->widget->isSettingSet[3] == 1) {
                float dy = sketchgui->toolSettings->widget->toolParameters[3];
                if (onSketchPos.y - firstPoint.y < 0) {
                    dy = -dy;
                }
                secondPoint.y = firstPoint.y + dy;
            }

            EditCurve[0] = firstPoint;
            EditCurve[2] = secondPoint;
            EditCurve[1] = Base::Vector2d(secondPoint.x, firstPoint.y);
            EditCurve[3] = Base::Vector2d(firstPoint.x, secondPoint.y);
            EditCurve[4] = firstPoint;
            drawEdit(EditCurve);

            sketchgui->toolSettings->widget->setParameterActive(0, 2);
            sketchgui->toolSettings->widget->setParameterActive(0, 3);
            sketchgui->toolSettings->widget->setParameterActive(1, 4);
            sketchgui->toolSettings->widget->setParameterFocus(4);
            Mode = STATUS_SEEK_Third;
        }
        else {
            double dx, dy, minX, minY, maxX, maxY;
            minX = min(firstPoint.x, secondPoint.x);
            maxX = max(firstPoint.x, secondPoint.x);
            minY = min(firstPoint.y, secondPoint.y);
            maxY = max(firstPoint.y, secondPoint.y);

            if (sketchgui->toolSettings->widget->isSettingSet[4] == 1) {
                thickness = -sketchgui->toolSettings->widget->toolParameters[4];
            }
            else {
                dx = min(abs(onSketchPos.x - minX), abs(onSketchPos.x - maxX));
                dy = min(abs(onSketchPos.y - minY), abs(onSketchPos.y - maxY));
                if (onSketchPos.x - minX > 0 && onSketchPos.x - maxX < 0 && !(onSketchPos.y - minY > 0 && onSketchPos.y - maxY < 0)) {
                    thickness = dy;
                }
                else if (onSketchPos.y - minY > 0 && onSketchPos.y - maxY < 0 && !(onSketchPos.x - minX > 0 && onSketchPos.x - maxX < 0)) {
                    thickness = dx;
                }
                else if (onSketchPos.y - minY > 0 && onSketchPos.y - maxY < 0 && onSketchPos.x - minX > 0 && onSketchPos.x - maxX < 0) {
                    thickness = -min(dx, dy);
                }
                else {
                    thickness = max(dx, dy);
                }
            }

            thirdPoint.x = firstPoint.x == minX ? minX - thickness : maxX + thickness;
            thirdPoint.y = firstPoint.y == minY ? minY - thickness : maxY + thickness;
            fourthPoint.x = secondPoint.x == minX ? minX - thickness : maxX + thickness;
            fourthPoint.y = secondPoint.y == minY ? minY - thickness : maxY + thickness;

            EditCurve.resize(10);
            EditCurve[0] = firstPoint;
            EditCurve[1] = Base::Vector2d(secondPoint.x, firstPoint.y);
            EditCurve[2] = secondPoint;
            EditCurve[3] = Base::Vector2d(firstPoint.x, secondPoint.y);
            EditCurve[4] = firstPoint;
            EditCurve[5] = thirdPoint;
            EditCurve[6] = Base::Vector2d(fourthPoint.x, thirdPoint.y);
            EditCurve[7] = fourthPoint;
            EditCurve[8] = Base::Vector2d(thirdPoint.x, fourthPoint.y);
            EditCurve[9] = thirdPoint;
            Mode = STATUS_End;
        }
        return true;
    }

    virtual bool releaseButton(Base::Vector2d onSketchPos)
    {
        Q_UNUSED(onSketchPos);
        if (Mode == STATUS_End) {
            unsetCursor();
            resetPositionText();
            int firstCurve = getHighestCurveIndex() + 1;

            try {
                Gui::Command::openCommand(QT_TRANSLATE_NOOP("Command", "Add sketch box"));
                Gui::Command::doCommand(Gui::Command::Doc,
                    "geoList = []\n"
                    "geoList.append(Part.LineSegment(App.Vector(%f,%f,0),App.Vector(%f,%f,0)))\n"
                    "geoList.append(Part.LineSegment(App.Vector(%f,%f,0),App.Vector(%f,%f,0)))\n"
                    "geoList.append(Part.LineSegment(App.Vector(%f,%f,0),App.Vector(%f,%f,0)))\n"
                    "geoList.append(Part.LineSegment(App.Vector(%f,%f,0),App.Vector(%f,%f,0)))\n"
                    "geoList.append(Part.LineSegment(App.Vector(%f,%f,0),App.Vector(%f,%f,0)))\n"
                    "geoList.append(Part.LineSegment(App.Vector(%f,%f,0),App.Vector(%f,%f,0)))\n"
                    "geoList.append(Part.LineSegment(App.Vector(%f,%f,0),App.Vector(%f,%f,0)))\n"
                    "geoList.append(Part.LineSegment(App.Vector(%f,%f,0),App.Vector(%f,%f,0)))\n"
                    "geoList.append(Part.LineSegment(App.Vector(%f,%f,0),App.Vector(%f,%f,0)))\n"
                    "geoList.append(Part.LineSegment(App.Vector(%f,%f,0),App.Vector(%f,%f,0)))\n"
                    "geoList.append(Part.LineSegment(App.Vector(%f,%f,0),App.Vector(%f,%f,0)))\n"
                    "geoList.append(Part.LineSegment(App.Vector(%f,%f,0),App.Vector(%f,%f,0)))\n"
                    "%s.addGeometry(geoList,%s)\n"
                    "conList = []\n"
                    "conList.append(Sketcher.Constraint('Coincident',%i,2,%i,1))\n"
                    "conList.append(Sketcher.Constraint('Coincident',%i,2,%i,1))\n"
                    "conList.append(Sketcher.Constraint('Coincident',%i,2,%i,1))\n"
                    "conList.append(Sketcher.Constraint('Coincident',%i,2,%i,1))\n"
                    "conList.append(Sketcher.Constraint('Coincident',%i,2,%i,1))\n"
                    "conList.append(Sketcher.Constraint('Coincident',%i,2,%i,1))\n"
                    "conList.append(Sketcher.Constraint('Coincident',%i,2,%i,1))\n"
                    "conList.append(Sketcher.Constraint('Coincident',%i,2,%i,1))\n"

                    "conList.append(Sketcher.Constraint('Coincident',%i,2,%i,1))\n"
                    "conList.append(Sketcher.Constraint('Coincident',%i,1,%i,1))\n"
                    "conList.append(Sketcher.Constraint('Coincident',%i,2,%i,1))\n"
                    "conList.append(Sketcher.Constraint('Coincident',%i,1,%i,1))\n"
                    "conList.append(Sketcher.Constraint('Coincident',%i,2,%i,1))\n"
                    "conList.append(Sketcher.Constraint('Coincident',%i,1,%i,1))\n"
                    "conList.append(Sketcher.Constraint('Coincident',%i,2,%i,1))\n"
                    "conList.append(Sketcher.Constraint('Coincident',%i,1,%i,1))\n"
                    "conList.append(Sketcher.Constraint('Horizontal',%i))\n"
                    "conList.append(Sketcher.Constraint('Horizontal',%i))\n"
                    "conList.append(Sketcher.Constraint('Vertical',%i))\n"
                    "conList.append(Sketcher.Constraint('Vertical',%i))\n"
                    "conList.append(Sketcher.Constraint('Horizontal',%i))\n"
                    "conList.append(Sketcher.Constraint('Horizontal',%i))\n"
                    "conList.append(Sketcher.Constraint('Vertical',%i))\n"
                    "conList.append(Sketcher.Constraint('Vertical',%i))\n"
                    "conList.append(Sketcher.Constraint('Perpendicular',%i,%i))\n"
                    "conList.append(Sketcher.Constraint('Perpendicular',%i,%i))\n"
                    "conList.append(Sketcher.Constraint('Perpendicular',%i,%i))\n"
                    "%s.addConstraint(conList)\n"
                    "del geoList, conList\n",
                    EditCurve[0].x, EditCurve[0].y, EditCurve[1].x, EditCurve[1].y, // rectangle 1 line 1
                    EditCurve[1].x, EditCurve[1].y, EditCurve[2].x, EditCurve[2].y, // rectangle 1 line 2
                    EditCurve[2].x, EditCurve[2].y, EditCurve[3].x, EditCurve[3].y, // rectangle 1 line 3
                    EditCurve[3].x, EditCurve[3].y, EditCurve[0].x, EditCurve[0].y, // rectangle 1 line 4
                    EditCurve[5].x, EditCurve[5].y, EditCurve[6].x, EditCurve[6].y, // rectangle 2 line 1
                    EditCurve[6].x, EditCurve[6].y, EditCurve[7].x, EditCurve[7].y, // rectangle 2 line 2
                    EditCurve[7].x, EditCurve[7].y, EditCurve[8].x, EditCurve[8].y, // rectangle 2 line 3
                    EditCurve[8].x, EditCurve[8].y, EditCurve[5].x, EditCurve[5].y, // rectangle 2 line 4
                    EditCurve[5].x, EditCurve[5].y, EditCurve[0].x, EditCurve[0].y, // support line 1
                    EditCurve[6].x, EditCurve[6].y, EditCurve[1].x, EditCurve[1].y, // support line 2
                    EditCurve[7].x, EditCurve[7].y, EditCurve[2].x, EditCurve[2].y, // support line 3
                    EditCurve[8].x, EditCurve[8].y, EditCurve[3].x, EditCurve[3].y, // support line 4
                    Gui::Command::getObjectCmd(sketchgui->getObject()).c_str(), // the sketch
                    geometryCreationMode == Construction ? "True" : "False", // geometry as construction or not
                    firstCurve, firstCurve + 1, // coincident1
                    firstCurve + 1, firstCurve + 2, // coincident2
                    firstCurve + 2, firstCurve + 3, // coincident3
                    firstCurve + 3, firstCurve, // coincident4
                    firstCurve + 4, firstCurve + 5, // coincident5
                    firstCurve + 5, firstCurve + 6, // coincident6
                    firstCurve + 6, firstCurve + 7, // coincident7
                    firstCurve + 7, firstCurve + 4, // coincident8

                    firstCurve + 8, firstCurve, // coincident9
                    firstCurve + 8, firstCurve + 4, // coincident10
                    firstCurve + 9, firstCurve + 1, // coincident11
                    firstCurve + 9, firstCurve + 5, // coincident12
                    firstCurve + 10, firstCurve + 2, // coincident13
                    firstCurve + 10, firstCurve + 6, // coincident14
                    firstCurve + 11, firstCurve + 3, // coincident15
                    firstCurve + 11, firstCurve + 7, // coincident16

                    firstCurve, // horizontal1
                    firstCurve + 2, // horizontal2
                    firstCurve + 1, // vertical1
                    firstCurve + 3, // vertical2
                    firstCurve + 4, // horizontal3
                    firstCurve + 6, // horizontal4
                    firstCurve + 5, // vertical3
                    firstCurve + 7, // vertical4
                    firstCurve + 8, firstCurve + 9, // Perpendicular of support lines
                    firstCurve + 9, firstCurve + 10, // Perpendicular of support lines
                    firstCurve + 10, firstCurve + 11, // Perpendicular of support lines
                    Gui::Command::getObjectCmd(sketchgui->getObject()).c_str()); // the sketch

                if (geometryCreationMode != Construction) {
                    Gui::cmdAppObjectArgs(sketchgui->getObject(), "toggleConstruction(%d) ", firstCurve + 8);
                    Gui::cmdAppObjectArgs(sketchgui->getObject(), "toggleConstruction(%d) ", firstCurve + 9);
                    Gui::cmdAppObjectArgs(sketchgui->getObject(), "toggleConstruction(%d) ", firstCurve + 10);
                    Gui::cmdAppObjectArgs(sketchgui->getObject(), "toggleConstruction(%d) ", firstCurve + 11);
                }

                //add constraint if user typed in some dimensions in tool widget
                if (sketchgui->toolSettings->widget->isSettingSet[0] + sketchgui->toolSettings->widget->isSettingSet[1] + sketchgui->toolSettings->widget->isSettingSet[2] + sketchgui->toolSettings->widget->isSettingSet[3] + sketchgui->toolSettings->widget->isSettingSet[4]  != 0) {
                    if (sketchgui->toolSettings->widget->isSettingSet[0] == 1) {
                        distanceXYorPointOnObject(0, firstCurve, Sketcher::PointPos::start, sketchgui->toolSettings->widget->toolParameters[0]);
                    }
                    if (sketchgui->toolSettings->widget->isSettingSet[1] == 1) {
                        distanceXYorPointOnObject(1, firstCurve, Sketcher::PointPos::start, sketchgui->toolSettings->widget->toolParameters[1]);
                    }
                    if (sketchgui->toolSettings->widget->isSettingSet[2] == 1) {
                        Gui::cmdAppObjectArgs(sketchgui->getObject(), "addConstraint(Sketcher.Constraint('Distance',%d,%d,%d,%d,%f)) ",
                            firstCurve, 1, firstCurve, 2, sketchgui->toolSettings->widget->toolParameters[2]);
                    }
                    if (sketchgui->toolSettings->widget->isSettingSet[3] == 1) {
                        Gui::cmdAppObjectArgs(sketchgui->getObject(), "addConstraint(Sketcher.Constraint('Distance',%d,%d,%d,%d,%f)) ",
                            firstCurve + 3, 1, firstCurve + 3, 2, sketchgui->toolSettings->widget->toolParameters[3]);
                    }
                    if (sketchgui->toolSettings->widget->isSettingSet[4] == 1) {
                        int GeoId1 = firstCurve + 3;
                        int GeoId2 = firstCurve + 7;
                        if (secondPoint.x - firstPoint.x < 0) {
                            GeoId1 = firstCurve + 1;
                            GeoId2 = firstCurve + 5;
                        }
                        Gui::cmdAppObjectArgs(sketchgui->getObject(), "addConstraint(Sketcher.Constraint('DistanceX',%d,%d,%d,%d,%f)) ",
                            GeoId1, 1, GeoId2, 1, sketchgui->toolSettings->widget->toolParameters[4]);
                    }
                }

                Gui::Command::commitCommand();
            }
            catch (const Base::Exception& e) {
                Base::Console().Error("Failed to add frame: %s\n", e.what());
                Gui::Command::abortCommand();
            }

            // add auto constraints at the start of the first side
            if (sugConstr1.size() > 0) {
                createAutoConstraints(sugConstr1, getHighestCurveIndex() - 3, Sketcher::PointPos::start);
                sugConstr1.clear();
            }

            // add auto constraints at the end of the second side
            if (sugConstr2.size() > 0) {
                createAutoConstraints(sugConstr2, getHighestCurveIndex() - 2, Sketcher::PointPos::end);
                sugConstr2.clear();
            }

            tryAutoRecomputeIfNotSolve(static_cast<Sketcher::SketchObject*>(sketchgui->getObject()));

            ParameterGrp::handle hGrp = App::GetApplication().GetParameterGroupByPath("User parameter:BaseApp/Preferences/Mod/Sketcher");
            bool continuousMode = hGrp->GetBool("ContinuousCreationMode", true);

            if (continuousMode) {
                // This code enables the continuous creation mode.
                sketchgui->toolSettings->widget->setSettings(15);
                Mode = STATUS_SEEK_First;
                EditCurve.clear();
                drawEdit(EditCurve);
                EditCurve.resize(5);
                applyCursor();
                /* this is ok not to call to purgeHandler
                * in continuous creation mode because the
                * handler is destroyed by the quit() method on pressing the
                * right button of the mouse */
            }
            else {
                sketchgui->purgeHandler(); // no code after this line, Handler get deleted in ViewProvider
            }
        }
        return true;
    }
protected:
    BoxMode Mode;
    Base::Vector2d firstPoint, secondPoint, thirdPoint, fourthPoint;
    double thickness;
    std::vector<Base::Vector2d> EditCurve;
    std::vector<AutoConstraint> sugConstr1, sugConstr2;
};

DEF_STD_CMD_AU(CmdSketcherCreateFrame)

CmdSketcherCreateFrame::CmdSketcherCreateFrame()
    : Command("Sketcher_CreateFrame")
{
    sAppModule = "Sketcher";
    sGroup = "Sketcher";
    sMenuText = QT_TR_NOOP("Create a frame");
    sToolTipText = QT_TR_NOOP("Create a rectangle frame in the sketch");
    sWhatsThis = "Sketcher_CreateFrame";
    sStatusTip = sToolTipText;
    sPixmap = "Sketcher_CreateFrame";
    sAccel = "G, F";
    eType = ForEdit;
}

void CmdSketcherCreateFrame::activated(int iMsg)
{
    Q_UNUSED(iMsg);
    ActivateHandler(getActiveGuiDocument(), new DrawSketchHandlerFrame());
}

void CmdSketcherCreateFrame::updateAction(int mode)
{
    switch (mode) {
    case Normal:
        if (getAction())
            getAction()->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateFrame"));
        break;
    case Construction:
        if (getAction())
            getAction()->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateFrame_Constr"));
        break;
    }
}

bool CmdSketcherCreateFrame::isActive(void)
{
    return isCreateGeoActive(getActiveGuiDocument());
}

/* Rectangles Comp command =========================================*/

DEF_STD_CMD_ACLU(CmdSketcherCompCreateRectangles)

CmdSketcherCompCreateRectangles::CmdSketcherCompCreateRectangles()
    : Command("Sketcher_CompCreateRectangles")
{
    sAppModule = "Sketcher";
    sGroup = "Sketcher";
    sMenuText = QT_TR_NOOP("Create rectangles");
    sToolTipText = QT_TR_NOOP("Creates a rectangle in the sketch");
    sWhatsThis = "Sketcher_CompCreateRectangles";
    sStatusTip = sToolTipText;
    eType = ForEdit;
}

void CmdSketcherCompCreateRectangles::activated(int iMsg)
{
    if (iMsg == 0)
        ActivateHandler(getActiveGuiDocument(), new DrawSketchHandlerBox(DrawSketchHandlerBox::Diagonal));
    else if (iMsg == 1)
        ActivateHandler(getActiveGuiDocument(), new DrawSketchHandlerBox(DrawSketchHandlerBox::CenterAndCorner));
    else if (iMsg == 2)
        ActivateHandler(getActiveGuiDocument(), new DrawSketchHandlerOblong());
    else if (iMsg == 3)
        ActivateHandler(getActiveGuiDocument(), new DrawSketchHandlerFrame());
    else
        return;

    // Since the default icon is reset when enabling/disabling the command we have
    // to explicitly set the icon of the used command.
    Gui::ActionGroup* pcAction = qobject_cast<Gui::ActionGroup*>(_pcAction);
    QList<QAction*> a = pcAction->actions();

    assert(iMsg < a.size());
    pcAction->setIcon(a[iMsg]->icon());
}

Gui::Action* CmdSketcherCompCreateRectangles::createAction(void)
{
    Gui::ActionGroup* pcAction = new Gui::ActionGroup(this, Gui::getMainWindow());
    pcAction->setDropDownMenu(true);
    applyCommandData(this->className(), pcAction);

    QAction* arc1 = pcAction->addAction(QString());
    arc1->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateRectangle"));
    QAction* arc2 = pcAction->addAction(QString());
    arc2->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateRectangle_Center"));
    QAction* arc3 = pcAction->addAction(QString());
    arc3->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateOblong"));
    QAction* arc4 = pcAction->addAction(QString());
    arc4->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateFrame"));

    _pcAction = pcAction;
    languageChange();

    pcAction->setIcon(arc1->icon());
    int defaultId = 0;
    pcAction->setProperty("defaultAction", QVariant(defaultId));

    return pcAction;
}

void CmdSketcherCompCreateRectangles::updateAction(int mode)
{
    Gui::ActionGroup* pcAction = qobject_cast<Gui::ActionGroup*>(getAction());
    if (!pcAction)
        return;

    QList<QAction*> a = pcAction->actions();
    int index = pcAction->property("defaultAction").toInt();
    switch (mode) {
    case Normal:
        a[0]->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateRectangle"));
        a[1]->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateRectangle_Center"));
        a[2]->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateOblong"));
        a[3]->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateFrame"));
        getAction()->setIcon(a[index]->icon());
        break;
    case Construction:
        a[0]->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateRectangle_Constr"));
        a[1]->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateRectangle_Center_Constr"));
        a[2]->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateOblong_Constr"));
        a[3]->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateFrame_Constr"));
        getAction()->setIcon(a[index]->icon());
        break;
    }
}

void CmdSketcherCompCreateRectangles::languageChange()
{
    Command::languageChange();

    if (!_pcAction)
        return;
    Gui::ActionGroup* pcAction = qobject_cast<Gui::ActionGroup*>(_pcAction);
    QList<QAction*> a = pcAction->actions();

    QAction* rectangle1 = a[0];
    rectangle1->setText(QApplication::translate("CmdSketcherCompCreateRectangles", "Rectangle"));
    rectangle1->setToolTip(QApplication::translate("Sketcher_CreateRectangle", "Create a rectangle"));
    rectangle1->setStatusTip(rectangle1->toolTip());
    QAction* rectangle2 = a[1];
    rectangle2->setText(QApplication::translate("CmdSketcherCompCreateRectangles", "Centered rectangle"));
    rectangle2->setToolTip(QApplication::translate("Sketcher_CreateRectangle_Center", "Create a centered rectangle"));
    rectangle2->setStatusTip(rectangle2->toolTip());
    QAction* rectangle3 = a[2];
    rectangle3->setText(QApplication::translate("CmdSketcherCompCreateRectangles", "Rounded rectangle"));
    rectangle3->setToolTip(QApplication::translate("Sketcher_CreateOblong", "Create a rounded rectangle"));
    rectangle3->setStatusTip(rectangle3->toolTip());
    QAction* rectangle4 = a[3];
    rectangle4->setText(QApplication::translate("CmdSketcherCompCreateRectangles", "Frame"));
    rectangle4->setToolTip(QApplication::translate("Sketcher_CreateFrame", "Create a frame"));
    rectangle4->setStatusTip(rectangle3->toolTip());
}

bool CmdSketcherCompCreateRectangles::isActive(void)
{
    return isCreateGeoActive(getActiveGuiDocument());
}

/* Polyline command =================================================*/

class DrawSketchHandlerLineSet: public DrawSketchHandler
{
public:
    DrawSketchHandlerLineSet()
      : Mode(STATUS_SEEK_First), SegmentMode(SEGMENT_MODE_Line)
      , TransitionMode(TRANSITION_MODE_Free)
      , SnapMode(SNAP_MODE_Free)
      , suppressTransition(false)
      , EditCurve(2)
      , firstCurve(-1)
      , previousCurve(-1)
      , firstPosId(Sketcher::PointPos::none)
      , previousPosId(Sketcher::PointPos::none)
      , startAngle(0)
      , endAngle(0)
      , arcRadius(0)
      , firstsegment(true)
    {
    }
    virtual ~DrawSketchHandlerLineSet() {
        sketchgui->toolSettings->widget->setSettings(0);
    }
    /// mode table
    enum SELECT_MODE {
        STATUS_SEEK_First,      /**< enum value ----. */
        STATUS_SEEK_Second,     /**< enum value ----. */
        STATUS_Do,
        STATUS_Close
    };

    enum SEGMENT_MODE
    {
        SEGMENT_MODE_Arc,
        SEGMENT_MODE_Line
    };

    enum TRANSITION_MODE
    {
        TRANSITION_MODE_Free,
        TRANSITION_MODE_Tangent,
        TRANSITION_MODE_Perpendicular_L,
        TRANSITION_MODE_Perpendicular_R
    };

    enum SNAP_MODE
    {
        SNAP_MODE_Free,
        SNAP_MODE_45Degree
    };

    virtual void registerPressedKey(bool pressed, int key)
    {
        if (Mode != STATUS_SEEK_Second)
            return; // SegmentMode can be changed only in STATUS_SEEK_Second mode

        if ( (key == SoKeyboardEvent::RIGHT_SHIFT || key == SoKeyboardEvent::LEFT_SHIFT) && pressed && previousCurve != -1) {
            //Note: for key press to be registered by the registerPressedKey when tool settings is being used (has focus), then the key has to be mapped in taskSketcherTool.cpp
            // loop through the following modes:
            // SEGMENT_MODE_Line, TRANSITION_MODE_Free / TRANSITION_MODE_Tangent
            // SEGMENT_MODE_Line, TRANSITION_MODE_Perpendicular_L
            // SEGMENT_MODE_Line, TRANSITION_MODE_Tangent / TRANSITION_MODE_Free
            // SEGMENT_MODE_Arc, TRANSITION_MODE_Tangent
            // SEGMENT_MODE_Arc, TRANSITION_MODE_Perpendicular_L
            // SEGMENT_MODE_Arc, TRANSITION_MODE_Perpendicular_R

            SnapMode = SNAP_MODE_Free;

            Base::Vector2d onSketchPos;
            if (SegmentMode == SEGMENT_MODE_Line)
                onSketchPos = EditCurve[EditCurve.size()-1];
            else
                onSketchPos = EditCurve[29];

            const Part::Geometry *geom = sketchgui->getSketchObject()->getGeometry(previousCurve);

            if (SegmentMode == SEGMENT_MODE_Line) {
                switch (TransitionMode) {
                    case TRANSITION_MODE_Free:
                        if (geom->getTypeId() == Part::GeomArcOfCircle::getClassTypeId()) { // 3rd mode
                            SegmentMode = SEGMENT_MODE_Arc;
                            TransitionMode = TRANSITION_MODE_Tangent;
                        }
                        else // 1st mode
                            TransitionMode = TRANSITION_MODE_Perpendicular_L;
                        break;
                    case TRANSITION_MODE_Perpendicular_L: // 2nd mode
                        if (geom->getTypeId() == Part::GeomArcOfCircle::getClassTypeId())
                            TransitionMode = TRANSITION_MODE_Free;
                        else
                            TransitionMode = TRANSITION_MODE_Tangent;
                        break;
                    case TRANSITION_MODE_Tangent:
                        if (geom->getTypeId() == Part::GeomArcOfCircle::getClassTypeId()) // 1st mode
                            TransitionMode = TRANSITION_MODE_Perpendicular_L;
                        else { // 3rd mode
                            SegmentMode = SEGMENT_MODE_Arc;
                            TransitionMode = TRANSITION_MODE_Tangent;
                        }
                        break;
                    default: // unexpected mode
                        TransitionMode = TRANSITION_MODE_Free;
                        break;
                }
            }
            else {
                switch (TransitionMode) {
                    case TRANSITION_MODE_Tangent: // 4th mode
                        TransitionMode = TRANSITION_MODE_Perpendicular_L;
                        break;
                    case TRANSITION_MODE_Perpendicular_L: // 5th mode
                        TransitionMode = TRANSITION_MODE_Perpendicular_R;
                        break;
                    default: // 6th mode (Perpendicular_R) + unexpected mode
                        SegmentMode = SEGMENT_MODE_Line;
                        if (geom->getTypeId() == Part::GeomArcOfCircle::getClassTypeId())
                            TransitionMode = TRANSITION_MODE_Tangent;
                        else
                            TransitionMode = TRANSITION_MODE_Free;
                        break;
                }
            }

            if (SegmentMode == SEGMENT_MODE_Line)
                EditCurve.resize(TransitionMode == TRANSITION_MODE_Free ? 2 : 3);
            else
                EditCurve.resize(32);
            mouseMove(onSketchPos); // trigger an update of EditCurve
        }
    }

    virtual void activated(ViewProviderSketch *)
    {
        setCrosshairCursor("Sketcher_Pointer_Create_Lineset");
        sketchgui->toolSettings->widget->setSettings(5);
    }

    virtual void mouseMove(Base::Vector2d onSketchPos)
    {
        suppressTransition = false;
        if (Mode==STATUS_SEEK_First) {
            setPositionText(onSketchPos);
            if (seekAutoConstraint(sugConstr1, onSketchPos, Base::Vector2d(0.f,0.f))) {
                renderSuggestConstraintsCursor(sugConstr1);
                return;
            }
            if (sketchgui->toolSettings->widget->isSettingSet[0] + sketchgui->toolSettings->widget->isSettingSet[1] == 2) {
                pressButton(onSketchPos);
                releaseButton(onSketchPos);
            }
        }
        else if (Mode==STATUS_SEEK_Second){
            if (SegmentMode == SEGMENT_MODE_Line) {
                if (sketchgui->toolSettings->widget->isSettingSet[2] == 1) {
                    EditCurve[EditCurve.size() - 1].x = sketchgui->toolSettings->widget->toolParameters[2];
                }
                else {
                    EditCurve[EditCurve.size() - 1].x = onSketchPos.x;
                }
                if (sketchgui->toolSettings->widget->isSettingSet[3] == 1) {
                    EditCurve[EditCurve.size() - 1].y = sketchgui->toolSettings->widget->toolParameters[3];
                }
                else {
                    EditCurve[EditCurve.size() - 1].y = onSketchPos.y;
                }
                if (TransitionMode == TRANSITION_MODE_Tangent) {
                    Base::Vector2d Tangent(dirVec.x,dirVec.y);
                    EditCurve[1].ProjectToLine(EditCurve[2] - EditCurve[0], Tangent);
                    if (EditCurve[1] * Tangent < 0) {
                        EditCurve[1] = EditCurve[2];
                        suppressTransition = true;
                    }
                    else
                        EditCurve[1] = EditCurve[0] + EditCurve[1];
                }
                else if (TransitionMode == TRANSITION_MODE_Perpendicular_L ||
                         TransitionMode == TRANSITION_MODE_Perpendicular_R) {
                    Base::Vector2d Perpendicular(-dirVec.y,dirVec.x);
                    EditCurve[1].ProjectToLine(EditCurve[2] - EditCurve[0], Perpendicular);
                    EditCurve[1] = EditCurve[0] + EditCurve[1];
                }

                drawEdit(EditCurve);

                float length = (EditCurve[1] - EditCurve[0]).Length();
                float angle = (EditCurve[1] - EditCurve[0]).GetAngle(Base::Vector2d(1.f,0.f));

                SbString text;
                text.sprintf(" (%.1f,%.1fdeg)", length, angle * 180 / M_PI);
                setPositionText(EditCurve[1], text);

                if (TransitionMode == TRANSITION_MODE_Free) {
                    if (seekAutoConstraint(sugConstr2, onSketchPos, onSketchPos - EditCurve[0])) {
                        renderSuggestConstraintsCursor(sugConstr2);
                        return;
                    }
                }
            }
            else if (SegmentMode == SEGMENT_MODE_Arc) {
                
                Base::Vector2d secondPoint; //replace onSketchPos if parameters are typed in tool settings
                if (sketchgui->toolSettings->widget->isSettingSet[2] == 1) {
                    secondPoint.x = sketchgui->toolSettings->widget->toolParameters[2];
                }
                else {
                    secondPoint.x = onSketchPos.x;
                }
                if (sketchgui->toolSettings->widget->isSettingSet[3] == 1) {
                    secondPoint.y = sketchgui->toolSettings->widget->toolParameters[3];
                }
                else {
                    secondPoint.y = onSketchPos.y;
                }

                if(QApplication::keyboardModifiers() == Qt::ControlModifier)
                    SnapMode = SNAP_MODE_45Degree;
                else
                    SnapMode = SNAP_MODE_Free;

                Base::Vector2d Tangent;
                if  (TransitionMode == TRANSITION_MODE_Tangent)
                    Tangent = Base::Vector2d(dirVec.x,dirVec.y);
                else if  (TransitionMode == TRANSITION_MODE_Perpendicular_L)
                    Tangent = Base::Vector2d(-dirVec.y,dirVec.x);
                else if  (TransitionMode == TRANSITION_MODE_Perpendicular_R)
                    Tangent = Base::Vector2d(dirVec.y,-dirVec.x);

                double theta = Tangent.GetAngle(secondPoint - EditCurve[0]);

                arcRadius = (secondPoint - EditCurve[0]).Length()/(2.0*sin(theta));

                // At this point we need a unit normal vector pointing towards
                // the center of the arc we are drawing. Derivation of the formula
                // used here can be found at http://people.richland.edu/james/lecture/m116/matrices/area.html
                double x1 = EditCurve[0].x;
                double y1 = EditCurve[0].y;
                double x2 = x1 + Tangent.x;
                double y2 = y1 + Tangent.y;
                double x3 = secondPoint.x;
                double y3 = secondPoint.y;
                if ((x2*y3-x3*y2)-(x1*y3-x3*y1)+(x1*y2-x2*y1) > 0)
                    arcRadius *= -1;
                if (boost::math::isnan(arcRadius) || boost::math::isinf(arcRadius))
                    arcRadius = 0.f;

                CenterPoint = EditCurve[0] + Base::Vector2d(arcRadius * Tangent.y, -arcRadius * Tangent.x);

                double rx = EditCurve[0].x - CenterPoint.x;
                double ry = EditCurve[0].y - CenterPoint.y;

                startAngle = atan2(ry,rx);

                double rxe = secondPoint.x - CenterPoint.x;
                double rye = secondPoint.y - CenterPoint.y;
                double arcAngle = atan2(-rxe*ry + rye*rx, rxe*rx + rye*ry);
                if (boost::math::isnan(arcAngle) || boost::math::isinf(arcAngle))
                    arcAngle = 0.f;
                if (arcRadius >= 0 && arcAngle > 0)
                    arcAngle -=  2*M_PI;
                if (arcRadius < 0 && arcAngle < 0)
                    arcAngle +=  2*M_PI;

                if (SnapMode == SNAP_MODE_45Degree)
                    arcAngle = round(arcAngle / (M_PI/4)) * M_PI/4;

                endAngle = startAngle + arcAngle;

                for (int i=1; i <= 29; i++) {
                    double angle = i*arcAngle/29.0;
                    double dx = rx * cos(angle) - ry * sin(angle);
                    double dy = rx * sin(angle) + ry * cos(angle);
                    EditCurve[i] = Base::Vector2d(CenterPoint.x + dx, CenterPoint.y + dy);
                }

                EditCurve[30] = CenterPoint;
                EditCurve[31] = EditCurve[0];

                drawEdit(EditCurve);

                SbString text;
                text.sprintf(" (%.1fR,%.1fdeg)", std::abs(arcRadius), arcAngle * 180 / M_PI);
                setPositionText(secondPoint, text);

                if (seekAutoConstraint(sugConstr2, secondPoint, Base::Vector2d(0.f,0.f))) {
                    renderSuggestConstraintsCursor(sugConstr2);
                    return;
                }
            }
            if (sketchgui->toolSettings->widget->isSettingSet[2] == 1 && sketchgui->toolSettings->widget->isSettingSet[3] == 1) {
                pressButton(onSketchPos);
                releaseButton(onSketchPos);
            }
        }
        applyCursor();
    }

    virtual bool pressButton(Base::Vector2d onSketchPos)
    {
        if (Mode == STATUS_SEEK_First) {
            if (sketchgui->toolSettings->widget->isSettingSet[0] == 1) {
                EditCurve[0].x = sketchgui->toolSettings->widget->toolParameters[0];
            }
            else {
                EditCurve[0].x = onSketchPos.x;
            }
            if (sketchgui->toolSettings->widget->isSettingSet[1] == 1) {
                EditCurve[0].y = sketchgui->toolSettings->widget->toolParameters[1];
            }
            else {
                EditCurve[0].y = onSketchPos.y;
            }
            // EditCurve[0] may be overwritten if previousCurve is found

            sketchgui->toolSettings->widget->setParameterActive(0, 0);
            sketchgui->toolSettings->widget->setParameterActive(0, 1);
            sketchgui->toolSettings->widget->setParameterActive(1, 2);
            sketchgui->toolSettings->widget->setParameterActive(1, 3);
            sketchgui->toolSettings->widget->setParameterFocus(2);

            virtualsugConstr1 = sugConstr1; // store original autoconstraints.

            // here we check if there is a preselected point and
            // we set up a transition from the neighbouring segment.
            // (peviousCurve, previousPosId, dirVec, TransitionMode)
            for (unsigned int i=0; i < sugConstr1.size(); i++)
                if (sugConstr1[i].Type == Sketcher::Coincident) {
                    const Part::Geometry *geom = sketchgui->getSketchObject()->getGeometry(sugConstr1[i].GeoId);
                    if ((geom->getTypeId() == Part::GeomLineSegment::getClassTypeId() ||
                         geom->getTypeId() == Part::GeomArcOfCircle::getClassTypeId()) &&
                        (sugConstr1[i].PosId == Sketcher::PointPos::start ||
                         sugConstr1[i].PosId == Sketcher::PointPos::end)) {
                        previousCurve = sugConstr1[i].GeoId;
                        previousPosId = sugConstr1[i].PosId;
                        updateTransitionData(previousCurve,previousPosId); // -> dirVec, EditCurve[0]
                        if (geom->getTypeId() == Part::GeomArcOfCircle::getClassTypeId()) {
                            TransitionMode = TRANSITION_MODE_Tangent;
                            SnapMode = SNAP_MODE_Free;
                        }
                        sugConstr1.erase(sugConstr1.begin()+i); // actually we should clear the vector completely
                        break;
                    }
                }

            // remember our first point (even if we are doing a transition from a previous curve)
            firstCurve = getHighestCurveIndex() + 1;
            firstPosId = Sketcher::PointPos::start;

            if (SegmentMode == SEGMENT_MODE_Line)
                EditCurve.resize(TransitionMode == TRANSITION_MODE_Free ? 2 : 3);
            else if (SegmentMode == SEGMENT_MODE_Arc)
                EditCurve.resize(32);
            Mode = STATUS_SEEK_Second;
        }
        else if (Mode == STATUS_SEEK_Second) {
            // exit on clicking exactly at the same position (e.g. double click)
            if (onSketchPos == EditCurve[0]) {
                unsetCursor();
                resetPositionText();
                EditCurve.clear();
                drawEdit(EditCurve);

                sketchgui->toolSettings->widget->setSettings(0);
                ParameterGrp::handle hGrp = App::GetApplication().GetParameterGroupByPath("User parameter:BaseApp/Preferences/Mod/Sketcher");
                bool continuousMode = hGrp->GetBool("ContinuousCreationMode",true);

                if(continuousMode){
                    // This code enables the continuous creation mode.
                    sketchgui->toolSettings->widget->setSettings(5);
                    Mode=STATUS_SEEK_First;
                    SegmentMode=SEGMENT_MODE_Line;
                    TransitionMode=TRANSITION_MODE_Free;
                    SnapMode = SNAP_MODE_Free;
                    suppressTransition=false;
                    firstCurve=-1;
                    previousCurve=-1;
                    firstPosId=Sketcher::PointPos::none;
                    previousPosId=Sketcher::PointPos::none;
                    EditCurve.clear();
                    drawEdit(EditCurve);
                    EditCurve.resize(2);
                    applyCursor();
                    /* this is ok not to call to purgeHandler
                    * in continuous creation mode because the
                    * handler is destroyed by the quit() method on pressing the
                    * right button of the mouse */
                    return true;
                }
                else{
                    sketchgui->purgeHandler(); // no code after this line, Handler get deleted in ViewProvider
                    return true;
                }
            }

            Mode = STATUS_Do;

            if (getPreselectPoint() != -1 && firstPosId != Sketcher::PointPos::none) {
                int GeoId;
                Sketcher::PointPos PosId;
                sketchgui->getSketchObject()->getGeoVertexIndex(getPreselectPoint(),GeoId,PosId);
                if (sketchgui->getSketchObject()->arePointsCoincident(GeoId,PosId,firstCurve,firstPosId))
                    Mode = STATUS_Close;
            }
            else if (getPreselectCross() == 0 && firstPosId != Sketcher::PointPos::none) {
                // close line started at root point
                if (sketchgui->getSketchObject()->arePointsCoincident(-1,Sketcher::PointPos::start,firstCurve,firstPosId))
                    Mode = STATUS_Close;
            }
        }
        return true;
    }

    virtual bool releaseButton(Base::Vector2d onSketchPos)
    {
        if (Mode == STATUS_Do || Mode == STATUS_Close) {
            bool addedGeometry = true;
            // issue the geometry
            if (SegmentMode == SEGMENT_MODE_Line) {
                try {
                    // open the transaction
                    Gui::Command::openCommand(QT_TRANSLATE_NOOP("Command", "Add line to sketch wire"));
                    Gui::cmdAppObjectArgs(sketchgui->getObject(), "addGeometry(Part.LineSegment(App.Vector(%f,%f,0),App.Vector(%f,%f,0)),%s)",
                        EditCurve[0].x,EditCurve[0].y,EditCurve[1].x,EditCurve[1].y,
                        geometryCreationMode==Construction?"True":"False");
                }
                catch (const Base::Exception& e) {
                    addedGeometry = false;
                    Base::Console().Error("Failed to add line: %s\n", e.what());
                    Gui::Command::abortCommand();
                }

                firstsegment=false;
            }
            else if (SegmentMode == SEGMENT_MODE_Arc) { // We're dealing with an Arc
                if (!boost::math::isnormal(arcRadius)) {
                    Mode = STATUS_SEEK_Second;
                    return true;
                }

                try {
                    Gui::Command::openCommand(QT_TRANSLATE_NOOP("Command", "Add arc to sketch wire"));
                    Gui::cmdAppObjectArgs(sketchgui->getObject(), "addGeometry(Part.ArcOfCircle"
                        "(Part.Circle(App.Vector(%f,%f,0),App.Vector(0,0,1),%f),%f,%f),%s)",
                        CenterPoint.x, CenterPoint.y, std::abs(arcRadius),
                        std::min(startAngle,endAngle), std::max(startAngle,endAngle),
                        geometryCreationMode==Construction?"True":"False");
                }
                catch (const Base::Exception& e) {
                    addedGeometry = false;
                    Base::Console().Error("Failed to add arc: %s\n", e.what());
                    Gui::Command::abortCommand();
                }

                firstsegment=false;
            }

            int lastCurve = getHighestCurveIndex();
            // issue the constraint
            if (addedGeometry && (previousPosId != Sketcher::PointPos::none)) {
                Sketcher::PointPos lastStartPosId = (SegmentMode == SEGMENT_MODE_Arc && startAngle > endAngle) ?
                                                    Sketcher::PointPos::end : Sketcher::PointPos::start;
                Sketcher::PointPos lastEndPosId = (SegmentMode == SEGMENT_MODE_Arc && startAngle > endAngle) ?
                                                  Sketcher::PointPos::start : Sketcher::PointPos::end;
                // in case of a tangency constraint, the coincident constraint is redundant
                std::string constrType = "Coincident";
                if (!suppressTransition && previousCurve != -1) {
                    if (TransitionMode == TRANSITION_MODE_Tangent)
                        constrType = "Tangent";
                    else if (TransitionMode == TRANSITION_MODE_Perpendicular_L ||
                             TransitionMode == TRANSITION_MODE_Perpendicular_R)
                        constrType = "Perpendicular";
                }
                Gui::cmdAppObjectArgs(sketchgui->getObject(), "addConstraint(Sketcher.Constraint('%s',%i,%i,%i,%i)) ",
                     constrType.c_str(), previousCurve, static_cast<int>(previousPosId), lastCurve, static_cast<int>(lastStartPosId));

                if(SnapMode == SNAP_MODE_45Degree && Mode != STATUS_Close) {
                    // -360, -315, -270, -225, -180, -135, -90, -45,  0, 45,  90, 135, 180, 225, 270, 315, 360
                    //  N/A,    a, perp,    a,  par,    a,perp,   a,N/A,  a,perp,   a, par,   a,perp,   a, N/A

                    // #3974: if in radians, the printf %f defaults to six decimals, which leads to loss of precision
                    double arcAngle = abs(round( (endAngle - startAngle) / (M_PI/4)) * 45); // in degrees

                    Gui::cmdAppObjectArgs(sketchgui->getObject(), "addConstraint(Sketcher.Constraint('Angle',%i,App.Units.Quantity('%f deg'))) ",
                                          lastCurve, arcAngle);
                }
                if (Mode == STATUS_Close) {
                    // close the loop by constrain to the first curve point
                    Gui::cmdAppObjectArgs(sketchgui->getObject(), "addConstraint(Sketcher.Constraint('Coincident',%i,%i,%i,%i)) ",
                                          lastCurve,static_cast<int>(lastEndPosId),firstCurve,static_cast<int>(firstPosId));
                }
                Gui::Command::commitCommand();

                tryAutoRecomputeIfNotSolve(static_cast<Sketcher::SketchObject *>(sketchgui->getObject()));
                }

            ParameterGrp::handle hGrp = App::GetApplication().GetParameterGroupByPath("User parameter:BaseApp/Preferences/Mod/Sketcher");
            bool avoidredundant = sketchgui->AvoidRedundant.getValue()  && sketchgui->Autoconstraints.getValue();

            if (Mode == STATUS_Close) {

                if(avoidredundant) {
                    if (SegmentMode == SEGMENT_MODE_Line) { // avoid redundant constraints.
                        if (sugConstr1.size() > 0)
                            removeRedundantHorizontalVertical(static_cast<Sketcher::SketchObject *>(sketchgui->getObject()),sugConstr1,sugConstr2);
                        else
                            removeRedundantHorizontalVertical(static_cast<Sketcher::SketchObject *>(sketchgui->getObject()),virtualsugConstr1,sugConstr2);
                    }
                }

                if (sugConstr2.size() > 0) {
                    // exclude any coincidence constraints
                    std::vector<AutoConstraint> sugConstr;
                    for (unsigned int i=0; i < sugConstr2.size(); i++) {
                        if (sugConstr2[i].Type != Sketcher::Coincident)
                            sugConstr.push_back(sugConstr2[i]);
                    }
                    createAutoConstraints(sugConstr, getHighestCurveIndex(), Sketcher::PointPos::end);
                    sugConstr2.clear();
                }

                tryAutoRecomputeIfNotSolve(static_cast<Sketcher::SketchObject *>(sketchgui->getObject()));

                unsetCursor();

                resetPositionText();
                EditCurve.clear();
                drawEdit(EditCurve);

                sketchgui->toolSettings->widget->setSettings(0);
                ParameterGrp::handle hGrp = App::GetApplication().GetParameterGroupByPath("User parameter:BaseApp/Preferences/Mod/Sketcher");
                bool continuousMode = hGrp->GetBool("ContinuousCreationMode",true);

                if(continuousMode){
                    // This code enables the continuous creation mode.
                    sketchgui->toolSettings->widget->setSettings(5);
                    Mode=STATUS_SEEK_First;
                    SegmentMode=SEGMENT_MODE_Line;
                    TransitionMode=TRANSITION_MODE_Free;
                    SnapMode = SNAP_MODE_Free;
                    suppressTransition=false;
                    firstCurve=-1;
                    previousCurve=-1;
                    firstPosId=Sketcher::PointPos::none;
                    previousPosId=Sketcher::PointPos::none;
                    EditCurve.clear();
                    drawEdit(EditCurve);
                    EditCurve.resize(2);
                    applyCursor();
                    /* this is ok not to call to purgeHandler
                    * in continuous creation mode because the
                    * handler is destroyed by the quit() method on pressing the
                    * right button of the mouse */
                }
                else{
                    sketchgui->purgeHandler(); // no code after this line, Handler get deleted in ViewProvider
                }
            }
            else {
                //Constraint if user tool setting was used to create the point.
                if (sketchgui->toolSettings->widget->isSettingSet[0] + sketchgui->toolSettings->widget->isSettingSet[1] + sketchgui->toolSettings->widget->isSettingSet[2] + sketchgui->toolSettings->widget->isSettingSet[3] != 0) {
                    if (firstCurve == lastCurve) { //First point constrained only in case of the first curve
                        if (sketchgui->toolSettings->widget->isSettingSet[0] == 1) {
                            distanceXYorPointOnObject(0, lastCurve, Sketcher::PointPos::start, sketchgui->toolSettings->widget->toolParameters[0]);
                        }
                        if (sketchgui->toolSettings->widget->isSettingSet[1] == 1) {
                            distanceXYorPointOnObject(1, lastCurve, Sketcher::PointPos::start, sketchgui->toolSettings->widget->toolParameters[1]);
                        }
                    }
                    bool firstCstrCreated = 0;

                    if (sketchgui->toolSettings->widget->isSettingSet[2] == 1) {
                        if (SegmentMode == SEGMENT_MODE_Arc) {
                            distanceXYorPointOnObject(0, lastCurve, (startAngle > endAngle) ? Sketcher::PointPos::start : Sketcher::PointPos::end, sketchgui->toolSettings->widget->toolParameters[2]);
                        }
                        else {
                            distanceXYorPointOnObject(0, lastCurve, Sketcher::PointPos::end, sketchgui->toolSettings->widget->toolParameters[2]);
                        }
                        firstCstrCreated = 1;
                    }
                    if (sketchgui->toolSettings->widget->isSettingSet[3] == 1
                        && ((TransitionMode != TRANSITION_MODE_Perpendicular_L && TransitionMode != TRANSITION_MODE_Perpendicular_R) || !firstCstrCreated)
                        && (TransitionMode != TRANSITION_MODE_Tangent || SegmentMode == SEGMENT_MODE_Arc || !firstCstrCreated)) {
                        //complex if to avoid over-constraining due to tangent and perpendicular constraints
                        if (SegmentMode == SEGMENT_MODE_Arc) {
                            distanceXYorPointOnObject(1, lastCurve, (startAngle > endAngle) ? Sketcher::PointPos::start : Sketcher::PointPos::end, sketchgui->toolSettings->widget->toolParameters[3]);
                        }
                        else {
                            distanceXYorPointOnObject(1, lastCurve, Sketcher::PointPos::end, sketchgui->toolSettings->widget->toolParameters[3]);
                        }
                    }
                }

                Gui::Command::commitCommand();

                // Add auto constraints
                if (sugConstr1.size() > 0) { // this is relevant only to the very first point
                    createAutoConstraints(sugConstr1, getHighestCurveIndex(), Sketcher::PointPos::start);
                    sugConstr1.clear();
                }


                if(avoidredundant) {
                    if (SegmentMode == SEGMENT_MODE_Line) { // avoid redundant constraints.
                        if (sugConstr1.size() > 0)
                            removeRedundantHorizontalVertical(static_cast<Sketcher::SketchObject *>(sketchgui->getObject()),sugConstr1,sugConstr2);
                        else
                            removeRedundantHorizontalVertical(static_cast<Sketcher::SketchObject *>(sketchgui->getObject()),virtualsugConstr1,sugConstr2);
                    }
                }

                virtualsugConstr1 = sugConstr2; // these are the initial constraints for the next iteration.

                if (sugConstr2.size() > 0) {
                    createAutoConstraints(sugConstr2, getHighestCurveIndex(),
                                          (SegmentMode == SEGMENT_MODE_Arc && startAngle > endAngle) ?
                                            Sketcher::PointPos::start : Sketcher::PointPos::end);
                    sugConstr2.clear();
                }

                tryAutoRecomputeIfNotSolve(static_cast<Sketcher::SketchObject *>(sketchgui->getObject()));

                // remember the vertex for the next rounds constraint..
                previousCurve = getHighestCurveIndex();
                previousPosId = (SegmentMode == SEGMENT_MODE_Arc && startAngle > endAngle) ?
                                 Sketcher::PointPos::start : Sketcher::PointPos::end; // cw arcs are rendered in reverse

                // setup for the next line segment
                // calculate dirVec and EditCurve[0]
                updateTransitionData(previousCurve,previousPosId);

                applyCursor();

                //reset the tool parameters
                sketchgui->toolSettings->widget->setSettings(0);
                sketchgui->toolSettings->widget->setSettings(6);

                Mode = STATUS_SEEK_Second;
                if (SegmentMode == SEGMENT_MODE_Arc) {
                    TransitionMode = TRANSITION_MODE_Tangent;
                    EditCurve.resize(3);
                    EditCurve[2] = EditCurve[0];
                }
                else {
                    TransitionMode = TRANSITION_MODE_Free;
                    EditCurve.resize(2);
                }
                SegmentMode = SEGMENT_MODE_Line;
                SnapMode = SNAP_MODE_Free;
                EditCurve[1] = EditCurve[0];
                mouseMove(onSketchPos); // trigger an update of EditCurve
            }
        }
        return true;
    }

    virtual void quit(void) {
        // We must see if we need to create a B-spline before cancelling everything
        // and now just like any other Handler,

        sketchgui->toolSettings->widget->setSettings(0);
        ParameterGrp::handle hGrp = App::GetApplication().GetParameterGroupByPath("User parameter:BaseApp/Preferences/Mod/Sketcher");

        bool continuousMode = hGrp->GetBool("ContinuousCreationMode",true);

        if (firstsegment) {
            // user when right-clicking with no segment in really wants to exit
            DrawSketchHandler::quit();
        }
        else {

            if(!continuousMode){
                DrawSketchHandler::quit();
            }
            else {
                sketchgui->toolSettings->widget->setSettings(5);
                // This code disregards existing data and enables the continuous creation mode.
                Mode=STATUS_SEEK_First;
                SegmentMode=SEGMENT_MODE_Line;
                TransitionMode=TRANSITION_MODE_Free;
                SnapMode = SNAP_MODE_Free;
                suppressTransition=false;
                firstCurve=-1;
                previousCurve=-1;
                firstPosId=Sketcher::PointPos::none;
                previousPosId=Sketcher::PointPos::none;
                firstsegment=true;
                EditCurve.clear();
                drawEdit(EditCurve);
                EditCurve.resize(2);
                applyCursor();
            }
        }
    }

protected:
    SELECT_MODE Mode;
    SEGMENT_MODE SegmentMode;
    TRANSITION_MODE TransitionMode;
    SNAP_MODE SnapMode;
    bool suppressTransition;

    std::vector<Base::Vector2d> EditCurve;
    int firstCurve;
    int previousCurve;
    Sketcher::PointPos firstPosId;
    Sketcher::PointPos previousPosId;
    // the latter stores those constraints that a first point would have been given in absence of the transition mechanism
    std::vector<AutoConstraint> sugConstr1, sugConstr2, virtualsugConstr1;

    Base::Vector2d CenterPoint;
    Base::Vector3d dirVec;
    double startAngle, endAngle, arcRadius;

    bool firstsegment;

    void updateTransitionData(int GeoId, Sketcher::PointPos PosId) {

        // Use updated startPoint/endPoint as autoconstraints can modify the position
        const Part::Geometry *geom = sketchgui->getSketchObject()->getGeometry(GeoId);
        if (geom->getTypeId() == Part::GeomLineSegment::getClassTypeId()) {
            const Part::GeomLineSegment *lineSeg = static_cast<const Part::GeomLineSegment *>(geom);
            dirVec.Set(lineSeg->getEndPoint().x - lineSeg->getStartPoint().x,
                       lineSeg->getEndPoint().y - lineSeg->getStartPoint().y,
                       0.f);
            if (PosId == Sketcher::PointPos::start) {
                dirVec *= -1;
                EditCurve[0] = Base::Vector2d(lineSeg->getStartPoint().x, lineSeg->getStartPoint().y);
            }
            else
                EditCurve[0] = Base::Vector2d(lineSeg->getEndPoint().x, lineSeg->getEndPoint().y);
        }
        else if (geom->getTypeId() == Part::GeomArcOfCircle::getClassTypeId()) {
            const Part::GeomArcOfCircle *arcSeg = static_cast<const Part::GeomArcOfCircle *>(geom);
            if (PosId == Sketcher::PointPos::start) {
                EditCurve[0] = Base::Vector2d(arcSeg->getStartPoint(/*emulateCCW=*/true).x,arcSeg->getStartPoint(/*emulateCCW=*/true).y);
                dirVec = Base::Vector3d(0.f,0.f,-1.0) % (arcSeg->getStartPoint(/*emulateCCW=*/true)-arcSeg->getCenter());
            }
            else {
                EditCurve[0] = Base::Vector2d(arcSeg->getEndPoint(/*emulateCCW=*/true).x,arcSeg->getEndPoint(/*emulateCCW=*/true).y);
                dirVec = Base::Vector3d(0.f,0.f,1.0) % (arcSeg->getEndPoint(/*emulateCCW=*/true)-arcSeg->getCenter());
            }
        }
        dirVec.Normalize();
    }
};

DEF_STD_CMD_AU(CmdSketcherCreatePolyline)

CmdSketcherCreatePolyline::CmdSketcherCreatePolyline()
  : Command("Sketcher_CreatePolyline")
{
    sAppModule      = "Sketcher";
    sGroup          = "Sketcher";
    sMenuText       = QT_TR_NOOP("Create polyline");
    sToolTipText    = QT_TR_NOOP("Create a polyline in the sketch. 'Space' Key cycles behaviour");
    sWhatsThis      = "Sketcher_CreatePolyline";
    sStatusTip      = sToolTipText;
    sPixmap         = "Sketcher_CreatePolyline";
    sAccel          = "G, M";
    eType           = ForEdit;
}

void CmdSketcherCreatePolyline::activated(int iMsg)
{
    Q_UNUSED(iMsg);
    ActivateHandler(getActiveGuiDocument(),new DrawSketchHandlerLineSet() );
}

void CmdSketcherCreatePolyline::updateAction(int mode)
{
    switch (mode) {
    case Normal:
        if (getAction())
            getAction()->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreatePolyline"));
        break;
    case Construction:
        if (getAction())
            getAction()->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreatePolyline_Constr"));
        break;
    }
}

bool CmdSketcherCreatePolyline::isActive(void)
{
    return isCreateGeoActive(getActiveGuiDocument());
}


/* Arc command =========================================================*/

class DrawSketchHandlerArc : public DrawSketchHandler
{
public:
    DrawSketchHandlerArc()
      : Mode(STATUS_SEEK_First)
      , EditCurve(2)
      , rx(0), ry(0)
      , startAngle(0)
      , endAngle(0)
      , arcAngle(0)
    {
    }
    virtual ~DrawSketchHandlerArc(){
        sketchgui->toolSettings->widget->setSettings(0);
    }
    /// mode table
    enum SelectMode {
        STATUS_SEEK_First,      /**< enum value ----. */
        STATUS_SEEK_Second,     /**< enum value ----. */
        STATUS_SEEK_Third,      /**< enum value ----. */
        STATUS_End
    };

    virtual void activated(ViewProviderSketch *)
    {
        setCrosshairCursor("Sketcher_Pointer_Create_Arc");
        sketchgui->toolSettings->widget->setSettings(3);
    }

    virtual void mouseMove(Base::Vector2d onSketchPos)
    {
        if (Mode==STATUS_SEEK_First) {
            setPositionText(onSketchPos);
            if (seekAutoConstraint(sugConstr1, onSketchPos, Base::Vector2d(0.f,0.f))) {
                renderSuggestConstraintsCursor(sugConstr1);
                return;
            }
            if (sketchgui->toolSettings->widget->isSettingSet[0] + sketchgui->toolSettings->widget->isSettingSet[1] == 2) {
                pressButton(onSketchPos);
                releaseButton(onSketchPos);
            }
        }
        else if (Mode==STATUS_SEEK_Second) {

            double dx_ = onSketchPos.x - CenterPoint.x;
            double dy_ = onSketchPos.y - CenterPoint.y;
            float angle = atan2f(dy_, dx_);

            if (sketchgui->toolSettings->widget->isSettingSet[2] == 1) {
                dx_ = cos(angle) * sketchgui->toolSettings->widget->toolParameters[2];
                dy_ = sin(angle) * sketchgui->toolSettings->widget->toolParameters[2];
            }

            for (int i=0; i < 16; i++) {
                double angle = i*M_PI/16.0;
                double dx = dx_ * cos(angle) + dy_ * sin(angle);
                double dy = -dx_ * sin(angle) + dy_ * cos(angle);
                EditCurve[1+i] = Base::Vector2d(EditCurve[0].x + dx, EditCurve[0].y + dy);
                EditCurve[17+i] = Base::Vector2d(EditCurve[0].x - dx, EditCurve[0].y - dy);
            }
            EditCurve[33] = EditCurve[1];

            // Display radius and start angle
            float radius = (onSketchPos - EditCurve[0]).Length();

            SbString text;
            text.sprintf(" (%.1fR,%.1fdeg)", radius, angle * 180 / M_PI);
            setPositionText(onSketchPos, text);

            drawEdit(EditCurve);
            if (seekAutoConstraint(sugConstr2, onSketchPos, Base::Vector2d(0.f,0.f))) {
                renderSuggestConstraintsCursor(sugConstr2);
                return;
            }
        }
        else if (Mode==STATUS_SEEK_Third) {
            double angle1 = atan2(onSketchPos.y - CenterPoint.y,
                                 onSketchPos.x - CenterPoint.x) - startAngle;
            double angle2 = angle1 + (angle1 < 0. ? 2 : -2) * M_PI ;
            arcAngle = abs(angle1-arcAngle) < abs(angle2-arcAngle) ? angle1 : angle2;
            for (int i=1; i <= 29; i++) {
                double angle = i*arcAngle/29.0;
                double dx = rx * cos(angle) - ry * sin(angle);
                double dy = rx * sin(angle) + ry * cos(angle);
                EditCurve[i] = Base::Vector2d(CenterPoint.x + dx, CenterPoint.y + dy);
            }

            // Display radius and arc angle
            float radius = (onSketchPos - EditCurve[0]).Length();

            SbString text;
            text.sprintf(" (%.1fR,%.1fdeg)", radius, arcAngle * 180 / M_PI);
            setPositionText(onSketchPos, text);

            drawEdit(EditCurve);
            if (seekAutoConstraint(sugConstr3, onSketchPos, Base::Vector2d(0.0,0.0))) {
                renderSuggestConstraintsCursor(sugConstr3);
                return;
            }
        }
        applyCursor();

    }

    virtual bool pressButton(Base::Vector2d onSketchPos)
    {
        if (Mode==STATUS_SEEK_First){
            EditCurve.resize(34);
            if (sketchgui->toolSettings->widget->isSettingSet[0] == 1) {
                CenterPoint.x = sketchgui->toolSettings->widget->toolParameters[0];
            }
            else {
                CenterPoint.x = onSketchPos.x;
            }
            if (sketchgui->toolSettings->widget->isSettingSet[1] == 1) {
                CenterPoint.y = sketchgui->toolSettings->widget->toolParameters[1];
            }
            else {
                CenterPoint.y = onSketchPos.y;
            }
            EditCurve[0] = CenterPoint;

            sketchgui->toolSettings->widget->setParameterActive(0, 0);
            sketchgui->toolSettings->widget->setParameterActive(0, 1);
            sketchgui->toolSettings->widget->setParameterActive(1, 2);
            sketchgui->toolSettings->widget->setParameterFocus(2);
            Mode = STATUS_SEEK_Second;
        }
        else if (Mode==STATUS_SEEK_Second){
            rx = onSketchPos.x - CenterPoint.x;
            ry = onSketchPos.y - CenterPoint.y;
            startAngle = atan2f(ry, rx);

            if (sketchgui->toolSettings->widget->isSettingSet[2] == 1) {
                rx = cos(startAngle) * sketchgui->toolSettings->widget->toolParameters[2];
                ry = sin(startAngle) * sketchgui->toolSettings->widget->toolParameters[2];
            }

            EditCurve.resize(31);
            EditCurve[0].x = rx + CenterPoint.x;
            EditCurve[0].y = ry + CenterPoint.y;
                
            EditCurve[30] = CenterPoint;
            arcAngle = 0.;

            sketchgui->toolSettings->widget->setParameterActive(0, 2);
            Mode = STATUS_SEEK_Third;
        }
        else {
            EditCurve.resize(30);
            double angle1 = atan2(onSketchPos.y - CenterPoint.y,
                                 onSketchPos.x - CenterPoint.x) - startAngle;
            double angle2 = angle1 + (angle1 < 0. ? 2 : -2) * M_PI ;
            arcAngle = abs(angle1-arcAngle) < abs(angle2-arcAngle) ? angle1 : angle2;
            if (arcAngle > 0)
                endAngle = startAngle + arcAngle;
            else {
                endAngle = startAngle;
                startAngle += arcAngle;
            }

            drawEdit(EditCurve);
            applyCursor();
            Mode = STATUS_End;
        }

        return true;
    }

    virtual bool releaseButton(Base::Vector2d onSketchPos)
    {
        Q_UNUSED(onSketchPos);
        if (Mode==STATUS_End) {
            unsetCursor();
            resetPositionText();

            int firstCurve = getHighestCurveIndex() + 1;
            try {
                Gui::Command::openCommand(QT_TRANSLATE_NOOP("Command", "Add sketch arc"));
                Gui::cmdAppObjectArgs(sketchgui->getObject(), "addGeometry(Part.ArcOfCircle"
                    "(Part.Circle(App.Vector(%f,%f,0),App.Vector(0,0,1),%f),%f,%f),%s)",
                          CenterPoint.x, CenterPoint.y, sqrt(rx*rx + ry*ry),
                          startAngle, endAngle,
                          geometryCreationMode==Construction?"True":"False"); //arcAngle > 0 ? 0 : 1);

            //add constraint if user typed in some dimensions in tool widget
                if (sketchgui->toolSettings->widget->isSettingSet[0] + sketchgui->toolSettings->widget->isSettingSet[1] + sketchgui->toolSettings->widget->isSettingSet[2] + sketchgui->toolSettings->widget->isSettingSet[3] != 0) {
                    if (sketchgui->toolSettings->widget->isSettingSet[0] == 1) {
                        distanceXYorPointOnObject(0, firstCurve, Sketcher::PointPos::mid, sketchgui->toolSettings->widget->toolParameters[0]);
                    }
                    if (sketchgui->toolSettings->widget->isSettingSet[1] == 1) {
                        distanceXYorPointOnObject(1, firstCurve, Sketcher::PointPos::mid, sketchgui->toolSettings->widget->toolParameters[1]);
                    }
                    if (sketchgui->toolSettings->widget->isSettingSet[2] == 1) {
                        Gui::cmdAppObjectArgs(sketchgui->getObject(), "addConstraint(Sketcher.Constraint('Radius',%d,%f)) ",
                            firstCurve, sketchgui->toolSettings->widget->toolParameters[2]);
                    }
                }

                Gui::Command::commitCommand();
            }
            catch (const Base::Exception& e) {
                Base::Console().Error("Failed to add arc: %s\n", e.what());
                Gui::Command::abortCommand();
            }

            // Auto Constraint center point
            if (sugConstr1.size() > 0) {
                createAutoConstraints(sugConstr1, getHighestCurveIndex(), Sketcher::PointPos::mid);
                sugConstr1.clear();
            }

            // Auto Constraint first picked point
            if (sugConstr2.size() > 0) {
                createAutoConstraints(sugConstr2, getHighestCurveIndex(), (arcAngle > 0) ? Sketcher::PointPos::start : Sketcher::PointPos::end );
                sugConstr2.clear();
            }

            // Auto Constraint second picked point
            if (sugConstr3.size() > 0) {
                createAutoConstraints(sugConstr3, getHighestCurveIndex(), (arcAngle > 0) ? Sketcher::PointPos::end : Sketcher::PointPos::start);
                sugConstr3.clear();
            }


            tryAutoRecomputeIfNotSolve(static_cast<Sketcher::SketchObject *>(sketchgui->getObject()));

            sketchgui->toolSettings->widget->setSettings(0);
            ParameterGrp::handle hGrp = App::GetApplication().GetParameterGroupByPath("User parameter:BaseApp/Preferences/Mod/Sketcher");
            bool continuousMode = hGrp->GetBool("ContinuousCreationMode",true);
            if(continuousMode){
                // This code enables the continuous creation mode.
                sketchgui->toolSettings->widget->setSettings(3);
                Mode=STATUS_SEEK_First;
                EditCurve.clear();
                drawEdit(EditCurve);
                EditCurve.resize(2);
                applyCursor();
                /* this is ok not to call to purgeHandler
                * in continuous creation mode because the
                * handler is destroyed by the quit() method on pressing the
                * right button of the mouse */
            }
            else{
                sketchgui->purgeHandler(); // no code after this line, Handler get deleted in ViewProvider
            }
        }
        return true;
    }
protected:
    SelectMode Mode;
    std::vector<Base::Vector2d> EditCurve;
    Base::Vector2d CenterPoint;
    double rx, ry, startAngle, endAngle, arcAngle;
    std::vector<AutoConstraint> sugConstr1, sugConstr2, sugConstr3;
};

DEF_STD_CMD_A(CmdSketcherCreateArc)

CmdSketcherCreateArc::CmdSketcherCreateArc()
  : Command("Sketcher_CreateArc")
{
    sAppModule      = "Sketcher";
    sGroup          = "Sketcher";
    sMenuText       = QT_TR_NOOP("Create arc by center");
    sToolTipText    = QT_TR_NOOP("Create an arc by its center and by its end points");
    sWhatsThis      = "Sketcher_CreateArc";
    sStatusTip      = sToolTipText;
    sPixmap         = "Sketcher_CreateArc";
    sAccel          = "G, A";
    eType           = ForEdit;
}

void CmdSketcherCreateArc::activated(int iMsg)
{
    Q_UNUSED(iMsg);
    ActivateHandler(getActiveGuiDocument(),new DrawSketchHandlerArc() );
}

bool CmdSketcherCreateArc::isActive(void)
{
    return isCreateGeoActive(getActiveGuiDocument());
}


/* 3 points arc command ==================================================*/

class DrawSketchHandler3PointArc : public DrawSketchHandler
{
public:
    DrawSketchHandler3PointArc()
      : Mode(STATUS_SEEK_First), EditCurve(2)
      , radius(0), startAngle(0)
      , endAngle(0), arcAngle(0)
      , arcPos1(Sketcher::PointPos::none)
      , arcPos2(Sketcher::PointPos::none)
    {
    }
    virtual ~DrawSketchHandler3PointArc(){
        sketchgui->toolSettings->widget->setSettings(0);
    }
    /// mode table
    enum SelectMode {
        STATUS_SEEK_First,      /**< enum value ----. */
        STATUS_SEEK_Second,     /**< enum value ----. */
        STATUS_SEEK_Third,      /**< enum value ----. */
        STATUS_End
    };

    virtual void activated(ViewProviderSketch *)
    {
        setCrosshairCursor("Sketcher_Pointer_Create_3PointArc");
        sketchgui->toolSettings->widget->setSettings(5);
    }

    virtual void mouseMove(Base::Vector2d onSketchPos)
    {
        if (Mode==STATUS_SEEK_First) {
            setPositionText(onSketchPos);
            if (seekAutoConstraint(sugConstr1, onSketchPos, Base::Vector2d(0.f,0.f))) {
                renderSuggestConstraintsCursor(sugConstr1);
                return;
            }
            if (sketchgui->toolSettings->widget->isSettingSet[0] + sketchgui->toolSettings->widget->isSettingSet[1] == 2) {
                pressButton(onSketchPos);
                releaseButton(onSketchPos);
            }
        }
        else if (Mode==STATUS_SEEK_Second) {
            if (sketchgui->toolSettings->widget->isSettingSet[2] == 1) {
                SecondPoint.x = sketchgui->toolSettings->widget->toolParameters[2];
            }
            else {
                SecondPoint.x = onSketchPos.x;
            }
            if (sketchgui->toolSettings->widget->isSettingSet[3] == 1) {
                SecondPoint.y = sketchgui->toolSettings->widget->toolParameters[3];
            }
            else {
                SecondPoint.y = onSketchPos.y;
            }
            CenterPoint  = EditCurve[0] = (SecondPoint - FirstPoint)/2 + FirstPoint;
            EditCurve[1] = EditCurve[33] = SecondPoint;
            radius = (SecondPoint - CenterPoint).Length();
            double lineAngle = GetPointAngle(CenterPoint, SecondPoint);

            // Build a 32 point circle ignoring already constructed points
            for (int i=1; i <= 32; i++) {
                // Start at current angle
                double angle = (i-1)*2*M_PI/32.0 + lineAngle; // N point closed circle has N segments
                if (i != 1 && i != 17 ) {
                    EditCurve[i] = Base::Vector2d(CenterPoint.x + radius*cos(angle),
                                                  CenterPoint.y + radius*sin(angle));
                }
            }

            // Display radius and start angle
            // This lineAngle will report counter-clockwise from +X, not relatively
            SbString text;
            text.sprintf(" (%.1fR,%.1fdeg)", (float) radius, (float) lineAngle * 180 / M_PI);
            setPositionText(onSketchPos, text);

            drawEdit(EditCurve);
            if (seekAutoConstraint(sugConstr2, onSketchPos, Base::Vector2d(0.f,0.f))) {
                renderSuggestConstraintsCursor(sugConstr2);
                return;
            }
            if (sketchgui->toolSettings->widget->isSettingSet[2] == 1 && sketchgui->toolSettings->widget->isSettingSet[3] == 1) {
                pressButton(onSketchPos);
                releaseButton(onSketchPos);
            }
        }
        else if (Mode==STATUS_SEEK_Third) {
            /*
            Centerline inverts when the arc flips sides.  Easily taken care of by replacing
            centerline with a point.  It happens because the direction the curve is being drawn
            reverses.
            */
            try {
                CenterPoint = EditCurve[30] = Part::Geom2dCircle::getCircleCenter(FirstPoint, SecondPoint, onSketchPos);

                radius = (SecondPoint - CenterPoint).Length();

                double angle1 = GetPointAngle(CenterPoint, FirstPoint);
                double angle2 = GetPointAngle(CenterPoint, SecondPoint);
                double angle3 = GetPointAngle(CenterPoint, onSketchPos);

                // Always build arc counter-clockwise
                // Point 3 is between Point 1 and 2
                if ( angle3 > min(angle1, angle2) && angle3 < max(angle1, angle2) ) {
                    if (angle2 > angle1) {
                        EditCurve[0] =  FirstPoint;
                        EditCurve[29] = SecondPoint;
                        arcPos1 = Sketcher::PointPos::start;
                        arcPos2 = Sketcher::PointPos::end;
                    }
                    else {
                        EditCurve[0] =  SecondPoint;
                        EditCurve[29] = FirstPoint;
                        arcPos1 = Sketcher::PointPos::end;
                        arcPos2 = Sketcher::PointPos::start;
                    }
                    startAngle = min(angle1, angle2);
                    endAngle   = max(angle1, angle2);
                    arcAngle = endAngle - startAngle;
                }
                // Point 3 is not between Point 1 and 2
                else {
                    if (angle2 > angle1) {
                        EditCurve[0] =  SecondPoint;
                        EditCurve[29] = FirstPoint;
                        arcPos1 = Sketcher::PointPos::end;
                        arcPos2 = Sketcher::PointPos::start;
                    }
                    else {
                        EditCurve[0] =  FirstPoint;
                        EditCurve[29] = SecondPoint;
                        arcPos1 = Sketcher::PointPos::start;
                        arcPos2 = Sketcher::PointPos::end;
                    }
                    startAngle = max(angle1, angle2);
                    endAngle   = min(angle1, angle2);
                    arcAngle = 2*M_PI - (startAngle - endAngle);
                }

                // Build a 30 point circle ignoring already constructed points
                for (int i=1; i <= 28; i++) {
                    double angle = startAngle + i*arcAngle/29.0; // N point arc has N-1 segments
                    EditCurve[i] = Base::Vector2d(CenterPoint.x + radius*cos(angle),
                                                CenterPoint.y + radius*sin(angle));
                }

                SbString text;
                text.sprintf(" (%.1fR,%.1fdeg)", (float) radius, (float) arcAngle * 180 / M_PI);
                setPositionText(onSketchPos, text);

                drawEdit(EditCurve);
                if (seekAutoConstraint(sugConstr3, onSketchPos, Base::Vector2d(0.0,0.0),
                                    AutoConstraint::CURVE)) {
                    renderSuggestConstraintsCursor(sugConstr3);
                    return;
                }
            }
            catch(Base::ValueError &e) {
                e.ReportException();
            }
        }
        applyCursor();
    }

    virtual bool pressButton(Base::Vector2d onSketchPos)
    {
        if (Mode==STATUS_SEEK_First){
            if (sketchgui->toolSettings->widget->isSettingSet[0] == 1) {
                FirstPoint.x = sketchgui->toolSettings->widget->toolParameters[0];
            }
            else {
                FirstPoint.x = onSketchPos.x;
            }
            if (sketchgui->toolSettings->widget->isSettingSet[1] == 1) {
                FirstPoint.y = sketchgui->toolSettings->widget->toolParameters[1];
            }
            else {
                FirstPoint.y = onSketchPos.y;
            }

            // 32 point curve + center + endpoint
            EditCurve.resize(34);
            // 17 is circle halfway point (1+32/2)
            EditCurve[17] = FirstPoint;

            sketchgui->toolSettings->widget->setParameterActive(0, 0);
            sketchgui->toolSettings->widget->setParameterActive(0, 1);
            sketchgui->toolSettings->widget->setParameterActive(1, 2);
            sketchgui->toolSettings->widget->setParameterActive(1, 3);
            sketchgui->toolSettings->widget->setParameterFocus(2);
            Mode = STATUS_SEEK_Second;
        }
        else if (Mode==STATUS_SEEK_Second){
            // 30 point arc and center point
            EditCurve.resize(31);
            if (sketchgui->toolSettings->widget->isSettingSet[2] == 1) {
                SecondPoint.x = sketchgui->toolSettings->widget->toolParameters[2];
            }
            else {
                SecondPoint.x = onSketchPos.x;
            }
            if (sketchgui->toolSettings->widget->isSettingSet[3] == 1) {
                SecondPoint.y = sketchgui->toolSettings->widget->toolParameters[3];
            }
            else {
                SecondPoint.y = onSketchPos.y;
            }

            sketchgui->toolSettings->widget->setParameterActive(0, 2);
            sketchgui->toolSettings->widget->setParameterActive(0, 3);
            Mode = STATUS_SEEK_Third;
        }
        else {
            EditCurve.resize(30);

            drawEdit(EditCurve);
            applyCursor();
            Mode = STATUS_End;
        }

        return true;
    }

    virtual bool releaseButton(Base::Vector2d onSketchPos)
    {
        Q_UNUSED(onSketchPos);
        // Need to look at.  rx might need fixing.
        if (Mode==STATUS_End) {
            unsetCursor();
            resetPositionText();

            int firstCurve = getHighestCurveIndex() + 1;
            try {
                Gui::Command::openCommand(QT_TRANSLATE_NOOP("Command", "Add sketch arc"));
                Gui::cmdAppObjectArgs(sketchgui->getObject(), "addGeometry(Part.ArcOfCircle"
                    "(Part.Circle(App.Vector(%f,%f,0),App.Vector(0,0,1),%f),%f,%f),%s)",
                          CenterPoint.x, CenterPoint.y, radius,
                          startAngle, endAngle,
                          geometryCreationMode==Construction?"True":"False");

                //add constraint if user typed in some dimensions in tool widget
                if (sketchgui->toolSettings->widget->isSettingSet[0] + sketchgui->toolSettings->widget->isSettingSet[1] + sketchgui->toolSettings->widget->isSettingSet[2] + sketchgui->toolSettings->widget->isSettingSet[3] != 0) {
                    if (sketchgui->toolSettings->widget->isSettingSet[0] == 1) {
                        distanceXYorPointOnObject(0, firstCurve, arcPos1, sketchgui->toolSettings->widget->toolParameters[0]);
                    }
                    if (sketchgui->toolSettings->widget->isSettingSet[1] == 1) {
                        distanceXYorPointOnObject(1, firstCurve, arcPos1, sketchgui->toolSettings->widget->toolParameters[1]);
                    }
                    if (sketchgui->toolSettings->widget->isSettingSet[2] == 1) {
                        distanceXYorPointOnObject(0, firstCurve, arcPos2, sketchgui->toolSettings->widget->toolParameters[2]);
                    }
                    if (sketchgui->toolSettings->widget->isSettingSet[3] == 1) {
                        distanceXYorPointOnObject(1, firstCurve, arcPos2, sketchgui->toolSettings->widget->toolParameters[3]);
                    }
                }

                Gui::Command::commitCommand();
            }
            catch (const Base::Exception& e) {
                Base::Console().Error("Failed to add arc: %s\n", e.what());
                Gui::Command::abortCommand();
            }
            
            // Auto Constraint first picked point
            if (sugConstr1.size() > 0) {
                createAutoConstraints(sugConstr1, getHighestCurveIndex(), arcPos1);
                sugConstr1.clear();
            }

            // Auto Constraint second picked point
            if (sugConstr2.size() > 0) {
                createAutoConstraints(sugConstr2, getHighestCurveIndex(), arcPos2);
                sugConstr2.clear();
            }

            // Auto Constraint third picked point
            if (sugConstr3.size() > 0) {
                createAutoConstraints(sugConstr3, getHighestCurveIndex(), Sketcher::PointPos::none);
                sugConstr3.clear();
            }

            tryAutoRecomputeIfNotSolve(static_cast<Sketcher::SketchObject *>(sketchgui->getObject()));

            sketchgui->toolSettings->widget->setSettings(0);
            ParameterGrp::handle hGrp = App::GetApplication().GetParameterGroupByPath("User parameter:BaseApp/Preferences/Mod/Sketcher");
            bool continuousMode = hGrp->GetBool("ContinuousCreationMode",true);
            if(continuousMode){
                // This code enables the continuous creation mode.
                sketchgui->toolSettings->widget->setSettings(5);
                Mode=STATUS_SEEK_First;
                EditCurve.clear();
                drawEdit(EditCurve);
                EditCurve.resize(2);
                applyCursor();
                /* this is ok not to call to purgeHandler
                * in continuous creation mode because the
                * handler is destroyed by the quit() method on pressing the
                * right button of the mouse */
            }
            else{
                sketchgui->purgeHandler(); // no code after this line, Handler get deleted in ViewProvider
            }
        }
        return true;
    }
protected:
    SelectMode Mode;
    std::vector<Base::Vector2d> EditCurve;
    Base::Vector2d CenterPoint, FirstPoint, SecondPoint;
    double radius, startAngle, endAngle, arcAngle;
    std::vector<AutoConstraint> sugConstr1, sugConstr2, sugConstr3;
    Sketcher::PointPos arcPos1, arcPos2;
};

DEF_STD_CMD_A(CmdSketcherCreate3PointArc)

CmdSketcherCreate3PointArc::CmdSketcherCreate3PointArc()
  : Command("Sketcher_Create3PointArc")
{
    sAppModule      = "Sketcher";
    sGroup          = "Sketcher";
    sMenuText       = QT_TR_NOOP("Create arc by three points");
    sToolTipText    = QT_TR_NOOP("Create an arc by its end points and a point along the arc");
    sWhatsThis      = "Sketcher_Create3PointArc";
    sStatusTip      = sToolTipText;
    sPixmap         = "Sketcher_Create3PointArc";
    sAccel          = "G, 3, A";
    eType           = ForEdit;
}

void CmdSketcherCreate3PointArc::activated(int iMsg)
{
    Q_UNUSED(iMsg);
    ActivateHandler(getActiveGuiDocument(),new DrawSketchHandler3PointArc() );
}

bool CmdSketcherCreate3PointArc::isActive(void)
{
    return isCreateGeoActive(getActiveGuiDocument());
}


DEF_STD_CMD_ACLU(CmdSketcherCompCreateArc)

CmdSketcherCompCreateArc::CmdSketcherCompCreateArc()
  : Command("Sketcher_CompCreateArc")
{
    sAppModule      = "Sketcher";
    sGroup          = "Sketcher";
    sMenuText       = QT_TR_NOOP("Create arc");
    sToolTipText    = QT_TR_NOOP("Create an arc in the sketcher");
    sWhatsThis      = "Sketcher_CompCreateArc";
    sStatusTip      = sToolTipText;
    eType           = ForEdit;
}

void CmdSketcherCompCreateArc::activated(int iMsg)
{
    if (iMsg==0)
        ActivateHandler(getActiveGuiDocument(),new DrawSketchHandlerArc());
    else if (iMsg==1)
        ActivateHandler(getActiveGuiDocument(),new DrawSketchHandler3PointArc());
    else
        return;

    // Since the default icon is reset when enabling/disabling the command we have
    // to explicitly set the icon of the used command.
    Gui::ActionGroup* pcAction = qobject_cast<Gui::ActionGroup*>(_pcAction);
    QList<QAction*> a = pcAction->actions();

    assert(iMsg < a.size());
    pcAction->setIcon(a[iMsg]->icon());
}

Gui::Action * CmdSketcherCompCreateArc::createAction(void)
{
    Gui::ActionGroup* pcAction = new Gui::ActionGroup(this, Gui::getMainWindow());
    pcAction->setDropDownMenu(true);
    applyCommandData(this->className(), pcAction);

    QAction* arc1 = pcAction->addAction(QString());
    arc1->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateArc"));
    QAction* arc2 = pcAction->addAction(QString());
    arc2->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_Create3PointArc"));

    _pcAction = pcAction;
    languageChange();

    pcAction->setIcon(arc1->icon());
    int defaultId = 0;
    pcAction->setProperty("defaultAction", QVariant(defaultId));

    return pcAction;
}

void CmdSketcherCompCreateArc::updateAction(int mode)
{
    Gui::ActionGroup* pcAction = qobject_cast<Gui::ActionGroup*>(getAction());
    if (!pcAction)
        return;

    QList<QAction*> a = pcAction->actions();
    int index = pcAction->property("defaultAction").toInt();
    switch (mode) {
    case Normal:
        a[0]->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateArc"));
        a[1]->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_Create3PointArc"));
        getAction()->setIcon(a[index]->icon());
        break;
    case Construction:
        a[0]->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateArc_Constr"));
        a[1]->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_Create3PointArc_Constr"));
        getAction()->setIcon(a[index]->icon());
        break;
    }
}

void CmdSketcherCompCreateArc::languageChange()
{
    Command::languageChange();

    if (!_pcAction)
        return;
    Gui::ActionGroup* pcAction = qobject_cast<Gui::ActionGroup*>(_pcAction);
    QList<QAction*> a = pcAction->actions();

    QAction* arc1 = a[0];
    arc1->setText(QApplication::translate("CmdSketcherCompCreateArc","Center and end points"));
    arc1->setToolTip(QApplication::translate("Sketcher_CreateArc","Create an arc by its center and by its end points"));
    arc1->setStatusTip(QApplication::translate("Sketcher_CreateArc","Create an arc by its center and by its end points"));
    QAction* arc2 = a[1];
    arc2->setText(QApplication::translate("CmdSketcherCompCreateArc","End points and rim point"));
    arc2->setToolTip(QApplication::translate("Sketcher_Create3PointArc","Create an arc by its end points and a point along the arc"));
    arc2->setStatusTip(QApplication::translate("Sketcher_Create3PointArc","Create an arc by its end points and a point along the arc"));
}

bool CmdSketcherCompCreateArc::isActive(void)
{
    return isCreateGeoActive(getActiveGuiDocument());
}


/* Circle command =======================================================*/

class DrawSketchHandlerCircle : public DrawSketchHandler
{
public:
    DrawSketchHandlerCircle() : Mode(STATUS_SEEK_First),EditCurve(34){}
    virtual ~DrawSketchHandlerCircle(){}
    /// mode table
    enum SelectMode {
        STATUS_SEEK_First,      /**< enum value ----. */
        STATUS_SEEK_Second,     /**< enum value ----. */
        STATUS_Close
    };

    virtual void activated(ViewProviderSketch *)
    {
        setCrosshairCursor("Sketcher_Pointer_Create_Circle");
        sketchgui->toolSettings->widget->setSettings(3);
    }

    virtual void mouseMove(Base::Vector2d onSketchPos)
    {
        if (Mode==STATUS_SEEK_First) {
            setPositionText(onSketchPos);
            if (seekAutoConstraint(sugConstr1, onSketchPos, Base::Vector2d(0.f,0.f))) {
                renderSuggestConstraintsCursor(sugConstr1);
                return;
            }
            if (sketchgui->toolSettings->widget->isSettingSet[0] + sketchgui->toolSettings->widget->isSettingSet[1] == 2) {
                pressButton(onSketchPos);
                releaseButton(onSketchPos);
            }
        }
        else if (Mode==STATUS_SEEK_Second) {
            double rx0, ry0;
            if (sketchgui->toolSettings->widget->isSettingSet[2] == 1) {
                rx0 = sketchgui->toolSettings->widget->toolParameters[2];
                ry0 = 0;
            }
            else {
                rx0 = onSketchPos.x - EditCurve[0].x;
                ry0 = onSketchPos.y - EditCurve[0].y;
            }

            for (int i=0; i < 16; i++) {
                double angle = i*M_PI/16.0;
                double rx = rx0 * cos(angle) + ry0 * sin(angle);
                double ry = -rx0 * sin(angle) + ry0 * cos(angle);
                EditCurve[1+i] = Base::Vector2d(EditCurve[0].x + rx, EditCurve[0].y + ry);
                EditCurve[17+i] = Base::Vector2d(EditCurve[0].x - rx, EditCurve[0].y - ry);
            }
            EditCurve[33] = EditCurve[1];

            // Display radius for user
            float radius = (onSketchPos - EditCurve[0]).Length();

            SbString text;
            text.sprintf(" (%.1fR)", radius);
            setPositionText(onSketchPos, text);

            drawEdit(EditCurve);
            if (seekAutoConstraint(sugConstr2, onSketchPos, onSketchPos - EditCurve[0],
                                   AutoConstraint::CURVE)) {
                renderSuggestConstraintsCursor(sugConstr2);
                return;
            }
            if (sketchgui->toolSettings->widget->isSettingSet[2] == 1) {
                pressButton(onSketchPos);
                releaseButton(onSketchPos);
            }

        }
        applyCursor();
    }

    virtual bool pressButton(Base::Vector2d onSketchPos)
    {
        if (Mode==STATUS_SEEK_First){
            if (sketchgui->toolSettings->widget->isSettingSet[0] == 1) {
                EditCurve[0].x = sketchgui->toolSettings->widget->toolParameters[0];
            }
            else {
                EditCurve[0].x = onSketchPos.x;
            }
            if (sketchgui->toolSettings->widget->isSettingSet[1] == 1) {
                EditCurve[0].y = sketchgui->toolSettings->widget->toolParameters[1];
            }
            else {
                EditCurve[0].y = onSketchPos.y;
            }

            sketchgui->toolSettings->widget->setParameterActive(0, 0);
            sketchgui->toolSettings->widget->setParameterActive(0, 1);
            sketchgui->toolSettings->widget->setParameterActive(1, 2);
            sketchgui->toolSettings->widget->setParameterFocus(2);
            Mode = STATUS_SEEK_Second;
        } 
        else {
            EditCurve[1] = onSketchPos;
            Mode = STATUS_Close;
        }
        return true;
    }

    virtual bool releaseButton(Base::Vector2d onSketchPos)
    {
        Q_UNUSED(onSketchPos);
        if (Mode==STATUS_Close) {
            double rx, ry;
            if (sketchgui->toolSettings->widget->isSettingSet[2] == 1) {
                rx = sketchgui->toolSettings->widget->toolParameters[2];
                ry = 0;
            }
            else {
                rx = EditCurve[1].x - EditCurve[0].x;
                ry = EditCurve[1].y - EditCurve[0].y;
            }
            unsetCursor();
            resetPositionText();

            int firstCurve = getHighestCurveIndex() + 1;
            try {
                Gui::Command::openCommand(QT_TRANSLATE_NOOP("Command", "Add sketch circle"));
                Gui::cmdAppObjectArgs(sketchgui->getObject(), "addGeometry(Part.Circle"
                    "(App.Vector(%f,%f,0),App.Vector(0,0,1),%f),%s)",
                          EditCurve[0].x, EditCurve[0].y,
                          sqrt(rx*rx + ry*ry),
                          geometryCreationMode==Construction?"True":"False");

                //add constraint if user typed in some dimensions in tool widget
                if (sketchgui->toolSettings->widget->isSettingSet[0] + sketchgui->toolSettings->widget->isSettingSet[1] + sketchgui->toolSettings->widget->isSettingSet[2] + sketchgui->toolSettings->widget->isSettingSet[3] != 0) {
                    if (sketchgui->toolSettings->widget->isSettingSet[0] == 1) {
                        distanceXYorPointOnObject(0, firstCurve, Sketcher::PointPos::mid, sketchgui->toolSettings->widget->toolParameters[0]);
                    }
                    if (sketchgui->toolSettings->widget->isSettingSet[1] == 1) {
                        distanceXYorPointOnObject(1, firstCurve, Sketcher::PointPos::mid, sketchgui->toolSettings->widget->toolParameters[1]);
                    }
                    if (sketchgui->toolSettings->widget->isSettingSet[2] == 1) {
                        Gui::cmdAppObjectArgs(sketchgui->getObject(), "addConstraint(Sketcher.Constraint('Radius',%d,%f)) ",
                            firstCurve, rx);
                    }
                }

                Gui::Command::commitCommand();
            }
            catch (const Base::Exception& e) {
                Base::Console().Error("Failed to add circle: %s\n", e.what());
                Gui::Command::abortCommand();
            }
            
            // add auto constraints for the center point
            if (sugConstr1.size() > 0) {
                createAutoConstraints(sugConstr1, getHighestCurveIndex(), Sketcher::PointPos::mid);
                sugConstr1.clear();
            }

            // add suggested constraints for circumference
            if (sugConstr2.size() > 0) {
                createAutoConstraints(sugConstr2, getHighestCurveIndex(), Sketcher::PointPos::none);
                sugConstr2.clear();
            }

            tryAutoRecomputeIfNotSolve(static_cast<Sketcher::SketchObject *>(sketchgui->getObject()));

            sketchgui->toolSettings->widget->setSettings(0);
            ParameterGrp::handle hGrp = App::GetApplication().GetParameterGroupByPath("User parameter:BaseApp/Preferences/Mod/Sketcher");
            bool continuousMode = hGrp->GetBool("ContinuousCreationMode",true);
            if(continuousMode){
                // This code enables the continuous creation mode.
                sketchgui->toolSettings->widget->setSettings(3);
                Mode=STATUS_SEEK_First;
                EditCurve.clear();
                drawEdit(EditCurve);
                EditCurve.resize(34);
                applyCursor();
                /* this is ok not to call to purgeHandler
                * in continuous creation mode because the
                * handler is destroyed by the quit() method on pressing the
                * right button of the mouse */
            }
            else{
                sketchgui->purgeHandler(); // no code after this line, Handler get deleted in ViewProvider
            }
        }
        return true;
    }
protected:
    SelectMode Mode;
    std::vector<Base::Vector2d> EditCurve;
    std::vector<AutoConstraint> sugConstr1, sugConstr2;

};

DEF_STD_CMD_A(CmdSketcherCreateCircle)

CmdSketcherCreateCircle::CmdSketcherCreateCircle()
  : Command("Sketcher_CreateCircle")
{
    sAppModule      = "Sketcher";
    sGroup          = "Sketcher";
    sMenuText       = QT_TR_NOOP("Create circle");
    sToolTipText    = QT_TR_NOOP("Create a circle in the sketch");
    sWhatsThis      = "Sketcher_CreateCircle";
    sStatusTip      = sToolTipText;
    sPixmap         = "Sketcher_CreateCircle";
    sAccel          = "G, C";
    eType           = ForEdit;
}

void CmdSketcherCreateCircle::activated(int iMsg)
{
    Q_UNUSED(iMsg);
    ActivateHandler(getActiveGuiDocument(),new DrawSketchHandlerCircle() );
}

bool CmdSketcherCreateCircle::isActive(void)
{
    return isCreateGeoActive(getActiveGuiDocument());
}

// ======================================================================================


/**
 * @brief This class handles user interaction to draw and save the ellipse
 *
 * Two construction methods are implemented:
 *   -Periapsis, apoapsis, and b; and
 *   -Center, periapsis, and b.
 *
 * The first method limits the ellipse to a circle, while the second method allows for
 * swapping of the semi-major and semi-minor axes.
 *
 * We use three reference frames in this class.  The first (and primary), is the cartesian
 * frame of the sketcher; all our work begins and ends in this frame.  The second is the
 * perifocal frame of the ellipse using polar coordinates.  We use this frame for naming
 * conventions and working with the ellipse.  The last is a rotated right-handed cartesian
 * frame centered at the ellipse center with the +X direction towards periapsis, +Z out of
 * screen.
 *
 * When working with an ellipse in the perifocal frame, the following equations are useful:
 *
 *    \f{eqnarray*}{
 *        r &\equiv& \textrm{ radial distance from the focus to a point on the ellipse}\\
 *        r_a &\equiv& \textrm{ radial distance from the focus to apopasis}\\
 *        r_p &\equiv& \textrm{ radial distance from the focus to periapsis}\\
 *        a &\equiv& \textrm{ length of the semi-major axis, colloquially 'radius'}\\
 *        b &\equiv& \textrm{ length of the semi-minor axis, colloquially 'radius'}\\
 *        e &\equiv& \textrm{ eccentricity of the ellipse}\\
 *        \theta_b &\equiv& \textrm{ angle to the intersection of the semi-minor axis and the ellipse, relative to the focus}\\
 *        ae &\equiv& \textrm{ distance from the focus to the centroid}\\
 *        r &=& \frac{a(1-e^2)}{1+e\cos(\theta)} = \frac{r_a(1-e)}{1+e\cos(\theta)} = \frac{r_p(1+e)}{1+e\cos(\theta)}\\
 *        r_a &=& a(1-e)\\
 *        r_p &=& a(1+e)\\
 *        a &=& \frac{r_p+r_a}{2}\\
 *        b &=& a\sqrt{1-e^2}\\
 *        e &=& \frac{r_a-r_p}{r_a+r_p} = \sqrt{1-\frac{b^2}{a^2}}\\
 *        \theta_b &=& \left[\pi - \arctan\left(\frac{b}{ae}\right)\right] \pm N\pi
 *   \f}
 *
 */
class DrawSketchHandlerEllipse : public DrawSketchHandler
{
public:
    DrawSketchHandlerEllipse(int constructionMethod)
      : mode(STATUS_Close)
      , method(CENTER_PERIAPSIS_B)
      , constrMethod(constructionMethod)
      , a(0), b(0), e(0), ratio(0), ae(0)
      , num(0), r(0), theta(0), phi(0)
      , editCurve(33), fixedAxisLength(0)
    {
    }
    virtual ~DrawSketchHandlerEllipse(){}
    /// Mode table, describes what step of the process we are in
    enum SelectMode {
        STATUS_SEEK_PERIAPSIS,  /**< enum value, looking for click to set periapsis. */
        STATUS_SEEK_APOAPSIS,   /**< enum value, looking for click to set apoapsis. */
        STATUS_SEEK_CENTROID,   /**< enum value, looking for click to set centroid. */
        STATUS_SEEK_A,          /**< enum value, looking for click to set a. */
        STATUS_SEEK_B,          /**< enum value, looking for click to set b. */
        STATUS_Close            /**< enum value, finalizing and saving ellipse. */
    };
    /// Construction methods, describes the method used to construct the ellipse
    enum ConstructionMethod {
        CENTER_PERIAPSIS_B,     /**< enum value, click on center, then periapsis, then b point. */
        PERIAPSIS_APOAPSIS_B    /**< enum value, click on periapsis, then apoapsis, then b point. */
    };

    /**
     * @brief Slot called when the create ellipse command is activated
     * @param sketchgui A pointer to the active sketch
     */
    virtual void activated(ViewProviderSketch *)
    {
        sketchgui->toolSettings->widget->setSettings(7);
        setCrosshairCursor("Sketcher_Pointer_Create_Ellipse");
        if (constrMethod == 0) {
            method = CENTER_PERIAPSIS_B;
            mode = STATUS_SEEK_CENTROID;
            sketchgui->toolSettings->widget->setLabel(QApplication::translate("ConstrainContextually",
                "x of Center Point"), 0);
            sketchgui->toolSettings->widget->setLabel(QApplication::translate("ConstrainContextually",
                "y of Center Point"), 1);
        } else {
            method = PERIAPSIS_APOAPSIS_B;
            mode = STATUS_SEEK_PERIAPSIS;
        }
    }

    /**
     * @brief Updates the ellipse when the cursor moves
     * @param onSketchPos the position of the cursor on the sketch
     */
    virtual void mouseMove(Base::Vector2d onSketchPos)
    {
        if (method == PERIAPSIS_APOAPSIS_B) {
            if (mode == STATUS_SEEK_PERIAPSIS) {
                setPositionText(onSketchPos);
                if (seekAutoConstraint(sugConstr1, onSketchPos, Base::Vector2d(0.f,0.f),
                    AutoConstraint::CURVE)) {
                    renderSuggestConstraintsCursor(sugConstr1);
                    return;
                }
                if (sketchgui->toolSettings->widget->isSettingSet[0] + sketchgui->toolSettings->widget->isSettingSet[1] == 2) {
                    pressButton(onSketchPos);
                    releaseButton(onSketchPos);
                }
            } 
            else if (mode == STATUS_SEEK_APOAPSIS) {
                double dx, dy;
                if (sketchgui->toolSettings->widget->isSettingSet[2] == 1) {
                    double dx1 = onSketchPos.x - periapsis.x;
                    double dy1 = onSketchPos.y - periapsis.y;

                    dx = sketchgui->toolSettings->widget->toolParameters[2] * 2 * dx1 / sqrt(dx1 * dx1 + dy1 * dy1);
                    dy = sketchgui->toolSettings->widget->toolParameters[2] * 2 * dy1 / sqrt(dx1 * dx1 + dy1 * dy1);
                }
                else {
                    dx = onSketchPos.x - periapsis.x;
                    dy = onSketchPos.y - periapsis.y;
                }
                double length = sqrt(dx * dx + dy * dy);

                if (sketchgui->toolSettings->widget->isSettingSet[3] == 1) {
                    dx = cos(sketchgui->toolSettings->widget->toolParameters[3] * M_PI / 180) * length;
                    dy = sin(sketchgui->toolSettings->widget->toolParameters[3] * M_PI / 180) * length;
                }
                apoapsis.x = periapsis.x + dx;
                apoapsis.y = periapsis.y + dy;

                solveEllipse(apoapsis);
                approximateEllipse();

                // Display radius for user
                float semiMajorRadius = a * 2;
                SbString text;
                text.sprintf(" (%.1fR,%.1fR)", semiMajorRadius,semiMajorRadius);
                setPositionText(onSketchPos, text);

                drawEdit(editCurve);
                // Suggestions for ellipse and curves are disabled because many tangent constraints
                // need an intermediate point or line.
                if (seekAutoConstraint(sugConstr2, onSketchPos, Base::Vector2d(0.f,0.f),
                    AutoConstraint::CURVE)) {
                    renderSuggestConstraintsCursor(sugConstr2);
                    return;
                }
                if (sketchgui->toolSettings->widget->isSettingSet[2] == 1 && sketchgui->toolSettings->widget->isSettingSet[3] == 1) {
                    pressButton(onSketchPos);
                    releaseButton(onSketchPos);
                }
            } 
            else if (mode == STATUS_SEEK_B) {
                Base::Vector2d pointAtRadius = onSketchPos;
                if (sketchgui->toolSettings->widget->isSettingSet[4] == 1) {
                    //get a point at distance SettingSet[4]
                    double theta = atan((periapsis.x - apoapsis.x) / (periapsis.y - apoapsis.y));
                    pointAtRadius.x = apoapsis.x - sketchgui->toolSettings->widget->toolParameters[4] * cos(theta);
                    pointAtRadius.y = apoapsis.y + sketchgui->toolSettings->widget->toolParameters[4] * sin(theta);
                }
                solveEllipse(pointAtRadius);
                approximateEllipse();

                // Display radius for user
                SbString text;
                text.sprintf(" (%.1fR,%.1fR)", a, b);
                setPositionText(onSketchPos, text);

                drawEdit(editCurve);
                if (seekAutoConstraint(sugConstr3, onSketchPos, Base::Vector2d(0.f,0.f),
                    AutoConstraint::CURVE)) {
                    renderSuggestConstraintsCursor(sugConstr3);
                    return;
                }
                if (sketchgui->toolSettings->widget->isSettingSet[4] == 1) {
                    pressButton(onSketchPos);
                    releaseButton(onSketchPos);
                }
            }
        } 
        else { // method is CENTER_PERIAPSIS_B
            if (mode == STATUS_SEEK_CENTROID) {
                setPositionText(onSketchPos);
                if (seekAutoConstraint(sugConstr1, onSketchPos, Base::Vector2d(0.f,0.f))) { // TODO: ellipse prio 1
                    renderSuggestConstraintsCursor(sugConstr1);
                    return;
                }
                if (sketchgui->toolSettings->widget->isSettingSet[0] + sketchgui->toolSettings->widget->isSettingSet[1] == 2) {
                    pressButton(onSketchPos);
                    releaseButton(onSketchPos);
                }
            } 
            else if (mode == STATUS_SEEK_PERIAPSIS) {
                double dx, dy;
                if (sketchgui->toolSettings->widget->isSettingSet[2] == 1) {
                    double dx1 = onSketchPos.x - centroid.x;
                    double dy1 = onSketchPos.y - centroid.y;

                    dx = sketchgui->toolSettings->widget->toolParameters[2] * dx1 / sqrt(dx1 * dx1 + dy1 * dy1);
                    dy = sketchgui->toolSettings->widget->toolParameters[2] * dy1 / sqrt(dx1 * dx1 + dy1 * dy1);
                }
                else {
                    dx = onSketchPos.x - centroid.x;
                    dy = onSketchPos.y - centroid.y;
                }
                length1 = sqrt(dx * dx + dy * dy);

                if (sketchgui->toolSettings->widget->isSettingSet[3] == 1) {
                    dx = cos(-sketchgui->toolSettings->widget->toolParameters[3] * M_PI / 180) * length1;
                    dy = sin(-sketchgui->toolSettings->widget->toolParameters[3] * M_PI / 180) * length1;
                }
                periapsis.x = centroid.x + dx;
                periapsis.y = centroid.y + dy;

                solveEllipse(periapsis);
                approximateEllipse();

                // Display radius for user
                float semiMajorRadius = a * 2;
                SbString text;
                text.sprintf(" (%.1fR,%.1fR)", semiMajorRadius,semiMajorRadius);
                setPositionText(onSketchPos, text);

                drawEdit(editCurve);
                if (seekAutoConstraint(sugConstr2, onSketchPos, onSketchPos - centroid,
                    AutoConstraint::CURVE)) {
                    renderSuggestConstraintsCursor(sugConstr2);
                    return;
                }
                if (sketchgui->toolSettings->widget->isSettingSet[2] == 1 && sketchgui->toolSettings->widget->isSettingSet[3] == 1) {
                    pressButton(onSketchPos);
                    releaseButton(onSketchPos);
                }
            } 
            else if ((mode == STATUS_SEEK_A) || (mode == STATUS_SEEK_B)) {
                Base::Vector2d pointAtRadius = onSketchPos;
                if (sketchgui->toolSettings->widget->isSettingSet[4] == 1) {
                    //get a point at distance SettingSet[4]
                    double theta = atan((periapsis.x - centroid.x) / (periapsis.y - centroid.y));
                    pointAtRadius.x = centroid.x - sketchgui->toolSettings->widget->toolParameters[4] * cos(theta);
                    pointAtRadius.y = centroid.y + sketchgui->toolSettings->widget->toolParameters[4] * sin(theta);
                }

                //find length2
                Base::Vector2d cursor = Base::Vector2d(pointAtRadius - centroid);
                Base::Vector2d w_1 = cursor;
                w_1.ProjectToLine(cursor, (fixedAxis - centroid)); // projection of cursor line onto fixed axis line
                Base::Vector2d w_2 = (cursor - w_1);
                length2 = w_2.Length();

                solveEllipse(pointAtRadius);
                approximateEllipse();

                // Display radius for user
                SbString text;
                text.sprintf(" (%.1fR,%.1fR)", a, b);
                setPositionText(onSketchPos, text);

                drawEdit(editCurve);
                if (seekAutoConstraint(sugConstr3, onSketchPos, onSketchPos - centroid,
                    AutoConstraint::CURVE)) {
                    renderSuggestConstraintsCursor(sugConstr3);
                    return;
                }
                if (sketchgui->toolSettings->widget->isSettingSet[4] == 1) {
                    pressButton(onSketchPos);
                    releaseButton(onSketchPos);
                }
            }
        }
        applyCursor();
    }

    /**
     * @brief Changes drawing mode on user-click
     * @param onSketchPos the position of the cursor on the sketch
     * @return
     */
    virtual bool pressButton(Base::Vector2d onSketchPos)
    {
        if (method == PERIAPSIS_APOAPSIS_B) {
            if (mode == STATUS_SEEK_PERIAPSIS) {
                periapsis = onSketchPos;
                if (sketchgui->toolSettings->widget->isSettingSet[0] == 1) {
                    periapsis.x = sketchgui->toolSettings->widget->toolParameters[0];
                }
                if (sketchgui->toolSettings->widget->isSettingSet[1] == 1) {
                    periapsis.y = sketchgui->toolSettings->widget->toolParameters[1];
                }

                sketchgui->toolSettings->widget->setParameterActive(0, 0);
                sketchgui->toolSettings->widget->setParameterActive(0, 1);
                sketchgui->toolSettings->widget->setParameterActive(1, 2);
                sketchgui->toolSettings->widget->setParameterActive(1, 3);
                sketchgui->toolSettings->widget->setParameterFocus(2);
                mode = STATUS_SEEK_APOAPSIS;
            }
            else if (mode == STATUS_SEEK_APOAPSIS) {
                double dx, dy;
                if (sketchgui->toolSettings->widget->isSettingSet[2] == 1) {
                    double dx1 = onSketchPos.x - periapsis.x;
                    double dy1 = onSketchPos.y - periapsis.y;

                    dx = sketchgui->toolSettings->widget->toolParameters[2] * 2 * dx1 / sqrt(dx1 * dx1 + dy1 * dy1);
                    dy = sketchgui->toolSettings->widget->toolParameters[2] * 2 * dy1 / sqrt(dx1 * dx1 + dy1 * dy1);
                }
                else {
                    dx = onSketchPos.x - periapsis.x;
                    dy = onSketchPos.y - periapsis.y;
                }
                double length = sqrt(dx * dx + dy * dy);

                if (sketchgui->toolSettings->widget->isSettingSet[3] == 1) {
                    dx = cos(-sketchgui->toolSettings->widget->toolParameters[3] * M_PI / 180) * length;
                    dy = sin(-sketchgui->toolSettings->widget->toolParameters[3] * M_PI / 180) * length;
                }
                apoapsis.x = periapsis.x + dx;
                apoapsis.y = periapsis.y + dy;

                sketchgui->toolSettings->widget->setParameterActive(0, 2);
                sketchgui->toolSettings->widget->setParameterActive(0, 3);
                sketchgui->toolSettings->widget->setParameterActive(1, 4);
                sketchgui->toolSettings->widget->setParameterFocus(4);
                mode = STATUS_SEEK_B;
            }
            else {
                mode = STATUS_Close;
            }
        } 
        else { // method is CENTER_PERIAPSIS_B
            if (mode == STATUS_SEEK_CENTROID) {
                centroid = onSketchPos;
                if (sketchgui->toolSettings->widget->isSettingSet[0] == 1) {
                    centroid.x = sketchgui->toolSettings->widget->toolParameters[0];
                }
                if (sketchgui->toolSettings->widget->isSettingSet[1] == 1) {
                    centroid.y = sketchgui->toolSettings->widget->toolParameters[1];
                }

                sketchgui->toolSettings->widget->setParameterActive(0, 0);
                sketchgui->toolSettings->widget->setParameterActive(0, 1);
                sketchgui->toolSettings->widget->setParameterActive(1, 2);
                sketchgui->toolSettings->widget->setParameterActive(1, 3);
                sketchgui->toolSettings->widget->setParameterFocus(2);

                mode = STATUS_SEEK_PERIAPSIS;
            }
            else if (mode == STATUS_SEEK_PERIAPSIS) {
                double dx, dy;
                if (sketchgui->toolSettings->widget->isSettingSet[2] == 1) {
                    double dx1 = onSketchPos.x - centroid.x;
                    double dy1 = onSketchPos.y - centroid.y;

                    dx = sketchgui->toolSettings->widget->toolParameters[2] * dx1 / sqrt(dx1 * dx1 + dy1 * dy1);
                    dy = sketchgui->toolSettings->widget->toolParameters[2] * dy1 / sqrt(dx1 * dx1 + dy1 * dy1);
                }
                else {
                    dx = onSketchPos.x - centroid.x;
                    dy = onSketchPos.y - centroid.y;
                }
                double length = sqrt(dx * dx + dy * dy);
                if (sketchgui->toolSettings->widget->isSettingSet[3] == 1) {
                    dx = cos(-sketchgui->toolSettings->widget->toolParameters[3] * M_PI / 180) * length;
                    dy = sin(-sketchgui->toolSettings->widget->toolParameters[3] * M_PI / 180) * length;
                }
                periapsis.x = centroid.x + dx;
                periapsis.y = centroid.y + dy;

                sketchgui->toolSettings->widget->setParameterActive(0, 2);
                sketchgui->toolSettings->widget->setParameterActive(0, 3);
                sketchgui->toolSettings->widget->setParameterActive(1, 4);
                sketchgui->toolSettings->widget->setParameterFocus(4);
                mode = STATUS_SEEK_B;
            }
            else {
                mode = STATUS_Close;
            }
        }
        return true;
    }

    /**
     * @brief Calls \c saveEllipse() after last user input
     * @param onSketchPos the position of the cursor on the sketch
     * @return
     */
    virtual bool releaseButton(Base::Vector2d onSketchPos)
    {
        Q_UNUSED(onSketchPos);
        if (mode == STATUS_Close) {
            saveEllipse();

            ParameterGrp::handle hGrp = App::GetApplication().GetParameterGroupByPath("User parameter:BaseApp/Preferences/Mod/Sketcher");
            bool continuousMode = hGrp->GetBool("ContinuousCreationMode",true);

            if(continuousMode){
                sketchgui->toolSettings->widget->setSettings(7);
                if (constrMethod == 0) {
                    method = CENTER_PERIAPSIS_B;
                    mode = STATUS_SEEK_CENTROID;
                    sketchgui->toolSettings->widget->setLabel(QApplication::translate("ConstrainContextually",
                        "x of Center Point"), 0);
                    sketchgui->toolSettings->widget->setLabel(QApplication::translate("ConstrainContextually",
                        "y of Center Point"), 0);
                } else {
                    method = PERIAPSIS_APOAPSIS_B;
                    mode = STATUS_SEEK_PERIAPSIS;
                }
            }
        }
        return true;
    }
protected:
    std::vector<AutoConstraint> sugConstr1, sugConstr2, sugConstr3;
private:
    SelectMode mode;
    /// the method of constructing the ellipse
    ConstructionMethod method;
    int constrMethod;
    /// periapsis position vector, in standard position in sketch coordinate system
    Base::Vector2d periapsis;
    /// apoapsis position vector, in standard position in sketch coordinate system
    Base::Vector2d apoapsis;
    /// centroid position vector, in standard position in sketch coordinate system
    Base::Vector2d centroid;
    /**
     * @brief position vector of positive b point, in standard position in sketch coordinate system
     * I.E. in polar perifocal system, the first intersection of the semiminor axis with the ellipse
     * as theta increases from 0. This always happens when:
     *    \f{eqnarray*}{
     *        \theta_b &=& \left[\pi - \arctan\left(\frac{b}{ae}\right)\right]  \pm N 2\pi
     *   \f}
     *
     * In a rotated R^3 cartesian system, centered at the centroid, +X towards periapsis, and
     * +Z coming out of the sketch, this b position is in the +Y direction from the centroid.
     */
    Base::Vector2d positiveB;
    /// the other b position
    Base::Vector2d negativeB;
    /// cart. position vector for primary focus
    Base::Vector2d f;
    /// cart. position vector for other focus
    Base::Vector2d fPrime;
    /// Unit vector for apse line
    Base::Vector2d apseHat;
    /// first radius entered/selected
    double length1;
    /// second radius entered/selected
    double length2;
    /// length of semimajor axis, i.e. 'radius' colloquially
    double a;
    /// length of semiminor axis, i.e. 'radius' colloquially
    double b;
    /// eccentricity [unitless]
    double e;
    /// optimization, holds a term that helps calculate b in terms of a and e
    double ratio;
    /// holds product of a * e
    double ae;
    /// holds numerator of orbit equation of form a(1-e^2)
    double num;
    /// holds a radial distance from f to the ellipse for a given theta
    double r;
    /// angle of a point in a perifocal frame centered at f
    double theta;
    /// angle of apse line relative to sketch coordinate system
    double phi;
    /// holds a position vector for a point on the ellipse from f
    Base::Vector2d pos;
    /// holds a position vector for a point on the ellipse from fPrime
    Base::Vector2d posPrime;
    /// holds position vectors for a points on the ellipse
    std::vector<Base::Vector2d> editCurve;
    /// local i_hat vector for ellipse, from centroid to periapsis
    Base::Vector3d iPrime;
    /// local j_hat vector for ellipse, from centroid to b point
    Base::Vector3d jPrime;
    /// length (radius) of the fixed axis
    double fixedAxisLength;
    /// position vector of fixed axis point in sketch coordinates
    Base::Vector2d fixedAxis;

    /**
     * @brief Computes a vector of 2D points representing an ellipse
     * @param onSketchPos Current position of the cursor on the sketch
     */
    void solveEllipse(Base::Vector2d onSketchPos)
    {
        const double GOLDEN_RATIO = 1.6180339887;
        Base::Vector3d k(0,0,1);

        if (method == PERIAPSIS_APOAPSIS_B) {
            if (mode == STATUS_SEEK_APOAPSIS) {
                apoapsis = onSketchPos;
            }
            a = (apoapsis - periapsis).Length() / 2;
            apseHat = (periapsis - apoapsis);
            apseHat.Normalize();
            centroid = apseHat;
            centroid.Scale(-1 * a);
            centroid = periapsis + centroid;
            if (mode == STATUS_SEEK_APOAPSIS) {
                // for first step, we draw an ellipse inscribed in a golden rectangle
                ratio = 1 / GOLDEN_RATIO;   // ~= 0.6180339887
                e = sqrt(ratio);            // ~= 0.7861513777
                b = a * ratio;
            }
            else if (mode == STATUS_SEEK_B) {
                // Get the closest distance from onSketchPos to apse line, as a 'requested' value for b
                Base::Vector2d cursor = Base::Vector2d(onSketchPos - f); // vector from f to cursor pos
                // decompose cursor with a projection, then length of w_2 will give us b
                Base::Vector2d w_1 = cursor;
                w_1.ProjectToLine(cursor, (periapsis - apoapsis)); // projection of cursor line onto apse line
                Base::Vector2d w_2 = (cursor - w_1);
                b = w_2.Length();

                // limit us to ellipse or circles
                if (b > a) {
                    b = a;
                }

                e = sqrt(1 - ((b * b) / (a * a)));
                ratio = sqrt(1 - (e*e));
            }
            ae = a * e;
            f = apseHat;
            f.Scale(ae);
            f = centroid + f;
            fPrime = apseHat;
            fPrime.Scale(-1 * ae);
            fPrime = centroid + fPrime;
            phi = atan2(apseHat.y, apseHat.x);
            num = a * (1 - (e * e));
            // The ellipse is now solved
        } else { // method == CENTER_PERIAPSIS_B
            if (mode == STATUS_SEEK_PERIAPSIS) {
                // solve the ellipse inscribed in a golden rectangle
                periapsis = onSketchPos;
                a = (centroid - periapsis).Length();
                iPrime.x = periapsis.x - centroid.x;
                iPrime.y = periapsis.y - centroid.y;
                iPrime.z = 0;
                jPrime = k % iPrime;   // j = k cross i

                // these are constant for any ellipse inscribed in a golden rectangle
                ratio = 1 / GOLDEN_RATIO;   // ~= 0.6180339887
                e = sqrt(ratio);            // ~= 0.7861513777

                b = a * ratio;
                ae = a * e;
                apseHat = (periapsis - centroid);
                apseHat.Normalize();
                f = apseHat;
                f.Scale(ae);
                f = centroid + f;
                fPrime = apseHat;
                fPrime.Scale(-1 * ae);
                fPrime = centroid + fPrime;
                apoapsis = apseHat;
                apoapsis.Scale(-1 * a);
                apoapsis = centroid + apoapsis;
                phi = atan2(apseHat.y, apseHat.x);
                num = a * (1 - (e * e));
                fixedAxisLength = a;
                fixedAxis = periapsis;
            } else if ((mode == STATUS_SEEK_B) || (mode == STATUS_SEEK_A)) {
                // while looking for the last click, we may switch back and forth
                // between looking for a b point and looking for periapsis, so ensure
                // we are in the right mode
                Base::Vector2d cursor = Base::Vector2d(onSketchPos - centroid); // vector from centroid to cursor pos
                // decompose cursor with a projection, then length of w_2 will give us b
                Base::Vector2d w_1 = cursor;
                w_1.ProjectToLine(cursor, (fixedAxis - centroid)); // projection of cursor line onto fixed axis line
                Base::Vector2d w_2 = (cursor - w_1);
                if (w_2.Length() > fixedAxisLength) {
                    // b is fixed, we are seeking a
                    mode = STATUS_SEEK_A;
                    jPrime.x = (fixedAxis - centroid).x;
                    jPrime.y = (fixedAxis - centroid).y;
                    jPrime.Normalize();
                    iPrime = jPrime % k;    // cross
                    b = fixedAxisLength;
                    a = w_2.Length();
                } else {
                    // a is fixed, we are seeking b
                    mode = STATUS_SEEK_B;
                    iPrime.x = (fixedAxis - centroid).x;
                    iPrime.y = (fixedAxis - centroid).y;
                    iPrime.Normalize();
                    jPrime = k % iPrime;    // cross
                    a = fixedAxisLength;
                    b = w_2.Length();
                }
                // now finish solving the ellipse
                periapsis.x = centroid.x + (iPrime * a).x;
                periapsis.y = centroid.y + (iPrime * a).y;
                e = sqrt(1 - ((b * b) / (a * a)));
                ratio = sqrt(1 - (e*e));
                ae = a * e;
                apseHat = (periapsis - centroid);
                apseHat.Normalize();
                f = apseHat;
                f.Scale(ae);
                f = centroid + f;
                fPrime = apseHat;
                fPrime.Scale(-1 * ae);
                fPrime = centroid + fPrime;
                apoapsis = apseHat;
                apoapsis.Scale(-1 * a);
                apoapsis = centroid + apoapsis;
                phi = atan2(apseHat.y, apseHat.x);
                num = a * (1 - (e * e));
            }
        }
    }


    /**
     * @brief Computes a sequence of 2D vectors to approximate the ellipse
     */
    void approximateEllipse()
    {
        // We will approximate the ellipse as a sequence of connected chords
        // Number of points per quadrant of the ellipse
        int n = static_cast<int>((editCurve.size() - 1) / 4);

        // We choose points in the perifocal frame then translate them to sketch cartesian.
        // This gives us a better approximation of an ellipse, i.e. more points where the
        // curvature is higher.  If the eccentricity is high, we shift the points a bit towards
        // the semi-minor axis.
        double partitionAngle = (M_PI - atan2(b, ae)) / n;
        double radianShift = 0;
        if (e > 0.8) {radianShift = (partitionAngle / 5) * 4;}
        for (int i=0; i < n; i++) {
            theta = i * partitionAngle;
            if (i > 0) {theta = theta + radianShift;}
            r = num / (1 + (e * cos(theta)));
            // r(pi/2) is semi-latus rectum, if we need it
            pos.x = r*cos(theta+phi);  // phi rotates, sin/cos translate
            pos.y = r*sin(theta+phi);
            pos = pos + f;
            posPrime.x = r*cos(theta+phi+M_PI);
            posPrime.y = r*sin(theta+phi+M_PI);
            posPrime = posPrime + fPrime;
            // over the loop, loads Quadrant I points, by using f as origin
            editCurve[i] = pos;
            // over the loop, loads Quadrant III points, by using fPrime as origin
            editCurve[(2*n) + i] = posPrime;
            // load points with negative theta angles (i.e. cw)
            if (i>0) {
                pos.x = r*cos(-1*theta+phi);
                pos.y = r*sin(-1*theta+phi);
                pos = pos + f;
                // loads Quadrant IV points
                editCurve[(4*n) - i] = pos;
                posPrime.x = r*cos(-1*theta+phi+M_PI);
                posPrime.y = r*sin(-1*theta+phi+M_PI);
                posPrime = posPrime + fPrime;
                // loads Quadrant II points
                editCurve[(2*n) - i] = posPrime;
            }
        }
        // load pos & neg b points
        theta = M_PI - atan2(b, ae);        // the angle from f to the positive b point
        r = num / (1 + (e * cos(theta)));
        pos.x = r*cos(theta+phi);
        pos.y = r*sin(theta+phi);
        pos = pos + f;
        editCurve[n] = pos; // positive
        pos.x = r*cos(-1*theta+phi);
        pos.y = r*sin(-1*theta+phi);
        pos = pos + f;
        editCurve[(3*n)] = pos; // negative
        // force the curve to be a closed shape
        editCurve[(4*n)] = editCurve[0];
    }

    /**
     * @brief Prints the ellipse data to STDOUT as an GNU Octave script
     * @param onSketchPos position of the cursor on the sketch
     */
    void ellipseToOctave(Base::Vector2d /*onSketchPos*/)
    {
        int n = static_cast<int>((editCurve.size() - 1) / 4);

        // send a GNU Octave script to stdout to plot points for debugging
        std::ostringstream octave;
        octave << std::fixed << std::setprecision(12);
        octave << "\nclear all;\nclose all;\nclc;\n\n";
        octave << "periapsis = [" << periapsis.x << ", " << periapsis.y << "];\n";
        octave << "apoapsis = [" << apoapsis.x << ", " << apoapsis.y << "];\n";
        octave << "positiveB = [" << editCurve[n].x << ", " << editCurve[n].y << "];\n";
        octave << "apseHat = [" << apseHat.x << ", " << apseHat.y << "];\n";
        octave << "a = " << a << ";\n";
        octave << "b = " << b << ";\n";
        octave << "eccentricity = " << e << ";\n";
        octave << "centroid = [" << centroid.x << ", " << centroid.y << "];\n";
        octave << "f = [" << f.x << ", " << f.y << "];\n";
        octave << "fPrime = [" << fPrime.x << ", " << fPrime.y << "];\n";
        octave << "phi = " << phi << ";\n\n";
        octave << "x = [";
        for (int i=0; i < 4*n + 1; i++) {
            octave << editCurve[i].x;
            if (i < 4*n) {
                octave << ", ";
            }
        }
        octave << "];\n";
        octave << "y = [";
        for (int i=0; i < 4*n + 1; i++) {
            octave << editCurve[i].y;
            if (i < 4*n) {
                octave << ", ";
            }
        }
        octave << "];\n\n";
        octave << "% Draw ellipse points in red;\n";
        octave << "plot (x, y, \"r.\", \"markersize\", 5);\n";
        octave << "axis ([-300, 300, -300, 300], \"square\");grid on;\n";
        octave << "hold on;\n\n";
        octave << "% Draw centroid in blue, f in cyan, and fPrime in magenta;\n";
        octave << "plot(centroid(1), centroid(2), \"b.\", \"markersize\", 5);\n";
        octave << "plot(f(1), f(2), \"c.\", \"markersize\", 5);\n";
        octave << "plot(fPrime(1), fPrime(2), \"m.\", \"markersize\", 5);\n";
        octave << "n = [periapsis(1) - f(1), periapsis(2) - f(2)];\n";
        octave << "h = quiver(f(1),f(2),n(1),n(2), 0);\n";
        octave << "set (h, \"maxheadsize\", 0.1);\n\n";
        octave << "% Draw the three position vectors used for Gui::Command::doCommand(...)\n";
        octave << "periapsisVec = quiver(0,0,periapsis(1),periapsis(2), 0);\n";
        octave << "set (periapsisVec, \"maxheadsize\", 0.01, \"color\", \"black\");\n";
        octave << "centroidVec = quiver(0,0,centroid(1),centroid(2), 0);\n";
        octave << "set (centroidVec, \"maxheadsize\", 0.01, \"color\", \"black\");\n";
        octave << "bVec = quiver(0,0,positiveB(1),positiveB(2), 0);\n";
        octave << "set (bVec, \"maxheadsize\", 0.01, \"color\", \"black\");\n\n";
        octave << "% Draw the local x & y basis vectors, scaled to a and b, in red and blue, respectively\n";
        octave << "xLocalVec = quiver(centroid(1),centroid(2),periapsis(1)-centroid(1),periapsis(2)-centroid(2), 0);\n";
        octave << "set (xLocalVec, \"maxheadsize\", 0.01, \"color\", \"red\");\n";
        octave << "yLocalVec = quiver(centroid(1),centroid(2), positiveB(1)-centroid(1), positiveB(2)-centroid(2), 0);\n";
        octave << "set (yLocalVec, \"maxheadsize\", 0.01, \"color\", \"blue\");\nhold off;\n";
        qDebug() << QString::fromStdString(octave.str());
    }

    /**
     * @brief Finalizes and saves the drawn ellipse
     * @return nothing
     */
    void saveEllipse()
    {
        unsetCursor();
        resetPositionText();

        /* There are a couple of issues with Gui::Command::doCommand(...) and
         * GC_MakeEllipse(...) that cause bugs if not handled properly, even
         * when we give them a mathematically-correct ellipse.
         *
         * GC_MakeEllipse may fail with a gce_InvertAxis error for a small
         * circular ellipse when floating point roundoff or representation
         * errors make the b axis slightly larger than the a axis.
         *
         * A similar, larger, issue arises in Gui::Command::doCommand(...) because
         * we cast our double vector components into strings with a fixed
         * precision of six, and then create new doubles from the strings
         * in EllipsePy::PyInit(...).  Thus, by the time we call GC_MakeEllipse(...)
         * in EllipsePy::PyInit(...), our ellipse may not be valid anymore
         * because b is now greater than a.
         *
         * To handle these issues, we simulate the effects Gui::Command::doCommand(...)
         * has on our ellipse, and we adjust our ellipse parameters until
         * GC_MakeEllipse successfully creates an ellipse with our mangled
         * parameters.
         *
         * In almost all cases, we only have to make our test ellipse one time;
         * it is only in the rare edge cases that require repeated test ellipses
         * until we get a valid one, or fail due to excessive attempts. With a
         * limit of 25 attempts, I have been unable to make it fail.
         */

        // simulate loss of precision in centroid, periapsis, and apoapsis
        char cx[64];
        char cy[64];
        char px[64];
        char py[64];
        char ax[64];
        char ay[64];
        sprintf(cx, "%.6lf\n", centroid.x);
        sprintf(cy, "%.6lf\n", centroid.y);
        sprintf(px, "%.6lf\n", periapsis.x);
        sprintf(py, "%.6lf\n", periapsis.y);
        sprintf(ax, "%.6lf\n", apoapsis.x);
        sprintf(ay, "%.6lf\n", apoapsis.y);
        centroid.x = atof(cx);
        centroid.y = atof(cy);
        periapsis.x = atof(px);
        periapsis.y = atof(py);
        apoapsis.x = atof(ax);
        apoapsis.y = atof(ay);
        double majorLength = (periapsis - apoapsis).Length();
        double minorLength = 0;

        /* GC_MakeEllipse requires a right-handed coordinate system, with +X
         * from centroid to periapsis, +Z out of the page.
         */
        Base::Vector3d k(0,0,1);
        Base::Vector3d i(periapsis.x - centroid.x, periapsis.y - centroid.y, 0);
        Base::Vector3d j = k % i;   // j = k cross i
        double beta = 1e-7;
        int count = 0;
        int limit = 25;             // no infinite loops!
        bool success = false;
        double tempB = b;

        // adjust b until our mangled vectors produce a good ellipse in GC_MakeEllipse
        // and the mangled major and minor lines in LinePy::PyInit(...) are such that
        // major is at least slightly larger than minor
        do {
            tempB = b - double(count * beta);
            j = j.Normalize() * tempB;
            positiveB.x = centroid.x + j.x;
            positiveB.y = centroid.y + j.y;
            negativeB.x = centroid.x + (j.x * -1);
            negativeB.y = centroid.y + (j.y * -1);
            char bpx[64];
            char bpy[64];
            char bnx[64];
            char bny[64];
            sprintf(bpx, "%.6lf\n", positiveB.x);
            sprintf(bpy, "%.6lf\n", positiveB.y);
            sprintf(bnx, "%.6lf\n", negativeB.x);
            sprintf(bny, "%.6lf\n", negativeB.y);
            positiveB.x = atof(bpx);
            positiveB.y = atof(bpy);
            negativeB.x = atof(bnx);
            negativeB.y = atof(bny);
            GC_MakeEllipse me(gp_Pnt(periapsis.x,periapsis.y,0),
                              gp_Pnt(positiveB.x,positiveB.y,0),
                              gp_Pnt(centroid.x,centroid.y,0));
            minorLength = (negativeB - positiveB).Length();
            count++;
            success = me.IsDone() && (minorLength + beta < majorLength);
        } while (!success && (count <= limit));
        if (!success) {
            qDebug() << "Failed to create a valid mangled ellipse after" << count << "attempts";
        }

        // save any changes to b, then recalculate ellipse as required due to change in b
        b = tempB;
        e = sqrt(1 - ((b * b) / (a * a)));
        ae = a * e;
        f = apseHat;
        f.Scale(ae);
        f = centroid + f;
        fPrime = apseHat;
        fPrime.Scale(-1 * ae);
        fPrime = centroid + fPrime;

        int currentgeoid = getHighestCurveIndex(); // index of the ellipse we just created

        try {
            Gui::Command::openCommand(QT_TRANSLATE_NOOP("Command", "Add sketch ellipse"));
            Gui::cmdAppObjectArgs(sketchgui->getObject(), "addGeometry(Part.Ellipse"
                                    "(App.Vector(%f,%f,0),App.Vector(%f,%f,0),App.Vector(%f,%f,0)),%s)",
                                    periapsis.x, periapsis.y,
                                    positiveB.x, positiveB.y,
                                    centroid.x, centroid.y,
                                    geometryCreationMode==Construction?"True":"False");

            currentgeoid++;

            Gui::cmdAppObjectArgs(sketchgui->getObject(), "exposeInternalGeometry(%d)", currentgeoid);

            int firstAxis = currentgeoid + 1;
            int secondAxis = currentgeoid + 2;

            //add constraint if user typed in some dimensions in tool widget
            if (method == CENTER_PERIAPSIS_B) {
                if (length1 < length2) {
                    firstAxis = currentgeoid + 2;
                    secondAxis = currentgeoid + 1;
                }
                if (sketchgui->toolSettings->widget->isSettingSet[0] + sketchgui->toolSettings->widget->isSettingSet[1] + sketchgui->toolSettings->widget->isSettingSet[2] + sketchgui->toolSettings->widget->isSettingSet[3] + sketchgui->toolSettings->widget->isSettingSet[4] != 0) {
                    if (sketchgui->toolSettings->widget->isSettingSet[0] == 1) {
                        distanceXYorPointOnObject(0, currentgeoid, Sketcher::PointPos::mid, sketchgui->toolSettings->widget->toolParameters[0]);
                    }
                    if (sketchgui->toolSettings->widget->isSettingSet[1] == 1) {
                        distanceXYorPointOnObject(1, currentgeoid, Sketcher::PointPos::mid, sketchgui->toolSettings->widget->toolParameters[1]);
                    }
                    if (sketchgui->toolSettings->widget->isSettingSet[2] == 1) {
                        Gui::cmdAppObjectArgs(sketchgui->getObject(), "addConstraint(Sketcher.Constraint('Distance',%d,%d,%d,%d,%f)) ",
                            currentgeoid, 3, firstAxis, 1, sketchgui->toolSettings->widget->toolParameters[2]);
                    }
                    if (sketchgui->toolSettings->widget->isSettingSet[3] == 1) {
                        if (sketchgui->toolSettings->widget->toolParameters[3] == 0) {
                            Gui::cmdAppObjectArgs(sketchgui->getObject(), "addConstraint(Sketcher.Constraint('Horizontal',%d)) ", firstAxis);
                        }
                        else if (sketchgui->toolSettings->widget->toolParameters[3] == 90) {
                            Gui::cmdAppObjectArgs(sketchgui->getObject(), "addConstraint(Sketcher.Constraint('Vertical',%d)) ", firstAxis);
                        }
                        else {
                            Gui::cmdAppObjectArgs(sketchgui->getObject(), "addConstraint(Sketcher.Constraint('Angle',%d,%d,%f)) ",
                                firstAxis, Sketcher::GeoEnum::HAxis, sketchgui->toolSettings->widget->toolParameters[3] * M_PI / 180);
                        }
                    }
                    if (sketchgui->toolSettings->widget->isSettingSet[4] == 1) {
                        Gui::cmdAppObjectArgs(sketchgui->getObject(), "addConstraint(Sketcher.Constraint('Distance',%d,%d,%d,%d,%f)) ",
                            currentgeoid, 3, secondAxis, 1, sketchgui->toolSettings->widget->toolParameters[4]);
                    }
                }
            }
            else if (method == PERIAPSIS_APOAPSIS_B) {
                if (sketchgui->toolSettings->widget->isSettingSet[0] + sketchgui->toolSettings->widget->isSettingSet[1] + sketchgui->toolSettings->widget->isSettingSet[2] + sketchgui->toolSettings->widget->isSettingSet[3] + sketchgui->toolSettings->widget->isSettingSet[4] != 0) {
                    if (sketchgui->toolSettings->widget->isSettingSet[0] == 1) {
                        distanceXYorPointOnObject(0, firstAxis, Sketcher::PointPos::start, sketchgui->toolSettings->widget->toolParameters[0]);
                    }
                    if (sketchgui->toolSettings->widget->isSettingSet[1] == 1) {
                        distanceXYorPointOnObject(1, firstAxis, Sketcher::PointPos::start, sketchgui->toolSettings->widget->toolParameters[1]);
                    }
                    if (sketchgui->toolSettings->widget->isSettingSet[2] == 1) {
                        Gui::cmdAppObjectArgs(sketchgui->getObject(), "addConstraint(Sketcher.Constraint('Distance',%d,%d,%d,%d,%f)) ",
                            currentgeoid, 3, firstAxis, 1, sketchgui->toolSettings->widget->toolParameters[2]);
                    }
                    if (sketchgui->toolSettings->widget->isSettingSet[3] == 1) {
                        if (sketchgui->toolSettings->widget->toolParameters[3] == 0) {
                            Gui::cmdAppObjectArgs(sketchgui->getObject(), "addConstraint(Sketcher.Constraint('Horizontal',%d)) ", firstAxis);
                        }
                        else if (sketchgui->toolSettings->widget->toolParameters[3] == 90) {
                            Gui::cmdAppObjectArgs(sketchgui->getObject(), "addConstraint(Sketcher.Constraint('Vertical',%d)) ", firstAxis);
                        }
                        else {
                            Gui::cmdAppObjectArgs(sketchgui->getObject(), "addConstraint(Sketcher.Constraint('Angle',%d,%d,%f)) ",
                                firstAxis, Sketcher::GeoEnum::HAxis, sketchgui->toolSettings->widget->toolParameters[3] * M_PI / 180);
                        }
                    }
                    if (sketchgui->toolSettings->widget->isSettingSet[4] == 1 && length1 > length2) {
                        //ellipse by 3 points do not support changing swapping and big radius so sedond radius cannot be bigger than first
                        Gui::cmdAppObjectArgs(sketchgui->getObject(), "addConstraint(Sketcher.Constraint('Distance',%d,%d,%d,%d,%f)) ",
                            currentgeoid, 3, secondAxis, 1, sketchgui->toolSettings->widget->toolParameters[4]);
                    }
                }
            }
        }
        catch (const Base::Exception& e) {
            Base::Console().Error("%s\n", e.what());
            Gui::Command::abortCommand();

            tryAutoRecomputeIfNotSolve(static_cast<Sketcher::SketchObject *>(sketchgui->getObject()));

            return;
        }

        Gui::Command::commitCommand();

        if (method == CENTER_PERIAPSIS_B) {
            // add auto constraints for the center point
            if (sugConstr1.size() > 0) {
                createAutoConstraints(sugConstr1, currentgeoid, Sketcher::PointPos::mid);
                sugConstr1.clear();
            }
            if (sugConstr2.size() > 0) {
                createAutoConstraints(sugConstr2, currentgeoid, Sketcher::PointPos::none);
                sugConstr2.clear();
            }
            if (sugConstr3.size() > 0) {
                createAutoConstraints(sugConstr3, currentgeoid, Sketcher::PointPos::none);
                sugConstr3.clear();
            }
        }

        if (method == PERIAPSIS_APOAPSIS_B) {
            if (sugConstr1.size() > 0) {
                createAutoConstraints(sugConstr1, currentgeoid, Sketcher::PointPos::none);
                sugConstr1.clear();
            }
            if (sugConstr2.size() > 0) {
                createAutoConstraints(sugConstr2, currentgeoid, Sketcher::PointPos::none);
                sugConstr2.clear();
            }
            if (sugConstr3.size() > 0) {
                createAutoConstraints(sugConstr3, currentgeoid, Sketcher::PointPos::none);
                sugConstr3.clear();
            }
        }

        tryAutoRecomputeIfNotSolve(static_cast<Sketcher::SketchObject *>(sketchgui->getObject()));

        // This code enables the continuous creation mode.
        if (constrMethod == 0) {
            method = CENTER_PERIAPSIS_B;
            mode = STATUS_SEEK_CENTROID;
        } else {
            method = PERIAPSIS_APOAPSIS_B;
            mode = STATUS_SEEK_PERIAPSIS;
        }
        editCurve.clear();
        drawEdit(editCurve);

        ParameterGrp::handle hGrp = App::GetApplication().GetParameterGroupByPath("User parameter:BaseApp/Preferences/Mod/Sketcher");
        bool continuousMode = hGrp->GetBool("ContinuousCreationMode",true);


        if(continuousMode){
            // This code enables the continuous creation mode.
            editCurve.resize(33);
            applyCursor();
            /* It is ok not to call to purgeHandler
            * in continuous creation mode because the
            * handler is destroyed by the quit() method on pressing the
            * right button of the mouse */
        }
        else{
            sketchgui->purgeHandler(); // no code after this line, Handler get deleted in ViewProvider
        }

    }
};

/// @brief Macro that declares a new sketcher command class 'CmdSketcherCreateEllipseByCenter'
DEF_STD_CMD_A(CmdSketcherCreateEllipseByCenter)

/**
 * @brief ctor
 */
CmdSketcherCreateEllipseByCenter::CmdSketcherCreateEllipseByCenter()
  : Command("Sketcher_CreateEllipseByCenter")
{
    sAppModule      = "Sketcher";
    sGroup          = "Sketcher";
    sMenuText       = QT_TR_NOOP("Create ellipse by center");
    sToolTipText    = QT_TR_NOOP("Create an ellipse by center in the sketch");
    sWhatsThis      = "Sketcher_CreateEllipseByCenter";
    sStatusTip      = sToolTipText;
    sPixmap         = "Sketcher_Conics_Ellipse_Center";
    sAccel          = "G, E, E";
    eType           = ForEdit;
}

void CmdSketcherCreateEllipseByCenter::activated(int iMsg)
{
    Q_UNUSED(iMsg);
    ActivateHandler(getActiveGuiDocument(),new DrawSketchHandlerEllipse(0) );
}

bool CmdSketcherCreateEllipseByCenter::isActive(void)
{
    return isCreateGeoActive(getActiveGuiDocument());
}

/// @brief Macro that declares a new sketcher command class 'CmdSketcherCreateEllipseBy3Points'
DEF_STD_CMD_A(CmdSketcherCreateEllipseBy3Points)

/**
 * @brief ctor
 */
CmdSketcherCreateEllipseBy3Points::CmdSketcherCreateEllipseBy3Points()
  : Command("Sketcher_CreateEllipseBy3Points")
{
    sAppModule      = "Sketcher";
    sGroup          = "Sketcher";
    sMenuText       = QT_TR_NOOP("Create ellipse by 3 points");
    sToolTipText    = QT_TR_NOOP("Create an ellipse by 3 points in the sketch");
    sWhatsThis      = "Sketcher_CreateEllipseBy3Points";
    sStatusTip      = sToolTipText;
    sPixmap         = "Sketcher_CreateEllipse_3points";
    sAccel          = "G, 3, E";
    eType           = ForEdit;
}

void CmdSketcherCreateEllipseBy3Points::activated(int iMsg)
{
    Q_UNUSED(iMsg);
    ActivateHandler(getActiveGuiDocument(),new DrawSketchHandlerEllipse(1) );
}

bool CmdSketcherCreateEllipseBy3Points::isActive(void)
{
    return isCreateGeoActive(getActiveGuiDocument());
}

class DrawSketchHandlerArcOfEllipse : public DrawSketchHandler
{
public:
    DrawSketchHandlerArcOfEllipse()
        : Mode(STATUS_SEEK_First), EditCurve(34)
        , rx(0), ry(0), startAngle(0), endAngle(0)
        , arcAngle(0), arcAngle_t(0)
    {
    }
    virtual ~DrawSketchHandlerArcOfEllipse(){}
    /// mode table
    enum SelectMode {
        STATUS_SEEK_First,      /**< enum value ----. */
        STATUS_SEEK_Second,     /**< enum value ----. */
        STATUS_SEEK_Third,      /**< enum value ----. */
        STATUS_SEEK_Fourth,     /**< enum value ----. */
        STATUS_Close
    };

    virtual void activated(ViewProviderSketch *)
    {
        setCrosshairCursor("Sketcher_Pointer_Create_ArcOfEllipse");
    }

    virtual void mouseMove(Base::Vector2d onSketchPos)
    {
        if (Mode==STATUS_SEEK_First) {
            setPositionText(onSketchPos);
            if (seekAutoConstraint(sugConstr1, onSketchPos, Base::Vector2d(0.f,0.f))) { // TODO: ellipse prio 1
                renderSuggestConstraintsCursor(sugConstr1);
                return;
            }
        }
        else if (Mode==STATUS_SEEK_Second) {
            double rx0 = onSketchPos.x - EditCurve[0].x;
            double ry0 = onSketchPos.y - EditCurve[0].y;
            for (int i=0; i < 16; i++) {
                double angle = i*M_PI/16.0;
                double rx1 = rx0 * cos(angle) + ry0 * sin(angle);
                double ry1 = -rx0 * sin(angle) + ry0 * cos(angle);
                EditCurve[1+i] = Base::Vector2d(EditCurve[0].x + rx1, EditCurve[0].y + ry1);
                EditCurve[17+i] = Base::Vector2d(EditCurve[0].x - rx1, EditCurve[0].y - ry1);
            }
            EditCurve[33] = EditCurve[1];

            // Display radius for user
            float radius = (onSketchPos - EditCurve[0]).Length();

            SbString text;
            text.sprintf(" (%.1fR,%.1fR)", radius,radius);
            setPositionText(onSketchPos, text);

            drawEdit(EditCurve);
            if (seekAutoConstraint(sugConstr2, onSketchPos, onSketchPos - centerPoint,
                                   AutoConstraint::CURVE)) {
                renderSuggestConstraintsCursor(sugConstr2);
                return;
            }
        }
        else if (Mode==STATUS_SEEK_Third) {
            // angle between the major axis of the ellipse and the X axis
            double a = (EditCurve[1]-EditCurve[0]).Length();
            double phi = atan2(EditCurve[1].y-EditCurve[0].y,EditCurve[1].x-EditCurve[0].x);

            // This is the angle at cursor point
            double angleatpoint = acos((onSketchPos.x-EditCurve[0].x+(onSketchPos.y-EditCurve[0].y)*tan(phi))/(a*(cos(phi)+tan(phi)*sin(phi))));
            double b=(onSketchPos.y-EditCurve[0].y-a*cos(angleatpoint)*sin(phi))/(sin(angleatpoint)*cos(phi));

            for (int i=1; i < 16; i++) {
                double angle = i*M_PI/16.0;
                double rx1 = a * cos(angle) * cos(phi) - b * sin(angle) * sin(phi);
                double ry1 = a * cos(angle) * sin(phi) + b * sin(angle) * cos(phi);
                EditCurve[1+i] = Base::Vector2d(EditCurve[0].x + rx1, EditCurve[0].y + ry1);
                EditCurve[17+i] = Base::Vector2d(EditCurve[0].x - rx1, EditCurve[0].y - ry1);
            }
            EditCurve[33] = EditCurve[1];
            EditCurve[17] = EditCurve[16];

            // Display radius for user
            SbString text;
            text.sprintf(" (%.1fR,%.1fR)", a, b);
            setPositionText(onSketchPos, text);

            drawEdit(EditCurve);
            if (seekAutoConstraint(sugConstr3, onSketchPos, Base::Vector2d(0.f,0.f))) {
                renderSuggestConstraintsCursor(sugConstr3);
                return;
            }
        }
        else if (Mode==STATUS_SEEK_Fourth) { // here we differ from ellipse creation
            // angle between the major axis of the ellipse and the X axis
            double a = (axisPoint-centerPoint).Length();
            double phi = atan2(axisPoint.y-centerPoint.y,axisPoint.x-centerPoint.x);

            // This is the angle at cursor point
            double angleatpoint = acos((startingPoint.x-centerPoint.x+(startingPoint.y-centerPoint.y)*tan(phi))/(a*(cos(phi)+tan(phi)*sin(phi))));
            double b=abs((startingPoint.y-centerPoint.y-a*cos(angleatpoint)*sin(phi))/(sin(angleatpoint)*cos(phi)));

            double rxs = startingPoint.x - centerPoint.x;
            double rys = startingPoint.y - centerPoint.y;
            startAngle = atan2(a*(rys*cos(phi)-rxs*sin(phi)), b*(rxs*cos(phi)+rys*sin(phi))); // eccentric anomaly angle

            double angle1 = atan2(a*((onSketchPos.y - centerPoint.y)*cos(phi)-(onSketchPos.x - centerPoint.x)*sin(phi)),
                                  b*((onSketchPos.x - centerPoint.x)*cos(phi)+(onSketchPos.y - centerPoint.y)*sin(phi)))- startAngle;

            double angle2 = angle1 + (angle1 < 0. ? 2 : -2) * M_PI ;
            arcAngle = abs(angle1-arcAngle) < abs(angle2-arcAngle) ? angle1 : angle2;

            for (int i=0; i < 34; i++) {
                double angle = startAngle+i*arcAngle/34.0;
                double rx1 = a * cos(angle) * cos(phi) - b * sin(angle) * sin(phi);
                double ry1 = a * cos(angle) * sin(phi) + b * sin(angle) * cos(phi);
                EditCurve[i] = Base::Vector2d(centerPoint.x + rx1, centerPoint.y + ry1);
            }
//             EditCurve[33] = EditCurve[1];
//             EditCurve[17] = EditCurve[16];

            // Display radii and angle for user
            SbString text;
            text.sprintf(" (%.1fR,%.1fR,%.1fdeg)", a, b, arcAngle * 180 / M_PI);
            setPositionText(onSketchPos, text);

            drawEdit(EditCurve);
            if (seekAutoConstraint(sugConstr4, onSketchPos, Base::Vector2d(0.f,0.f))) {
                renderSuggestConstraintsCursor(sugConstr4);
                return;
            }
        }



        applyCursor();
    }

    virtual bool pressButton(Base::Vector2d onSketchPos)
    {
        if (Mode==STATUS_SEEK_First){
            EditCurve[0] = onSketchPos;
            centerPoint = onSketchPos;
            Mode = STATUS_SEEK_Second;
        }
        else if(Mode==STATUS_SEEK_Second) {
            EditCurve[1] = onSketchPos;
            axisPoint = onSketchPos;
            Mode = STATUS_SEEK_Third;
        }
        else if(Mode==STATUS_SEEK_Third) {
            startingPoint = onSketchPos;
            arcAngle = 0.;
            arcAngle_t= 0.;
            Mode = STATUS_SEEK_Fourth;
        }
        else { // Fourth
            endPoint = onSketchPos;

            Mode = STATUS_Close;
        }
        return true;
    }

    virtual bool releaseButton(Base::Vector2d onSketchPos)
    {
        Q_UNUSED(onSketchPos);
        if (Mode==STATUS_Close) {
            unsetCursor();
            resetPositionText();

            // angle between the major axis of the ellipse and the X axisEllipse
            double a = (axisPoint-centerPoint).Length();
            double phi = atan2(axisPoint.y-centerPoint.y,axisPoint.x-centerPoint.x);

            // This is the angle at cursor point
            double angleatpoint = acos((startingPoint.x-centerPoint.x+(startingPoint.y-centerPoint.y)*tan(phi))/(a*(cos(phi)+tan(phi)*sin(phi))));
            double b=abs((startingPoint.y-centerPoint.y-a*cos(angleatpoint)*sin(phi))/(sin(angleatpoint)*cos(phi)));

            double angle1 = atan2(a*((endPoint.y - centerPoint.y)*cos(phi)-(endPoint.x - centerPoint.x)*sin(phi)),
                                  b*((endPoint.x - centerPoint.x)*cos(phi)+(endPoint.y - centerPoint.y)*sin(phi)))- startAngle;

            double angle2 = angle1 + (angle1 < 0. ? 2 : -2) * M_PI ;
            arcAngle = abs(angle1-arcAngle) < abs(angle2-arcAngle) ? angle1 : angle2;

            bool isOriginalArcCCW=true;

            if (arcAngle > 0)
                endAngle = startAngle + arcAngle;
            else {
                endAngle = startAngle;
                startAngle += arcAngle;
                isOriginalArcCCW=false;
            }

            Base::Vector2d majAxisDir,minAxisDir,minAxisPoint,majAxisPoint;
            // We always create a CCW ellipse, because we want our XY reference system to be in the +X +Y direction
            // Our normal will then always be in the +Z axis (local +Z axis of the sketcher)

            if(a>b)
            {
                // force second semidiameter to be perpendicular to first semidiamater
                majAxisDir = axisPoint - centerPoint;
                Base::Vector2d perp(-majAxisDir.y,majAxisDir.x);
                perp.Normalize();
                perp.Scale(abs(b));
                minAxisPoint = centerPoint+perp;
                majAxisPoint = centerPoint+majAxisDir;
            }
            else {
                // force second semidiameter to be perpendicular to first semidiamater
                minAxisDir = axisPoint - centerPoint;
                Base::Vector2d perp(minAxisDir.y,-minAxisDir.x);
                perp.Normalize();
                perp.Scale(abs(b));
                majAxisPoint = centerPoint+perp;
                minAxisPoint = centerPoint+minAxisDir;
                endAngle +=  M_PI/2;
                startAngle += M_PI/2;
                phi-=M_PI/2;
                double t=a; a=b; b=t;//swap a,b
            }

            int currentgeoid = getHighestCurveIndex();

            try {
                Gui::Command::openCommand(QT_TRANSLATE_NOOP("Command", "Add sketch arc of ellipse"));

                Gui::cmdAppObjectArgs(sketchgui->getObject(), "addGeometry(Part.ArcOfEllipse"
                    "(Part.Ellipse(App.Vector(%f,%f,0),App.Vector(%f,%f,0),App.Vector(%f,%f,0)),%f,%f),%s)",
                        majAxisPoint.x, majAxisPoint.y,
                        minAxisPoint.x, minAxisPoint.y,
                        centerPoint.x, centerPoint.y,
                        startAngle, endAngle,
                        geometryCreationMode==Construction?"True":"False");

                currentgeoid++;

                Gui::cmdAppObjectArgs(sketchgui->getObject(), "exposeInternalGeometry(%d)", currentgeoid);
            }
            catch (const Base::Exception& e) {
                Base::Console().Error("%s\n", e.what());
                Gui::Command::abortCommand();

                tryAutoRecomputeIfNotSolve(static_cast<Sketcher::SketchObject *>(sketchgui->getObject()));

                return false;
            }

            Gui::Command::commitCommand();

            // add auto constraints for the center point
            if (sugConstr1.size() > 0) {
                createAutoConstraints(sugConstr1, currentgeoid, Sketcher::PointPos::mid);
                sugConstr1.clear();
            }

            // add suggested constraints for arc
            if (sugConstr2.size() > 0) {
                createAutoConstraints(sugConstr2, currentgeoid, Sketcher::PointPos::none);
                sugConstr2.clear();
            }

            // add suggested constraints for start of arc
            if (sugConstr3.size() > 0) {
                createAutoConstraints(sugConstr3, currentgeoid, isOriginalArcCCW?Sketcher::PointPos::start:Sketcher::PointPos::end);
                sugConstr3.clear();
            }

            // add suggested constraints for start of arc
            if (sugConstr4.size() > 0) {
                createAutoConstraints(sugConstr4, currentgeoid, isOriginalArcCCW?Sketcher::PointPos::end:Sketcher::PointPos::start);
                sugConstr4.clear();
            }

            tryAutoRecomputeIfNotSolve(static_cast<Sketcher::SketchObject *>(sketchgui->getObject()));

            ParameterGrp::handle hGrp = App::GetApplication().GetParameterGroupByPath("User parameter:BaseApp/Preferences/Mod/Sketcher");
            bool continuousMode = hGrp->GetBool("ContinuousCreationMode",true);
            if(continuousMode){
                // This code enables the continuous creation mode.
                Mode=STATUS_SEEK_First;
                EditCurve.clear();
                drawEdit(EditCurve);
                EditCurve.resize(34);
                applyCursor();
                /* this is ok not to call to purgeHandler
                * in continuous creation mode because the
                * handler is destroyed by the quit() method on pressing the
                * right button of the mouse */
            }
            else{
                sketchgui->purgeHandler(); // no code after this line, Handler get deleted in ViewProvider
            }
        }
        return true;
    }
protected:
    SelectMode Mode;
    std::vector<Base::Vector2d> EditCurve;
    Base::Vector2d centerPoint, axisPoint, startingPoint, endPoint;
    double rx, ry, startAngle, endAngle, arcAngle, arcAngle_t;
    std::vector<AutoConstraint> sugConstr1, sugConstr2, sugConstr3, sugConstr4;
};

DEF_STD_CMD_A(CmdSketcherCreateArcOfEllipse)

CmdSketcherCreateArcOfEllipse::CmdSketcherCreateArcOfEllipse()
  : Command("Sketcher_CreateArcOfEllipse")
{
    sAppModule      = "Sketcher";
    sGroup          = "Sketcher";
    sMenuText       = QT_TR_NOOP("Create an arc of ellipse");
    sToolTipText    = QT_TR_NOOP("Create an arc of ellipse in the sketch");
    sWhatsThis      = "Sketcher_CreateArcOfEllipse";
    sStatusTip      = sToolTipText;
    sPixmap         = "Sketcher_CreateElliptical_Arc";
    sAccel          = "G, E, A";
    eType           = ForEdit;
}

void CmdSketcherCreateArcOfEllipse::activated(int iMsg)
{
    Q_UNUSED(iMsg);
    ActivateHandler(getActiveGuiDocument(),new DrawSketchHandlerArcOfEllipse() );
}

bool CmdSketcherCreateArcOfEllipse::isActive(void)
{
    return isCreateGeoActive(getActiveGuiDocument());
}

class DrawSketchHandlerArcOfHyperbola : public DrawSketchHandler
{
public:
    DrawSketchHandlerArcOfHyperbola()
      : Mode(STATUS_SEEK_First)
      , EditCurve(34)
      , arcAngle(0)
      , arcAngle_t(0)
    {
    }
    virtual ~DrawSketchHandlerArcOfHyperbola(){}
    /// mode table
    enum SelectMode {
        STATUS_SEEK_First,      /**< enum value ----. */
        STATUS_SEEK_Second,     /**< enum value ----. */
        STATUS_SEEK_Third,     /**< enum value ----. */
        STATUS_SEEK_Fourth,     /**< enum value ----. */
        STATUS_Close
    };

    virtual void activated(ViewProviderSketch * /*sketchgui*/)
    {
        setCrosshairCursor("Sketcher_Pointer_Create_ArcOfHyperbola");
    }

    virtual void mouseMove(Base::Vector2d onSketchPos)
    {
        if (Mode==STATUS_SEEK_First) {
            setPositionText(onSketchPos);
            if (seekAutoConstraint(sugConstr1, onSketchPos, Base::Vector2d(0.f,0.f))) {
                renderSuggestConstraintsCursor(sugConstr1);
                return;
            }
        }
        else if (Mode==STATUS_SEEK_Second) {
            EditCurve[1]= onSketchPos;

            // Display radius for user
            float radius = (onSketchPos - centerPoint).Length();

            SbString text;
            text.sprintf(" (%.1fR,%.1fR)", radius,radius);
            setPositionText(onSketchPos, text);

            drawEdit(EditCurve);
            if (seekAutoConstraint(sugConstr2, onSketchPos, Base::Vector2d(0.f,0.f),
                                   AutoConstraint::CURVE)) {
                renderSuggestConstraintsCursor(sugConstr2);
                return;
            }
        }
        else if (Mode==STATUS_SEEK_Third) {
            // angle between the major axis of the hyperbola and the X axis
            double a = (axisPoint-centerPoint).Length();
            double phi = atan2(axisPoint.y-centerPoint.y,axisPoint.x-centerPoint.x);

            // This is the angle at cursor point
            double angleatpoint = acosh(((onSketchPos.x-centerPoint.x)*cos(phi)+(onSketchPos.y-centerPoint.y)*sin(phi))/a);
            double b=(onSketchPos.y-centerPoint.y-a*cosh(angleatpoint)*sin(phi))/(sinh(angleatpoint)*cos(phi));

            if(!boost::math::isnan(b)){
                for (int i=15; i >= -15; i--) {
                    // P(U) = O + MajRad*Cosh(U)*XDir + MinRad*Sinh(U)*YDir
                    //double angle = i*M_PI/16.0;
                    double angle=i*angleatpoint/15;
                    double rx = a * cosh(angle) * cos(phi) - b * sinh(angle) * sin(phi);
                    double ry = a * cosh(angle) * sin(phi) + b * sinh(angle) * cos(phi);
                    EditCurve[15+i] = Base::Vector2d(centerPoint.x + rx, centerPoint.y + ry);
                }

                // Display radius for user
                SbString text;
                text.sprintf(" (%.1fR,%.1fR)", a, b);
                setPositionText(onSketchPos, text);
            }

            drawEdit(EditCurve);
            if (seekAutoConstraint(sugConstr3, onSketchPos, Base::Vector2d(0.f,0.f))) {
                renderSuggestConstraintsCursor(sugConstr3);
                return;
            }
        }
        else if (Mode==STATUS_SEEK_Fourth) {
            // angle between the major axis of the hyperbola and the X axis
            double a = (axisPoint-centerPoint).Length();
            double phi = atan2(axisPoint.y-centerPoint.y,axisPoint.x-centerPoint.x);

            // This is the angle at cursor point
            double angleatstartingpoint = acosh(((startingPoint.x-centerPoint.x)*cos(phi)+(startingPoint.y-centerPoint.y)*sin(phi))/a);
            double b=(startingPoint.y-centerPoint.y-a*cosh(angleatstartingpoint)*sin(phi))/(sinh(angleatstartingpoint)*cos(phi));

            double startAngle = angleatstartingpoint;

            //double angleatpoint = acosh(((onSketchPos.x-centerPoint.x)*cos(phi)+(onSketchPos.y-centerPoint.y)*sin(phi))/a);

            double angleatpoint = atanh( (((onSketchPos.y-centerPoint.y)*cos(phi)-(onSketchPos.x-centerPoint.x)*sin(phi))*a) /
                                         (((onSketchPos.x-centerPoint.x)*cos(phi)+(onSketchPos.y-centerPoint.y)*sin(phi))*b)  );

            /*double angle1 = angleatpoint - startAngle;

            double angle2 = angle1 + (angle1 < 0. ? 2 : -2) * M_PI ;
            arcAngle = abs(angle1-arcAngle) < abs(angle2-arcAngle) ? angle1 : angle2;*/

            arcAngle = angleatpoint - startAngle;

            //if(!boost::math::isnan(angle1) && !boost::math::isnan(angle2)){
            if (!boost::math::isnan(arcAngle)) {
                EditCurve.resize(33);
                for (int i=0; i < 33; i++) {
                    // P(U) = O + MajRad*Cosh(U)*XDir + MinRad*Sinh(U)*YDir
                    //double angle=i*angleatpoint/16;
                    double angle = startAngle+i*arcAngle/32.0;
                    double rx = a * cosh(angle) * cos(phi) - b * sinh(angle) * sin(phi);
                    double ry = a * cosh(angle) * sin(phi) + b * sinh(angle) * cos(phi);
                    EditCurve[i] = Base::Vector2d(centerPoint.x + rx, centerPoint.y + ry);
                }

                // Display radius for user
                SbString text;
                text.sprintf(" (%.1fR,%.1fR)", a, b);
                setPositionText(onSketchPos, text);
            }
            else {
                arcAngle=0.;
            }

            drawEdit(EditCurve);
            if (seekAutoConstraint(sugConstr4, onSketchPos, Base::Vector2d(0.f,0.f))) {
                renderSuggestConstraintsCursor(sugConstr4);
                return;
            }
        }

        applyCursor();
    }

    virtual bool pressButton(Base::Vector2d onSketchPos)
    {
        if (Mode==STATUS_SEEK_First){
            EditCurve[0] = onSketchPos;
            centerPoint = onSketchPos;
            EditCurve.resize(2);
            Mode = STATUS_SEEK_Second;
        }
        else if(Mode==STATUS_SEEK_Second) {
            EditCurve[1] = onSketchPos;
            axisPoint = onSketchPos;
            EditCurve.resize(31);
            Mode = STATUS_SEEK_Third;
        }
        else if(Mode==STATUS_SEEK_Third) {
            startingPoint = onSketchPos;
            arcAngle = 0.;
            arcAngle_t= 0.;
            Mode = STATUS_SEEK_Fourth;
        }
        else { // Fourth
            endPoint = onSketchPos;

            Mode = STATUS_Close;
        }
        return true;
    }

    virtual bool releaseButton(Base::Vector2d /*onSketchPos*/)
    {
        if (Mode==STATUS_Close) {
            unsetCursor();
            resetPositionText();


            // angle between the major axis of the hyperbola and the X axis
            double a = (axisPoint-centerPoint).Length();
            double phi = atan2(axisPoint.y-centerPoint.y,axisPoint.x-centerPoint.x);

            // This is the angle at cursor point
            double angleatstartingpoint = acosh(((startingPoint.x-centerPoint.x)*cos(phi)+(startingPoint.y-centerPoint.y)*sin(phi))/a);
            double b=(startingPoint.y-centerPoint.y-a*cosh(angleatstartingpoint)*sin(phi))/(sinh(angleatstartingpoint)*cos(phi));

            double startAngle = angleatstartingpoint;

            //double angleatpoint = acosh(((onSketchPos.x-centerPoint.x)*cos(phi)+(onSketchPos.y-centerPoint.y)*sin(phi))/a);

            double endAngle = atanh( (((endPoint.y-centerPoint.y)*cos(phi)-(endPoint.x-centerPoint.x)*sin(phi))*a) /
                                         (((endPoint.x-centerPoint.x)*cos(phi)+(endPoint.y-centerPoint.y)*sin(phi))*b)  );

            if (boost::math::isnan(startAngle) || boost::math::isnan(endAngle)) {
                sketchgui->purgeHandler();
                Base::Console().Error("Cannot create arc of hyperbola from invalid angles, try again!\n");
                return false;
            }


            bool isOriginalArcCCW=true;

            if (arcAngle > 0)
                endAngle = startAngle + arcAngle;
            else {
                endAngle = startAngle;
                startAngle += arcAngle;
                isOriginalArcCCW=false;
            }

            Base::Vector2d majAxisDir,minAxisDir,minAxisPoint,majAxisPoint;
            // We always create a CCW hyperbola, because we want our XY reference system to be in the +X +Y direction
            // Our normal will then always be in the +Z axis (local +Z axis of the sketcher)

            if(a>b)
            {
                // force second semidiameter to be perpendicular to first semidiamater
                majAxisDir = axisPoint - centerPoint;
                Base::Vector2d perp(-majAxisDir.y,majAxisDir.x);
                perp.Normalize();
                perp.Scale(abs(b));
                minAxisPoint = centerPoint+perp;
                majAxisPoint = centerPoint+majAxisDir;
            }
            else {
                // force second semidiameter to be perpendicular to first semidiamater
                minAxisDir = axisPoint - centerPoint;
                Base::Vector2d perp(minAxisDir.y,-minAxisDir.x);
                perp.Normalize();
                perp.Scale(abs(b));
                majAxisPoint = centerPoint+perp;
                minAxisPoint = centerPoint+minAxisDir;
                endAngle +=  M_PI/2;
                startAngle += M_PI/2;
            }

            int currentgeoid = getHighestCurveIndex();

            try {

                Gui::Command::openCommand(QT_TRANSLATE_NOOP("Command", "Add sketch arc of hyperbola"));

                //Add arc of hyperbola, point and constrain point as focus2. We add focus2 for it to balance
                //the intrinsic focus1, in order to balance out the intrinsic invisible focus1 when AOE is
                //dragged by its center
                Gui::cmdAppObjectArgs(sketchgui->getObject(), "addGeometry(Part.ArcOfHyperbola"
                    "(Part.Hyperbola(App.Vector(%f,%f,0),App.Vector(%f,%f,0),App.Vector(%f,%f,0)),%f,%f),%s)",
                    majAxisPoint.x, majAxisPoint.y,
                    minAxisPoint.x, minAxisPoint.y,
                    centerPoint.x, centerPoint.y,
                    startAngle, endAngle,
                    geometryCreationMode==Construction?"True":"False");

                currentgeoid++;

                Gui::cmdAppObjectArgs(sketchgui->getObject(), "exposeInternalGeometry(%d)", currentgeoid);
            }
            catch (const Base::Exception& e) {
                Base::Console().Error("%s\n", e.what());
                Gui::Command::abortCommand();

                tryAutoRecomputeIfNotSolve(static_cast<Sketcher::SketchObject *>(sketchgui->getObject()));

                return false;
            }

            Gui::Command::commitCommand();

            // add auto constraints for the center point
            if (sugConstr1.size() > 0) {
                createAutoConstraints(sugConstr1, currentgeoid, Sketcher::PointPos::mid);
                sugConstr1.clear();
            }

            // add suggested constraints for arc
            if (sugConstr2.size() > 0) {
                createAutoConstraints(sugConstr2, currentgeoid, Sketcher::PointPos::none);
                sugConstr2.clear();
            }

            // add suggested constraints for start of arc
            if (sugConstr3.size() > 0) {
                createAutoConstraints(sugConstr3, currentgeoid, isOriginalArcCCW?Sketcher::PointPos::start:Sketcher::PointPos::end);
                sugConstr3.clear();
            }

            // add suggested constraints for start of arc
            if (sugConstr4.size() > 0) {
                createAutoConstraints(sugConstr4, currentgeoid, isOriginalArcCCW?Sketcher::PointPos::end:Sketcher::PointPos::start);
                sugConstr4.clear();
            }

            tryAutoRecomputeIfNotSolve(static_cast<Sketcher::SketchObject *>(sketchgui->getObject()));

            ParameterGrp::handle hGrp = App::GetApplication().GetParameterGroupByPath("User parameter:BaseApp/Preferences/Mod/Sketcher");
            bool continuousMode = hGrp->GetBool("ContinuousCreationMode",true);

            if(continuousMode){
                // This code enables the continuous creation mode.
                Mode = STATUS_SEEK_First;
                EditCurve.clear();
                drawEdit(EditCurve);
                EditCurve.resize(34);
                applyCursor();
                /* It is ok not to call to purgeHandler
                 * in continuous creation mode because the
                 * handler is destroyed by the quit() method on pressing the
                 * right button of the mouse */
            }
            else{
                sketchgui->purgeHandler(); // no code after this line, Handler get deleted in ViewProvider
            }
        }
        return true;
    }
protected:
    SelectMode Mode;
    std::vector<Base::Vector2d> EditCurve;
    Base::Vector2d centerPoint, axisPoint, startingPoint, endPoint;
    double arcAngle, arcAngle_t;
    std::vector<AutoConstraint> sugConstr1, sugConstr2, sugConstr3, sugConstr4;

};

DEF_STD_CMD_A(CmdSketcherCreateArcOfHyperbola)

CmdSketcherCreateArcOfHyperbola::CmdSketcherCreateArcOfHyperbola()
  : Command("Sketcher_CreateArcOfHyperbola")
{
    sAppModule      = "Sketcher";
    sGroup          = "Sketcher";
    sMenuText       = QT_TR_NOOP("Create an arc of hyperbola");
    sToolTipText    = QT_TR_NOOP("Create an arc of hyperbola in the sketch");
    sWhatsThis      = "Sketcher_CreateArcOfHyperbola";
    sStatusTip      = sToolTipText;
    sPixmap         = "Sketcher_CreateHyperbolic_Arc";
    sAccel          = "G, H";
    eType           = ForEdit;
}

void CmdSketcherCreateArcOfHyperbola::activated(int /*iMsg*/)
{
    ActivateHandler(getActiveGuiDocument(),new DrawSketchHandlerArcOfHyperbola() );
}

bool CmdSketcherCreateArcOfHyperbola::isActive(void)
{
    return isCreateGeoActive(getActiveGuiDocument());
}

class DrawSketchHandlerArcOfParabola : public DrawSketchHandler
{
public:
    DrawSketchHandlerArcOfParabola()
        : Mode(STATUS_SEEK_First)
        , EditCurve(34)
        , startAngle(0)
        , endAngle(0)
        , arcAngle(0)
        , arcAngle_t(0)
    {
    }
    virtual ~DrawSketchHandlerArcOfParabola(){}
    /// mode table
    enum SelectMode {
        STATUS_SEEK_First,      /**< enum value ----. */
        STATUS_SEEK_Second,     /**< enum value ----. */
        STATUS_SEEK_Third,      /**< enum value ----. */
        STATUS_SEEK_Fourth,     /**< enum value ----. */
        STATUS_Close
    };

    virtual void activated(ViewProviderSketch * /*sketchgui*/)
    {
        setCrosshairCursor("Sketcher_Pointer_Create_ArcOfParabola");
    }

    virtual void mouseMove(Base::Vector2d onSketchPos)
    {
        if (Mode==STATUS_SEEK_First) {
            setPositionText(onSketchPos);
            if (seekAutoConstraint(sugConstr1, onSketchPos, Base::Vector2d(0.f,0.f))) {
                renderSuggestConstraintsCursor(sugConstr1);
                return;
            }
        }
        else if (Mode==STATUS_SEEK_Second) {
            EditCurve[1]= onSketchPos;

            // Display radius for user
            float radius = (onSketchPos - focusPoint).Length();

            SbString text;
            text.sprintf(" (F%.1f)", radius);
            setPositionText(onSketchPos, text);

            drawEdit(EditCurve);
            if (seekAutoConstraint(sugConstr2, onSketchPos, Base::Vector2d(0.f,0.f))) {
                renderSuggestConstraintsCursor(sugConstr2);
                return;
            }
        }
        else if (Mode==STATUS_SEEK_Third) {
            double focal = (axisPoint-focusPoint).Length();
            double phi = atan2(focusPoint.y-axisPoint.y,focusPoint.x-axisPoint.x);

            // P(U) = O + U*U/(4.*F)*XDir + U*YDir
            //
            // pnt = Base::Vector3d(pnt0.x + angle * angle / 4 / focal * cos(phi) - angle * sin(phi),
            //                      pnt0.y + angle * angle / 4 / focal * sin(phi) + angle * cos(phi),
            //                      0.f);

            // This is the angle at cursor point
            double u =
            ( cos(phi) * (onSketchPos.y - axisPoint.y) - (onSketchPos.x - axisPoint.x) * sin(phi));

            for (int i=15; i >= -15; i--) {
                double angle=i*u/15;
                double rx = angle * angle / 4 / focal * cos(phi) - angle * sin(phi);
                double ry = angle * angle / 4 / focal * sin(phi) + angle * cos(phi);
                EditCurve[15+i] = Base::Vector2d(axisPoint.x + rx, axisPoint.y + ry);
            }

            // Display radius for user
            SbString text;
            text.sprintf(" (F%.1f)", focal);
            setPositionText(onSketchPos, text);

            drawEdit(EditCurve);

            if (seekAutoConstraint(sugConstr3, onSketchPos, Base::Vector2d(0.f,0.f))) {
                renderSuggestConstraintsCursor(sugConstr3);
                return;
            }
        }
        else if (Mode==STATUS_SEEK_Fourth) {
            double focal = (axisPoint-focusPoint).Length();
            double phi = atan2(focusPoint.y-axisPoint.y,focusPoint.x-axisPoint.x);

            // P(U) = O + U*U/(4.*F)*XDir + U*YDir
            //
            // pnt = Base::Vector3d(pnt0.x + angle * angle / 4 / focal * cos(phi) - angle * sin(phi),
            //                      pnt0.y + angle * angle / 4 / focal * sin(phi) + angle * cos(phi),
            //                      0.f);

            // This is the angle at starting point
            double ustartpoint =
            ( cos(phi) * (startingPoint.y - axisPoint.y) - (startingPoint.x - axisPoint.x) * sin(phi));

            double startValue = ustartpoint;

            double u =
            ( cos(phi) * (onSketchPos.y - axisPoint.y) - (onSketchPos.x - axisPoint.x) * sin(phi));


            arcAngle = u - startValue;

            if (!boost::math::isnan(arcAngle)) {
                EditCurve.resize(33);
                for (std::size_t i=0; i < 33; i++) {
                    double angle = startValue+i*arcAngle/32.0;
                    double rx = angle * angle / 4 / focal * cos(phi) - angle * sin(phi);
                    double ry = angle * angle / 4 / focal * sin(phi) + angle * cos(phi);
                    EditCurve[i] = Base::Vector2d(axisPoint.x + rx, axisPoint.y + ry);
                }

                SbString text;
                text.sprintf(" (F%.1f)", focal);
                setPositionText(onSketchPos, text);
            }
            else {
                arcAngle=0.;
            }

            drawEdit(EditCurve);
            if (seekAutoConstraint(sugConstr4, onSketchPos, Base::Vector2d(0.f,0.f))) {
                renderSuggestConstraintsCursor(sugConstr4);
                return;
            }
        }

        applyCursor();
    }

    virtual bool pressButton(Base::Vector2d onSketchPos)
    {
        if (Mode==STATUS_SEEK_First){
            EditCurve[0] = onSketchPos;
            focusPoint = onSketchPos;
            EditCurve.resize(2);
            Mode = STATUS_SEEK_Second;
        }
        else if(Mode==STATUS_SEEK_Second) {
            EditCurve[1] = onSketchPos;
            axisPoint = onSketchPos;
            EditCurve.resize(31);
            Mode = STATUS_SEEK_Third;
        }
        else if(Mode==STATUS_SEEK_Third) {
            startingPoint = onSketchPos;
            arcAngle = 0.;
            arcAngle_t= 0.;
            Mode = STATUS_SEEK_Fourth;
        }
        else { // Fourth
            endPoint = onSketchPos;
            Mode = STATUS_Close;
        }
        return true;
    }

    virtual bool releaseButton(Base::Vector2d /*onSketchPos*/)
    {
        if (Mode==STATUS_Close) {
            unsetCursor();
            resetPositionText();

            double phi = atan2(focusPoint.y-axisPoint.y,focusPoint.x-axisPoint.x);

            double ustartpoint =
            ( cos(phi) * (startingPoint.y - axisPoint.y) - (startingPoint.x - axisPoint.x) * sin(phi));

            double uendpoint =
            ( cos(phi) * (endPoint.y - axisPoint.y) - (endPoint.x - axisPoint.x) * sin(phi));

            double startAngle = ustartpoint;

            double endAngle = uendpoint;

            bool isOriginalArcCCW=true;

            if (arcAngle > 0) {
                endAngle = startAngle + arcAngle;
            }
            else {
                endAngle = startAngle;
                startAngle += arcAngle;
                isOriginalArcCCW=false;
            }

            int currentgeoid = getHighestCurveIndex();

            try {
                Gui::Command::openCommand(QT_TRANSLATE_NOOP("Command", "Add sketch arc of Parabola"));

                //Add arc of parabola
                Gui::cmdAppObjectArgs(sketchgui->getObject(), "addGeometry(Part.ArcOfParabola"
                    "(Part.Parabola(App.Vector(%f,%f,0),App.Vector(%f,%f,0),App.Vector(0,0,1)),%f,%f),%s)",
                        focusPoint.x, focusPoint.y,
                        axisPoint.x, axisPoint.y,
                        startAngle, endAngle,
                        geometryCreationMode==Construction?"True":"False");

                currentgeoid++;

                Gui::cmdAppObjectArgs(sketchgui->getObject(), "exposeInternalGeometry(%d)", currentgeoid);
            }
            catch (const Base::Exception& e) {
                Base::Console().Error("%s\n", e.what());
                Gui::Command::abortCommand();

                tryAutoRecomputeIfNotSolve(static_cast<Sketcher::SketchObject *>(sketchgui->getObject()));

                return false;
            }

            Gui::Command::commitCommand();

            // add auto constraints for the focus point
            if (sugConstr1.size() > 0) {
                createAutoConstraints(sugConstr1, currentgeoid+1, Sketcher::PointPos::start);
                sugConstr1.clear();
            }

            // add suggested constraints for vertex point
            if (sugConstr2.size() > 0) {
                createAutoConstraints(sugConstr2, currentgeoid, Sketcher::PointPos::mid);
                sugConstr2.clear();
            }

            // add suggested constraints for start of arc
            if (sugConstr3.size() > 0) {
                createAutoConstraints(sugConstr3, currentgeoid, isOriginalArcCCW?Sketcher::PointPos::start:Sketcher::PointPos::end);
                sugConstr3.clear();
            }

            // add suggested constraints for start of arc
            if (sugConstr4.size() > 0) {
                createAutoConstraints(sugConstr4, currentgeoid, isOriginalArcCCW?Sketcher::PointPos::end:Sketcher::PointPos::start);
                sugConstr4.clear();
            }

            tryAutoRecomputeIfNotSolve(static_cast<Sketcher::SketchObject *>(sketchgui->getObject()));

            ParameterGrp::handle hGrp = App::GetApplication().GetParameterGroupByPath("User parameter:BaseApp/Preferences/Mod/Sketcher");
            bool continuousMode = hGrp->GetBool("ContinuousCreationMode",true);
            if (continuousMode) {
                // This code enables the continuous creation mode.
                Mode = STATUS_SEEK_First;
                EditCurve.clear();
                drawEdit(EditCurve);
                EditCurve.resize(34);
                applyCursor();
                /* It is ok not to call to purgeHandler
                 * in continuous creation mode because the
                 * handler is destroyed by the quit() method on pressing the
                 * right button of the mouse */
            }
            else {
                sketchgui->purgeHandler(); // no code after this line, Handler get deleted in ViewProvider
            }
        }
        return true;
    }

protected:
    SelectMode Mode;
    std::vector<Base::Vector2d> EditCurve;
    Base::Vector2d focusPoint, axisPoint, startingPoint, endPoint;
    double startAngle, endAngle, arcAngle, arcAngle_t;
    std::vector<AutoConstraint> sugConstr1, sugConstr2, sugConstr3, sugConstr4;
};

DEF_STD_CMD_A(CmdSketcherCreateArcOfParabola)

CmdSketcherCreateArcOfParabola::CmdSketcherCreateArcOfParabola()
  : Command("Sketcher_CreateArcOfParabola")
{
    sAppModule      = "Sketcher";
    sGroup          = "Sketcher";
    sMenuText       = QT_TR_NOOP("Create an arc of parabola");
    sToolTipText    = QT_TR_NOOP("Create an arc of parabola in the sketch");
    sWhatsThis      = "Sketcher_CreateArcOfParabola";
    sStatusTip      = sToolTipText;
    sPixmap         = "Sketcher_CreateParabolic_Arc";
    sAccel          = "G, J";
    eType           = ForEdit;
}

void CmdSketcherCreateArcOfParabola::activated(int /*iMsg*/)
{
    ActivateHandler(getActiveGuiDocument(),new DrawSketchHandlerArcOfParabola() );
}

bool CmdSketcherCreateArcOfParabola::isActive(void)
{
    return isCreateGeoActive(getActiveGuiDocument());
}




/// @brief Macro that declares a new sketcher command class 'CmdSketcherCompCreateEllipse'
DEF_STD_CMD_ACLU(CmdSketcherCompCreateConic)

/**
 * @brief ctor
 */
CmdSketcherCompCreateConic::CmdSketcherCompCreateConic()
  : Command("Sketcher_CompCreateConic")
{
    sAppModule      = "Sketcher";
    sGroup          = "Sketcher";
    sMenuText       = QT_TR_NOOP("Create a conic");
    sToolTipText    = QT_TR_NOOP("Create a conic in the sketch");
    sWhatsThis      = "Sketcher_CompCreateConic";
    sStatusTip      = sToolTipText;
    eType           = ForEdit;
}

/**
 * @brief Instantiates the conic handler when the conic command activated
 * @param int iMsg
 */
void CmdSketcherCompCreateConic::activated(int iMsg)
{
    if (iMsg == 0) {
        ActivateHandler(getActiveGuiDocument(), new DrawSketchHandlerEllipse(iMsg));
    } else if (iMsg == 1) {
        ActivateHandler(getActiveGuiDocument(), new DrawSketchHandlerEllipse(iMsg));
    } else if (iMsg == 2) {
        ActivateHandler(getActiveGuiDocument(), new DrawSketchHandlerArcOfEllipse());
    } else if (iMsg == 3) {
        ActivateHandler(getActiveGuiDocument(), new DrawSketchHandlerArcOfHyperbola());
    } else if (iMsg == 4) {
        ActivateHandler(getActiveGuiDocument(), new DrawSketchHandlerArcOfParabola());
    } else {
        return;
    }

    // Since the default icon is reset when enabling/disabling the command we have
    // to explicitly set the icon of the used command.
    Gui::ActionGroup* pcAction = qobject_cast<Gui::ActionGroup*>(_pcAction);
    QList<QAction*> a = pcAction->actions();

    assert(iMsg < a.size());
    pcAction->setIcon(a[iMsg]->icon());
}

Gui::Action * CmdSketcherCompCreateConic::createAction(void)
{
    Gui::ActionGroup* pcAction = new Gui::ActionGroup(this, Gui::getMainWindow());
    pcAction->setDropDownMenu(true);
    applyCommandData(this->className(), pcAction);

    QAction* ellipseByCenter = pcAction->addAction(QString());
    ellipseByCenter->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateEllipse"));
     /// @todo replace with correct icon
    QAction* ellipseBy3Points = pcAction->addAction(QString());
    ellipseBy3Points->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateEllipse_3points"));

    QAction* arcofellipse = pcAction->addAction(QString());
    arcofellipse->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateElliptical_Arc"));

    QAction* arcofhyperbola = pcAction->addAction(QString());
    arcofhyperbola->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateHyperbolic_Arc"));

    QAction* arcofparabola = pcAction->addAction(QString());
    arcofparabola->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateParabolic_Arc"));

    _pcAction = pcAction;
    languageChange();

    // set ellipse by center, a, b as default method
    pcAction->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_Conics"));
    int defaultId = 0;
    pcAction->setProperty("defaultAction", QVariant(defaultId));

    return pcAction;
}

void CmdSketcherCompCreateConic::updateAction(int mode)
{
    Gui::ActionGroup* pcAction = qobject_cast<Gui::ActionGroup*>(getAction());
    if (!pcAction)
        return;

    QList<QAction*> a = pcAction->actions();
    int index = pcAction->property("defaultAction").toInt();
    switch (mode) {
    case Normal:
        a[0]->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateEllipse"));
        a[1]->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateEllipse_3points"));
        a[2]->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateElliptical_Arc"));
        a[3]->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateHyperbolic_Arc"));
        a[4]->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateParabolic_Arc"));
        getAction()->setIcon(a[index]->icon());
        break;
    case Construction:
        a[0]->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateEllipse_Constr"));
        a[1]->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateEllipse_3points_Constr"));
        a[2]->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateElliptical_Arc_Constr"));
        a[3]->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateHyperbolic_Arc_Constr"));
        a[4]->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateParabolic_Arc_Constr"));
        getAction()->setIcon(a[index]->icon());
        break;
    }
}

void CmdSketcherCompCreateConic::languageChange()
{
    Command::languageChange();

    if (!_pcAction)
        return;
    Gui::ActionGroup* pcAction = qobject_cast<Gui::ActionGroup*>(_pcAction);
    QList<QAction*> a = pcAction->actions();

    QAction* ellipseByCenter = a[0];
    ellipseByCenter->setText(QApplication::translate("CmdSketcherCompCreateConic","Ellipse by center, major radius, point"));
    ellipseByCenter->setToolTip(QApplication::translate("Sketcher_CreateEllipseByCenter","Create an ellipse by center, major radius and point"));
    ellipseByCenter->setStatusTip(QApplication::translate("Sketcher_CreateEllipseByCenter","Create an ellipse by center, major radius and point"));
    QAction* ellipseBy3Points = a[1];
    ellipseBy3Points->setText(QApplication::translate("CmdSketcherCompCreateConic","Ellipse by periapsis, apoapsis, minor radius"));
    ellipseBy3Points->setToolTip(QApplication::translate("Sketcher_CreateEllipseBy3Points","Create a ellipse by periapsis, apoapsis, and minor radius"));
    ellipseBy3Points->setStatusTip(QApplication::translate("Sketcher_CreateEllipseBy3Points","Create a ellipse by periapsis, apoapsis, and minor radius"));
    QAction* arcofellipse = a[2];
    arcofellipse->setText(QApplication::translate("CmdSketcherCompCreateConic","Arc of ellipse by center, major radius, endpoints"));
    arcofellipse->setToolTip(QApplication::translate("Sketcher_CreateArcOfEllipse","Create an arc of ellipse by its center, major radius, and endpoints"));
    arcofellipse->setStatusTip(QApplication::translate("Sketcher_CreateArcOfEllipse","Create an arc of ellipse by its center, major radius, and endpoints"));
    QAction* arcofhyperbola = a[3];
    arcofhyperbola->setText(QApplication::translate("CmdSketcherCompCreateConic","Arc of hyperbola by center, major radius, endpoints"));
    arcofhyperbola->setToolTip(QApplication::translate("Sketcher_CreateArcOfHyperbola","Create an arc of hyperbola by its center, major radius, and endpoints"));
    arcofhyperbola->setStatusTip(QApplication::translate("Sketcher_CreateArcOfHyperbola","Create an arc of hyperbola by its center, major radius, and endpoints"));
    QAction* arcofparabola = a[4];
    arcofparabola->setText(QApplication::translate("CmdSketcherCompCreateConic","Arc of parabola by focus, vertex, endpoints"));
    arcofparabola->setToolTip(QApplication::translate("Sketcher_CreateArcOfParabola","Create an arc of parabola by its focus, vertex, and endpoints"));
    arcofparabola->setStatusTip(QApplication::translate("Sketcher_CreateArcOfParabola","Create an arc of parabola by its focus, vertex, and endpoints"));
}

bool CmdSketcherCompCreateConic::isActive(void)
{
    return isCreateGeoActive(getActiveGuiDocument());
}

// ======================================================================================

class DrawSketchHandlerBSpline: public DrawSketchHandler
{
public:
    DrawSketchHandlerBSpline(int constructionMethod)
      : Mode(STATUS_SEEK_FIRST_CONTROLPOINT)
      , EditCurve(2)
      , CurrentConstraint(0)
      , ConstrMethod(constructionMethod)
      , IsClosed(false)
      , FirstPoleGeoId(-2000)
    {
        std::vector<AutoConstraint> sugConstr1;
        sugConstr.push_back(sugConstr1);
    }

    virtual ~DrawSketchHandlerBSpline() {}
    /// modes
    enum SELECT_MODE {
        STATUS_SEEK_FIRST_CONTROLPOINT,
        STATUS_SEEK_ADDITIONAL_CONTROLPOINTS,
        STATUS_CLOSE
    };

    virtual void activated(ViewProviderSketch *)
    {
        setCrosshairCursor("Sketcher_Pointer_Create_BSpline");
    }

    virtual void mouseMove(Base::Vector2d onSketchPos)
    {
        if (Mode==STATUS_SEEK_FIRST_CONTROLPOINT) {
            setPositionText(onSketchPos);
            if (seekAutoConstraint(sugConstr[CurrentConstraint], onSketchPos, Base::Vector2d(0.f,0.f))) {
                renderSuggestConstraintsCursor(sugConstr[CurrentConstraint]);
                return;
            }
        }
        else if (Mode==STATUS_SEEK_ADDITIONAL_CONTROLPOINTS){

            EditCurve[EditCurve.size()-1] = onSketchPos;

            drawEdit(EditCurve);

            float length = (EditCurve[EditCurve.size()-1] - EditCurve[EditCurve.size()-2]).Length();
            float angle = (EditCurve[EditCurve.size()-1] - EditCurve[EditCurve.size()-2]).GetAngle(Base::Vector2d(1.f,0.f));

            SbString text;
            text.sprintf(" (%.1f,%.1fdeg)", length, angle * 180 / M_PI);
            setPositionText(EditCurve[EditCurve.size()-1], text);

            if (seekAutoConstraint(sugConstr[CurrentConstraint], onSketchPos, Base::Vector2d(0.f,0.f))) {
                renderSuggestConstraintsCursor(sugConstr[CurrentConstraint]);
                return;
            }

        }
        applyCursor();
    }

    virtual bool pressButton(Base::Vector2d onSketchPos)
    {
        if (Mode == STATUS_SEEK_FIRST_CONTROLPOINT) {

            EditCurve[0] = onSketchPos;

            Mode = STATUS_SEEK_ADDITIONAL_CONTROLPOINTS;

            // insert circle point for pole, defer internal alignment constraining.
            try {

                Gui::Command::openCommand(QT_TRANSLATE_NOOP("Command", "Add Pole circle"));

                //Add pole
                Gui::cmdAppObjectArgs(sketchgui->getObject(), "addGeometry(Part.Circle(App.Vector(%f,%f,0),App.Vector(0,0,1),10),True)",
                                      EditCurve[0].x,EditCurve[0].y);

            }
            catch (const Base::Exception& e) {
                Base::Console().Error("%s\n", e.what());
                Gui::Command::abortCommand();

                static_cast<Sketcher::SketchObject *>(sketchgui->getObject())->solve();

                return false;
            }

            //Gui::Command::commitCommand();

            //static_cast<Sketcher::SketchObject *>(sketchgui->getObject())->solve();

            FirstPoleGeoId = getHighestCurveIndex();

            // add auto constraints on pole
            if (sugConstr[CurrentConstraint].size() > 0) {
                createAutoConstraints(sugConstr[CurrentConstraint], FirstPoleGeoId, Sketcher::PointPos::mid, false);
            }

            static_cast<Sketcher::SketchObject *>(sketchgui->getObject())->solve();

            std::vector<AutoConstraint> sugConstrN;
            sugConstr.push_back(sugConstrN);
            CurrentConstraint++;

        }
        else if (Mode == STATUS_SEEK_ADDITIONAL_CONTROLPOINTS) {
            EditCurve[EditCurve.size()-1] = onSketchPos;

            // check if coincident with first pole
            for(std::vector<AutoConstraint>::const_iterator it = sugConstr[CurrentConstraint].begin(); it != sugConstr[CurrentConstraint].end(); it++) {
                if( (*it).Type == Sketcher::Coincident && (*it).GeoId == FirstPoleGeoId && (*it).PosId == Sketcher::PointPos::mid ) {

                    IsClosed = true;
                    }
            }

            if (IsClosed) {
                Mode = STATUS_CLOSE;

                if (ConstrMethod == 1) { // if periodic we do not need the last pole
                    EditCurve.pop_back();
                    sugConstr.pop_back();

                    return true;
                }


            }

            // insert circle point for pole, defer internal alignment constraining.
            try {

                //Gui::Command::openCommand(QT_TRANSLATE_NOOP("Command", "Add Pole circle"));

                //Add pole
                Gui::cmdAppObjectArgs(sketchgui->getObject(), "addGeometry(Part.Circle(App.Vector(%f,%f,0),App.Vector(0,0,1),10),True)",
                                      EditCurve[EditCurve.size()-1].x,EditCurve[EditCurve.size()-1].y);

                if(EditCurve.size() == 2) {
                    Gui::cmdAppObjectArgs(sketchgui->getObject(), "addConstraint(Sketcher.Constraint('Weight',%d,%f)) ",
                                          FirstPoleGeoId, 1.0 ); // First pole defaults to 1.0 weight
                }

                Gui::cmdAppObjectArgs(sketchgui->getObject(), "addConstraint(Sketcher.Constraint('Equal',%d,%d)) ",
                                      FirstPoleGeoId, FirstPoleGeoId+ EditCurve.size()-1);
            }
            catch (const Base::Exception& e) {
                Base::Console().Error("%s\n", e.what());
                Gui::Command::abortCommand();

                static_cast<Sketcher::SketchObject *>(sketchgui->getObject())->solve();

                return false;
            }

            //Gui::Command::commitCommand();

            //static_cast<Sketcher::SketchObject *>(sketchgui->getObject())->solve();

            // add auto constraints on pole
            if (sugConstr[CurrentConstraint].size() > 0) {
                createAutoConstraints(sugConstr[CurrentConstraint], FirstPoleGeoId + EditCurve.size()-1, Sketcher::PointPos::mid, false);
            }

            //static_cast<Sketcher::SketchObject *>(sketchgui->getObject())->solve();

            if (!IsClosed) {
                EditCurve.resize(EditCurve.size() + 1); // add one place for a pole
                std::vector<AutoConstraint> sugConstrN;
                sugConstr.push_back(sugConstrN);
                CurrentConstraint++;
            }

        }
        return true;
    }

    virtual bool releaseButton(Base::Vector2d /*onSketchPos*/)
    {
        if (Mode==STATUS_CLOSE) {
            unsetCursor();
            resetPositionText();

            std::stringstream stream;

            for (std::vector<Base::Vector2d>::const_iterator it=EditCurve.begin();
                it != EditCurve.end(); ++it) {
                stream << "App.Vector(" << (*it).x << "," << (*it).y << "),";
            }

            std::string controlpoints = stream.str();

            // remove last comma and add brackets
            int index = controlpoints.rfind(',');
            controlpoints.resize(index);

            controlpoints.insert(0,1,'[');
            controlpoints.append(1,']');

            int currentgeoid = getHighestCurveIndex();

            try {

                //Gui::Command::openCommand(QT_TRANSLATE_NOOP("Command", "Add B-spline curve"));

                /*Gui::cmdAppObjectArgs(sketchgui->getObject(), "addGeometry(Part.BSplineCurve"
                    "(%s,%s),"
                    "%s)",
                        controlpoints.c_str(),
                        ConstrMethod == 0 ?"False":"True",
                        geometryCreationMode==Construction?"True":"False"); */

                // {"poles", "mults", "knots", "periodic", "degree", "weights", "CheckRational", NULL};
                Gui::cmdAppObjectArgs(sketchgui->getObject(), "addGeometry(Part.BSplineCurve"
                                        "(%s,None,None,%s,3,None,False),%s)",
                                        controlpoints.c_str(),
                                        ConstrMethod == 0 ?"False":"True",
                                        geometryCreationMode==Construction?"True":"False");


                currentgeoid++;

                // autoconstraints were added to the circles of the poles, which is ok because they must go to the
                // right position, or the user will freak-out if they appear out of the autoconstrained position.
                // However, autoconstraints on the first and last pole, in normal non-periodic b-splines (with appropriate endpoint knot multiplicity)
                // as the ones created by this tool are intended for the b-spline endpoints, and not for the poles,
                // so here we retrieve any autoconstraint on those poles' center and mangle it to the endpoint.
                if (ConstrMethod == 0) {

                    for(auto & constr : static_cast<Sketcher::SketchObject *>(sketchgui->getObject())->Constraints.getValues()) {
                        if(constr->First == FirstPoleGeoId && constr->FirstPos == Sketcher::PointPos::mid) {
                            constr->First = currentgeoid;
                            constr->FirstPos = Sketcher::PointPos::start;
                        }
                        else if(constr->First == (FirstPoleGeoId + CurrentConstraint - 1) && constr->FirstPos == Sketcher::PointPos::mid) {
                            constr->First = currentgeoid;
                            constr->FirstPos = Sketcher::PointPos::end;
                        }
                    }
                }

                // Constraint pole circles to B-spline.
                std::stringstream cstream;

                cstream << "conList = []\n";

                for (size_t i = 0; i < EditCurve.size(); i++) {
                    cstream << "conList.append(Sketcher.Constraint('InternalAlignment:Sketcher::BSplineControlPoint'," << FirstPoleGeoId+i
                        << "," << static_cast<int>(Sketcher::PointPos::mid) << "," << currentgeoid << "," << i << "))\n";
                }

                cstream << Gui::Command::getObjectCmd(sketchgui->getObject()) << ".addConstraint(conList)\n";
                cstream << "del conList\n";

                Gui::Command::doCommand(Gui::Command::Doc, cstream.str().c_str());

                // for showing the knots on creation
                Gui::cmdAppObjectArgs(sketchgui->getObject(), "exposeInternalGeometry(%d)", currentgeoid);
            }
            catch (const Base::Exception& e) {
                Base::Console().Error("%s\n", e.what());
                Gui::Command::abortCommand();

                tryAutoRecomputeIfNotSolve(static_cast<Sketcher::SketchObject *>(sketchgui->getObject()));

                return false;
            }

            Gui::Command::commitCommand();

            tryAutoRecomputeIfNotSolve(static_cast<Sketcher::SketchObject *>(sketchgui->getObject()));

            ParameterGrp::handle hGrp = App::GetApplication().GetParameterGroupByPath("User parameter:BaseApp/Preferences/Mod/Sketcher");
            bool continuousMode = hGrp->GetBool("ContinuousCreationMode",true);

            if(continuousMode){
                // This code enables the continuous creation mode.
                Mode = STATUS_SEEK_FIRST_CONTROLPOINT;
                EditCurve.clear();
                drawEdit(EditCurve);
                EditCurve.resize(2);
                applyCursor();

                sugConstr.clear();

                std::vector<AutoConstraint> sugConstr1;
                sugConstr.push_back(sugConstr1);

                CurrentConstraint=0;
                IsClosed=false;

                /* It is ok not to call to purgeHandler
                 * in continuous creation mode because the
                 * handler is destroyed by the quit() method on pressing the
                 * right button of the mouse */
            }
            else{
                sketchgui->purgeHandler(); // no code after this line, Handler get deleted in ViewProvider
            }
        }
        return true;
    }

    virtual void quit(void) {
        // We must see if we need to create a B-spline before cancelling everything
        // and now just like any other Handler,

        ParameterGrp::handle hGrp = App::GetApplication().GetParameterGroupByPath("User parameter:BaseApp/Preferences/Mod/Sketcher");

        bool continuousMode = hGrp->GetBool("ContinuousCreationMode",true);

        if (CurrentConstraint > 1) {
            // create B-spline from existing poles
            Mode=STATUS_CLOSE;
            EditCurve.pop_back();
            this->releaseButton(Base::Vector2d(0.f,0.f));
        }
        else if(CurrentConstraint == 1) {
            // if we just have one point and we can not close anything, then cancel this creation but continue according to continuous mode
            //sketchgui->getDocument()->undo(1);

            Gui::Command::abortCommand();

            tryAutoRecomputeIfNotSolve(static_cast<Sketcher::SketchObject *>(sketchgui->getObject()));

            if(!continuousMode){
                DrawSketchHandler::quit();
            }
            else {
                // This code disregards existing data and enables the continuous creation mode.
                Mode = STATUS_SEEK_FIRST_CONTROLPOINT;
                EditCurve.clear();
                drawEdit(EditCurve);
                EditCurve.resize(2);
                applyCursor();

                sugConstr.clear();

                std::vector<AutoConstraint> sugConstr1;
                sugConstr.push_back(sugConstr1);

                CurrentConstraint=0;
                IsClosed=false;
            }
        }
        else { // we have no data (CurrentConstraint == 0) so user when right-clicking really wants to exit
            DrawSketchHandler::quit();
        }
    }

protected:
    SELECT_MODE Mode;

    std::vector<Base::Vector2d> EditCurve;

    std::vector<std::vector<AutoConstraint>> sugConstr;

    int CurrentConstraint;
    int ConstrMethod;
    bool IsClosed;
    int FirstPoleGeoId;
};

DEF_STD_CMD_A(CmdSketcherCreateBSpline)

CmdSketcherCreateBSpline::CmdSketcherCreateBSpline()
  : Command("Sketcher_CreateBSpline")
{
    sAppModule      = "Sketcher";
    sGroup          = "Sketcher";
    sMenuText       = QT_TR_NOOP("Create B-spline");
    sToolTipText    = QT_TR_NOOP("Create a B-spline via control points in the sketch.");
    sWhatsThis      = "Sketcher_CreateBSpline";
    sStatusTip      = sToolTipText;
    sPixmap         = "Sketcher_CreateBSpline";
    sAccel          = "G, B, B";
    eType           = ForEdit;
}

void CmdSketcherCreateBSpline::activated(int iMsg)
{
    Q_UNUSED(iMsg);
    ActivateHandler(getActiveGuiDocument(),new DrawSketchHandlerBSpline(0) );
}

/*void CmdSketcherCreateBSpline::updateAction(int mode)
{
    switch (mode) {
    case Normal:
        if (getAction())
            getAction()->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateBSpline"));
        break;
    case Construction:
        if (getAction())
            getAction()->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateBSpline_Constr"));
        break;
    }
}*/

bool CmdSketcherCreateBSpline::isActive(void)
{
    return isCreateGeoActive(getActiveGuiDocument());
}

/// @brief Macro that declares a new sketcher command class 'CmdSketcherCreateBSpline'
DEF_STD_CMD_A(CmdSketcherCreatePeriodicBSpline)

/**
 * @brief ctor
 */
CmdSketcherCreatePeriodicBSpline::CmdSketcherCreatePeriodicBSpline()
: Command("Sketcher_CreatePeriodicBSpline")
{
    sAppModule      = "Sketcher";
    sGroup          = "Sketcher";
    sMenuText       = QT_TR_NOOP("Create periodic B-spline");
    sToolTipText    = QT_TR_NOOP("Create a periodic B-spline via control points in the sketch.");
    sWhatsThis      = "Sketcher_CreatePeriodicBSpline";
    sStatusTip      = sToolTipText;
    sPixmap         = "Sketcher_Create_Periodic_BSpline";
    sAccel          = "G, B, P";
    eType           = ForEdit;
}

void CmdSketcherCreatePeriodicBSpline::activated(int iMsg)
{
    Q_UNUSED(iMsg);
    ActivateHandler(getActiveGuiDocument(),new DrawSketchHandlerBSpline(1) );
}

bool CmdSketcherCreatePeriodicBSpline::isActive(void)
{
    return isCreateGeoActive(getActiveGuiDocument());
}


/// @brief Macro that declares a new sketcher command class 'CmdSketcherCompCreateBSpline'
DEF_STD_CMD_ACLU(CmdSketcherCompCreateBSpline)

/**
 * @brief ctor
 */
CmdSketcherCompCreateBSpline::CmdSketcherCompCreateBSpline()
: Command("Sketcher_CompCreateBSpline")
{
    sAppModule      = "Sketcher";
    sGroup          = "Sketcher";
    sMenuText       = QT_TR_NOOP("Create a B-spline");
    sToolTipText    = QT_TR_NOOP("Create a B-spline in the sketch");
    sWhatsThis      = "Sketcher_CompCreateBSpline";
    sStatusTip      = sToolTipText;
    eType           = ForEdit;
}

/**
 * @brief Instantiates the B-spline handler when the B-spline command activated
 * @param int iMsg
 */
void CmdSketcherCompCreateBSpline::activated(int iMsg)
{
    if (iMsg == 0) {
        ActivateHandler(getActiveGuiDocument(), new DrawSketchHandlerBSpline(iMsg));
    } else if (iMsg == 1) {
        ActivateHandler(getActiveGuiDocument(), new DrawSketchHandlerBSpline(iMsg));
    } else {
        return;
    }

    // Since the default icon is reset when enabling/disabling the command we have
    // to explicitly set the icon of the used command.
    Gui::ActionGroup* pcAction = qobject_cast<Gui::ActionGroup*>(_pcAction);
    QList<QAction*> a = pcAction->actions();

    assert(iMsg < a.size());
    pcAction->setIcon(a[iMsg]->icon());
}

Gui::Action * CmdSketcherCompCreateBSpline::createAction(void)
{
    Gui::ActionGroup* pcAction = new Gui::ActionGroup(this, Gui::getMainWindow());
    pcAction->setDropDownMenu(true);
    applyCommandData(this->className(), pcAction);

    QAction* bspline = pcAction->addAction(QString());
    bspline->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateBSpline"));

    QAction* periodicbspline = pcAction->addAction(QString());
    periodicbspline->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_Create_Periodic_BSpline"));

    _pcAction = pcAction;
    languageChange();

    // default
    pcAction->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateBSpline"));
    int defaultId = 0;
    pcAction->setProperty("defaultAction", QVariant(defaultId));

    return pcAction;
}

void CmdSketcherCompCreateBSpline::updateAction(int mode)
{
    Gui::ActionGroup* pcAction = qobject_cast<Gui::ActionGroup*>(getAction());
    if (!pcAction)
        return;

    QList<QAction*> a = pcAction->actions();
    int index = pcAction->property("defaultAction").toInt();
    switch (mode) {
        case Normal:
            a[0]->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateBSpline"));
            a[1]->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_Create_Periodic_BSpline"));
            getAction()->setIcon(a[index]->icon());
            break;
        case Construction:
            a[0]->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateBSpline_Constr"));
            a[1]->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_Create_Periodic_BSpline_Constr"));
            getAction()->setIcon(a[index]->icon());
            break;
    }
}

void CmdSketcherCompCreateBSpline::languageChange()
{
    Command::languageChange();

    if (!_pcAction)
        return;
    Gui::ActionGroup* pcAction = qobject_cast<Gui::ActionGroup*>(_pcAction);
    QList<QAction*> a = pcAction->actions();

    QAction* bspline = a[0];
    bspline->setText(QApplication::translate("Sketcher_CreateBSpline","B-spline by control points"));
    bspline->setToolTip(QApplication::translate("Sketcher_CreateBSpline","Create a B-spline by control points"));
    bspline->setStatusTip(QApplication::translate("Sketcher_CreateBSpline","Create a B-spline by control points"));
    QAction* periodicbspline = a[1];
    periodicbspline->setText(QApplication::translate("Sketcher_Create_Periodic_BSpline","Periodic B-spline by control points"));
    periodicbspline->setToolTip(QApplication::translate("Sketcher_Create_Periodic_BSpline","Create a periodic B-spline by control points"));
    periodicbspline->setStatusTip(QApplication::translate("Sketcher_Create_Periodic_BSpline","Create a periodic B-spline by control points"));
}

bool CmdSketcherCompCreateBSpline::isActive(void)
{
    return isCreateGeoActive(getActiveGuiDocument());
}


// ======================================================================================

class DrawSketchHandler3PointCircle : public DrawSketchHandler
{
public:
    DrawSketchHandler3PointCircle()
      : Mode(STATUS_SEEK_First),EditCurve(2),radius(1),N(32.0){}
    virtual ~DrawSketchHandler3PointCircle(){
        sketchgui->toolSettings->widget->setSettings(0);
    }
    /// mode table
    enum SelectMode {
        STATUS_SEEK_First,      /**< enum value ----. */
        STATUS_SEEK_Second,     /**< enum value ----. */
        STATUS_SEEK_Third,      /**< enum value ----. */
        STATUS_End
    };

    virtual void activated(ViewProviderSketch *)
    {
        setCrosshairCursor("Sketcher_Pointer_Create_3PointCircle");
        sketchgui->toolSettings->widget->setSettings(5);
    }

    virtual void mouseMove(Base::Vector2d onSketchPos)
    {
        if (Mode == STATUS_SEEK_First) {
            setPositionText(onSketchPos);
            if (seekAutoConstraint(sugConstr1, onSketchPos, Base::Vector2d(0.f,0.f),
                                   AutoConstraint::CURVE)) {
                renderSuggestConstraintsCursor(sugConstr1);
                return;
            }
            if (sketchgui->toolSettings->widget->isSettingSet[0] + sketchgui->toolSettings->widget->isSettingSet[1] == 2) {
                pressButton(onSketchPos);
                releaseButton(onSketchPos);
            }
        }
        else if (Mode == STATUS_SEEK_Second) {
            try
            {
                if (sketchgui->toolSettings->widget->isSettingSet[2] == 1) {
                    SecondPoint.x = sketchgui->toolSettings->widget->toolParameters[2];
                }
                else {
                    SecondPoint.x = onSketchPos.x;
                }
                if (sketchgui->toolSettings->widget->isSettingSet[3] == 1) {
                    SecondPoint.y = sketchgui->toolSettings->widget->toolParameters[3];
                }
                else {
                    SecondPoint.y = onSketchPos.y;
                }
                CenterPoint = EditCurve[N + 1] = (SecondPoint - FirstPoint) / 2 + FirstPoint;
                                
                radius = (SecondPoint - CenterPoint).Length();
                double lineAngle = GetPointAngle(CenterPoint, onSketchPos);

                // Build a N point circle
                for (int i=1; i < N; i++) {
                    // Start at current angle
                    double angle = i*2*M_PI/N + lineAngle; // N point closed circle has N segments
                    EditCurve[i] = Base::Vector2d(CenterPoint.x + radius*cos(angle),
                                                CenterPoint.y + radius*sin(angle));
                }
                // Beginning and end of curve should be exact
                EditCurve[0] = EditCurve[N] = SecondPoint;

                // Display radius and start angle
                // This lineAngle will report counter-clockwise from +X, not relatively
                SbString text;
                text.sprintf(" (%.1fR,%.1fdeg)", (float) radius, (float) lineAngle * 180 / M_PI);
                setPositionText(onSketchPos, text);

                drawEdit(EditCurve);
                if (seekAutoConstraint(sugConstr2, SecondPoint, Base::Vector2d(0.f,0.f),
                                    AutoConstraint::CURVE)) {
                        renderSuggestConstraintsCursor(sugConstr2);
                        return;
                    }
                if (sketchgui->toolSettings->widget->isSettingSet[2] == 1 && sketchgui->toolSettings->widget->isSettingSet[3] == 1) {
                    pressButton(onSketchPos);
                    releaseButton(onSketchPos);
                }
            }
            catch(Base::ValueError &e) {
                e.ReportException();
            }
        }
        else if (Mode == STATUS_SEEK_Third) {
            try
            {
                CenterPoint = EditCurve[N + 1] = Part::Geom2dCircle::getCircleCenter(FirstPoint, SecondPoint, onSketchPos);
                
                radius = (onSketchPos - CenterPoint).Length();
                double lineAngle = GetPointAngle(CenterPoint, onSketchPos);

                // Build a N point circle
                for (int i = 1; i < N; i++) {
                    // Start at current angle
                    double angle = i * 2 * M_PI / N + lineAngle; // N point closed circle has N segments
                    EditCurve[i] = Base::Vector2d(CenterPoint.x + radius * cos(angle),
                        CenterPoint.y + radius * sin(angle));
                }
                // Beginning and end of curve should be exact
                EditCurve[0] = EditCurve[N] = onSketchPos;

                // Display radius and start angle
                // This lineAngle will report counter-clockwise from +X, not relatively
                SbString text;
                text.sprintf(" (%.1fR,%.1fdeg)", (float)radius, (float)lineAngle * 180 / M_PI);
                setPositionText(onSketchPos, text);

                drawEdit(EditCurve);
                
                if (seekAutoConstraint(sugConstr3, onSketchPos, Base::Vector2d(0.f, 0.f),
                    AutoConstraint::CURVE)) {
                    renderSuggestConstraintsCursor(sugConstr3);
                    return;
                }
            }
            catch (Base::ValueError& e) {
                e.ReportException();
            }
        }
        applyCursor();
    }

    virtual bool pressButton(Base::Vector2d onSketchPos)
    {
        if (Mode == STATUS_SEEK_First) {
            // N point curve + center + endpoint
            EditCurve.resize(N+2);

            if (sketchgui->toolSettings->widget->isSettingSet[0] == 1) {
                FirstPoint.x = sketchgui->toolSettings->widget->toolParameters[0];
            }
            else {
                FirstPoint.x = onSketchPos.x;
            }
            if (sketchgui->toolSettings->widget->isSettingSet[1] == 1) {
                FirstPoint.y = sketchgui->toolSettings->widget->toolParameters[1];
            }
            else {
                FirstPoint.y = onSketchPos.y;
            }

            sketchgui->toolSettings->widget->setParameterActive(0, 0);
            sketchgui->toolSettings->widget->setParameterActive(0, 1);
            sketchgui->toolSettings->widget->setParameterActive(1, 2);
            sketchgui->toolSettings->widget->setParameterActive(1, 3);
            sketchgui->toolSettings->widget->setParameterFocus(2);
            Mode = STATUS_SEEK_Second;
        }
        else if (Mode == STATUS_SEEK_Second) {
            if (sketchgui->toolSettings->widget->isSettingSet[2] == 1) {
                SecondPoint.x = sketchgui->toolSettings->widget->toolParameters[2];
            }
            else {
                SecondPoint.x = onSketchPos.x;
            }
            if (sketchgui->toolSettings->widget->isSettingSet[3] == 1) {
                SecondPoint.y = sketchgui->toolSettings->widget->toolParameters[3];
            }
            else {
                SecondPoint.y = onSketchPos.y;
            }

            sketchgui->toolSettings->widget->setParameterActive(0, 2);
            sketchgui->toolSettings->widget->setParameterActive(0, 3);
            Mode = STATUS_SEEK_Third;
        }
        else {
            EditCurve.resize(N);

            drawEdit(EditCurve);
            applyCursor();
            Mode = STATUS_End;
        }

        return true;
    }

    virtual bool releaseButton(Base::Vector2d onSketchPos)
    {
        Q_UNUSED(onSketchPos);
        // Need to look at.  rx might need fixing.
        if (Mode==STATUS_End) {
            unsetCursor();
            resetPositionText();

            try {
                Gui::Command::openCommand(QT_TRANSLATE_NOOP("Command", "Add sketch circle"));
                Gui::cmdAppObjectArgs(sketchgui->getObject(), "addGeometry(Part.Circle"
                    "(App.Vector(%f,%f,0),App.Vector(0,0,1),%f),%s)",
                          CenterPoint.x, CenterPoint.y,
                          radius,
                          geometryCreationMode==Construction?"True":"False");

                Gui::Command::commitCommand();
            }
            catch (const Base::Exception& e) {
                Base::Console().Error("Failed to add circle: %s\n", e.what());
                Gui::Command::abortCommand();
            }

            // Auto Constraint first picked point
            if (sugConstr1.size() > 0) {
                createAutoConstraints(sugConstr1, getHighestCurveIndex(), Sketcher::PointPos::none);
                sugConstr1.clear();
            }

            // Auto Constraint second picked point
            if (sugConstr2.size() > 0) {
                createAutoConstraints(sugConstr2, getHighestCurveIndex(), Sketcher::PointPos::none);
                sugConstr2.clear();
            }

            // Auto Constraint third picked point
            if (sugConstr3.size() > 0) {
                createAutoConstraints(sugConstr3, getHighestCurveIndex(), Sketcher::PointPos::none);
                sugConstr3.clear();
            }

            tryAutoRecomputeIfNotSolve(static_cast<Sketcher::SketchObject *>(sketchgui->getObject()));

            sketchgui->toolSettings->widget->setSettings(0);
            ParameterGrp::handle hGrp = App::GetApplication().GetParameterGroupByPath("User parameter:BaseApp/Preferences/Mod/Sketcher");
            bool continuousMode = hGrp->GetBool("ContinuousCreationMode",true);
            if(continuousMode){
                // This code enables the continuous creation mode.
                sketchgui->toolSettings->widget->setSettings(5);
                Mode=STATUS_SEEK_First;
                EditCurve.clear();
                drawEdit(EditCurve);
                EditCurve.resize(2);
                applyCursor();
                /* this is ok not to call to purgeHandler
                * in continuous creation mode because the
                * handler is destroyed by the quit() method on pressing the
                * right button of the mouse */
            }
            else{
                sketchgui->purgeHandler(); // no code after this line, Handler get deleted in ViewProvider
            }
        }
        return true;
    }
protected:
    SelectMode Mode;
    std::vector<Base::Vector2d> EditCurve;
    Base::Vector2d CenterPoint, FirstPoint, SecondPoint;
    double radius, N; // N should be even
    std::vector<AutoConstraint> sugConstr1, sugConstr2, sugConstr3;
};

DEF_STD_CMD_A(CmdSketcherCreate3PointCircle)

CmdSketcherCreate3PointCircle::CmdSketcherCreate3PointCircle()
  : Command("Sketcher_Create3PointCircle")
{
    sAppModule      = "Sketcher";
    sGroup          = "Sketcher";
    sMenuText       = QT_TR_NOOP("Create circle by three points");
    sToolTipText    = QT_TR_NOOP("Create a circle by 3 perimeter points");
    sWhatsThis      = "Sketcher_Create3PointCircle";
    sStatusTip      = sToolTipText;
    sPixmap         = "Sketcher_Create3PointCircle";
    sAccel          = "G, 3, C";
    eType           = ForEdit;
}

void CmdSketcherCreate3PointCircle::activated(int iMsg)
{
    Q_UNUSED(iMsg);
    ActivateHandler(getActiveGuiDocument(),new DrawSketchHandler3PointCircle() );
}

bool CmdSketcherCreate3PointCircle::isActive(void)
{
    return isCreateGeoActive(getActiveGuiDocument());
}


DEF_STD_CMD_ACLU(CmdSketcherCompCreateCircle)

CmdSketcherCompCreateCircle::CmdSketcherCompCreateCircle()
  : Command("Sketcher_CompCreateCircle")
{
    sAppModule      = "Sketcher";
    sGroup          = "Sketcher";
    sMenuText       = QT_TR_NOOP("Create circle");
    sToolTipText    = QT_TR_NOOP("Create a circle in the sketcher");
    sWhatsThis      = "Sketcher_CompCreateCircle";
    sStatusTip      = sToolTipText;
    eType           = ForEdit;
}

void CmdSketcherCompCreateCircle::activated(int iMsg)
{
    if (iMsg==0)
        ActivateHandler(getActiveGuiDocument(),new DrawSketchHandlerCircle());
    else if (iMsg==1)
        ActivateHandler(getActiveGuiDocument(),new DrawSketchHandler3PointCircle());
    else
        return;

    // Since the default icon is reset when enabling/disabling the command we have
    // to explicitly set the icon of the used command.
    Gui::ActionGroup* pcAction = qobject_cast<Gui::ActionGroup*>(_pcAction);
    QList<QAction*> a = pcAction->actions();

    assert(iMsg < a.size());
    pcAction->setIcon(a[iMsg]->icon());
}

Gui::Action * CmdSketcherCompCreateCircle::createAction(void)
{
    Gui::ActionGroup* pcAction = new Gui::ActionGroup(this, Gui::getMainWindow());
    pcAction->setDropDownMenu(true);
    applyCommandData(this->className(), pcAction);

    QAction* arc1 = pcAction->addAction(QString());
    arc1->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateCircle"));
    QAction* arc2 = pcAction->addAction(QString());
    arc2->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_Create3PointCircle"));

    _pcAction = pcAction;
    languageChange();

    pcAction->setIcon(arc1->icon());
    int defaultId = 0;
    pcAction->setProperty("defaultAction", QVariant(defaultId));

    return pcAction;
}

void CmdSketcherCompCreateCircle::updateAction(int mode)
{
    Gui::ActionGroup* pcAction = qobject_cast<Gui::ActionGroup*>(getAction());
    if (!pcAction)
        return;

    QList<QAction*> a = pcAction->actions();
    int index = pcAction->property("defaultAction").toInt();
    switch (mode) {
    case Normal:
        a[0]->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateCircle"));
        a[1]->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_Create3PointCircle"));
        getAction()->setIcon(a[index]->icon());
        break;
    case Construction:
        a[0]->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateCircle_Constr"));
        a[1]->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_Create3PointCircle_Constr"));
        getAction()->setIcon(a[index]->icon());
        break;
    }
}

void CmdSketcherCompCreateCircle::languageChange()
{
    Command::languageChange();

    if (!_pcAction)
        return;
    Gui::ActionGroup* pcAction = qobject_cast<Gui::ActionGroup*>(_pcAction);
    QList<QAction*> a = pcAction->actions();

    QAction* arc1 = a[0];
    arc1->setText(QApplication::translate("CmdSketcherCompCreateCircle", "Center and rim point"));
    arc1->setToolTip(QApplication::translate("Sketcher_CreateCircle", "Create a circle by its center and by a rim point"));
    arc1->setStatusTip(QApplication::translate("Sketcher_CreateCircle", "Create a circle by its center and by a rim point"));
    QAction* arc2 = a[1];
    arc2->setText(QApplication::translate("CmdSketcherCompCreateCircle", "3 rim points"));
    arc2->setToolTip(QApplication::translate("Sketcher_Create3PointCircle", "Create a circle by 3 rim points"));
    arc2->setStatusTip(QApplication::translate("Sketcher_Create3PointCircle", "Create a circle by 3 rim points"));
}

bool CmdSketcherCompCreateCircle::isActive(void)
{
    return isCreateGeoActive(getActiveGuiDocument());
}


// ======================================================================================

class DrawSketchHandlerPoint: public DrawSketchHandler
{
public:
    DrawSketchHandlerPoint() : selectionDone(false) {}
    virtual ~DrawSketchHandlerPoint() {
        sketchgui->toolSettings->widget->setSettings(0);
    }

    virtual void activated(ViewProviderSketch *)
    {
        setCrosshairCursor("Sketcher_Pointer_Create_Point");
        sketchgui->toolSettings->widget->setSettings(4);
    }

    virtual void mouseMove(Base::Vector2d onSketchPos)
    {
        setPositionText(onSketchPos);
        if (seekAutoConstraint(sugConstr, onSketchPos, Base::Vector2d(0.f,0.f))) {
            renderSuggestConstraintsCursor(sugConstr);
            return;
        }
        if (sketchgui->toolSettings->widget->isSettingSet[0] + sketchgui->toolSettings->widget->isSettingSet[1] == 2) {
            pressButton(onSketchPos);
            releaseButton(onSketchPos);
        }
        applyCursor();
    }

    virtual bool pressButton(Base::Vector2d onSketchPos)
    {
        if (sketchgui->toolSettings->widget->isSettingSet[0] == 1) {
            EditPoint.x = sketchgui->toolSettings->widget->toolParameters[0];
        }
        else {
            EditPoint.x = onSketchPos.x;
        }
        if (sketchgui->toolSettings->widget->isSettingSet[1] == 1) {
            EditPoint.y = sketchgui->toolSettings->widget->toolParameters[1];
        }
        else {
            EditPoint.y = onSketchPos.y;
        }
        selectionDone = true;
        return true;
    }

    virtual bool releaseButton(Base::Vector2d onSketchPos)
    {
        Q_UNUSED(onSketchPos);
        if (selectionDone){
            unsetCursor();
            resetPositionText();

            int firstCurve = getHighestCurveIndex() + 1;
            try {
                Gui::Command::openCommand(QT_TRANSLATE_NOOP("Command", "Add sketch point"));
                Gui::cmdAppObjectArgs(sketchgui->getObject(), "addGeometry(Part.Point(App.Vector(%f,%f,0)))",
                          EditPoint.x,EditPoint.y);

                //add constraint if user typed in some dimensions in tool widget
                if (sketchgui->toolSettings->widget->isSettingSet[0] + sketchgui->toolSettings->widget->isSettingSet[1] != 0) {
                    if (sketchgui->toolSettings->widget->isSettingSet[0] == 1) {
                        distanceXYorPointOnObject(0, firstCurve, Sketcher::PointPos::start, sketchgui->toolSettings->widget->toolParameters[0]);
                    }
                    if (sketchgui->toolSettings->widget->isSettingSet[1] == 1) {
                        distanceXYorPointOnObject(1, firstCurve, Sketcher::PointPos::start, sketchgui->toolSettings->widget->toolParameters[1]);
                    }
                }

                Gui::Command::commitCommand();
            }
            catch (const Base::Exception& e) {
                Base::Console().Error("Failed to add point: %s\n", e.what());
                Gui::Command::abortCommand();
            }

            // add auto constraints for the line segment start
            if (sugConstr.size() > 0) {
                createAutoConstraints(sugConstr, getHighestCurveIndex(), Sketcher::PointPos::start);
                sugConstr.clear();
            }

            tryAutoRecomputeIfNotSolve(static_cast<Sketcher::SketchObject *>(sketchgui->getObject()));

            sketchgui->toolSettings->widget->setSettings(0);
            ParameterGrp::handle hGrp = App::GetApplication().GetParameterGroupByPath("User parameter:BaseApp/Preferences/Mod/Sketcher");
            bool continuousMode = hGrp->GetBool("ContinuousCreationMode",true);
            if(continuousMode){
                // This code enables the continuous creation mode.
                sketchgui->toolSettings->widget->setSettings(4);
                applyCursor();
                /* It is ok not to call to purgeHandler
                * in continuous creation mode because the
                * handler is destroyed by the quit() method on pressing the
                * right button of the mouse */
            }
            else{
                sketchgui->purgeHandler(); // no code after this line, Handler get deleted in ViewProvider
            }
        }
        return true;
    }

protected:
    bool selectionDone;
    Base::Vector2d EditPoint;
    std::vector<AutoConstraint> sugConstr;
};

DEF_STD_CMD_A(CmdSketcherCreatePoint)

CmdSketcherCreatePoint::CmdSketcherCreatePoint()
  : Command("Sketcher_CreatePoint")
{
    sAppModule      = "Sketcher";
    sGroup          = "Sketcher";
    sMenuText       = QT_TR_NOOP("Create point");
    sToolTipText    = QT_TR_NOOP("Create a point in the sketch");
    sWhatsThis      = "Sketcher_CreatePoint";
    sStatusTip      = sToolTipText;
    sPixmap         = "Sketcher_CreatePoint";
    sAccel          = "G, Y";
    eType           = ForEdit;
}

void CmdSketcherCreatePoint::activated(int iMsg)
{
    Q_UNUSED(iMsg);
    ActivateHandler(getActiveGuiDocument(), new DrawSketchHandlerPoint());
}

bool CmdSketcherCreatePoint::isActive(void)
{
    return isCreateGeoActive(getActiveGuiDocument());
}


// ======================================================================================

DEF_STD_CMD_A(CmdSketcherCreateText)

CmdSketcherCreateText::CmdSketcherCreateText()
  : Command("Sketcher_CreateText")
{
    sAppModule      = "Sketcher";
    sGroup          = "Sketcher";
    sMenuText       = QT_TR_NOOP("Create text");
    sToolTipText    = QT_TR_NOOP("Create text in the sketch");
    sWhatsThis      = "Sketcher_CreateText";
    sStatusTip      = sToolTipText;
    sPixmap         = "Sketcher_CreateText";
    eType           = ForEdit;
}

void CmdSketcherCreateText::activated(int iMsg)
{
    Q_UNUSED(iMsg);
}

bool CmdSketcherCreateText::isActive(void)
{
    return false;
}


// ======================================================================================

DEF_STD_CMD_A(CmdSketcherCreateDraftLine)

CmdSketcherCreateDraftLine::CmdSketcherCreateDraftLine()
  : Command("Sketcher_CreateDraftLine")
{
    sAppModule      = "Sketcher";
    sGroup          = "Sketcher";
    sMenuText       = QT_TR_NOOP("Create draft line");
    sToolTipText    = QT_TR_NOOP("Create a draft line in the sketch");
    sWhatsThis      = "Sketcher_CreateDraftLine";
    sStatusTip      = sToolTipText;
    sPixmap         = "Sketcher_DraftLine";
    eType           = ForEdit;
}

void CmdSketcherCreateDraftLine::activated(int iMsg)
{
    Q_UNUSED(iMsg);
}

bool CmdSketcherCreateDraftLine::isActive(void)
{
    return false;
}

// Fillet and Chamfer ===================================================================

namespace SketcherGui {
    class FilletSelection : public Gui::SelectionFilterGate
    {
        App::DocumentObject* object;
    public:
        FilletSelection(App::DocumentObject* obj)
            : Gui::SelectionFilterGate((Gui::SelectionFilter*)0), object(obj)
        {}

        bool allow(App::Document * /*pDoc*/, App::DocumentObject *pObj, const char *sSubName)
        {
            if (pObj != this->object)
                return false;
            if (!sSubName || sSubName[0] == '\0')
                return false;
            std::string element(sSubName);
            if (element.substr(0,4) == "Edge") {
                int GeoId = std::atoi(element.substr(4,4000).c_str()) - 1;
                Sketcher::SketchObject *Sketch = static_cast<Sketcher::SketchObject*>(object);
                const Part::Geometry *geom = Sketch->getGeometry(GeoId);
                if (geom->getTypeId().isDerivedFrom(Part::GeomBoundedCurve::getClassTypeId()))
                    return true;
            }
            if (element.substr(0,6) == "Vertex") {
                int VtId = std::atoi(element.substr(6,4000).c_str()) - 1;
                Sketcher::SketchObject *Sketch = static_cast<Sketcher::SketchObject*>(object);
                std::vector<int> GeoIdList;
                std::vector<Sketcher::PointPos> PosIdList;
                Sketch->getDirectlyCoincidentPoints(VtId, GeoIdList, PosIdList);
                if (GeoIdList.size() == 2 && GeoIdList[0] >= 0  && GeoIdList[1] >= 0) {
                    const Part::Geometry *geom1 = Sketch->getGeometry(GeoIdList[0]);
                    const Part::Geometry *geom2 = Sketch->getGeometry(GeoIdList[1]);
                    if (geom1->getTypeId() == Part::GeomLineSegment::getClassTypeId() &&
                        geom2->getTypeId() == Part::GeomLineSegment::getClassTypeId())
                        return true;
                }
            }
            return  false;
        }
    };
}

class DrawSketchHandlerFillet: public DrawSketchHandler
{
public:

    enum FilletType {
        Fillet,
        Chamfer
    };

    DrawSketchHandlerFillet(FilletType filletType) :filletType(filletType), Mode(STATUS_SEEK_First), firstCurve(0) {}
    virtual ~DrawSketchHandlerFillet()
    {
        Gui::Selection().rmvSelectionGate();
    }

    enum SelectMode{
        STATUS_SEEK_First,
        STATUS_SEEK_Second
    };

    virtual void activated(ViewProviderSketch *)
    {
        sketchgui->toolSettings->widget->setSettings(11);
        if (filletType == Chamfer) {
            sketchgui->toolSettings->widget->setParameterVisible(1, 1);
        }
        Gui::Selection().rmvSelectionGate();
        Gui::Selection().addSelectionGate(new FilletSelection(sketchgui->getObject()));
        setCrosshairCursor("Sketcher_Pointer_Create_Fillet");
    }
    virtual void deactivated(ViewProviderSketch*)
    {
        sketchgui->toolSettings->widget->setSettings(0);
    }

    virtual void registerPressedKey(bool pressed, int key)
    {
        if ((key == SoKeyboardEvent::RIGHT_SHIFT || key == SoKeyboardEvent::LEFT_SHIFT) && pressed) {
            //Modifier key should switch between the 2 modes : Keep corner point or not.
        }
    }
    virtual void mouseMove(Base::Vector2d onSketchPos)
    {
        Q_UNUSED(onSketchPos);
    }

    virtual bool pressButton(Base::Vector2d onSketchPos)
    {
        Q_UNUSED(onSketchPos);
        return true;
    }

    virtual bool releaseButton(Base::Vector2d onSketchPos)
    {
        bool construction=false;
        //Case 1 : User selected a point. In this case the fillet will be made at this point (if there are two lines intersecting)
        int VtId = getPreselectPoint();
        if (Mode == STATUS_SEEK_First && VtId != -1) {
            int GeoId;
            Sketcher::PointPos PosId=Sketcher::PointPos::none;
            sketchgui->getSketchObject()->getGeoVertexIndex(VtId,GeoId,PosId);
            const Part::Geometry *geom = sketchgui->getSketchObject()->getGeometry(GeoId);
            if (geom->getTypeId() == Part::GeomLineSegment::getClassTypeId() &&
                (PosId == Sketcher::PointPos::start || PosId == Sketcher::PointPos::end)) {

                double radius=-1;
                if (sketchgui->toolSettings->widget->isSettingSet[0]) {
                    radius = sketchgui->toolSettings->widget->toolParameters[0];
                }
                else {
                    // guess fillet radius
                    std::vector<int> GeoIdList;
                    std::vector<Sketcher::PointPos> PosIdList;
                    sketchgui->getSketchObject()->getDirectlyCoincidentPoints(GeoId, PosId, GeoIdList, PosIdList);
                    if (GeoIdList.size() == 2 && GeoIdList[0] >= 0 && GeoIdList[1] >= 0) {
                        const Part::Geometry* geom1 = sketchgui->getSketchObject()->getGeometry(GeoIdList[0]);
                        const Part::Geometry* geom2 = sketchgui->getSketchObject()->getGeometry(GeoIdList[1]);
                        construction = Sketcher::GeometryFacade::getConstruction(geom1) && Sketcher::GeometryFacade::getConstruction(geom2);
                        if (geom1->getTypeId() == Part::GeomLineSegment::getClassTypeId() &&
                            geom2->getTypeId() == Part::GeomLineSegment::getClassTypeId()) {
                            const Part::GeomLineSegment* lineSeg1 = static_cast<const Part::GeomLineSegment*>(geom1);
                            const Part::GeomLineSegment* lineSeg2 = static_cast<const Part::GeomLineSegment*>(geom2);
                            Base::Vector3d dir1 = lineSeg1->getEndPoint() - lineSeg1->getStartPoint();
                            Base::Vector3d dir2 = lineSeg2->getEndPoint() - lineSeg2->getStartPoint();
                            if (PosIdList[0] == Sketcher::PointPos::end)
                                dir1 *= -1;
                            if (PosIdList[1] == Sketcher::PointPos::end)
                                dir2 *= -1;
                            double l1 = dir1.Length();
                            double l2 = dir2.Length();
                            double angle = dir1.GetAngle(dir2);
                            radius = (l1 < l2 ? l1 : l2) * 0.2 * sin(angle / 2);
                        }
                    }

                }
                if (radius < 0)
                    return false;

                int currentgeoid= getHighestCurveIndex();
                // create fillet at point
                try {
                    //nofAngles add support for chamfer and poly-chamfer and inward-poly-chamfer and inward-fillet. 1 is normal fillet
                    //-1 is inward fillet, 2 and -2 are chamfer, 3 is a two edge chamfer, -3 is two edge inward chamfer and so on.
                    int nofAngles = 1;
                    if (filletType == Chamfer) {
                        nofAngles = 2;
                        if (sketchgui->toolSettings->widget->isSettingSet[1]) {
                            nofAngles = max(static_cast <int>(floor(sketchgui->toolSettings->widget->toolParameters[1])) + 1, 2);
                        }
                    }
                    if (sketchgui->toolSettings->widget->isCheckBoxChecked(2)) {
                        nofAngles = -nofAngles;
                    }
                    bool pointFillet = sketchgui->toolSettings->widget->isCheckBoxChecked(1);
                    Gui::Command::openCommand(QT_TRANSLATE_NOOP("Command", "Create fillet"));
                    Gui::cmdAppObjectArgs(sketchgui->getObject(), "fillet(%d,%d,%f,%s,%s,%d)", GeoId, static_cast<int>(PosId), radius, "True",
                        pointFillet ? "True":"False", nofAngles);

                    if (construction) {
                        Gui::cmdAppObjectArgs(sketchgui->getObject(), "toggleConstruction(%d) ", currentgeoid+1);
                    }

                    //add constraint if user typed in some dimensions in tool widget
                    if (sketchgui->toolSettings->widget->isSettingSet[0]) {
                        if (sketchgui->toolSettings->widget->isSettingSet[0] == 1) {
                            Gui::cmdAppObjectArgs(sketchgui->getObject(), "addConstraint(Sketcher.Constraint('Radius',%d,%f)) ",
                                currentgeoid + 1, radius);
                        }
                    }

                    Gui::Command::commitCommand();
                }
                catch (const Base::Exception& e) {
                    Base::Console().Error("Failed to create fillet: %s\n", e.what());
                    Gui::Command::abortCommand();
                }

                tryAutoRecomputeIfNotSolve(static_cast<Sketcher::SketchObject *>(sketchgui->getObject()));
                sketchgui->toolSettings->widget->setParameterFocus(0);
            }
            return true;
        }

        //Case 2 : User selected a curve. Then the fillet will be made between this curve and next selected curve
        int GeoId = getPreselectCurve();
        if (GeoId > -1) {
            const Part::Geometry *geom = sketchgui->getSketchObject()->getGeometry(GeoId);
            if (geom->getTypeId().isDerivedFrom(Part::GeomBoundedCurve::getClassTypeId())) {
                if (Mode==STATUS_SEEK_First) {
                    firstCurve = GeoId;
                    firstPos = onSketchPos;
                    Mode = STATUS_SEEK_Second;
                    // add the line to the selection
                    std::stringstream ss;
                    ss << "Edge" << firstCurve + 1;
                    Gui::Selection().addSelection(sketchgui->getSketchObject()->getDocument()->getName()
                                                 ,sketchgui->getSketchObject()->getNameInDocument()
                                                 ,ss.str().c_str()
                                                 ,onSketchPos.x
                                                 ,onSketchPos.y
                                                 ,0.f);
                }
                else if (Mode==STATUS_SEEK_Second) {
                    int secondCurve = GeoId;
                    Base::Vector2d secondPos = onSketchPos;

                    Base::Vector3d refPnt1(firstPos.x, firstPos.y, 0.f);
                    Base::Vector3d refPnt2(secondPos.x, secondPos.y, 0.f);

                    const Part::Geometry *geom1 = sketchgui->getSketchObject()->getGeometry(firstCurve);

                    double radius = 0;

                    if( geom->getTypeId() == Part::GeomLineSegment::getClassTypeId() &&
                        geom1->getTypeId() == Part::GeomLineSegment::getClassTypeId()) {
                        // guess fillet radius
                        const Part::GeomLineSegment *lineSeg1 = static_cast<const Part::GeomLineSegment *>
                                                                (sketchgui->getSketchObject()->getGeometry(firstCurve));
                        const Part::GeomLineSegment *lineSeg2 = static_cast<const Part::GeomLineSegment *>
                                                                (sketchgui->getSketchObject()->getGeometry(secondCurve));

                        if (sketchgui->toolSettings->widget->isSettingSet[0]) {
                            radius = sketchgui->toolSettings->widget->toolParameters[0];
                        }
                        else {
                            radius = Part::suggestFilletRadius(lineSeg1, lineSeg2, refPnt1, refPnt2);
                        }
                        if (radius < 0)
                            return false;

                        construction=Sketcher::GeometryFacade::getConstruction(lineSeg1) && Sketcher::GeometryFacade::getConstruction(lineSeg2);
                    }
                    else { // other supported curves
                        const Part::Geometry *geo1 = static_cast<const Part::Geometry *>
                                                                (sketchgui->getSketchObject()->getGeometry(firstCurve));
                        const Part::Geometry *geo2 = static_cast<const Part::Geometry *>
                                                                (sketchgui->getSketchObject()->getGeometry(secondCurve));

                        construction=Sketcher::GeometryFacade::getConstruction(geo1) && Sketcher::GeometryFacade::getConstruction(geo2);
                    }


                    int currentgeoid= getHighestCurveIndex();

                    // create fillet between lines
                    try {
                        int nofAngles = 1;
                        if (filletType == Chamfer) {
                            nofAngles = 2;
                            if (sketchgui->toolSettings->widget->isSettingSet[1]) {
                                nofAngles = static_cast <int>(floor(sketchgui->toolSettings->widget->toolParameters[1])) + 1;
                            }
                        }
                        if (sketchgui->toolSettings->widget->isCheckBoxChecked(2)) {
                            nofAngles = -nofAngles;
                        }
                        bool pointFillet = sketchgui->toolSettings->widget->isCheckBoxChecked(1);
                        Gui::Command::openCommand(QT_TRANSLATE_NOOP("Command", "Create fillet"));
                        Gui::cmdAppObjectArgs(sketchgui->getObject(), "fillet(%d,%d,App.Vector(%f,%f,0),App.Vector(%f,%f,0),%f,%s,%s,%d)",
                                  firstCurve, secondCurve,
                                  firstPos.x, firstPos.y,
                                  secondPos.x, secondPos.y, radius,
                                  "True", pointFillet ? "True":"False", nofAngles);

                        //Set the fillet as construction if the selected lines were construction lines.
                        if (construction) {
                            Gui::cmdAppObjectArgs(sketchgui->getObject(), "toggleConstruction(%d) ",
                                currentgeoid + 1);
                        }

                        //add constraint if user typed in some dimensions in tool widget
                        if (sketchgui->toolSettings->widget->isSettingSet[0] + sketchgui->toolSettings->widget->isSettingSet[1] + sketchgui->toolSettings->widget->isSettingSet[2]) {
                            if (sketchgui->toolSettings->widget->isSettingSet[0] == 1 && radius > 0 ) {
                                Gui::cmdAppObjectArgs(sketchgui->getObject(), "addConstraint(Sketcher.Constraint('Radius',%d,%f)) ",
                                    currentgeoid + 1, radius);
                            }
                        }

                        Gui::Command::commitCommand();
                    }
                    catch (const Base::CADKernelError& e) {
                        e.ReportException();
                        if(e.getTranslatable()) {
                            QMessageBox::warning(Gui::getMainWindow(), QObject::tr("CAD Kernel Error"),
                                                QObject::tr(e.getMessage().c_str()));
                        }
                        Gui::Selection().clearSelection();
                        Gui::Command::abortCommand();
                        Mode = STATUS_SEEK_First;
                    }
                    catch (const Base::ValueError& e) {
                        e.ReportException();
                        Gui::Selection().clearSelection();
                        Gui::Command::abortCommand();
                        Mode = STATUS_SEEK_First;
                    }

                    tryAutoRecompute(static_cast<Sketcher::SketchObject *>(sketchgui->getObject()));
                    sketchgui->toolSettings->widget->setParameterFocus(0);
                    Gui::Selection().clearSelection();
                    Mode = STATUS_SEEK_First;
                }
            }
        }

        if (VtId < 0 && GeoId < 0) // exit the fillet tool if the user clicked on empty space
            sketchgui->purgeHandler(); // no code after this line, Handler get deleted in ViewProvider

        return true;
    }

protected:
    int filletType;
    SelectMode Mode;
    int firstCurve;
    Base::Vector2d firstPos;
};

DEF_STD_CMD_A(CmdSketcherCreateFillet)

CmdSketcherCreateFillet::CmdSketcherCreateFillet()
  : Command("Sketcher_CreateFillet")
{
    sAppModule      = "Sketcher";
    sGroup          = "Sketcher";
    sMenuText       = QT_TR_NOOP("Create fillet");
    sToolTipText    = QT_TR_NOOP("Create a fillet between two lines or at a coincident point");
    sWhatsThis      = "Sketcher_CreateFillet";
    sStatusTip      = sToolTipText;
    sPixmap         = "Sketcher_CreateFillet";
    sAccel          = "G, F, F";
    eType           = ForEdit;
}

void CmdSketcherCreateFillet::activated(int iMsg)
{
    Q_UNUSED(iMsg);
    ActivateHandler(getActiveGuiDocument(), new DrawSketchHandlerFillet(DrawSketchHandlerFillet::Fillet));
}

bool CmdSketcherCreateFillet::isActive(void)
{
    return isCreateGeoActive(getActiveGuiDocument());
}

// Chamfer ===============================================================================

DEF_STD_CMD_A(CmdSketcherCreateChamfer)

CmdSketcherCreateChamfer::CmdSketcherCreateChamfer()
    : Command("Sketcher_CreateChamfer")
{
    sAppModule = "Sketcher";
    sGroup = "Sketcher";
    sMenuText = QT_TR_NOOP("Create chamfer");
    sToolTipText = QT_TR_NOOP("Create a chamfer between two lines or at a coincident point");
    sWhatsThis = "Sketcher_CreateChamfer";
    sStatusTip = sToolTipText;
    sPixmap = "Sketcher_CreateChamfer";
    sAccel = "";
    eType = ForEdit;
}

void CmdSketcherCreateChamfer::activated(int iMsg)
{
    Q_UNUSED(iMsg);
    ActivateHandler(getActiveGuiDocument(), new DrawSketchHandlerFillet(DrawSketchHandlerFillet::Chamfer));
}

bool CmdSketcherCreateChamfer::isActive(void)
{
    return isCreateGeoActive(getActiveGuiDocument());
}

/// @brief Macro that declares a new sketcher command class 'CmdSketcherCompCreateFillets'
DEF_STD_CMD_ACLU(CmdSketcherCompCreateFillets)

/**
 * @brief ctor
 */
CmdSketcherCompCreateFillets::CmdSketcherCompCreateFillets()
  : Command("Sketcher_CompCreateFillets")
{
    sAppModule      = "Sketcher";
    sGroup          = "Sketcher";
    sMenuText       = QT_TR_NOOP("Fillets");
    sToolTipText    = QT_TR_NOOP("Create a fillet between two lines");
    sWhatsThis      = "Sketcher_CompCreateFillets";
    sStatusTip      = sToolTipText;
    eType           = ForEdit;
}

/**
 * @brief Instantiates the fillet handler when the fillet command activated
 * @param int iMsg
 */
void CmdSketcherCompCreateFillets::activated(int iMsg)
{
    switch (iMsg) {
    case 0: //fillet tool
        ActivateHandler(getActiveGuiDocument(), new DrawSketchHandlerFillet(DrawSketchHandlerFillet::Fillet)); break;
    case 1: //fillet with point tool
        ActivateHandler(getActiveGuiDocument(), new DrawSketchHandlerFillet(DrawSketchHandlerFillet::Chamfer)); break;
    default:
        return;
    }

    // Since the default icon is reset when enabling/disabling the command we have
    // to explicitly set the icon of the used command.
    Gui::ActionGroup* pcAction = qobject_cast<Gui::ActionGroup*>(_pcAction);
    QList<QAction*> a = pcAction->actions();

    assert(iMsg < a.size());
    pcAction->setIcon(a[iMsg]->icon());
}

Gui::Action * CmdSketcherCompCreateFillets::createAction(void)
{
    Gui::ActionGroup* pcAction = new Gui::ActionGroup(this, Gui::getMainWindow());
    pcAction->setDropDownMenu(true);
    applyCommandData(this->className(), pcAction);

    QAction* filletAction = pcAction->addAction(QString());
    filletAction->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateFillet"));

    QAction* chamferAction = pcAction->addAction(QString());
    chamferAction->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateChamfer"));

    _pcAction = pcAction;
    languageChange();

    pcAction->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateFillet"));
    int defaultId = 0;
    pcAction->setProperty("defaultAction", QVariant(defaultId));

    return pcAction;
}

void CmdSketcherCompCreateFillets::updateAction(int mode)
{
    Q_UNUSED(mode);
    Gui::ActionGroup* pcAction = qobject_cast<Gui::ActionGroup*>(getAction());
    if (!pcAction)
        return;

    QList<QAction*> a = pcAction->actions();
    int index = pcAction->property("defaultAction").toInt();
    a[0]->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateFillet"));
    a[1]->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateChamfer"));
    getAction()->setIcon(a[index]->icon());
}

void CmdSketcherCompCreateFillets::languageChange()
{
    Command::languageChange();

    if (!_pcAction)
        return;
    Gui::ActionGroup* pcAction = qobject_cast<Gui::ActionGroup*>(_pcAction);
    QList<QAction*> a = pcAction->actions();

    QAction* filletAction = a[0];
    filletAction->setText(QApplication::translate("CmdSketcherCompCreateFillets","Sketch fillet"));
    filletAction->setToolTip(QApplication::translate("Sketcher_CreateFillet","Creates a radius between two lines"));
    filletAction->setStatusTip(QApplication::translate("Sketcher_CreateFillet","Creates a radius between two lines"));
    QAction* chamferAction = a[1];
    chamferAction->setText(QApplication::translate("CmdSketcherCompCreateFillets", "Sketch chamfer"));
    chamferAction->setToolTip(QApplication::translate("Sketcher_CreateChamfer", "Create a chamfer between two lines"));
    chamferAction->setStatusTip(QApplication::translate("Sketcher_CreateChamfer", "Create a chamfer between two lines"));

}

bool CmdSketcherCompCreateFillets::isActive(void)
{
    return isCreateGeoActive(getActiveGuiDocument());
}

// ======================================================================================

namespace SketcherGui {
    class TrimmingSelection : public Gui::SelectionFilterGate
    {
        App::DocumentObject* object;
    public:
        TrimmingSelection(App::DocumentObject* obj)
            : Gui::SelectionFilterGate((Gui::SelectionFilter*)0), object(obj)
        {}

        bool allow(App::Document * /*pDoc*/, App::DocumentObject *pObj, const char *sSubName)
        {
            if (pObj != this->object)
                return false;
            if (!sSubName || sSubName[0] == '\0')
                return false;
            std::string element(sSubName);
            if (element.substr(0,4) == "Edge") {
                int GeoId = std::atoi(element.substr(4,4000).c_str()) - 1;
                Sketcher::SketchObject *Sketch = static_cast<Sketcher::SketchObject*>(object);
                const Part::Geometry *geom = Sketch->getGeometry(GeoId);
                if (geom->getTypeId().isDerivedFrom(Part::GeomTrimmedCurve::getClassTypeId())   ||
                    geom->getTypeId() == Part::GeomCircle::getClassTypeId()                     ||
                    geom->getTypeId() == Part::GeomEllipse::getClassTypeId()                    ||
                    geom->getTypeId() == Part::GeomBSplineCurve::getClassTypeId()
                ) {
                    // We do not trim internal geometry of complex geometries
                    if( Sketcher::GeometryFacade::isInternalType(geom, Sketcher::InternalType::None))
                        return true;
                }
            }
            return  false;
        }
    };
}

class DrawSketchHandlerTrimming: public DrawSketchHandler
{
public:
    DrawSketchHandlerTrimming() {}
    virtual ~DrawSketchHandlerTrimming()
    {
        Gui::Selection().rmvSelectionGate();
    }

    virtual void activated(ViewProviderSketch *sketchgui)
    {
        Gui::Selection().clearSelection();
        Gui::Selection().rmvSelectionGate();
        Gui::Selection().addSelectionGate(new TrimmingSelection(sketchgui->getObject()));
        setCrosshairCursor("Sketcher_Pointer_Trimming");
    }

    virtual void mouseMove(Base::Vector2d onSketchPos)
    {
        Q_UNUSED(onSketchPos);

        int GeoId = getPreselectCurve();

        if (GeoId > -1) {
            auto sk = static_cast<Sketcher::SketchObject *>(sketchgui->getObject());
            int GeoId1, GeoId2;
            Base::Vector3d intersect1, intersect2;
            if(sk->seekTrimPoints(GeoId, Base::Vector3d(onSketchPos.x,onSketchPos.y,0),
                                  GeoId1, intersect1,
                                  GeoId2, intersect2)) {

                EditMarkers.resize(0);

                if(GeoId1 != Sketcher::GeoEnum::GeoUndef)
                    EditMarkers.emplace_back(intersect1.x, intersect1.y);
                else {
                    auto start = sk->getPoint(GeoId, Sketcher::PointPos::start);
                    EditMarkers.emplace_back(start.x, start.y);
                }

                if(GeoId2 != Sketcher::GeoEnum::GeoUndef)
                    EditMarkers.emplace_back(intersect2.x, intersect2.y);
                else {
                    auto end = sk->getPoint(GeoId, Sketcher::PointPos::end);
                    EditMarkers.emplace_back( end.x, end.y);
                }

                drawEditMarkers(EditMarkers, 2); // maker augmented by two sizes (see supported marker sizes)
            }
        }
        else {
            EditMarkers.resize(0);
            drawEditMarkers(EditMarkers, 2);
        }
    }

    virtual bool pressButton(Base::Vector2d onSketchPos)
    {
        Q_UNUSED(onSketchPos);
        return true;
    }

    virtual bool releaseButton(Base::Vector2d onSketchPos)
    {
        int GeoId = getPreselectCurve();
        if (GeoId > -1) {
            const Part::Geometry *geom = sketchgui->getSketchObject()->getGeometry(GeoId);
            if (geom->getTypeId().isDerivedFrom(Part::GeomTrimmedCurve::getClassTypeId())   ||
                geom->getTypeId() == Part::GeomCircle::getClassTypeId()                     ||
                geom->getTypeId() == Part::GeomEllipse::getClassTypeId() ||
                geom->getTypeId() == Part::GeomBSplineCurve::getClassTypeId() ) {
                try {
                    Gui::Command::openCommand(QT_TRANSLATE_NOOP("Command", "Trim edge"));
                    Gui::cmdAppObjectArgs(sketchgui->getObject(), "trim(%d,App.Vector(%f,%f,0))",
                              GeoId, onSketchPos.x, onSketchPos.y);
                    Gui::Command::commitCommand();
                    tryAutoRecompute(static_cast<Sketcher::SketchObject *>(sketchgui->getObject()));
                }
                catch (const Base::Exception& e) {
                    Base::Console().Error("Failed to trim edge: %s\n", e.what());
                    Gui::Command::abortCommand();
                }
            }

            EditMarkers.resize(0);
            drawEditMarkers(EditMarkers);
        }
        else // exit the trimming tool if the user clicked on empty space
            sketchgui->purgeHandler(); // no code after this line, Handler get deleted in ViewProvider

        return true;
    }
private:
    std::vector<Base::Vector2d> EditMarkers;
};

DEF_STD_CMD_A(CmdSketcherTrimming)

CmdSketcherTrimming::CmdSketcherTrimming()
  : Command("Sketcher_Trimming")
{
    sAppModule      = "Sketcher";
    sGroup          = "Sketcher";
    sMenuText       = QT_TR_NOOP("Trim edge");
    sToolTipText    = QT_TR_NOOP("Trim an edge with respect to the picked position");
    sWhatsThis      = "Sketcher_Trimming";
    sStatusTip      = sToolTipText;
    sPixmap         = "Sketcher_Trimming";
    sAccel          = "G, T";
    eType           = ForEdit;
}

void CmdSketcherTrimming::activated(int iMsg)
{
    Q_UNUSED(iMsg);
    ActivateHandler(getActiveGuiDocument(), new DrawSketchHandlerTrimming());
}

bool CmdSketcherTrimming::isActive(void)
{
    return isCreateGeoActive(getActiveGuiDocument());
}


// ======================================================================================

namespace SketcherGui {
    class ExtendSelection : public Gui::SelectionFilterGate
    {
        App::DocumentObject* object;
    public:
        ExtendSelection(App::DocumentObject* obj)
            : Gui::SelectionFilterGate((Gui::SelectionFilter*)0)
            , object(obj)
            , disabled(false)
        {}

        bool allow(App::Document * /*pDoc*/, App::DocumentObject *pObj, const char *sSubName)
        {
            if (pObj != this->object)
                return false;
            if (!sSubName || sSubName[0] == '\0')
                return false;
            if (disabled)
                return true;
            std::string element(sSubName);
            if (element.substr(0, 4) == "Edge") {
                int GeoId = std::atoi(element.substr(4, 4000).c_str()) - 1;
                Sketcher::SketchObject *Sketch = static_cast<Sketcher::SketchObject*>(object);
                const Part::Geometry *geom = Sketch->getGeometry(GeoId);
                if (geom->getTypeId() == Part::GeomLineSegment::getClassTypeId() ||
                    geom->getTypeId() == Part::GeomArcOfCircle::getClassTypeId())
                    return true;
            }
            return false;
        }

        void setDisabled(bool isDisabled) {
            disabled = isDisabled;
        }
    protected:
        bool disabled;
    };
}

class DrawSketchHandlerExtend: public DrawSketchHandler
{
public:
    DrawSketchHandlerExtend()
        : Mode(STATUS_SEEK_First)
        , EditCurve(2)
        , BaseGeoId(-1)
        , ExtendFromStart(false)
        , SavedExtendFromStart(false)
        , Increment(0)
    {
    }
    virtual ~DrawSketchHandlerExtend()
    {
        Gui::Selection().rmvSelectionGate();
    }
    enum SelectMode {
        STATUS_SEEK_First,
        STATUS_SEEK_Second,
    };

    virtual void activated(ViewProviderSketch *sketchgui)
    {
        Q_UNUSED(sketchgui)
        Gui::Selection().clearSelection();
        Gui::Selection().rmvSelectionGate();
        filterGate = new ExtendSelection(sketchgui->getObject());
        Gui::Selection().addSelectionGate(filterGate);
        setCrosshairCursor("Sketcher_Pointer_Extension");
    }

    virtual void mouseMove(Base::Vector2d onSketchPos)
    {
        Q_UNUSED(onSketchPos);
        if (Mode == STATUS_SEEK_Second) {
            const Part::Geometry *geom = sketchgui->getSketchObject()->getGeometry(BaseGeoId);
            if (geom->getTypeId() == Part::GeomLineSegment::getClassTypeId()) {
                const Part::GeomLineSegment *lineSeg = static_cast<const Part::GeomLineSegment *>(geom);
                // project point to the existing curve
                Base::Vector3d start3d = lineSeg->getStartPoint();
                Base::Vector3d end3d = lineSeg->getEndPoint();

                Base::Vector2d startPoint = Base::Vector2d(start3d.x, start3d.y);
                Base::Vector2d endPoint = Base::Vector2d(end3d.x, end3d.y);
                Base::Vector2d recenteredLine = endPoint - startPoint;
                Base::Vector2d recenteredPoint = onSketchPos - startPoint;
                Base::Vector2d projection;
                projection.ProjectToLine(recenteredPoint, recenteredLine);
                if (recenteredPoint.Length() < recenteredPoint.Distance(recenteredLine)) {
                    EditCurve[0] = startPoint + projection;
                    EditCurve[1] = endPoint;
                } else {
                    EditCurve[0] = startPoint;
                    EditCurve[1] = startPoint + projection;
                }
                /**
                 * If in-curve, the intuitive behavior is for the line to shrink an amount from
                 * the original click-point.
                 *
                 * If out-of-curve, the intuitive behavior is for the closest line endpoint to
                 * expand.
                 */
                bool inCurve = (projection.Length() < recenteredLine.Length()
                    && projection.GetAngle(recenteredLine) < 0.1); // Two possible values here, M_PI and 0, but 0.1 is to avoid floating point problems.
                if (inCurve) {
                    Increment = SavedExtendFromStart ? -1 * projection.Length() : projection.Length() - recenteredLine.Length();
                    ExtendFromStart = SavedExtendFromStart;
                } else {
                    ExtendFromStart = onSketchPos.Distance(startPoint) < onSketchPos.Distance(endPoint);
                    Increment = ExtendFromStart ? projection.Length() : projection.Length() - recenteredLine.Length();
                }
                drawEdit(EditCurve);

            } else if (geom->getTypeId() == Part::GeomArcOfCircle::getClassTypeId()) {
                const Part::GeomArcOfCircle *arc = static_cast<const Part::GeomArcOfCircle *>(geom);
                Base::Vector3d center = arc->getCenter();
                double radius = arc->getRadius();

                double start, end;
                arc->getRange(start, end, true);
                double arcAngle = end - start;

                Base::Vector2d angle = Base::Vector2d(onSketchPos.x - center.x, onSketchPos.y - center.y);
                Base::Vector2d startAngle = Base::Vector2d(cos(start), sin(start));
                Base::Vector2d endAngle = Base::Vector2d(cos(end), sin(end));

                Base::Vector2d arcHalf = Base::Vector2d(cos(start + arcAngle/ 2.0), sin(start+ arcAngle / 2.0));
                double angleToEndAngle = angle.GetAngle(endAngle);
                double angleToStartAngle = angle.GetAngle(startAngle);


                double modStartAngle = start;
                double modArcAngle = end - start;
                bool outOfArc = arcHalf.GetAngle(angle) * 2.0 > arcAngle;
                if (ExtendFromStart) {
                    bool isCCWFromStart = crossProduct(angle, startAngle) < 0;
                    if (outOfArc) {
                        if (isCCWFromStart) {
                            modStartAngle -= 2*M_PI - angleToStartAngle;
                            modArcAngle += 2*M_PI - angleToStartAngle;
                        } else {
                            modStartAngle -= angleToStartAngle;
                            modArcAngle += angleToStartAngle;
                        }
                    } else {
                        if (isCCWFromStart) {
                            modStartAngle += angleToStartAngle;
                            modArcAngle -= angleToStartAngle;
                        } else {
                            modStartAngle += 2*M_PI - angleToStartAngle;
                            modArcAngle -= 2*M_PI - angleToStartAngle;
                        }
                    }
                } else {
                    bool isCWFromEnd = crossProduct(angle, endAngle) >= 0;
                    if (outOfArc) {
                        if (isCWFromEnd) {
                            modArcAngle += 2*M_PI - angleToEndAngle;
                        } else {
                            modArcAngle += angleToEndAngle;
                        }
                    } else {
                        if (isCWFromEnd) {
                            modArcAngle -= angleToEndAngle;
                        } else {
                            modArcAngle -= 2*M_PI - angleToEndAngle;
                        }
                    }
                }
                Increment = modArcAngle - (end - start);
                for (int i = 0; i < 31; i++) {
                    double angle = modStartAngle + i * modArcAngle/30.0;
                    EditCurve[i] = Base::Vector2d(center.x + radius * cos(angle), center.y + radius * sin(angle));
                }
                drawEdit(EditCurve);
            }
            int curveId = getPreselectCurve();
            if (BaseGeoId != curveId && seekAutoConstraint(SugConstr, onSketchPos, Base::Vector2d(0.f,0.f))) {
                renderSuggestConstraintsCursor(SugConstr);
                return;
            }
        }
    }

    virtual bool pressButton(Base::Vector2d onSketchPos)
    {
        Q_UNUSED(onSketchPos);
        return true;
    }

    virtual bool releaseButton(Base::Vector2d onSketchPos)
    {
        Q_UNUSED(onSketchPos);
        if (Mode == STATUS_SEEK_First) {
            BaseGeoId = getPreselectCurve();
            if (BaseGeoId > -1) {
                const Part::Geometry *geom = sketchgui->getSketchObject()->getGeometry(BaseGeoId);
                if (geom->getTypeId() == Part::GeomLineSegment::getClassTypeId()) {
                    const Part::GeomLineSegment *seg = static_cast<const Part::GeomLineSegment *>(geom);
                    Base::Vector3d start3d = seg->getStartPoint();
                    Base::Vector3d end3d = seg->getEndPoint();
                    Base::Vector2d start = Base::Vector2d(start3d.x, start3d.y);
                    Base::Vector2d end = Base::Vector2d(end3d.x, end3d.y);
                    SavedExtendFromStart = (onSketchPos.Distance(start) < onSketchPos.Distance(end));
                    ExtendFromStart = SavedExtendFromStart;
                    Mode = STATUS_SEEK_Second;
                } else if (geom->getTypeId() == Part::GeomArcOfCircle::getClassTypeId()) {
                    const Part::GeomArcOfCircle *arc = static_cast<const Part::GeomArcOfCircle *>(geom);
                    double start, end;
                    arc->getRange(start, end, true);

                    Base::Vector3d center = arc->getCenter();
                    Base::Vector2d angle = Base::Vector2d(onSketchPos.x - center.x, onSketchPos.y - center.y);
                    double angleToStart = angle.GetAngle(Base::Vector2d(cos(start), sin(start)));
                    double angleToEnd = angle.GetAngle(Base::Vector2d(cos(end), sin(end)));
                    ExtendFromStart = (angleToStart < angleToEnd); // move start point if closer to angle than end point
                    EditCurve.resize(31);
                    Mode = STATUS_SEEK_Second;
                }
                filterGate->setDisabled(true);
            }
        } else if (Mode == STATUS_SEEK_Second) {
            try {
                Gui::Command::openCommand(QT_TRANSLATE_NOOP("Command", "Extend edge"));
                Gui::cmdAppObjectArgs(sketchgui->getObject(), "extend(%d, %f, %d)\n", // GeoId, increment, PointPos
                    BaseGeoId, Increment, ExtendFromStart ? static_cast<int>(Sketcher::PointPos::start) : static_cast<int>(Sketcher::PointPos::end));
                Gui::Command::commitCommand();

                ParameterGrp::handle hGrp = App::GetApplication().GetParameterGroupByPath("User parameter:BaseApp/Preferences/Mod/Sketcher");
                bool autoRecompute = hGrp->GetBool("AutoRecompute",false);
                if(autoRecompute)
                    Gui::Command::updateActive();

                // constrain chosen point
                if (SugConstr.size() > 0) {
                    createAutoConstraints(SugConstr, BaseGeoId, (ExtendFromStart) ? Sketcher::PointPos::start : Sketcher::PointPos::end);
                    SugConstr.clear();
                }
                bool continuousMode = hGrp->GetBool("ContinuousCreationMode",true);

                if(continuousMode){
                    // This code enables the continuous creation mode.
                    Mode=STATUS_SEEK_First;
                    filterGate->setDisabled(false);
                    EditCurve.clear();
                    drawEdit(EditCurve);
                    EditCurve.resize(2);
                    applyCursor();
                    /* this is ok not to call to purgeHandler
                    * in continuous creation mode because the
                    * handler is destroyed by the quit() method on pressing the
                    * right button of the mouse */
                } else{
                    sketchgui->purgeHandler(); // no code after this line, Handler get deleted in ViewProvider
                }
            }
            catch (const Base::Exception& e) {
                Base::Console().Error("Failed to extend edge: %s\n", e.what());
                Gui::Command::abortCommand();
            }

        } else { // exit extension tool if user clicked on empty space
            BaseGeoId = -1;
            sketchgui->purgeHandler(); // no code after this line, Handler get deleted in ViewProvider
        }
        return true;
    }

protected:
    SelectMode Mode;
    std::vector<Base::Vector2d> EditCurve;
    int BaseGeoId;
    ExtendSelection* filterGate = nullptr;
    bool ExtendFromStart; // if true, extend from start, else extend from end (circle only)
    bool SavedExtendFromStart;
    double Increment;
    std::vector<AutoConstraint> SugConstr;

private:
    int crossProduct(Base::Vector2d &vec1, Base::Vector2d &vec2) {
        return vec1.x * vec2.y - vec1.y * vec2.x;
    }
};

DEF_STD_CMD_A(CmdSketcherExtend)

//TODO: fix the translations for this
CmdSketcherExtend::CmdSketcherExtend()
  : Command("Sketcher_Extend")
{
    sAppModule      = "Sketcher";
    sGroup          = "Sketcher";
    sMenuText       = QT_TR_NOOP("Extend edge");
    sToolTipText    = QT_TR_NOOP("Extend an edge with respect to the picked position");
    sWhatsThis      = "Sketcher_Extend";
    sStatusTip      = sToolTipText;
    sPixmap         = "Sketcher_Extend";
    sAccel          = "G, Q";
    eType           = ForEdit;
}

void CmdSketcherExtend::activated(int iMsg)
{
    Q_UNUSED(iMsg);
    ActivateHandler(getActiveGuiDocument(), new DrawSketchHandlerExtend());
}

bool CmdSketcherExtend::isActive(void)
{
    return isCreateGeoActive(getActiveGuiDocument());
}


// Splitting ==============================================================*/

namespace SketcherGui {
    class SplittingSelection : public Gui::SelectionFilterGate
    {
        App::DocumentObject* object;
    public:
        SplittingSelection(App::DocumentObject* obj)
            : Gui::SelectionFilterGate((Gui::SelectionFilter*)0), object(obj)
        {}

        bool allow(App::Document * /*pDoc*/, App::DocumentObject *pObj, const char *sSubName)
        {
            if (pObj != this->object)
                return false;
            if (!sSubName || sSubName[0] == '\0')
                return false;
            std::string element(sSubName);
            if (element.substr(0,4) == "Edge") {
                int GeoId = std::atoi(element.substr(4,4000).c_str()) - 1;
                Sketcher::SketchObject *Sketch = static_cast<Sketcher::SketchObject*>(object);
                const Part::Geometry *geom = Sketch->getGeometry(GeoId);
                if (geom->getTypeId() == Part::GeomLineSegment::getClassTypeId()
                    || geom->getTypeId() == Part::GeomCircle::getClassTypeId()
                    || geom->getTypeId() == Part::GeomArcOfCircle::getClassTypeId()) {
                    return true;
                }
            }
            return  false;
        }
    };
}

class DrawSketchHandlerSplitting: public DrawSketchHandler
{
public:
    DrawSketchHandlerSplitting() {}
    virtual ~DrawSketchHandlerSplitting()
    {
        Gui::Selection().rmvSelectionGate();
    }

    virtual void activated(ViewProviderSketch *sketchgui)
    {
        Gui::Selection().clearSelection();
        Gui::Selection().rmvSelectionGate();
        Gui::Selection().addSelectionGate(new SplittingSelection(sketchgui->getObject()));
        setCrosshairCursor("Sketcher_Pointer_Splitting");
    }

    virtual void mouseMove(Base::Vector2d onSketchPos)
    {
        Q_UNUSED(onSketchPos);
    }

    virtual bool pressButton(Base::Vector2d onSketchPos)
    {
        Q_UNUSED(onSketchPos);
        return true;
    }

    virtual bool releaseButton(Base::Vector2d onSketchPos)
    {
        int GeoId = getPreselectCurve();
        if (GeoId >= 0) {
            const Part::Geometry *geom = sketchgui->getSketchObject()->getGeometry(GeoId);
            if (geom->getTypeId() == Part::GeomLineSegment::getClassTypeId()
                || geom->getTypeId() == Part::GeomCircle::getClassTypeId()
                || geom->getTypeId() == Part::GeomArcOfCircle::getClassTypeId()) {
                try {
                    Gui::Command::openCommand(QT_TRANSLATE_NOOP("Command", "Split edge"));
                    Gui::cmdAppObjectArgs(sketchgui->getObject(), "split(%d,App.Vector(%f,%f,0))",
                              GeoId, onSketchPos.x, onSketchPos.y);
                    Gui::Command::commitCommand();
                    tryAutoRecompute(static_cast<Sketcher::SketchObject *>(sketchgui->getObject()));
                }
                catch (const Base::Exception& e) {
                    Base::Console().Error("Failed to split edge: %s\n", e.what());
                    Gui::Command::abortCommand();
                }
            }
        }
        else {
            sketchgui->purgeHandler();
        }

        return true;
    }
};

DEF_STD_CMD_A(CmdSketcherSplit)

//TODO: fix the translations for this
CmdSketcherSplit::CmdSketcherSplit()
  : Command("Sketcher_Split")
{
    sAppModule      = "Sketcher";
    sGroup          = "Sketcher";
    sMenuText       = QT_TR_NOOP("Split edge");
    sToolTipText    = QT_TR_NOOP("Splits an edge into two while preserving constraints");
    sWhatsThis      = "Sketcher_Split";
    sStatusTip      = sToolTipText;
    sPixmap         = "Sketcher_Split";
    sAccel          = "G, Z";
    eType           = ForEdit;
}

void CmdSketcherSplit::activated(int iMsg)
{
    Q_UNUSED(iMsg);
    ActivateHandler(getActiveGuiDocument(), new DrawSketchHandlerSplitting());
}

bool CmdSketcherSplit::isActive(void)
{
    return isCreateGeoActive(getActiveGuiDocument());
}


namespace SketcherGui {
    class ExternalSelection : public Gui::SelectionFilterGate
    {
        App::DocumentObject* object;
    public:
        ExternalSelection(App::DocumentObject* obj)
            : Gui::SelectionFilterGate((Gui::SelectionFilter*)0), object(obj)
        {}

        bool allow(App::Document *pDoc, App::DocumentObject *pObj, const char *sSubName)
        {
            Sketcher::SketchObject *sketch = static_cast<Sketcher::SketchObject*>(object);

            this->notAllowedReason = "";
            Sketcher::SketchObject::eReasonList msg;
            if (!sketch->isExternalAllowed(pDoc, pObj, &msg)){
                switch(msg){
                case Sketcher::SketchObject::rlCircularReference:
                    this->notAllowedReason = QT_TR_NOOP("Linking this will cause circular dependency.");
                    break;
                case Sketcher::SketchObject::rlOtherDoc:
                    this->notAllowedReason = QT_TR_NOOP("This object is in another document.");
                    break;
                case Sketcher::SketchObject::rlOtherBody:
                    this->notAllowedReason = QT_TR_NOOP("This object belongs to another body, can't link.");
                    break;
                case Sketcher::SketchObject::rlOtherPart:
                    this->notAllowedReason = QT_TR_NOOP("This object belongs to another part, can't link.");
                    break;
                default:
                    break;
                }
                return false;
            }

            // Note: its better to search the support of the sketch in case the sketch support is a base plane
            //Part::BodyBase* body = Part::BodyBase::findBodyOf(sketch);
            //if ( body && body->hasFeature ( pObj ) && body->isAfter ( pObj, sketch ) ) {
                // Don't allow selection after the sketch in the same body
                // NOTE: allowness of features in other bodies is handled by SketchObject::isExternalAllowed()
                // TODO may be this should be in SketchObject::isExternalAllowed() (2015-08-07, Fat-Zer)
                //return false;
            //}

            if (!sSubName || sSubName[0] == '\0')
                return false;
            std::string element(sSubName);
            if ((element.size() > 4 && element.substr(0,4) == "Edge") ||
                (element.size() > 6 && element.substr(0,6) == "Vertex") ||
                (element.size() > 4 && element.substr(0,4) == "Face")) {
                return true;
            }
            if (pObj->getTypeId().isDerivedFrom(App::Plane::getClassTypeId()) ||
                pObj->getTypeId().isDerivedFrom(Part::Datum::getClassTypeId()))
                return true;
            return  false;
        }
    };
}

class DrawSketchHandlerExternal: public DrawSketchHandler
{
public:
    DrawSketchHandlerExternal() {}
    virtual ~DrawSketchHandlerExternal()
    {
        Gui::Selection().rmvSelectionGate();
    }

    virtual void activated(ViewProviderSketch *sketchgui)
    {
        setAxisPickStyle(false);
        Gui::MDIView *mdi = Gui::Application::Instance->activeDocument()->getActiveView();
        Gui::View3DInventorViewer *viewer;
        viewer = static_cast<Gui::View3DInventor *>(mdi)->getViewer();

        SoNode* root = viewer->getSceneGraph();
        static_cast<Gui::SoFCUnifiedSelection*>(root)->selectionRole.setValue(true);

        Gui::Selection().clearSelection();
        Gui::Selection().rmvSelectionGate();
        Gui::Selection().addSelectionGate(new ExternalSelection(sketchgui->getObject()));
        setCrosshairCursor("Sketcher_Pointer_External");
    }

    virtual void deactivated(ViewProviderSketch *sketchgui)
    {
        Q_UNUSED(sketchgui);
        setAxisPickStyle(true);
    }

    virtual void mouseMove(Base::Vector2d onSketchPos)
    {
        Q_UNUSED(onSketchPos);
        if (Gui::Selection().getPreselection().pObjectName)
            applyCursor();
    }

    virtual bool pressButton(Base::Vector2d onSketchPos)
    {
        Q_UNUSED(onSketchPos);
        return true;
    }

    virtual bool releaseButton(Base::Vector2d onSketchPos)
    {
        Q_UNUSED(onSketchPos);
        /* this is ok not to call to purgeHandler
        * in continuous creation mode because the
        * handler is destroyed by the quit() method on pressing the
        * right button of the mouse */
        return true;
    }

    virtual bool onSelectionChanged(const Gui::SelectionChanges& msg)
    {
        if (msg.Type == Gui::SelectionChanges::AddSelection) {
            App::DocumentObject* obj = sketchgui->getObject()->getDocument()->getObject(msg.pObjectName);
            if (obj == NULL)
                throw Base::ValueError("Sketcher: External geometry: Invalid object in selection");
            std::string subName(msg.pSubName);
            if (obj->getTypeId().isDerivedFrom(App::Plane::getClassTypeId()) ||
                obj->getTypeId().isDerivedFrom(Part::Datum::getClassTypeId()) ||
                (subName.size() > 4 && subName.substr(0,4) == "Edge") ||
                (subName.size() > 6 && subName.substr(0,6) == "Vertex") ||
                (subName.size() > 4 && subName.substr(0,4) == "Face")) {
                try {
                    Gui::Command::openCommand(QT_TRANSLATE_NOOP("Command", "Add external geometry"));
                    Gui::cmdAppObjectArgs(sketchgui->getObject(), "addExternal(\"%s\",\"%s\")",
                              msg.pObjectName, msg.pSubName);
                    Gui::Command::commitCommand();

                    // adding external geometry does not require a solve() per se (the DoF is the same),
                    // however a solve is required to update the amount of solver geometry, because we only
                    // redraw a changed Sketch if the solver geometry amount is the same as the SkethObject
                    // geometry amount (as this avoids other issues).
                    // This solver is a very low cost one anyway (there is actually nothing to solve).
                    tryAutoRecomputeIfNotSolve(static_cast<Sketcher::SketchObject *>(sketchgui->getObject()));

                    Gui::Selection().clearSelection();
                /* this is ok not to call to purgeHandler
                * in continuous creation mode because the
                * handler is destroyed by the quit() method on pressing the
                * right button of the mouse */
                }
                catch (const Base::Exception& e) {
                    Base::Console().Error("Failed to add external geometry: %s\n", e.what());
                    Gui::Selection().clearSelection();
                    Gui::Command::abortCommand();
                }
                return true;
            }
        }
        return false;
    }
};

DEF_STD_CMD_A(CmdSketcherExternal)

CmdSketcherExternal::CmdSketcherExternal()
  : Command("Sketcher_External")
{
    sAppModule      = "Sketcher";
    sGroup          = "Sketcher";
    sMenuText       = QT_TR_NOOP("External geometry");
    sToolTipText    = QT_TR_NOOP("Create an edge linked to an external geometry");
    sWhatsThis      = "Sketcher_External";
    sStatusTip      = sToolTipText;
    sPixmap         = "Sketcher_External";
    sAccel          = "G, X";
    eType           = ForEdit;
}

void CmdSketcherExternal::activated(int iMsg)
{
    Q_UNUSED(iMsg);
    ActivateHandler(getActiveGuiDocument(), new DrawSketchHandlerExternal());
}

bool CmdSketcherExternal::isActive(void)
{
    return isCreateGeoActive(getActiveGuiDocument());
}

/* Carbon Copy============================================================*/

namespace SketcherGui {
    class CarbonCopySelection : public Gui::SelectionFilterGate
    {
        App::DocumentObject* object;
    public:
        CarbonCopySelection(App::DocumentObject* obj)
        : Gui::SelectionFilterGate((Gui::SelectionFilter*)0), object(obj)
        {}

        bool allow(App::Document *pDoc, App::DocumentObject *pObj, const char *sSubName)
        {
            Q_UNUSED(sSubName);

            Sketcher::SketchObject *sketch = static_cast<Sketcher::SketchObject*>(object);
            sketch->setAllowOtherBody(QApplication::keyboardModifiers() == Qt::ControlModifier || QApplication::keyboardModifiers() == (Qt::ControlModifier | Qt::AltModifier));
            sketch->setAllowUnaligned(QApplication::keyboardModifiers() == (Qt::ControlModifier | Qt::AltModifier));

            this->notAllowedReason = "";
            Sketcher::SketchObject::eReasonList msg;
            // Reusing code: All good reasons not to allow a carbon copy
            bool xinv = false, yinv = false;
            if (!sketch->isCarbonCopyAllowed(pDoc, pObj, xinv, yinv, &msg)){
                switch(msg){
                    case Sketcher::SketchObject::rlCircularReference:
                        this->notAllowedReason = QT_TR_NOOP("Carbon copy would cause a circular dependency.");
                        break;
                    case Sketcher::SketchObject::rlOtherDoc:
                        this->notAllowedReason = QT_TR_NOOP("This object is in another document.");
                        break;
                    case Sketcher::SketchObject::rlOtherBody:
                        this->notAllowedReason = QT_TR_NOOP("This object belongs to another body. Hold Ctrl to allow cross-references.");
                        break;
                    case Sketcher::SketchObject::rlOtherBodyWithLinks:
                        this->notAllowedReason = QT_TR_NOOP("This object belongs to another body and it contains external geometry. Cross-reference not allowed.");
                        break;
                    case Sketcher::SketchObject::rlOtherPart:
                        this->notAllowedReason = QT_TR_NOOP("This object belongs to another part.");
                        break;
                    case Sketcher::SketchObject::rlNonParallel:
                        this->notAllowedReason = QT_TR_NOOP("The selected sketch is not parallel to this sketch. Hold Ctrl+Alt to allow non-parallel sketches.");
                        break;
                    case Sketcher::SketchObject::rlAxesMisaligned:
                        this->notAllowedReason = QT_TR_NOOP("The XY axes of the selected sketch do not have the same direction as this sketch. Hold Ctrl+Alt to disregard it.");
                        break;
                    case Sketcher::SketchObject::rlOriginsMisaligned:
                        this->notAllowedReason = QT_TR_NOOP("The origin of the selected sketch is not aligned with the origin of this sketch. Hold Ctrl+Alt to disregard it.");
                        break;
                    default:
                        break;
                }
                return false;
            }
            // Carbon copy only works on sketches that are not disallowed (e.g. would produce a circular reference)
            return  true;
        }
    };
}

class DrawSketchHandlerCarbonCopy: public DrawSketchHandler
{
public:
    DrawSketchHandlerCarbonCopy() {}
    virtual ~DrawSketchHandlerCarbonCopy()
    {
        Gui::Selection().rmvSelectionGate();
    }

    virtual void activated(ViewProviderSketch *sketchgui)
    {
        setAxisPickStyle(false);
        Gui::MDIView *mdi = Gui::Application::Instance->activeDocument()->getActiveView();
        Gui::View3DInventorViewer *viewer;
        viewer = static_cast<Gui::View3DInventor *>(mdi)->getViewer();

        SoNode* root = viewer->getSceneGraph();
        static_cast<Gui::SoFCUnifiedSelection*>(root)->selectionRole.setValue(true);

        Gui::Selection().clearSelection();
        Gui::Selection().rmvSelectionGate();
        Gui::Selection().addSelectionGate(new CarbonCopySelection(sketchgui->getObject()));
        setCrosshairCursor("Sketcher_Pointer_CarbonCopy");
    }

    virtual void deactivated(ViewProviderSketch *sketchgui)
    {
        Q_UNUSED(sketchgui);
        setAxisPickStyle(true);
    }

    virtual void mouseMove(Base::Vector2d onSketchPos)
    {
        Q_UNUSED(onSketchPos);
        if (Gui::Selection().getPreselection().pObjectName)
            applyCursor();
    }

    virtual bool pressButton(Base::Vector2d onSketchPos)
    {
        Q_UNUSED(onSketchPos);
        return true;
    }

    virtual bool releaseButton(Base::Vector2d onSketchPos)
    {
        Q_UNUSED(onSketchPos);
        /* this is ok not to call to purgeHandler
            * in continuous creation mode because the
            * handler is destroyed by the quit() method on pressing the
            * right button of the mouse */
        return true;
    }

    virtual bool onSelectionChanged(const Gui::SelectionChanges& msg)
    {
        if (msg.Type == Gui::SelectionChanges::AddSelection) {
            App::DocumentObject* obj = sketchgui->getObject()->getDocument()->getObject(msg.pObjectName);
            if (obj == NULL)
                throw Base::ValueError("Sketcher: Carbon Copy: Invalid object in selection");

            if (obj->getTypeId() == Sketcher::SketchObject::getClassTypeId()) {

                try {
                    Gui::Command::openCommand(QT_TRANSLATE_NOOP("Command", "Add carbon copy"));
                    Gui::cmdAppObjectArgs(sketchgui->getObject(), "carbonCopy(\"%s\",%s)",
                                            msg.pObjectName, geometryCreationMode==Construction?"True":"False");

                    Gui::Command::commitCommand();

                    tryAutoRecomputeIfNotSolve(static_cast<Sketcher::SketchObject *>(sketchgui->getObject()));

                    Gui::Selection().clearSelection();
                    /* this is ok not to call to purgeHandler
                        * in continuous creation mode because the
                        * handler is destroyed by the quit() method on pressing the
                        * right button of the mouse */
                }
                catch (const Base::Exception& e) {
                    Base::Console().Error("Failed to add carbon copy: %s\n", e.what());
                    Gui::Command::abortCommand();
                }
                return true;
                }
        }
        return false;
    }
};

DEF_STD_CMD_AU(CmdSketcherCarbonCopy)

CmdSketcherCarbonCopy::CmdSketcherCarbonCopy()
: Command("Sketcher_CarbonCopy")
{
    sAppModule      = "Sketcher";
    sGroup          = "Sketcher";
    sMenuText       = QT_TR_NOOP("Carbon copy");
    sToolTipText    = QT_TR_NOOP("Copies the geometry of another sketch");
    sWhatsThis      = "Sketcher_CarbonCopy";
    sStatusTip      = sToolTipText;
    sPixmap         = "Sketcher_CarbonCopy";
    sAccel          = "G, W";
    eType           = ForEdit;
}

void CmdSketcherCarbonCopy::activated(int iMsg)
{
    Q_UNUSED(iMsg);
    ActivateHandler(getActiveGuiDocument(), new DrawSketchHandlerCarbonCopy());
}

bool CmdSketcherCarbonCopy::isActive(void)
{
    return isCreateGeoActive(getActiveGuiDocument());
}

void CmdSketcherCarbonCopy::updateAction(int mode)
{
    switch (mode) {
        case Normal:
            if (getAction())
                getAction()->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CarbonCopy"));
            break;
        case Construction:
            if (getAction())
                getAction()->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CarbonCopy_Constr"));
            break;
    }
}

/* Create Slot =========================================================*/

class DrawSketchHandlerSlot : public DrawSketchHandler
{
public:

    DrawSketchHandlerSlot()
        : Mode(STATUS_SEEK_First)
        , SnapMode(SNAP_MODE_Free)
        , SnapDir(SNAP_DIR_Horz)
        , dx(0), dy(0), r(0)
        , EditCurve(35)
    {}
    virtual ~DrawSketchHandlerSlot() {
        sketchgui->toolSettings->widget->setSettings(0);
    }
    /// mode table
    enum BoxMode {
        STATUS_SEEK_First,      /**< enum value ----. */
        STATUS_SEEK_Second,      /**< enum value ----. */
        STATUS_SEEK_Third,      /**< enum value ----. */
        STATUS_End
    };

    enum SNAP_MODE
    {
        SNAP_MODE_Free,
        SNAP_MODE_Straight
    };

    enum SNAP_DIR
    {
        SNAP_DIR_Horz,
        SNAP_DIR_Vert
    };

    virtual void activated(ViewProviderSketch*)
    {
        setCrosshairCursor("Sketcher_Pointer_Slot");
        sketchgui->toolSettings->widget->setSettings(12);
    }

    virtual void mouseMove(Base::Vector2d onSketchPos)
    {

        if (Mode == STATUS_SEEK_First) {
            setPositionText(onSketchPos);

            if (sketchgui->toolSettings->widget->isSettingSet[0] + sketchgui->toolSettings->widget->isSettingSet[1] == 2) {
                pressButton(onSketchPos);
                releaseButton(onSketchPos);
            }

            if (seekAutoConstraint(sugConstr1, onSketchPos, Base::Vector2d(0.f, 0.f))) {
                renderSuggestConstraintsCursor(sugConstr1);
                return;
            }
        }
        else if (Mode == STATUS_SEEK_Second) {

            if (sketchgui->toolSettings->widget->isSettingSet[2] == 1) {
                double dx1 = onSketchPos.x - StartPos.x;
                double dy1 = onSketchPos.y - StartPos.y;

                dx = sketchgui->toolSettings->widget->toolParameters[2] * dx1 /sqrt(dx1*dx1 + dy1*dy1);
                dy = sketchgui->toolSettings->widget->toolParameters[2] * dy1 / sqrt(dx1 * dx1 + dy1 * dy1);
            }
            else {
                dx = onSketchPos.x - StartPos.x;
                dy = onSketchPos.y - StartPos.y;
            }

            double a = 0;
            double rev = 0;
            if (fabs(dx) > fabs(dy)) {
                rev = Base::sgn(dx);
            }
            else {
                a = 8;
                rev = Base::sgn(dy);
            }
            double length = sqrt(dx * dx + dy * dy);
            r = length / 5; //radius choosen at 1/5 of length

            if (sketchgui->toolSettings->widget->isSettingSet[3] == 1) {
                dx = cos(-sketchgui->toolSettings->widget->toolParameters[3] * M_PI / 180) * length;
                dy = sin(-sketchgui->toolSettings->widget->toolParameters[3] * M_PI / 180) * length;
            }
            else{
                if (QApplication::keyboardModifiers() == Qt::ControlModifier)
                    SnapMode = SNAP_MODE_Straight;
                else
                    SnapMode = SNAP_MODE_Free;

                if (fabs(dx) > fabs(dy)) {
                    SnapDir = SNAP_DIR_Horz;
                    if (SnapMode == SNAP_MODE_Straight) {
                        dy = 0;
                        if (sketchgui->toolSettings->widget->isSettingSet[2] == 1) {
                            dx = sketchgui->toolSettings->widget->toolParameters[2];
                        }
                    }
                }
                else {
                    SnapDir = SNAP_DIR_Vert;
                    if (SnapMode == SNAP_MODE_Straight) {
                        dx = 0;
                        if (sketchgui->toolSettings->widget->isSettingSet[2] == 1) {
                            dy = sketchgui->toolSettings->widget->toolParameters[2];
                        }
                    }
                }
            }


            // draw the arcs with each 16 segments
            for (int i = 0; i < 17; i++) {
                // first get the position at the arc
                // if a is 0, the end points of the arc are at the y-axis, if it is 8, they are on the x-axis
                double angle = (i + a) * M_PI / 16.0;
                double rx = -r * rev * sin(angle);
                double ry = r * rev * cos(angle);
                // now apply the rotation matrix according to the angle between StartPos and onSketchPos
                if (!(dx == 0 || dy == 0)) {
                    double rotAngle = atan(dy / dx);
                    if (a > 0)
                        rotAngle = -atan(dx / dy);
                    double rxRot = rx * cos(rotAngle) - ry * sin(rotAngle);
                    double ryRot = rx * sin(rotAngle) + ry * cos(rotAngle);
                    rx = rxRot;
                    ry = ryRot;
                }
                EditCurve[i] = Base::Vector2d(StartPos.x + rx, StartPos.y + ry);
                EditCurve[17 + i] = Base::Vector2d(StartPos.x + dx - rx, StartPos.y + dy - ry);
            }
            EditCurve[34] = EditCurve[0];

            SbString text;
            text.sprintf(" (%.1fR %.1fL)", r, length);
            setPositionText(onSketchPos, text);

            drawEdit(EditCurve);
            if (seekAutoConstraint(sugConstr2, onSketchPos, Base::Vector2d(dx, dy), AutoConstraint::VERTEX_NO_TANGENCY)) {
                renderSuggestConstraintsCursor(sugConstr2);
                return;
            }

            if (sketchgui->toolSettings->widget->isSettingSet[2] == 1 && sketchgui->toolSettings->widget->isSettingSet[3] == 1) {
                pressButton(onSketchPos);
                releaseButton(onSketchPos);
            }
        }
        else if (Mode == STATUS_SEEK_Third) {
            double a = 0;
            double rev = 0;

            /*To follow the cursor, r should adapt depending on the position of the cursor. If cursor is 'between' the center points, 
            then its distance to that line and not distance to the second center.
            A is "between" B and C if angle ∠ABC and angle ∠ACB are both less than or equal to ninety degrees.
            An angle ∠ABC is greater than ninety degrees iff AB^2 + BC^2 < AC^2.*/

            double L = sqrt(dx * dx + dy * dy); //distance between the two centers
            if (sketchgui->toolSettings->widget->isSettingSet[4] == 1) {
                r = sketchgui->toolSettings->widget->toolParameters[4];
            }
            else {
                double L1 = sqrt(pow(onSketchPos.x - StartPos.x, 2) + pow(onSketchPos.y - StartPos.y, 2)); //distance between first center and onSketchPos
                double L2 = sqrt(pow(onSketchPos.x - SecondPos.x, 2) + pow(onSketchPos.y - SecondPos.y, 2)); //distance between second center and onSketchPos

                if ((L1 * L1 + L * L > L2 * L2) && (L2 * L2 + L * L > L1 * L1)) {
                    //distance of onSketchPos to the line StartPos-SecondPos
                    r = (abs((SecondPos.y - StartPos.y) * onSketchPos.x - (SecondPos.x - StartPos.x) * onSketchPos.y + SecondPos.x * StartPos.y - SecondPos.y * StartPos.x)) /
                        (pow((pow(SecondPos.y - StartPos.y, 2) + pow(SecondPos.x - StartPos.x, 2)), 0.5));
                }
                else {
                    r = min(L1, L2);
                }
            }

            if (fabs(dx) > fabs(dy)) {
                rev = Base::sgn(dx);
                SnapDir = SNAP_DIR_Horz;
            }
            else {
                a = 8;
                rev = Base::sgn(dy);
                SnapDir = SNAP_DIR_Vert;
            }

            // draw the arcs with each 16 segments
            for (int i = 0; i < 17; i++) {
                // first get the position at the arc
                // if a is 0, the end points of the arc are at the y-axis, if it is 8, they are on the x-axis
                double angle = (i + a) * M_PI / 16.0;
                double rx = -r * rev * sin(angle);
                double ry = r * rev * cos(angle);
                // now apply the rotation matrix according to the angle between StartPos and onSketchPos
                if (!(dx == 0 || dy == 0)) {
                    double rotAngle = atan(dy / dx);
                    if (a > 0)
                        rotAngle = -atan(dx / dy);
                    double rxRot = rx * cos(rotAngle) - ry * sin(rotAngle);
                    double ryRot = rx * sin(rotAngle) + ry * cos(rotAngle);
                    rx = rxRot;
                    ry = ryRot;
                }
                EditCurve[i] = Base::Vector2d(StartPos.x + rx, StartPos.y + ry);
                EditCurve[17 + i] = Base::Vector2d(StartPos.x + dx - rx, StartPos.y + dy - ry);
            }
            EditCurve[34] = EditCurve[0];


            SbString text;
            text.sprintf(" (%.1fR %.1fL)", r, L);
            setPositionText(onSketchPos, text);

            drawEdit(EditCurve);

            if (sketchgui->toolSettings->widget->isSettingSet[4] == 1) {
                pressButton(onSketchPos);
                releaseButton(onSketchPos);
            }
        }
        applyCursor();
    }

    virtual bool pressButton(Base::Vector2d onSketchPos)
    {
        if (Mode == STATUS_SEEK_First) {
            StartPos = onSketchPos;
            if (sketchgui->toolSettings->widget->isSettingSet[0] == 1) {
                StartPos.x = sketchgui->toolSettings->widget->toolParameters[0];
            }
            if (sketchgui->toolSettings->widget->isSettingSet[1] == 1) {
                StartPos.y = sketchgui->toolSettings->widget->toolParameters[1];
            }

            sketchgui->toolSettings->widget->setParameterActive(0, 0);
            sketchgui->toolSettings->widget->setParameterActive(0, 1);
            sketchgui->toolSettings->widget->setParameterActive(1, 2);
            sketchgui->toolSettings->widget->setParameterActive(1, 3);
            sketchgui->toolSettings->widget->setParameterFocus(2);
            Mode = STATUS_SEEK_Second;
        }
        else if (Mode == STATUS_SEEK_Second) {
            SecondPos.x = StartPos.x + dx;
            SecondPos.y = StartPos.y + dy;
            sketchgui->toolSettings->widget->setParameterActive(0, 2);
            sketchgui->toolSettings->widget->setParameterActive(0, 3);
            sketchgui->toolSettings->widget->setParameterActive(1, 4);
            sketchgui->toolSettings->widget->setParameterFocus(4);
            Mode = STATUS_SEEK_Third;
        }
        else {
            Mode = STATUS_End;
        }

        return true;
    }

    virtual bool releaseButton(Base::Vector2d onSketchPos)
    {
        Q_UNUSED(onSketchPos);
        if (Mode == STATUS_End) {
            unsetCursor();
            resetPositionText();

            int firstCurve = getHighestCurveIndex() + 1;
            // add the geometry to the sketch
            // first determine the rotation angle for the first arc
            double start, end;
            if (fabs(dx) > fabs(dy)) {
                if (dx > 0) {
                    start = 0.5 * M_PI;
                    end = 1.5 * M_PI;
                }
                else {
                    start = 1.5 * M_PI;
                    end = 0.5 * M_PI;
                }
            }
            else {
                if (dy > 0) {
                    start = -M_PI;
                    end = 0;
                }
                else {
                    start = 0;
                    end = -M_PI;
                }
            }

            try {
                Gui::Command::openCommand(QT_TRANSLATE_NOOP("Command", "Add slot"));

                AutoConstraint lastCons = {Sketcher::None, Sketcher::GeoEnum::GeoUndef, Sketcher::PointPos::none};

                if (!sugConstr2.empty()) lastCons = sugConstr2.back();

                ostringstream snapCon = ostringstream("");
                if (SnapMode == SNAP_MODE_Straight) {
                    snapCon << "conList.append(Sketcher.Constraint('";
                    if (SnapDir == SNAP_DIR_Horz) {
                        snapCon << "Horizontal";
                    }
                    else {
                        snapCon << "Vertical";
                    }
                    snapCon << "'," << firstCurve + 2 << "))\n";

                    // If horizontal/vertical already applied because of snap, do not duplicate with Autocontraint
                    if (lastCons.Type == Sketcher::Horizontal || lastCons.Type == Sketcher::Vertical)
                        sugConstr2.pop_back();
                }
                else {
                    // If horizontal/vertical Autoconstraint suggested, applied it on first line (rather than last arc)
                    if (lastCons.Type == Sketcher::Horizontal || lastCons.Type == Sketcher::Vertical)
                        sugConstr2.back().GeoId = firstCurve + 2;
                }

                Gui::Command::doCommand(Gui::Command::Doc,
                    "geoList = []\n"
                    "geoList.append(Part.ArcOfCircle(Part.Circle(App.Vector(%f, %f, 0), App.Vector(0, 0, 1), %f), %f, %f))\n"
                    "geoList.append(Part.ArcOfCircle(Part.Circle(App.Vector(%f, %f ,0), App.Vector(0, 0, 1), %f), %f, %f))\n"
                    "geoList.append(Part.LineSegment(App.Vector(%f, %f, 0), App.Vector(%f, %f, 0)))\n"
                    "geoList.append(Part.LineSegment(App.Vector(%f, %f, 0), App.Vector(%f, %f, 0)))\n"
                    "%s.addGeometry(geoList, %s)\n"
                    "conList = []\n"
                    "conList.append(Sketcher.Constraint('Tangent', %i, 2, %i, 1))\n"
                    "conList.append(Sketcher.Constraint('Tangent', %i, 2, %i, 1))\n"
                    "conList.append(Sketcher.Constraint('Tangent', %i, 2, %i, 1))\n"
                    "conList.append(Sketcher.Constraint('Tangent', %i, 2, %i, 1))\n"
                    "conList.append(Sketcher.Constraint('Equal', %i, %i))\n"
                    "%s"
                    "%s.addConstraint(conList)\n"
                    "del geoList, conList\n",
                    StartPos.x, StartPos.y,           // center of the arc1
                    r,                                // radius arc1
                    start, end,                       // start and end angle of arc1
                    StartPos.x + dx, StartPos.y + dy, // center of the arc2
                    r,                                // radius arc2
                    end, end + M_PI,                  // start and end angle of arc2
                    EditCurve[16].x, EditCurve[16].y, EditCurve[17].x, EditCurve[17].y, // line1
                    EditCurve[33].x, EditCurve[33].y, EditCurve[34].x, EditCurve[34].y, // line2
                    Gui::Command::getObjectCmd(sketchgui->getObject()).c_str(), // the sketch
                    geometryCreationMode == Construction ? "True" : "False", // geometry as construction or not
                    firstCurve, firstCurve + 2,     // tangent1
                    firstCurve + 2, firstCurve + 1, // tangent2
                    firstCurve + 1, firstCurve + 3, // tangent3
                    firstCurve + 3, firstCurve,     // tangent4
                    firstCurve, firstCurve + 1,     // equal constraint
                    snapCon.str().c_str(),          // horizontal/vertical constraint if snapping
                    Gui::Command::getObjectCmd(sketchgui->getObject()).c_str()); // the sketch

                //add constraint if user typed in some dimensions in tool widget
                if (sketchgui->toolSettings->widget->isSettingSet[0] + sketchgui->toolSettings->widget->isSettingSet[1] + sketchgui->toolSettings->widget->isSettingSet[2] + sketchgui->toolSettings->widget->isSettingSet[3] + +sketchgui->toolSettings->widget->isSettingSet[4] != 0) {
                    if (sketchgui->toolSettings->widget->isSettingSet[0] == 1) {
                        distanceXYorPointOnObject(0, firstCurve, Sketcher::PointPos::mid, sketchgui->toolSettings->widget->toolParameters[0]);
                    }
                    if (sketchgui->toolSettings->widget->isSettingSet[1] == 1) {
                        distanceXYorPointOnObject(1, firstCurve, Sketcher::PointPos::mid, sketchgui->toolSettings->widget->toolParameters[1]);
                    }
                    if (sketchgui->toolSettings->widget->isSettingSet[2] == 1) {
                        Gui::cmdAppObjectArgs(sketchgui->getObject(), "addConstraint(Sketcher.Constraint('Distance',%d,%d,%d,%d,%f)) ",
                            firstCurve, 3, firstCurve + 1, 3, sketchgui->toolSettings->widget->toolParameters[2]);
                    }
                    if (sketchgui->toolSettings->widget->isSettingSet[3] == 1) {
                        if (sketchgui->toolSettings->widget->toolParameters[3] == 0 ) {
                            Gui::cmdAppObjectArgs(sketchgui->getObject(), "addConstraint(Sketcher.Constraint('Horizontal',%d)) ", firstCurve + 2);
                        }
                        else if (sketchgui->toolSettings->widget->toolParameters[3] == 90 ) {
                            Gui::cmdAppObjectArgs(sketchgui->getObject(), "addConstraint(Sketcher.Constraint('Vertical',%d)) ", firstCurve + 2);
                        }
                        else {
                            Gui::cmdAppObjectArgs(sketchgui->getObject(), "addConstraint(Sketcher.Constraint('Angle',%d,%d,%f)) ",
                                Sketcher::GeoEnum::HAxis, firstCurve + 2, sketchgui->toolSettings->widget->toolParameters[3] * M_PI / 180);
                        }
                    }
                    if (sketchgui->toolSettings->widget->isSettingSet[4] == 1) {
                        Gui::cmdAppObjectArgs(sketchgui->getObject(), "addConstraint(Sketcher.Constraint('Radius',%d,%f)) ",
                            firstCurve, sketchgui->toolSettings->widget->toolParameters[4]);
                    }
                }

                Gui::Command::commitCommand();


                // add auto constraints at the center of the first arc
                if (sugConstr1.size() > 0) {
                    createAutoConstraints(sugConstr1, getHighestCurveIndex() - 3, Sketcher::PointPos::mid);
                    sugConstr1.clear();
                }

                // add auto constraints at the center of the second arc
                if (sugConstr2.size() > 0) {
                    createAutoConstraints(sugConstr2, getHighestCurveIndex() - 2, Sketcher::PointPos::mid);
                    sugConstr2.clear();
                }

                tryAutoRecomputeIfNotSolve(static_cast<Sketcher::SketchObject*>(sketchgui->getObject()));
            }
            catch (const Base::Exception& e) {
                Base::Console().Error("Failed to add slot: %s\n", e.what());
                Gui::Command::abortCommand();

                tryAutoRecompute(static_cast<Sketcher::SketchObject*>(sketchgui->getObject()));
            }

            ParameterGrp::handle hGrp = App::GetApplication().GetParameterGroupByPath("User parameter:BaseApp/Preferences/Mod/Sketcher");
            bool continuousMode = hGrp->GetBool("ContinuousCreationMode", true);

            if (continuousMode) {
                // This code enables the continuous creation mode.
                sketchgui->toolSettings->widget->setSettings(12);
                Mode = STATUS_SEEK_First;
                EditCurve.clear();
                drawEdit(EditCurve);
                EditCurve.resize(35);
                applyCursor();
                /* this is ok not to call to purgeHandler
                * in continuous creation mode because the
                * handler is destroyed by the quit() method on pressing the
                * right button of the mouse */
            }
            else {
                sketchgui->purgeHandler(); // no code after this line, Handler get deleted in ViewProvider
            }
            SnapMode = SNAP_MODE_Straight;
        }
        return true;
    }
protected:
    BoxMode Mode;
    SNAP_MODE SnapMode;
    SNAP_DIR SnapDir;
    Base::Vector2d StartPos;
    Base::Vector2d SecondPos;
    double dx, dy, r;
    std::vector<Base::Vector2d> EditCurve;
    std::vector<AutoConstraint> sugConstr1, sugConstr2;
};

DEF_STD_CMD_A(CmdSketcherCreateSlot)

CmdSketcherCreateSlot::CmdSketcherCreateSlot()
    : Command("Sketcher_CreateSlot")
{
    sAppModule = "Sketcher";
    sGroup = "Sketcher";
    sMenuText = QT_TR_NOOP("Create slot");
    sToolTipText = QT_TR_NOOP("Create a slot in the sketch");
    sWhatsThis = "Sketcher_CreateSlot";
    sStatusTip = sToolTipText;
    sPixmap = "Sketcher_CreateSlot";
    sAccel  = "G, S";
    eType = ForEdit;
}

void CmdSketcherCreateSlot::activated(int iMsg)
{
    Q_UNUSED(iMsg);
    ActivateHandler(getActiveGuiDocument(), new DrawSketchHandlerSlot());
}

bool CmdSketcherCreateSlot::isActive(void)
{
    return isCreateGeoActive(getActiveGuiDocument());
}

/* Create Arc Slot =========================================================*/

class DrawSketchHandlerArcSlot : public DrawSketchHandler
{
public:

    DrawSketchHandlerArcSlot()
        : Mode(STATUS_SEEK_First)
        , SnapMode(SNAP_MODE_Free)
        , SnapDir(SNAP_DIR_Horz)
        , dx1(0), dy1(0), r(0)
        , EditCurve(33)
    {}
    virtual ~DrawSketchHandlerArcSlot() {}
    /// mode table
    enum BoxMode {
        STATUS_SEEK_First,      /**< enum value ----. */
        STATUS_SEEK_Second,      /**< enum value ----. */
        STATUS_SEEK_Third,      /**< enum value ----. */
        STATUS_SEEK_Fourth,     /**< enum value ----. */
        STATUS_End
    };

    enum SNAP_MODE
    {
        SNAP_MODE_Free,
        SNAP_MODE_Straight
    };

    enum SNAP_DIR
    {
        SNAP_DIR_Horz,
        SNAP_DIR_Vert
    };

    virtual void activated(ViewProviderSketch*)
    {
        setCrosshairCursor("Sketcher_Pointer_Slot");
        sketchgui->toolSettings->widget->setSettings(13);
    }

    virtual void mouseMove(Base::Vector2d onSketchPos)
    {

        if (Mode == STATUS_SEEK_First) {
            setPositionText(onSketchPos);

            if (sketchgui->toolSettings->widget->isSettingSet[0] + sketchgui->toolSettings->widget->isSettingSet[1] == 2) {
                pressButton(onSketchPos);
                releaseButton(onSketchPos);
            }

            if (seekAutoConstraint(sugConstr1, onSketchPos, Base::Vector2d(0.f, 0.f))) {
                renderSuggestConstraintsCursor(sugConstr1);
                return;
            }
        }
        else if (Mode == STATUS_SEEK_Second) {
            if (sketchgui->toolSettings->widget->isSettingSet[2] == 1) {
                double dx2 = onSketchPos.x - CenterPos.x;
                double dy2 = onSketchPos.y - CenterPos.y;

                dx1 = sketchgui->toolSettings->widget->toolParameters[2] * dx2 / sqrt(dx2 * dx2 + dy2 * dy2);
                dy1 = sketchgui->toolSettings->widget->toolParameters[2] * dy2 / sqrt(dx2 * dx2 + dy2 * dy2);
            }
            else {
                dx1 = onSketchPos.x - CenterPos.x;
                dy1 = onSketchPos.y - CenterPos.y;
            }

            //Arc radius
            ra = sqrt(dx1 * dx1 + dy1 * dy1);

            //Start angle
            if (sketchgui->toolSettings->widget->isSettingSet[3] == 1) {
                startAngle = sketchgui->toolSettings->widget->toolParameters[3] * M_PI / 180;
                dx1 = cos(startAngle) * ra;
                dy1 = sin(startAngle) * ra;
            }
            else {
                startAngle = atan2(dy1, dx1);
            }

            StartPos.x = CenterPos.x + dx1;
            StartPos.y = CenterPos.y + dy1;

            // draw the circle with 32 segments
            for (int i = 0; i < 16; i++) {
                double angle = i * M_PI / 16.0;
                double rx = dx1 * cos(angle) + dy1 * sin(angle);
                double ry = -dx1 * sin(angle) + dy1 * cos(angle);
                EditCurve[0 + i] = Base::Vector2d(CenterPos.x + rx, CenterPos.y + ry);
                EditCurve[16 + i] = Base::Vector2d(CenterPos.x - rx, CenterPos.y - ry);
            }
            EditCurve[32] = EditCurve[0];

            SbString text;
            text.sprintf(" (%.1fR %.1fdeg %.1fx %.1fy )", ra, startAngle * 180 / M_PI, onSketchPos.x, onSketchPos.x);
            setPositionText(onSketchPos, text);

            drawEdit(EditCurve);
            if (seekAutoConstraint(sugConstr2, onSketchPos, Base::Vector2d(0.f, 0.f))) {
                renderSuggestConstraintsCursor(sugConstr2);
                return;
            }

            if (sketchgui->toolSettings->widget->isSettingSet[2] == 1 && sketchgui->toolSettings->widget->isSettingSet[3] == 1) {
                pressButton(onSketchPos);
                releaseButton(onSketchPos);
            }
        }
        else if (Mode == STATUS_SEEK_Third) {
            dx2 = onSketchPos.x - CenterPos.x;
            dy2 = onSketchPos.y - CenterPos.y;

            if (sketchgui->toolSettings->widget->isSettingSet[4] == 1) {
                arcAngle = sketchgui->toolSettings->widget->toolParameters[4] * M_PI / 180;
                dx2 = cos(startAngle + arcAngle) * ra;
                dy2 = sin(startAngle + arcAngle) * ra;
            }
            else {
                double angle1 = atan2(onSketchPos.y - CenterPos.y,
                    onSketchPos.x - CenterPos.x) - startAngle;
                double angle2 = angle1 + (angle1 < 0. ? 2 : -2) * M_PI;
                arcAngle = abs(angle1 - arcAngle) < abs(angle2 - arcAngle) ? angle1 : angle2;
            }

            //find the center of the third arc.
            EndPos.x = sqrt(pow(ra, 2) / (pow(dy2 / dx2, 2) + 1)) * (dx2 < 0 ? -1 : 1) + CenterPos.x;
            EndPos.y = sqrt(pow(ra, 2) / (pow(dx2 / dy2, 2) + 1)) * (dy2 < 0 ? -1 : 1) + CenterPos.y;

            r = ra/10; //Auto radius to 1/10 of the arc radius

            EditCurve.resize(89);

            double dx11 = sqrt(pow(ra - r,2) / (pow(dy1 / dx1, 2) + 1));
            double dy11 = sqrt(pow(ra - r, 2) / (pow(dx1 / dy1, 2) + 1));
            double dx12 = sqrt(pow(ra + r, 2) / (pow(dy1 / dx1, 2) + 1));
            double dy12 = sqrt(pow(ra + r, 2) / (pow(dx1 / dy1, 2) + 1));
            if (dx1 < 0) { 
                dx11 = -dx11; 
                dx12 = -dx12;
            }
            if (dy1 < 0) { 
                dy11 = -dy11; 
                dy12 = -dy12;
            }
            for (int i = 0; i < 28; i++) {
                double angle = i * arcAngle / 27.0;
                double rx = dx11 * cos(angle) - dy11 * sin(angle);
                double ry = dx11 * sin(angle) + dy11 * cos(angle);
                EditCurve[i] = Base::Vector2d(CenterPos.x + rx, CenterPos.y + ry);
                double rx2 = dx12 * cos(angle) - dy12 * sin(angle);
                double ry2 = dx12 * sin(angle) + dy12 * cos(angle);
                EditCurve[28+16 +27-i] = Base::Vector2d(CenterPos.x + rx2, CenterPos.y + ry2);
            }

            double angle3 = arcAngle < 0 ? M_PI : -M_PI;
            for (int i = 0; i < 16; i++) {
                double angle = i * angle3 / 16.0;
                double rx = (EditCurve[71].x - StartPos.x) * cos(angle) - (EditCurve[71].y - StartPos.y) * sin(angle);
                double ry = (EditCurve[71].x - StartPos.x) * sin(angle) + (EditCurve[71].y - StartPos.y) * cos(angle);
                EditCurve[28 + 16 + 28 + i] = Base::Vector2d(StartPos.x + rx, StartPos.y + ry);
                double rx2 = (EditCurve[27].x - EndPos.x) * cos(angle) - (EditCurve[27].y - EndPos.y) * sin(angle);
                double ry2 = (EditCurve[27].x - EndPos.x) * sin(angle) + (EditCurve[27].y - EndPos.y) * cos(angle);
                EditCurve[28 + i] = Base::Vector2d(EndPos.x + rx2, EndPos.y + ry2);
            }
            EditCurve[88] = EditCurve[0];

            // Display radius and arc angle
            SbString text;
            text.sprintf(" (%.1fR,%.1fdeg %.1fx %.1fy )", ra, arcAngle * 180 / M_PI, onSketchPos.x, onSketchPos.x);
            setPositionText(onSketchPos, text);

            drawEdit(EditCurve);
            if (seekAutoConstraint(sugConstr3, onSketchPos, Base::Vector2d(0.0, 0.0))) {
                renderSuggestConstraintsCursor(sugConstr3);
                return;
            }

            if (sketchgui->toolSettings->widget->isSettingSet[4] == 1) {
                pressButton(onSketchPos);
                releaseButton(onSketchPos);
            }
        }
        else if (Mode == STATUS_SEEK_Fourth) {
            if (sketchgui->toolSettings->widget->isSettingSet[5] == 1) {
                r = sketchgui->toolSettings->widget->toolParameters[5];
            }
            else {
                //radius at sketch pos
                r = min(ra * 0.999, abs(ra - sqrt(pow(onSketchPos.x - CenterPos.x, 2) + pow(onSketchPos.y - CenterPos.y, 2))));
            }
            
            double dx11 = sqrt(pow(ra - r, 2) / (pow(dy1 / dx1, 2) + 1));
            double dy11 = sqrt(pow(ra - r, 2) / (pow(dx1 / dy1, 2) + 1));
            double dx12 = sqrt(pow(ra + r, 2) / (pow(dy1 / dx1, 2) + 1));
            double dy12 = sqrt(pow(ra + r, 2) / (pow(dx1 / dy1, 2) + 1));
            if (dx1 < 0) {
                dx11 = -dx11;
                dx12 = -dx12;
            }
            if (dy1 < 0) {
                dy11 = -dy11;
                dy12 = -dy12;
            }
            for (int i = 0; i < 28; i++) {
                double angle = i * arcAngle / 27.0;
                double rx = dx11 * cos(angle) - dy11 * sin(angle);
                double ry = dx11 * sin(angle) + dy11 * cos(angle);
                EditCurve[i] = Base::Vector2d(CenterPos.x + rx, CenterPos.y + ry);
                double rx2 = dx12 * cos(angle) - dy12 * sin(angle);
                double ry2 = dx12 * sin(angle) + dy12 * cos(angle);
                EditCurve[28 + 16 + 27 - i] = Base::Vector2d(CenterPos.x + rx2, CenterPos.y + ry2);
            }

            double angle3 = arcAngle < 0 ? M_PI : -M_PI;
            for (int i = 0; i < 16; i++) {
                double angle = i * angle3 / 16.0;
                double rx = (EditCurve[71].x - StartPos.x) * cos(angle) - (EditCurve[71].y - StartPos.y) * sin(angle);
                double ry = (EditCurve[71].x - StartPos.x) * sin(angle) + (EditCurve[71].y - StartPos.y) * cos(angle);
                EditCurve[28 + 16 + 28 + i] = Base::Vector2d(StartPos.x + rx, StartPos.y + ry);
                double rx2 = (EditCurve[27].x - EndPos.x) * cos(angle) - (EditCurve[27].y - EndPos.y) * sin(angle);
                double ry2 = (EditCurve[27].x - EndPos.x) * sin(angle) + (EditCurve[27].y - EndPos.y) * cos(angle);
                EditCurve[28 + i] = Base::Vector2d(EndPos.x + rx2, EndPos.y + ry2);
            }
            EditCurve[88] = EditCurve[0];

            // Display radius and arc angle

            SbString text;
            text.sprintf(" (%.1fR,%.1fdeg %.1fx %.1fy )", r, arcAngle * 180 / M_PI, onSketchPos.x, onSketchPos.x);
            setPositionText(onSketchPos, text);

            drawEdit(EditCurve);

            if (sketchgui->toolSettings->widget->isSettingSet[5] == 1) {
                pressButton(onSketchPos);
                releaseButton(onSketchPos);
            }
        }
        applyCursor();
    }

    virtual bool pressButton(Base::Vector2d onSketchPos)
    {
        if (Mode == STATUS_SEEK_First) {
            CenterPos = onSketchPos;
            if (sketchgui->toolSettings->widget->isSettingSet[0] == 1) {
                CenterPos.x = sketchgui->toolSettings->widget->toolParameters[0];
            }
            if (sketchgui->toolSettings->widget->isSettingSet[1] == 1) {
                CenterPos.y = sketchgui->toolSettings->widget->toolParameters[1];
            }

            sketchgui->toolSettings->widget->setParameterActive(0, 0);
            sketchgui->toolSettings->widget->setParameterActive(0, 1);
            sketchgui->toolSettings->widget->setParameterActive(1, 2);
            sketchgui->toolSettings->widget->setParameterActive(1, 3);
            sketchgui->toolSettings->widget->setParameterFocus(2);
            Mode = STATUS_SEEK_Second;
        }
        else if (Mode == STATUS_SEEK_Second) {
            arcAngle = 0.;

            sketchgui->toolSettings->widget->setParameterActive(0, 2);
            sketchgui->toolSettings->widget->setParameterActive(0, 3);
            sketchgui->toolSettings->widget->setParameterActive(1, 4);
            sketchgui->toolSettings->widget->setParameterFocus(4);
            Mode = STATUS_SEEK_Third;
        }
        else if (Mode == STATUS_SEEK_Third) {
            if (arcAngle > 0)
                endAngle = startAngle + arcAngle;
            else {
                endAngle = startAngle;
                startAngle += arcAngle;
            }

            sketchgui->toolSettings->widget->setParameterActive(0, 4);
            sketchgui->toolSettings->widget->setParameterActive(1, 5);
            sketchgui->toolSettings->widget->setParameterFocus(5);
            Mode = STATUS_SEEK_Fourth;
        }
        else {
            Mode = STATUS_End;
        }
        return true;
    }

    virtual bool releaseButton(Base::Vector2d onSketchPos)
    {
        Q_UNUSED(onSketchPos);
        if (Mode == STATUS_End) {
            unsetCursor();
            resetPositionText();

            int firstCurve = getHighestCurveIndex() + 1;
            
            try {
                Gui::Command::openCommand(QT_TRANSLATE_NOOP("Command", "Add slot"));

                AutoConstraint lastCons = { Sketcher::None, Sketcher::GeoEnum::GeoUndef, Sketcher::PointPos::none };

                if (!sugConstr2.empty()) lastCons = sugConstr2.back();

                Gui::Command::doCommand(Gui::Command::Doc,
                    "geoList = []\n"
                    "geoList.append(Part.ArcOfCircle(Part.Circle(App.Vector(%f, %f, 0), App.Vector(0, 0, 1), %f), %f, %f))\n"
                    "geoList.append(Part.ArcOfCircle(Part.Circle(App.Vector(%f, %f ,0), App.Vector(0, 0, 1), %f), %f, %f))\n"
                    "geoList.append(Part.ArcOfCircle(Part.Circle(App.Vector(%f, %f ,0), App.Vector(0, 0, 1), %f), %f, %f))\n"
                    "geoList.append(Part.ArcOfCircle(Part.Circle(App.Vector(%f, %f ,0), App.Vector(0, 0, 1), %f), %f, %f))\n"
                    "%s.addGeometry(geoList, %s)\n"
                    "conList = []\n"
                    "conList.append(Sketcher.Constraint('Coincident', %i, 3, %i, 3))\n"
                    "conList.append(Sketcher.Constraint('Tangent', %i, %i, %i, %i))\n"
                    "conList.append(Sketcher.Constraint('Tangent', %i, %i, %i, %i))\n"
                    "conList.append(Sketcher.Constraint('Tangent', %i, %i, %i, %i))\n"
                    "conList.append(Sketcher.Constraint('Tangent', %i, %i, %i, %i))\n"
                    "%s.addConstraint(conList)\n"
                    "del geoList, conList\n",
                    CenterPos.x, CenterPos.y,         // center of the arc1
                    ra - r,                           // radius arc1
                    startAngle, endAngle,             // start and end angle of arc1
                    CenterPos.x, CenterPos.y,         // center of the arc2
                    ra + r,                           // radius arc2
                    startAngle, endAngle,             // start and end angle of arc2
                    StartPos.x, StartPos.y,           // center of the arc3
                    r,                                // radius arc3
                    (arcAngle > 0) ? startAngle + M_PI : endAngle,
                    (arcAngle > 0) ? startAngle + 2 * M_PI : endAngle + M_PI,    // start and end angle of arc3
                    EndPos.x, EndPos.y,               // center of the arc4
                    r,                                // radius arc4
                    (arcAngle > 0) ? endAngle : startAngle + M_PI,
                    (arcAngle > 0) ? endAngle + M_PI :startAngle + 2*M_PI,        // start and end angle of arc4
                    Gui::Command::getObjectCmd(sketchgui->getObject()).c_str(), // the sketch
                    geometryCreationMode == Construction ? "True" : "False", // geometry as construction or not
                    firstCurve, firstCurve + 1,      // coicident1: mid of the two arcs
                    firstCurve    , (arcAngle > 0) ? Sketcher::PointPos::end : Sketcher::PointPos::start,     //tangent1
                    firstCurve + 3, (arcAngle > 0) ? Sketcher::PointPos::end : Sketcher::PointPos::start,     //tangent1
                    firstCurve    , (arcAngle > 0) ? Sketcher::PointPos::start : Sketcher::PointPos::end,     //tangent2
                    firstCurve + 2, (arcAngle > 0) ? Sketcher::PointPos::start : Sketcher::PointPos::end,     //tangent2
                    firstCurve + 1, (arcAngle > 0) ? Sketcher::PointPos::end : Sketcher::PointPos::start,     //tangent3
                    firstCurve + 3, (arcAngle > 0) ? Sketcher::PointPos::start : Sketcher::PointPos::end,     //tangent3
                    firstCurve + 1, (arcAngle > 0) ? Sketcher::PointPos::start : Sketcher::PointPos::end,     //tangent4
                    firstCurve + 2, (arcAngle > 0) ? Sketcher::PointPos::end : Sketcher::PointPos::start,     //tangent4
                    Gui::Command::getObjectCmd(sketchgui->getObject()).c_str()); // the sketch


                //add constraint if user typed in some dimensions in tool widget
                if (sketchgui->toolSettings->widget->isSettingSet[0] + sketchgui->toolSettings->widget->isSettingSet[1] + sketchgui->toolSettings->widget->isSettingSet[2] + sketchgui->toolSettings->widget->isSettingSet[5] != 0) {
                    if (sketchgui->toolSettings->widget->isSettingSet[0] == 1) {
                        distanceXYorPointOnObject(0, firstCurve, Sketcher::PointPos::mid, sketchgui->toolSettings->widget->toolParameters[0]);
                    }
                    if (sketchgui->toolSettings->widget->isSettingSet[1] == 1) {
                        distanceXYorPointOnObject(1, firstCurve, Sketcher::PointPos::mid, sketchgui->toolSettings->widget->toolParameters[1]);
                    }
                    if (sketchgui->toolSettings->widget->isSettingSet[2] == 1) {
                        Gui::cmdAppObjectArgs(sketchgui->getObject(), "addConstraint(Sketcher.Constraint('Distance',%d,%d,%d,%d,%f)) ",
                            firstCurve, 3 , firstCurve + 2, 3, sketchgui->toolSettings->widget->toolParameters[2]);
                    }
                    if (sketchgui->toolSettings->widget->isSettingSet[5] == 1) {
                        Gui::cmdAppObjectArgs(sketchgui->getObject(), "addConstraint(Sketcher.Constraint('Radius',%d,%f)) ",
                            firstCurve + 2, sketchgui->toolSettings->widget->toolParameters[5]);
                    }
                }

                Gui::Command::commitCommand();

                // add auto constraints at the center of the first arc
                if (sugConstr1.size() > 0) {
                    createAutoConstraints(sugConstr1, getHighestCurveIndex() - 3, Sketcher::PointPos::mid);
                    sugConstr1.clear();
                }

                // add auto constraints
                if (sugConstr2.size() > 0) {
                    createAutoConstraints(sugConstr2, getHighestCurveIndex() - 1, Sketcher::PointPos::mid);
                    sugConstr2.clear();
                }

                // add auto constraints
                if (sugConstr3.size() > 0) {
                    createAutoConstraints(sugConstr3, getHighestCurveIndex(), Sketcher::PointPos::mid);
                    sugConstr3.clear();
                }

                tryAutoRecomputeIfNotSolve(static_cast<Sketcher::SketchObject*>(sketchgui->getObject()));
            }
            catch (const Base::Exception& e) {
                Base::Console().Error("Failed to add slot: %s\n", e.what());
                Gui::Command::abortCommand();

                tryAutoRecompute(static_cast<Sketcher::SketchObject*>(sketchgui->getObject()));
            }
            ParameterGrp::handle hGrp = App::GetApplication().GetParameterGroupByPath("User parameter:BaseApp/Preferences/Mod/Sketcher");
            bool continuousMode = hGrp->GetBool("ContinuousCreationMode", true);

            if (continuousMode) {
                // This code enables the continuous creation mode.
                sketchgui->toolSettings->widget->setSettings(13);
                Mode = STATUS_SEEK_First;
                EditCurve.clear();
                drawEdit(EditCurve);
                EditCurve.resize(33);
                applyCursor();
                /* this is ok not to call to purgeHandler
                * in continuous creation mode because the
                * handler is destroyed by the quit() method on pressing the
                * right button of the mouse */
            }
            else {
                sketchgui->purgeHandler(); // no code after this line, Handler get deleted in ViewProvider
            }
            SnapMode = SNAP_MODE_Straight;
        }
        return true;
    }
protected:
    BoxMode Mode;
    int slotType;
    SNAP_MODE SnapMode;
    SNAP_DIR SnapDir;
    Base::Vector2d CenterPos;
    Base::Vector2d StartPos;
    Base::Vector2d EndPos;
    double dx1, dy1, dx2, dy2, startAngle, endAngle, arcAngle, r, ra;
    std::vector<Base::Vector2d> EditCurve;
    std::vector<AutoConstraint> sugConstr1, sugConstr2, sugConstr3;
};

DEF_STD_CMD_A(CmdSketcherCreateArcSlot)

CmdSketcherCreateArcSlot::CmdSketcherCreateArcSlot()
    : Command("Sketcher_CreateArcSlot")
{
    sAppModule = "Sketcher";
    sGroup = "Sketcher";
    sMenuText = QT_TR_NOOP("Create arc slot");
    sToolTipText = QT_TR_NOOP("Create an arc slot in the sketch");
    sWhatsThis = "Sketcher_CreateArcSlot";
    sStatusTip = sToolTipText;
    sPixmap = "Sketcher_CreateArcSlot";
    sAccel = "G, S, 2";
    eType = ForEdit;
}

void CmdSketcherCreateArcSlot::activated(int iMsg)
{
    Q_UNUSED(iMsg);
    ActivateHandler(getActiveGuiDocument(), new DrawSketchHandlerArcSlot());
}

bool CmdSketcherCreateArcSlot::isActive(void)
{
    return isCreateGeoActive(getActiveGuiDocument());
}

/* Create Rectangle Slot =========================================================*/

class DrawSketchHandlerRectangleSlot : public DrawSketchHandler
{
public:
    DrawSketchHandlerRectangleSlot()
        : Mode(STATUS_SEEK_First)
        , SnapMode(SNAP_MODE_Free)
        , SnapDir(SNAP_DIR_Horz)
        , dx1(0), dy1(0), r(0)
        , EditCurve(33)
    {}
    virtual ~DrawSketchHandlerRectangleSlot() {}
    /// mode table
    enum BoxMode {
        STATUS_SEEK_First,      /**< enum value ----. */
        STATUS_SEEK_Second,      /**< enum value ----. */
        STATUS_SEEK_Third,      /**< enum value ----. */
        STATUS_SEEK_Fourth,     /**< enum value ----. */
        STATUS_End
    };

    enum SNAP_MODE
    {
        SNAP_MODE_Free,
        SNAP_MODE_Straight
    };

    enum SNAP_DIR
    {
        SNAP_DIR_Horz,
        SNAP_DIR_Vert
    };

    virtual void activated(ViewProviderSketch*)
    {
        setCrosshairCursor("Sketcher_Pointer_Slot");
        sketchgui->toolSettings->widget->setSettings(14);

    }

    virtual void mouseMove(Base::Vector2d onSketchPos)
    {

        if (Mode == STATUS_SEEK_First) {
            setPositionText(onSketchPos);

            if (sketchgui->toolSettings->widget->isSettingSet[0] + sketchgui->toolSettings->widget->isSettingSet[1] == 2) {
                pressButton(onSketchPos);
                releaseButton(onSketchPos);
            }

            if (seekAutoConstraint(sugConstr1, onSketchPos, Base::Vector2d(0.f, 0.f))) {
                renderSuggestConstraintsCursor(sugConstr1);
                return;
            }
        }
        else if (Mode == STATUS_SEEK_Second) {
            if (sketchgui->toolSettings->widget->isSettingSet[2] == 1) {
                double dx2 = onSketchPos.x - CenterPos.x;
                double dy2 = onSketchPos.y - CenterPos.y;

                dx1 = sketchgui->toolSettings->widget->toolParameters[2] * dx2 / sqrt(dx2 * dx2 + dy2 * dy2);
                dy1 = sketchgui->toolSettings->widget->toolParameters[2] * dy2 / sqrt(dx2 * dx2 + dy2 * dy2);
            }
            else {
                dx1 = onSketchPos.x - CenterPos.x;
                dy1 = onSketchPos.y - CenterPos.y;
            }

            //Arc radius
            r = sqrt(dx1 * dx1 + dy1 * dy1);

            //Start angle
            if (sketchgui->toolSettings->widget->isSettingSet[3] == 1) {
                startAngle = sketchgui->toolSettings->widget->toolParameters[3] * M_PI / 180;
                dx1 = cos(startAngle) * r;
                dy1 = sin(startAngle) * r;
            }
            else {
                startAngle = atan2(dy1, dx1);
            }

            StartPos.x = CenterPos.x + dx1;
            StartPos.y = CenterPos.y + dy1;

            // draw the circle with 32 segments
            for (int i = 0; i < 16; i++) {
                double angle = i * M_PI / 16.0;
                double rx = dx1 * cos(angle) + dy1 * sin(angle);
                double ry = -dx1 * sin(angle) + dy1 * cos(angle);
                EditCurve[0 + i] = Base::Vector2d(CenterPos.x + rx, CenterPos.y + ry);
                EditCurve[16 + i] = Base::Vector2d(CenterPos.x - rx, CenterPos.y - ry);
            }
            EditCurve[32] = EditCurve[0];

            SbString text;
            text.sprintf(" (%.1fa %.1fR %.1fx %.1fy )", startAngle, r, onSketchPos.x, onSketchPos.x);
            setPositionText(onSketchPos, text);

            drawEdit(EditCurve);
            if (seekAutoConstraint(sugConstr2, onSketchPos, Base::Vector2d(0.f, 0.f))) {
                renderSuggestConstraintsCursor(sugConstr2);
                return;
            }
            if (sketchgui->toolSettings->widget->isSettingSet[2] == 1 && sketchgui->toolSettings->widget->isSettingSet[3] == 1) {
                pressButton(onSketchPos);
                releaseButton(onSketchPos);
            }
        }
        else if (Mode == STATUS_SEEK_Third) {

            //End angle
            dx2 = onSketchPos.x - CenterPos.x;
            dy2 = onSketchPos.y - CenterPos.y;

            if (sketchgui->toolSettings->widget->isSettingSet[5] == 1) {
                r2 = sketchgui->toolSettings->widget->toolParameters[5];
            }
            else {
                r2 = sqrt(pow(dx2, 2) + pow(dy2, 2));
            }

            if (sketchgui->toolSettings->widget->isSettingSet[4] == 1) {
                arcAngle = sketchgui->toolSettings->widget->toolParameters[4] * M_PI / 180;
                dx2 = cos(startAngle + arcAngle) * r2;
                dy2 = sin(startAngle + arcAngle) * r2;
            }
            else {
                double angle1 = atan2(onSketchPos.y - CenterPos.y,
                    onSketchPos.x - CenterPos.x) - startAngle;
                double angle2 = angle1 + (angle1 < 0. ? 2 : -2) * M_PI;
                arcAngle = abs(angle1 - arcAngle) < abs(angle2 - arcAngle) ? angle1 : angle2;
            }

            //find the center of the third arc.
            EndPos.x = dx2 + CenterPos.x;
            EndPos.y = dy2 + CenterPos.y;

            EditCurve.resize(57);

            for (int i = 0; i < 28; i++) {
                double angle = i * arcAngle / 27.0;
                double rx = dx1 * cos(angle) - dy1 * sin(angle);
                double ry = dx1 * sin(angle) + dy1 * cos(angle);
                EditCurve[i] = Base::Vector2d(CenterPos.x + rx, CenterPos.y + ry);
                double rx2 = dx2 * cos(-angle) - dy2 * sin(-angle);
                double ry2 = dx2 * sin(-angle) + dy2 * cos(-angle);
                EditCurve[28 + i] = Base::Vector2d(CenterPos.x + rx2, CenterPos.y + ry2);
            }
            EditCurve[56] = EditCurve[0];

            // Display radius and arc angle
            SbString text;
            text.sprintf(" (%.1fR,%.1fdeg)", r2, arcAngle * 180 / M_PI);
            setPositionText(onSketchPos, text);

            drawEdit(EditCurve);
            if (seekAutoConstraint(sugConstr3, onSketchPos, Base::Vector2d(0.0, 0.0))) {
                renderSuggestConstraintsCursor(sugConstr3);
                return;
            }
            if (sketchgui->toolSettings->widget->isSettingSet[4] + sketchgui->toolSettings->widget->isSettingSet[5] == 2) {
                pressButton(onSketchPos);
                releaseButton(onSketchPos);
            }
        }
        applyCursor();
    }

    virtual bool pressButton(Base::Vector2d onSketchPos)
    {
        if (Mode == STATUS_SEEK_First) {
            CenterPos = onSketchPos;
            if (sketchgui->toolSettings->widget->isSettingSet[0] == 1) {
                CenterPos.x = sketchgui->toolSettings->widget->toolParameters[0];
            }
            if (sketchgui->toolSettings->widget->isSettingSet[1] == 1) {
                CenterPos.y = sketchgui->toolSettings->widget->toolParameters[1];
            }

            sketchgui->toolSettings->widget->setParameterActive(0, 0);
            sketchgui->toolSettings->widget->setParameterActive(0, 1);
            sketchgui->toolSettings->widget->setParameterActive(1, 2);
            sketchgui->toolSettings->widget->setParameterActive(1, 3);
            sketchgui->toolSettings->widget->setParameterFocus(2);
            Mode = STATUS_SEEK_Second;
        }
        else if (Mode == STATUS_SEEK_Second) {
            arcAngle = 0.;
            sketchgui->toolSettings->widget->setParameterActive(0, 2);
            sketchgui->toolSettings->widget->setParameterActive(0, 3);
            sketchgui->toolSettings->widget->setParameterActive(1, 4);
            sketchgui->toolSettings->widget->setParameterActive(1, 5);
            sketchgui->toolSettings->widget->setParameterFocus(4);
            Mode = STATUS_SEEK_Third;
        }
        else if (Mode == STATUS_SEEK_Third) {
            if (arcAngle > 0)
                endAngle = startAngle + arcAngle;
            else {
                endAngle = startAngle;
                startAngle += arcAngle;
            }
            Mode = STATUS_End;
        }
        else {
            Mode = STATUS_End;
        }
        return true;
    }

    virtual bool releaseButton(Base::Vector2d onSketchPos)
    {
        Q_UNUSED(onSketchPos);
        if (Mode == STATUS_End) {
            unsetCursor();
            resetPositionText();

            int firstCurve = getHighestCurveIndex() + 1;

            try {
                Gui::Command::openCommand(QT_TRANSLATE_NOOP("Command", "Add slot"));

                AutoConstraint lastCons = { Sketcher::None, Sketcher::GeoEnum::GeoUndef, Sketcher::PointPos::none };

                if (!sugConstr2.empty()) lastCons = sugConstr2.back();

                Gui::Command::doCommand(Gui::Command::Doc,
                    "geoList = []\n"
                    "geoList.append(Part.ArcOfCircle(Part.Circle(App.Vector(%f, %f, 0), App.Vector(0, 0, 1), %f), %f, %f))\n"
                    "geoList.append(Part.ArcOfCircle(Part.Circle(App.Vector(%f, %f ,0), App.Vector(0, 0, 1), %f), %f, %f))\n"
                    "geoList.append(Part.LineSegment(App.Vector(%f, %f, 0), App.Vector(%f, %f, 0)))\n"
                    "geoList.append(Part.LineSegment(App.Vector(%f, %f, 0), App.Vector(%f, %f, 0)))\n"
                    "%s.addGeometry(geoList, %s)\n"
                    "conList = []\n"
                    "conList.append(Sketcher.Constraint('Perpendicular', %i, 0, %i, 0))\n"
                    "conList.append(Sketcher.Constraint('Perpendicular', %i, 0, %i, 0))\n"
                    "conList.append(Sketcher.Constraint('Coincident', %i, 3, %i, 3))\n"
                    "conList.append(Sketcher.Constraint('Coincident', %i, %i, %i, 2))\n"
                    "conList.append(Sketcher.Constraint('Coincident', %i, %i, %i, 1))\n"
                    "conList.append(Sketcher.Constraint('Coincident', %i, %i, %i, 1))\n"
                    "conList.append(Sketcher.Constraint('Coincident', %i, %i, %i, 2))\n"
                    "%s.addConstraint(conList)\n"
                    "del geoList, conList\n",
                    CenterPos.x, CenterPos.y,         // center of the arc1
                    r,                                // radius arc1
                    startAngle, endAngle,             // start and end angle of arc1
                    CenterPos.x, CenterPos.y,         // center of the arc2
                    r2,                               // radius arc2
                    startAngle, endAngle,             // start and end angle of arc2
                    EditCurve[27].x, EditCurve[27].y, EditCurve[28].x, EditCurve[28].y, // line1
                    EditCurve[55].x, EditCurve[55].y, EditCurve[56].x, EditCurve[56].y, // line2
                    Gui::Command::getObjectCmd(sketchgui->getObject()).c_str(), // the sketch
                    geometryCreationMode == Construction ? "True" : "False", // geometry as construction or not
                    firstCurve, firstCurve + 2,     // perpendicular1
                    firstCurve, firstCurve + 3,     // perpendicular2
                    firstCurve, firstCurve + 1,      // coicident1: mid of the two arcs
                    firstCurve, (arcAngle > 0) ? Sketcher::PointPos::start : Sketcher::PointPos::end, firstCurve + 3,     // coicident2
                    firstCurve, (arcAngle > 0) ? Sketcher::PointPos::end : Sketcher::PointPos::start, firstCurve + 2,     // coicident3
                    firstCurve + 1, (arcAngle > 0) ? Sketcher::PointPos::start : Sketcher::PointPos::end, firstCurve + 3, // coicident4
                    firstCurve + 1, (arcAngle > 0) ? Sketcher::PointPos::end : Sketcher::PointPos::start, firstCurve + 2, // coicident5
                    Gui::Command::getObjectCmd(sketchgui->getObject()).c_str()); // the sketch

                //add constraint if user typed in some dimensions in tool widget
                if (sketchgui->toolSettings->widget->isSettingSet[0] + sketchgui->toolSettings->widget->isSettingSet[1] + sketchgui->toolSettings->widget->isSettingSet[2] + sketchgui->toolSettings->widget->isSettingSet[5] != 0) {
                    if (sketchgui->toolSettings->widget->isSettingSet[0] == 1) {
                        distanceXYorPointOnObject(0, firstCurve, Sketcher::PointPos::mid, sketchgui->toolSettings->widget->toolParameters[0]);
                    }
                    if (sketchgui->toolSettings->widget->isSettingSet[1] == 1) {
                        distanceXYorPointOnObject(1, firstCurve, Sketcher::PointPos::mid, sketchgui->toolSettings->widget->toolParameters[1]);
                    }
                    if (sketchgui->toolSettings->widget->isSettingSet[2] == 1) {
                        Gui::cmdAppObjectArgs(sketchgui->getObject(), "addConstraint(Sketcher.Constraint('Radius',%d,%f)) ",
                            firstCurve, sketchgui->toolSettings->widget->toolParameters[2]);
                    }
                    if (sketchgui->toolSettings->widget->isSettingSet[3] == 1) {
                        if (sketchgui->toolSettings->widget->toolParameters[3] == 0 ) {
                            Gui::cmdAppObjectArgs(sketchgui->getObject(), "addConstraint(Sketcher.Constraint('Horizontal',%d)) ", firstCurve + 2);
                        }
                        else if (sketchgui->toolSettings->widget->toolParameters[3] == 90 ) {
                            Gui::cmdAppObjectArgs(sketchgui->getObject(), "addConstraint(Sketcher.Constraint('Vertical',%d)) ", firstCurve + 2);
                        }
                        else {
                            Gui::cmdAppObjectArgs(sketchgui->getObject(), "addConstraint(Sketcher.Constraint('Angle',%d,%d,%f)) ",
                                firstCurve + 2, Sketcher::GeoEnum::HAxis, sketchgui->toolSettings->widget->toolParameters[3] * M_PI / 180);
                        }
                    }
                    if (sketchgui->toolSettings->widget->isSettingSet[4] == 1) {
                        if (sketchgui->toolSettings->widget->toolParameters[4] == 0) {
                            Gui::cmdAppObjectArgs(sketchgui->getObject(), "addConstraint(Sketcher.Constraint('Horizontal',%d)) ", firstCurve + 3);
                        }
                        else if (sketchgui->toolSettings->widget->toolParameters[4] == 90) {
                            Gui::cmdAppObjectArgs(sketchgui->getObject(), "addConstraint(Sketcher.Constraint('Vertical',%d)) ", firstCurve + 3);
                        }
                        else {
                            Gui::cmdAppObjectArgs(sketchgui->getObject(), "addConstraint(Sketcher.Constraint('Angle',%d,%d,%f)) ",
                                firstCurve + 2, firstCurve + 3, sketchgui->toolSettings->widget->toolParameters[4] * M_PI / 180);
                        }
                    }
                    if (sketchgui->toolSettings->widget->isSettingSet[5] == 1) {
                        Gui::cmdAppObjectArgs(sketchgui->getObject(), "addConstraint(Sketcher.Constraint('Radius',%d,%f)) ",
                            firstCurve + 1, sketchgui->toolSettings->widget->toolParameters[5]);
                    }
                }

                Gui::Command::commitCommand();

                // add auto constraints at the center of the first arc
                if (sugConstr1.size() > 0) {
                    createAutoConstraints(sugConstr1, getHighestCurveIndex() - 3, Sketcher::PointPos::mid);
                    sugConstr1.clear();
                }

                // add auto constraints
                if (sugConstr2.size() > 0) {
                    createAutoConstraints(sugConstr2, getHighestCurveIndex() - 3, (arcAngle > 0) ? Sketcher::PointPos::start : Sketcher::PointPos::end);
                    sugConstr2.clear();
                }

                // add auto constraints
                if (sugConstr3.size() > 0) {
                    createAutoConstraints(sugConstr3, getHighestCurveIndex() - 2, (arcAngle > 0) ? Sketcher::PointPos::end : Sketcher::PointPos::start);
                    sugConstr3.clear();
                }

                tryAutoRecomputeIfNotSolve(static_cast<Sketcher::SketchObject*>(sketchgui->getObject()));
            }
            catch (const Base::Exception& e) {
                Base::Console().Error("Failed to add slot: %s\n", e.what());
                Gui::Command::abortCommand();

                tryAutoRecompute(static_cast<Sketcher::SketchObject*>(sketchgui->getObject()));
            }
            ParameterGrp::handle hGrp = App::GetApplication().GetParameterGroupByPath("User parameter:BaseApp/Preferences/Mod/Sketcher");
            bool continuousMode = hGrp->GetBool("ContinuousCreationMode", true);

            if (continuousMode) {
                // This code enables the continuous creation mode.
                sketchgui->toolSettings->widget->setSettings(14);
                Mode = STATUS_SEEK_First;
                EditCurve.clear();
                drawEdit(EditCurve);
                EditCurve.resize(33);
                applyCursor();
                /* this is ok not to call to purgeHandler
                * in continuous creation mode because the
                * handler is destroyed by the quit() method on pressing the
                * right button of the mouse */
            }
            else {
                sketchgui->purgeHandler(); // no code after this line, Handler get deleted in ViewProvider
            }
            SnapMode = SNAP_MODE_Straight;
        }
        return true;
    }
protected:
    BoxMode Mode;
    int slotType;
    SNAP_MODE SnapMode;
    SNAP_DIR SnapDir;
    Base::Vector2d CenterPos;
    Base::Vector2d StartPos;
    Base::Vector2d EndPos;
    double dx1, dy1, dx2, dy2, startAngle, endAngle, arcAngle, r, r2;
    std::vector<Base::Vector2d> EditCurve;
    std::vector<AutoConstraint> sugConstr1, sugConstr2, sugConstr3;
};

DEF_STD_CMD_A(CmdSketcherCreateRectangleSlot)

CmdSketcherCreateRectangleSlot::CmdSketcherCreateRectangleSlot()
    : Command("Sketcher_CreateRectangleSlot")
{
    sAppModule = "Sketcher";
    sGroup = "Sketcher";
    sMenuText = QT_TR_NOOP("Create rectangle slot");
    sToolTipText = QT_TR_NOOP("Create an rectangle slot in the sketch");
    sWhatsThis = "Sketcher_CreateRectangleSlot";
    sStatusTip = sToolTipText;
    sPixmap = "Sketcher_CreateRectangleSlot";
    sAccel = "G, S, 4";
    eType = ForEdit;
}

void CmdSketcherCreateRectangleSlot::activated(int iMsg)
{
    Q_UNUSED(iMsg);
    ActivateHandler(getActiveGuiDocument(), new DrawSketchHandlerRectangleSlot());
}

bool CmdSketcherCreateRectangleSlot::isActive(void)
{
    return isCreateGeoActive(getActiveGuiDocument());
}

/* Slot comp ============================================================*/

DEF_STD_CMD_ACLU(CmdSketcherCompCreateSlot)

CmdSketcherCompCreateSlot::CmdSketcherCompCreateSlot()
    : Command("Sketcher_CompCreateSlot")
{
    sAppModule = "Sketcher";
    sGroup = "Sketcher";
    sMenuText = QT_TR_NOOP("Create slot");
    sToolTipText = QT_TR_NOOP("Create a slot in the sketcher");
    sWhatsThis = "Sketcher_CompCreateSlot";
    sStatusTip = sToolTipText;
    sAccel = "G, S, S";
    eType = ForEdit;
}

void CmdSketcherCompCreateSlot::activated(int iMsg)
{
    switch (iMsg) {
    case 0:
        ActivateHandler(getActiveGuiDocument(), new DrawSketchHandlerSlot()); break;
    case 1:
        ActivateHandler(getActiveGuiDocument(), new DrawSketchHandlerArcSlot()); break;
    case 2:
        ActivateHandler(getActiveGuiDocument(), new DrawSketchHandlerRectangleSlot()); break;
    default:
        return;
    }

    // Since the default icon is reset when enabling/disabling the command we have
    // to explicitly set the icon of the used command.
    Gui::ActionGroup* pcAction = qobject_cast<Gui::ActionGroup*>(_pcAction);
    QList<QAction*> a = pcAction->actions();

    assert(iMsg < a.size());
    pcAction->setIcon(a[iMsg]->icon());
}

Gui::Action* CmdSketcherCompCreateSlot::createAction(void)
{
    Gui::ActionGroup* pcAction = new Gui::ActionGroup(this, Gui::getMainWindow());
    pcAction->setDropDownMenu(true);
    applyCommandData(this->className(), pcAction);

    QAction* slot = pcAction->addAction(QString());
    slot->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateSlot"));
    QAction* arcSlot = pcAction->addAction(QString());
    arcSlot->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateArcSlot"));
    QAction* rectangleSlot = pcAction->addAction(QString());
    rectangleSlot->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateRectangleSlot"));

    _pcAction = pcAction;
    languageChange();

    pcAction->setIcon(slot->icon());
    int defaultId = 0;
    pcAction->setProperty("defaultAction", QVariant(defaultId));

    return pcAction;
}

void CmdSketcherCompCreateSlot::updateAction(int mode)
{
    Gui::ActionGroup* pcAction = qobject_cast<Gui::ActionGroup*>(getAction());
    if (!pcAction)
        return;

    QList<QAction*> a = pcAction->actions();
    int index = pcAction->property("defaultAction").toInt();
    switch (mode) {
    case Normal:
        a[0]->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateSlot"));
        a[1]->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateArcSlot"));
        a[2]->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateRectangleSlot"));
        getAction()->setIcon(a[index]->icon());
        break;
    case Construction:
        a[0]->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateSlot_Constr"));
        a[1]->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateArcSlot_Constr"));
        a[2]->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateRectangleSlot_Constr"));
        getAction()->setIcon(a[index]->icon());
        break;
    }
}

void CmdSketcherCompCreateSlot::languageChange()
{
    Command::languageChange();

    if (!_pcAction)
        return;
    Gui::ActionGroup* pcAction = qobject_cast<Gui::ActionGroup*>(_pcAction);
    QList<QAction*> a = pcAction->actions();

    QAction* twoPointsSlot = a[0];
    twoPointsSlot->setText(QApplication::translate("CmdSketcherCompCreateSlot", "Slot"));
    twoPointsSlot->setToolTip(QApplication::translate("Sketcher_CreateTriangle", "Create a slot by its two center points and radius point"));
    twoPointsSlot->setStatusTip(QApplication::translate("Sketcher_CreateTriangle", "Create a slot by its two center points and radius point"));
    QAction* arcSlot = a[1];
    arcSlot->setText(QApplication::translate("CmdSketcherCompCreateSlot", "Arc slot"));
    arcSlot->setToolTip(QApplication::translate("Sketcher_CreateSquare", "Create a slot by its arc center first"));
    arcSlot->setStatusTip(QApplication::translate("Sketcher_CreateSquare", "Create a slot by its arc center first"));
    QAction* rectangleSlot = a[2];
    rectangleSlot->setText(QApplication::translate("CmdSketcherCompCreateSlot", "Rectangle slot"));
    rectangleSlot->setToolTip(QApplication::translate("Sketcher_CreateSquare", "Create a rectangle by its arc center first"));
    rectangleSlot->setStatusTip(QApplication::translate("Sketcher_CreateSquare", "Create a rectangle by its arc center first"));
}

bool CmdSketcherCompCreateSlot::isActive(void)
{
    return isCreateGeoActive(getActiveGuiDocument());
}

/* Create Regular Polygon ==============================================*/

class DrawSketchHandlerRegularPolygon: public DrawSketchHandler
{
public:
    DrawSketchHandlerRegularPolygon( size_t nof_corners ):
        Corners( nof_corners ),
        AngleOfSeparation( 2.0*M_PI/static_cast<double>(Corners) ),
        cos_v( cos( AngleOfSeparation ) ),
        sin_v( sin( AngleOfSeparation ) ),
        Mode(STATUS_SEEK_First),
        EditCurve(Corners+1)
    {
    }
    virtual ~DrawSketchHandlerRegularPolygon(){
        sketchgui->toolSettings->widget->setSettings(0);
    }
    /// mode table
    enum SelectMode {
        STATUS_SEEK_First,      /**< enum value ----. */
        STATUS_SEEK_Second,     /**< enum value ----. */
        STATUS_End
    };

    virtual void activated(ViewProviderSketch *)
    {
        setCrosshairCursor("Sketcher_Pointer_Regular_Polygon");
        sketchgui->toolSettings->widget->setSettings(3);
    }

    virtual void mouseMove(Base::Vector2d onSketchPos)
    {

        if (Mode==STATUS_SEEK_First) {
            setPositionText(onSketchPos);
            if (seekAutoConstraint(sugConstr1, onSketchPos, Base::Vector2d(0.f,0.f))) {
                renderSuggestConstraintsCursor(sugConstr1);
                return;
            }
            if (sketchgui->toolSettings->widget->isSettingSet[0] + sketchgui->toolSettings->widget->isSettingSet[1] == 2) {
                pressButton(onSketchPos);
                releaseButton(onSketchPos);
            }
        }
        else if (Mode==STATUS_SEEK_Second) {
            double rx = onSketchPos.x - StartPos.x;
            double ry = onSketchPos.y - StartPos.y;
            float angle = atan2f(ry, rx);

            if (sketchgui->toolSettings->widget->isSettingSet[2] == 1) {
                rx = cos(angle) * sketchgui->toolSettings->widget->toolParameters[2];
                ry = sin(angle) * sketchgui->toolSettings->widget->toolParameters[2];
            }

            Base::Vector2d dV = Base::Vector2d(rx,ry);

            EditCurve[0]= dV + StartPos;
            EditCurve[Corners]= dV + StartPos;

            for (int i=1; i < static_cast<int>(Corners); i++) {
                const double old_rx = rx;
                rx = cos_v * rx - sin_v * ry;
                ry = cos_v * ry + sin_v * old_rx;
                EditCurve[i] = Base::Vector2d(StartPos.x + rx, StartPos.y + ry);
            }


            SbString text;
            text.sprintf(" (%.1fR %.1fdeg)", dV.Length(), (180.0 / M_PI) * angle);
            setPositionText(onSketchPos, text);

            drawEdit(EditCurve);
            if (seekAutoConstraint(sugConstr2, onSketchPos, Base::Vector2d(0.f,0.f))) {
                renderSuggestConstraintsCursor(sugConstr2);
                return;
            }
        }
        applyCursor();
    }

    virtual bool pressButton(Base::Vector2d onSketchPos)
    {
        if (Mode==STATUS_SEEK_First){
            if (sketchgui->toolSettings->widget->isSettingSet[0] == 1) {
                StartPos.x = sketchgui->toolSettings->widget->toolParameters[0];
            }
            else {
                StartPos.x = onSketchPos.x;
            }
            if (sketchgui->toolSettings->widget->isSettingSet[1] == 1) {
                StartPos.y = sketchgui->toolSettings->widget->toolParameters[1];
            }
            else {
                StartPos.y = onSketchPos.y;
            }

            sketchgui->toolSettings->widget->setParameterActive(0, 0);
            sketchgui->toolSettings->widget->setParameterActive(0, 1);
            sketchgui->toolSettings->widget->setParameterActive(1, 2);
            sketchgui->toolSettings->widget->setParameterFocus(2);
            Mode = STATUS_SEEK_Second;
        }
        else {
            Mode = STATUS_End;
        }
        return true;
    }

    virtual bool releaseButton(Base::Vector2d onSketchPos)
    {
        Q_UNUSED(onSketchPos);
        if (Mode==STATUS_End){
            unsetCursor();
            resetPositionText();
            Gui::Command::openCommand(QT_TRANSLATE_NOOP("Command", "Add hexagon"));

            try {
                
                Gui::Command::doCommand(Gui::Command::Doc,
                        "import ProfileLib.RegularPolygon\n"
                        "ProfileLib.RegularPolygon.makeRegularPolygon(%s,%i,App.Vector(%f,%f,0),App.Vector(%f,%f,0),%s)",
                                            Gui::Command::getObjectCmd(sketchgui->getObject()).c_str(),
                                            Corners,
                                            StartPos.x,StartPos.y,EditCurve[0].x,EditCurve[0].y,
                                            geometryCreationMode==Construction?"True":"False");

                //add constraint if user typed in some dimensions in tool widget
                int lastCurve = getHighestCurveIndex(); //last geoID is the circle
                if (sketchgui->toolSettings->widget->isSettingSet[0] + sketchgui->toolSettings->widget->isSettingSet[1] + sketchgui->toolSettings->widget->isSettingSet[2] + sketchgui->toolSettings->widget->isSettingSet[3] != 0) {
                    if (sketchgui->toolSettings->widget->isSettingSet[0] == 1) {
                        distanceXYorPointOnObject(0, lastCurve, Sketcher::PointPos::mid, sketchgui->toolSettings->widget->toolParameters[0]);
                    }
                    if (sketchgui->toolSettings->widget->isSettingSet[1] == 1) {
                        distanceXYorPointOnObject(1, lastCurve, Sketcher::PointPos::mid, sketchgui->toolSettings->widget->toolParameters[1]);
                    }
                    if (sketchgui->toolSettings->widget->isSettingSet[2] == 1) {
                        Gui::cmdAppObjectArgs(sketchgui->getObject(), "addConstraint(Sketcher.Constraint('Radius',%d,%f)) ",
                            lastCurve, sketchgui->toolSettings->widget->toolParameters[2]);
                    }
                }

                Gui::Command::commitCommand();

                // add auto constraints at the center of the polygon
                if (sugConstr1.size() > 0) {
                    createAutoConstraints(sugConstr1, getHighestCurveIndex(), Sketcher::PointPos::mid);
                    sugConstr1.clear();
                }

                // add auto constraints to the last side of the polygon
                if (sugConstr2.size() > 0) {
                    createAutoConstraints(sugConstr2, getHighestCurveIndex() - 1, Sketcher::PointPos::end);
                    sugConstr2.clear();
                }

                tryAutoRecomputeIfNotSolve(static_cast<Sketcher::SketchObject *>(sketchgui->getObject()));
            }
            catch (const Base::Exception& e) {
                Base::Console().Error("Failed to add hexagon: %s\n", e.what());
                Gui::Command::abortCommand();

                tryAutoRecompute(static_cast<Sketcher::SketchObject *>(sketchgui->getObject()));
            }

            sketchgui->toolSettings->widget->setSettings(0);
            ParameterGrp::handle hGrp = App::GetApplication().GetParameterGroupByPath("User parameter:BaseApp/Preferences/Mod/Sketcher");
            bool continuousMode = hGrp->GetBool("ContinuousCreationMode",true);

            if(continuousMode){
                // This code enables the continuous creation mode.
                sketchgui->toolSettings->widget->setSettings(3);
                Mode=STATUS_SEEK_First;
                EditCurve.clear();
                drawEdit(EditCurve);
                EditCurve.resize(Corners+1);
                applyCursor();
                /* this is ok not to call to purgeHandler
                * in continuous creation mode because the
                * handler is destroyed by the quit() method on pressing the
                * right button of the mouse */
            }
            else{
                sketchgui->purgeHandler(); // no code after this line, Handler get deleted in ViewProvider
            }
        }
        return true;
    }
protected:
    const size_t Corners;
    const double AngleOfSeparation;
    const double cos_v, sin_v;
    SelectMode Mode;
    Base::Vector2d StartPos;
    std::vector<Base::Vector2d> EditCurve;
    std::vector<AutoConstraint> sugConstr1, sugConstr2;
};

DEF_STD_CMD_A(CmdSketcherCreateTriangle)

CmdSketcherCreateTriangle::CmdSketcherCreateTriangle()
  : Command("Sketcher_CreateTriangle")
{
    sAppModule      = "Sketcher";
    sGroup          = "Sketcher";
    sMenuText       = QT_TR_NOOP("Create equilateral triangle");
    sToolTipText    = QT_TR_NOOP("Create an equilateral triangle in the sketch");
    sWhatsThis      = "Sketcher_CreateTriangle";
    sStatusTip      = sToolTipText;
    sPixmap         = "Sketcher_CreateTriangle";
    sAccel          = "G, P, 3";
    eType           = ForEdit;
}

void CmdSketcherCreateTriangle::activated(int iMsg)
{
    Q_UNUSED(iMsg);
    ActivateHandler(getActiveGuiDocument(),new DrawSketchHandlerRegularPolygon(3) );
}

bool CmdSketcherCreateTriangle::isActive(void)
{
    return isCreateGeoActive(getActiveGuiDocument());
}

DEF_STD_CMD_A(CmdSketcherCreateSquare)

CmdSketcherCreateSquare::CmdSketcherCreateSquare()
  : Command("Sketcher_CreateSquare")
{
    sAppModule      = "Sketcher";
    sGroup          = "Sketcher";
    sMenuText       = QT_TR_NOOP("Create square");
    sToolTipText    = QT_TR_NOOP("Create a square in the sketch");
    sWhatsThis      = "Sketcher_CreateSquare";
    sStatusTip      = sToolTipText;
    sPixmap         = "Sketcher_CreateSquare";
    sAccel          = "G, P, 4";
    eType           = ForEdit;
}

void CmdSketcherCreateSquare::activated(int iMsg)
{
    Q_UNUSED(iMsg);
    ActivateHandler(getActiveGuiDocument(),new DrawSketchHandlerRegularPolygon(4) );
}

bool CmdSketcherCreateSquare::isActive(void)
{
    return isCreateGeoActive(getActiveGuiDocument());
}

DEF_STD_CMD_A(CmdSketcherCreatePentagon)

CmdSketcherCreatePentagon::CmdSketcherCreatePentagon()
  : Command("Sketcher_CreatePentagon")
{
    sAppModule      = "Sketcher";
    sGroup          = "Sketcher";
    sMenuText       = QT_TR_NOOP("Create pentagon");
    sToolTipText    = QT_TR_NOOP("Create a pentagon in the sketch");
    sWhatsThis      = "Sketcher_CreatePentagon";
    sStatusTip      = sToolTipText;
    sPixmap         = "Sketcher_CreatePentagon";
    sAccel          = "G, P, 5";
    eType           = ForEdit;
}

void CmdSketcherCreatePentagon::activated(int iMsg)
{
    Q_UNUSED(iMsg);
    ActivateHandler(getActiveGuiDocument(),new DrawSketchHandlerRegularPolygon(5) );
}

bool CmdSketcherCreatePentagon::isActive(void)
{
    return isCreateGeoActive(getActiveGuiDocument());
}


DEF_STD_CMD_A(CmdSketcherCreateHexagon)

CmdSketcherCreateHexagon::CmdSketcherCreateHexagon()
  : Command("Sketcher_CreateHexagon")
{
    sAppModule      = "Sketcher";
    sGroup          = "Sketcher";
    sMenuText       = QT_TR_NOOP("Create hexagon");
    sToolTipText    = QT_TR_NOOP("Create a hexagon in the sketch");
    sWhatsThis      = "Sketcher_CreateHexagon";
    sStatusTip      = sToolTipText;
    sPixmap         = "Sketcher_CreateHexagon";
    sAccel          = "G, P, 6";
    eType           = ForEdit;
}

void CmdSketcherCreateHexagon::activated(int iMsg)
{
    Q_UNUSED(iMsg);
    ActivateHandler(getActiveGuiDocument(),new DrawSketchHandlerRegularPolygon(6) );
}

bool CmdSketcherCreateHexagon::isActive(void)
{
    return isCreateGeoActive(getActiveGuiDocument());
}

DEF_STD_CMD_A(CmdSketcherCreateHeptagon)

CmdSketcherCreateHeptagon::CmdSketcherCreateHeptagon()
  : Command("Sketcher_CreateHeptagon")
{
    sAppModule      = "Sketcher";
    sGroup          = "Sketcher";
    sMenuText       = QT_TR_NOOP("Create heptagon");
    sToolTipText    = QT_TR_NOOP("Create a heptagon in the sketch");
    sWhatsThis      = "Sketcher_CreateHeptagon";
    sStatusTip      = sToolTipText;
    sPixmap         = "Sketcher_CreateHeptagon";
    sAccel          = "G, P, 7";
    eType           = ForEdit;
}

void CmdSketcherCreateHeptagon::activated(int iMsg)
{
    Q_UNUSED(iMsg);
    ActivateHandler(getActiveGuiDocument(),new DrawSketchHandlerRegularPolygon(7) );
}

bool CmdSketcherCreateHeptagon::isActive(void)
{
    return isCreateGeoActive(getActiveGuiDocument());
}

DEF_STD_CMD_A(CmdSketcherCreateOctagon)

CmdSketcherCreateOctagon::CmdSketcherCreateOctagon()
  : Command("Sketcher_CreateOctagon")
{
    sAppModule      = "Sketcher";
    sGroup          = "Sketcher";
    sMenuText       = QT_TR_NOOP("Create octagon");
    sToolTipText    = QT_TR_NOOP("Create an octagon in the sketch");
    sWhatsThis      = "Sketcher_CreateOctagon";
    sStatusTip      = sToolTipText;
    sPixmap         = "Sketcher_CreateOctagon";
    sAccel          = "G, P, 8";
    eType           = ForEdit;
}

void CmdSketcherCreateOctagon::activated(int iMsg)
{
    Q_UNUSED(iMsg);
    ActivateHandler(getActiveGuiDocument(),new DrawSketchHandlerRegularPolygon(8) );
}

bool CmdSketcherCreateOctagon::isActive(void)
{
    return isCreateGeoActive(getActiveGuiDocument());
}

DEF_STD_CMD_A(CmdSketcherCreateRegularPolygon)

CmdSketcherCreateRegularPolygon::CmdSketcherCreateRegularPolygon()
: Command("Sketcher_CreateRegularPolygon")
{
    sAppModule      = "Sketcher";
    sGroup          = "Sketcher";
    sMenuText       = QT_TR_NOOP("Create regular polygon");
    sToolTipText    = QT_TR_NOOP("Create a regular polygon in the sketch");
    sWhatsThis      = "Sketcher_CreateRegularPolygon";
    sStatusTip      = sToolTipText;
    sPixmap         = "Sketcher_CreateRegularPolygon";
    sAccel          = "G, P, R";
    eType           = ForEdit;
}

void CmdSketcherCreateRegularPolygon::activated(int iMsg)
{
    Q_UNUSED(iMsg);

    // Pop-up asking for values
    SketcherRegularPolygonDialog srpd;
    if (srpd.exec() == QDialog::Accepted)
        ActivateHandler(getActiveGuiDocument(),new DrawSketchHandlerRegularPolygon(srpd.sides));
}

bool CmdSketcherCreateRegularPolygon::isActive(void)
{
    return isCreateGeoActive(getActiveGuiDocument());
}

DEF_STD_CMD_ACLU(CmdSketcherCompCreateRegularPolygon)

CmdSketcherCompCreateRegularPolygon::CmdSketcherCompCreateRegularPolygon()
  : Command("Sketcher_CompCreateRegularPolygon")
{
    sAppModule      = "Sketcher";
    sGroup          = "Sketcher";
    sMenuText       = QT_TR_NOOP("Create regular polygon");
    sToolTipText    = QT_TR_NOOP("Create a regular polygon in the sketcher");
    sWhatsThis      = "Sketcher_CompCreateRegularPolygon";
    sStatusTip      = sToolTipText;
    sAccel          = "G, P, P";
    eType           = ForEdit;
}

void CmdSketcherCompCreateRegularPolygon::activated(int iMsg)
{
    switch( iMsg ){
    case 0:
        ActivateHandler(getActiveGuiDocument(),new DrawSketchHandlerRegularPolygon(3)); break;
    case 1:
        ActivateHandler(getActiveGuiDocument(),new DrawSketchHandlerRegularPolygon(4)); break;
    case 2:
        ActivateHandler(getActiveGuiDocument(),new DrawSketchHandlerRegularPolygon(5)); break;
    case 3:
        ActivateHandler(getActiveGuiDocument(),new DrawSketchHandlerRegularPolygon(6)); break;
    case 4:
        ActivateHandler(getActiveGuiDocument(),new DrawSketchHandlerRegularPolygon(7)); break;
    case 5:
        ActivateHandler(getActiveGuiDocument(),new DrawSketchHandlerRegularPolygon(8)); break;
    case 6:
    {
        // Pop-up asking for values
        SketcherRegularPolygonDialog srpd;
        if (srpd.exec() == QDialog::Accepted)
            ActivateHandler(getActiveGuiDocument(),new DrawSketchHandlerRegularPolygon(srpd.sides));
    }
    break;
    default:
        return;
    }

    // Since the default icon is reset when enabling/disabling the command we have
    // to explicitly set the icon of the used command.
    Gui::ActionGroup* pcAction = qobject_cast<Gui::ActionGroup*>(_pcAction);
    QList<QAction*> a = pcAction->actions();

    assert(iMsg < a.size());
    pcAction->setIcon(a[iMsg]->icon());
}

Gui::Action * CmdSketcherCompCreateRegularPolygon::createAction(void)
{
    Gui::ActionGroup* pcAction = new Gui::ActionGroup(this, Gui::getMainWindow());
    pcAction->setDropDownMenu(true);
    applyCommandData(this->className(), pcAction);

    QAction* triangle = pcAction->addAction(QString());
    triangle->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateTriangle"));
    QAction* square = pcAction->addAction(QString());
    square->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateSquare"));
    QAction* pentagon = pcAction->addAction(QString());
    pentagon->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreatePentagon"));
    QAction* hexagon = pcAction->addAction(QString());
    hexagon->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateHexagon"));
    QAction* heptagon = pcAction->addAction(QString());
    heptagon->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateHeptagon"));
    QAction* octagon = pcAction->addAction(QString());
    octagon->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateOctagon"));
    QAction* regular = pcAction->addAction(QString());
    regular->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateRegularPolygon"));

    _pcAction = pcAction;
    languageChange();

    pcAction->setIcon(hexagon->icon());
    int defaultId = 3;
    pcAction->setProperty("defaultAction", QVariant(defaultId));

    return pcAction;
}

void CmdSketcherCompCreateRegularPolygon::updateAction(int mode)
{
    Gui::ActionGroup* pcAction = qobject_cast<Gui::ActionGroup*>(getAction());
    if (!pcAction)
        return;

    QList<QAction*> a = pcAction->actions();
    int index = pcAction->property("defaultAction").toInt();
    switch (mode) {
    case Normal:
        a[0]->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateTriangle"));
        a[1]->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateSquare"));
        a[2]->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreatePentagon"));
        a[3]->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateHexagon"));
        a[4]->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateHeptagon"));
        a[5]->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateOctagon"));
        a[6]->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateRegularPolygon"));
        getAction()->setIcon(a[index]->icon());
        break;
    case Construction:
        a[0]->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateTriangle_Constr"));
        a[1]->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateSquare_Constr"));
        a[2]->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreatePentagon_Constr"));
        a[3]->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateHexagon_Constr"));
        a[4]->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateHeptagon_Constr"));
        a[5]->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateOctagon_Constr"));
        a[6]->setIcon(Gui::BitmapFactory().iconFromTheme("Sketcher_CreateRegularPolygon_Constr"));
        getAction()->setIcon(a[index]->icon());
        break;
    }
}

void CmdSketcherCompCreateRegularPolygon::languageChange()
{
    Command::languageChange();

    if (!_pcAction)
        return;
    Gui::ActionGroup* pcAction = qobject_cast<Gui::ActionGroup*>(_pcAction);
    QList<QAction*> a = pcAction->actions();

    QAction* triangle = a[0];
    triangle->setText(QApplication::translate("CmdSketcherCompCreateRegularPolygon","Triangle"));
    triangle->setToolTip(QApplication::translate("Sketcher_CreateTriangle","Create an equilateral triangle by its center and by one corner"));
    triangle->setStatusTip(QApplication::translate("Sketcher_CreateTriangle","Create an equilateral triangle by its center and by one corner"));
    QAction* square = a[1];
    square->setText(QApplication::translate("CmdSketcherCompCreateRegularPolygon","Square"));
    square->setToolTip(QApplication::translate("Sketcher_CreateSquare","Create a square by its center and by one corner"));
    square->setStatusTip(QApplication::translate("Sketcher_CreateSquare","Create a square by its center and by one corner"));
    QAction* pentagon = a[2];
    pentagon->setText(QApplication::translate("CmdSketcherCompCreateRegularPolygon","Pentagon"));
    pentagon->setToolTip(QApplication::translate("Sketcher_CreatePentagon","Create a pentagon by its center and by one corner"));
    pentagon->setStatusTip(QApplication::translate("Sketcher_CreatePentagon","Create a pentagon by its center and by one corner"));
    QAction* hexagon = a[3];
    hexagon->setText(QApplication::translate("CmdSketcherCompCreateRegularPolygon","Hexagon"));
    hexagon->setToolTip(QApplication::translate("Sketcher_CreateHexagon","Create a hexagon by its center and by one corner"));
    hexagon->setStatusTip(QApplication::translate("Sketcher_CreateHexagon","Create a hexagon by its center and by one corner"));
    QAction* heptagon = a[4];
    heptagon->setText(QApplication::translate("CmdSketcherCompCreateRegularPolygon","Heptagon"));
    heptagon->setToolTip(QApplication::translate("Sketcher_CreateHeptagon","Create a heptagon by its center and by one corner"));
    heptagon->setStatusTip(QApplication::translate("Sketcher_CreateHeptagon","Create a heptagon by its center and by one corner"));
    QAction* octagon = a[5];
    octagon->setText(QApplication::translate("CmdSketcherCompCreateRegularPolygon","Octagon"));
    octagon->setToolTip(QApplication::translate("Sketcher_CreateOctagon","Create an octagon by its center and by one corner"));
    octagon->setStatusTip(QApplication::translate("Sketcher_CreateOctagon","Create an octagon by its center and by one corner"));
    QAction* regular = a[6];
    regular->setText(QApplication::translate("CmdSketcherCompCreateRegularPolygon","Regular polygon"));
    regular->setToolTip(QApplication::translate("Sketcher_CreateOctagon","Create a regular polygon by its center and by one corner"));
    regular->setStatusTip(QApplication::translate("Sketcher_CreateOctagon","Create a regular polygon by its center and by one corner"));
}

bool CmdSketcherCompCreateRegularPolygon::isActive(void)
{
    return isCreateGeoActive(getActiveGuiDocument());
}

void CreateSketcherCommandsCreateGeo(void)
{
    Gui::CommandManager &rcCmdMgr = Gui::Application::Instance->commandManager();

    rcCmdMgr.addCommand(new CmdSketcherCreatePoint());
    rcCmdMgr.addCommand(new CmdSketcherCreateArc());
    rcCmdMgr.addCommand(new CmdSketcherCreate3PointArc());
    rcCmdMgr.addCommand(new CmdSketcherCompCreateArc());
    rcCmdMgr.addCommand(new CmdSketcherCreateCircle());
    rcCmdMgr.addCommand(new CmdSketcherCreate3PointCircle());
    rcCmdMgr.addCommand(new CmdSketcherCompCreateCircle());
    rcCmdMgr.addCommand(new CmdSketcherCreateEllipseByCenter());
    rcCmdMgr.addCommand(new CmdSketcherCreateEllipseBy3Points());
    rcCmdMgr.addCommand(new CmdSketcherCompCreateConic());
    rcCmdMgr.addCommand(new CmdSketcherCreateArcOfEllipse());
    rcCmdMgr.addCommand(new CmdSketcherCreateArcOfHyperbola());
    rcCmdMgr.addCommand(new CmdSketcherCreateArcOfParabola());
    rcCmdMgr.addCommand(new CmdSketcherCreateBSpline());
    rcCmdMgr.addCommand(new CmdSketcherCreatePeriodicBSpline());
    rcCmdMgr.addCommand(new CmdSketcherCompCreateBSpline());
    rcCmdMgr.addCommand(new CmdSketcherCreateLine());
    rcCmdMgr.addCommand(new CmdSketcherCreatePolyline());
    rcCmdMgr.addCommand(new CmdSketcherCreateRectangle());
    rcCmdMgr.addCommand(new CmdSketcherCreateRectangleCenter());
    rcCmdMgr.addCommand(new CmdSketcherCreateOblong());
    rcCmdMgr.addCommand(new CmdSketcherCreateFrame());
    rcCmdMgr.addCommand(new CmdSketcherCompCreateRegularPolygon());
    rcCmdMgr.addCommand(new CmdSketcherCreateTriangle());
    rcCmdMgr.addCommand(new CmdSketcherCreateSquare());
    rcCmdMgr.addCommand(new CmdSketcherCreatePentagon());
    rcCmdMgr.addCommand(new CmdSketcherCreateHexagon());
    rcCmdMgr.addCommand(new CmdSketcherCreateHeptagon());
    rcCmdMgr.addCommand(new CmdSketcherCreateOctagon());
    rcCmdMgr.addCommand(new CmdSketcherCreateRegularPolygon());
    rcCmdMgr.addCommand(new CmdSketcherCompCreateRectangles());
    rcCmdMgr.addCommand(new CmdSketcherCompCreateSlot());
    rcCmdMgr.addCommand(new CmdSketcherCreateSlot());
    rcCmdMgr.addCommand(new CmdSketcherCreateArcSlot());
    rcCmdMgr.addCommand(new CmdSketcherCreateRectangleSlot());
    rcCmdMgr.addCommand(new CmdSketcherCompCreateFillets());
    rcCmdMgr.addCommand(new CmdSketcherCreateFillet());
    rcCmdMgr.addCommand(new CmdSketcherCreateChamfer());
    //rcCmdMgr.addCommand(new CmdSketcherCreateText());
    //rcCmdMgr.addCommand(new CmdSketcherCreateDraftLine());
    rcCmdMgr.addCommand(new CmdSketcherTrimming());
    rcCmdMgr.addCommand(new CmdSketcherExtend());
    rcCmdMgr.addCommand(new CmdSketcherSplit());
    rcCmdMgr.addCommand(new CmdSketcherExternal());
    rcCmdMgr.addCommand(new CmdSketcherCarbonCopy());
}
