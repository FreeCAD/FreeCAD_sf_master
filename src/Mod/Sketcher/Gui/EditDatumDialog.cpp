/***************************************************************************
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
# include <Standard_math.hxx>
/// Qt Include Files
# include <QApplication>
# include <QDialog>
# include <QMessageBox>
# include <Inventor/sensors/SoSensor.h>
#endif

#include <Base/Tools.h>
#include <Gui/Application.h>
#include <Gui/CommandT.h>
#include <Gui/Document.h>
#include <Gui/MainWindow.h>
#include <Gui/View3DInventor.h>
#include <Gui/View3DInventorViewer.h>
#include <Mod/Sketcher/App/SketchObject.h>

#include "ViewProviderSketch.h"
#include "ui_InsertDatum.h"
#include "EditDatumDialog.h"
#include "CommandConstraints.h"


using namespace SketcherGui;

/* TRANSLATOR SketcherGui::EditDatumDialog */

EditDatumDialog::EditDatumDialog(ViewProviderSketch* vp, int ConstrNbr)
    : ConstrNbr(ConstrNbr)
{
    sketch = vp->getSketchObject();
    const std::vector<Sketcher::Constraint *> &Constraints = sketch->Constraints.getValues();
    Constr = Constraints[ConstrNbr];
}

EditDatumDialog::EditDatumDialog(Sketcher::SketchObject* pcSketch, int ConstrNbr)
    : sketch(pcSketch)
    , ConstrNbr(ConstrNbr)
{
    const std::vector<Sketcher::Constraint *> &Constraints = sketch->Constraints.getValues();
    Constr = Constraints[ConstrNbr];
}

EditDatumDialog::~EditDatumDialog()
{

}

void EditDatumDialog::exec(bool atCursor)
{
    // Return if constraint doesn't have editable value
    if (Constr->isDimensional()) {

        if (sketch->hasConflicts()) {
            QMessageBox::critical(qApp->activeWindow(), QObject::tr("Distance constraint"),
                                  QObject::tr("Not allowed to edit the datum because the sketch contains conflicting constraints"));
            return;
        }

        QDialog dlg(Gui::getMainWindow());
        if (ui_ins_datum == nullptr) {
            ui_ins_datum.reset(new Ui_InsertDatum);
            ui_ins_datum->setupUi(&dlg);
        }
        double datum = Constr->getValue();
        Base::Quantity init_val;

        if (Constr->Type == Sketcher::Angle) {
            datum = Base::toDegrees<double>(datum);
            dlg.setWindowTitle(tr("Insert angle"));
            init_val.setUnit(Base::Unit::Angle);
            ui_ins_datum->label->setText(tr("Angle:"));
            ui_ins_datum->labelEdit->setParamGrpPath(QByteArray("User parameter:BaseApp/History/SketcherAngle"));
        }
        else if (Constr->Type == Sketcher::Radius) {
            dlg.setWindowTitle(tr("Insert radius"));
            init_val.setUnit(Base::Unit::Length);
            ui_ins_datum->label->setText(tr("Radius:"));
            ui_ins_datum->labelEdit->setParamGrpPath(QByteArray("User parameter:BaseApp/History/SketcherLength"));
        }
        else if (Constr->Type == Sketcher::Diameter) {
            dlg.setWindowTitle(tr("Insert diameter"));
            init_val.setUnit(Base::Unit::Length);
            ui_ins_datum->label->setText(tr("Diameter:"));
            ui_ins_datum->labelEdit->setParamGrpPath(QByteArray("User parameter:BaseApp/History/SketcherLength"));
        }
        else if (Constr->Type == Sketcher::SnellsLaw) {
            dlg.setWindowTitle(tr("Refractive index ratio", "Constraint_SnellsLaw"));
            ui_ins_datum->label->setText(tr("Ratio n2/n1:", "Constraint_SnellsLaw"));
            ui_ins_datum->labelEdit->setParamGrpPath(QByteArray("User parameter:BaseApp/History/SketcherRefrIndexRatio"));
        }
        else {
            dlg.setWindowTitle(tr("Insert length"));
            init_val.setUnit(Base::Unit::Length);
            ui_ins_datum->label->setText(tr("Length:"));
            ui_ins_datum->labelEdit->setParamGrpPath(QByteArray("User parameter:BaseApp/History/SketcherLength"));
        }

        init_val.setValue(datum);

        ui_ins_datum->labelEdit->setValue(init_val);
        ui_ins_datum->labelEdit->pushToHistory();
        ui_ins_datum->labelEdit->selectNumber();
        ui_ins_datum->labelEdit->bind(sketch->Constraints.createPath(ConstrNbr));
        ui_ins_datum->name->setText(Base::Tools::fromStdString(Constr->Name));
        
        ui_ins_datum->cbDriving->setChecked(! Constr->isDriving);
        
        connect(ui_ins_datum->cbDriving, SIGNAL(toggled(bool)), this, SLOT(drivingToggled(bool)));
        connect(ui_ins_datum->labelEdit, SIGNAL(valueChanged(const Base::Quantity&)), this, SLOT(datumChanged()));
        connect(ui_ins_datum->labelEdit, SIGNAL(showFormulaDialog(bool)), this, SLOT(formEditorOpened(bool)));
        connect(&dlg, SIGNAL(accepted()), this, SLOT(accepted()));
        connect(&dlg, SIGNAL(rejected()), this, SLOT(rejected()));

        if (atCursor) {
            dlg.show(); // Need to show the dialog so geometry is computed
            QRect pg = dlg.parentWidget()->geometry();
            int Xmin = pg.x() + 10;
            int Ymin = pg.y() + 10;
            int Xmax = pg.x() + pg.width() - dlg.geometry().width() - 10;
            int Ymax = pg.y() + pg.height() - dlg.geometry().height() - 10;
            int x = Xmax < Xmin ? (Xmin + Xmax)/2 :  std::min(std::max(QCursor::pos().x(), Xmin), Xmax);
            int y = Ymax < Ymin ? (Ymin + Ymax)/2 :  std::min(std::max(QCursor::pos().y(), Ymin), Ymax);
            dlg.setGeometry(x, y, dlg.geometry().width(), dlg.geometry().height());
        }

        dlg.exec();
    }
}

void EditDatumDialog::accepted()
{
    Base::Quantity newQuant = ui_ins_datum->labelEdit->value();
    if (newQuant.isQuantity() || (Constr->Type == Sketcher::SnellsLaw && newQuant.isDimensionless())) {
        // save the value for the history
        ui_ins_datum->labelEdit->pushToHistory();
        
        double newDatum = newQuant.getValue();
        
        try {
            
            /*if (ui_ins_datum->cbDriving->isChecked() == Constr->isDriving) {
                Gui::cmdAppObjectArgs(sketch, "toggleDriving(%i)", ConstrNbr);
            }*/
            
            if (! ui_ins_datum->cbDriving->isChecked()) {
                if (ui_ins_datum->labelEdit->hasExpression())
                    ui_ins_datum->labelEdit->apply();
                else
                    Gui::cmdAppObjectArgs(sketch, "setDatum(%i,App.Units.Quantity('%f %s'))",
                                          ConstrNbr, newDatum, (const char*)newQuant.getUnit().getString().toUtf8());
            }
            
            QString constraintName = ui_ins_datum->name->text().trimmed();
            if (Base::Tools::toStdString(constraintName) != sketch->Constraints[ConstrNbr]->Name) {
                std::string escapedstr = Base::Tools::escapedUnicodeFromUtf8(constraintName.toUtf8().constData());
                Gui::cmdAppObjectArgs(sketch, "renameConstraint(%d, u'%s')",
                                      ConstrNbr, escapedstr.c_str());
            }
            
            Gui::Command::commitCommand();
            
            if (sketch->noRecomputes && sketch->ExpressionEngine.depsAreTouched()) {
                sketch->ExpressionEngine.execute();
                sketch->solve();
            }
            
            tryAutoRecompute(sketch);
        }
        catch (const Base::Exception& e) {
            QMessageBox::critical(qApp->activeWindow(), QObject::tr("Dimensional constraint"), QString::fromUtf8(e.what()));
            Gui::Command::abortCommand();
        }
    }
}

void EditDatumDialog::rejected()
{
    Gui::Command::abortCommand();
    sketch->recomputeFeature();
}

void EditDatumDialog::drivingToggled(bool state)
{
    if (state) {
        ui_ins_datum->labelEdit->setToLastUsedValue();
    }
    sketch->setDriving(ConstrNbr, !state);
}

void EditDatumDialog::datumChanged()
{
    if (ui_ins_datum->labelEdit->text() != ui_ins_datum->labelEdit->getHistory()[0]) {
        ui_ins_datum->cbDriving->setChecked(false);
    }
}

void EditDatumDialog::formEditorOpened(bool state)
{
    if (state) {
        ui_ins_datum->cbDriving->setChecked(false);
    }
}

#include "moc_EditDatumDialog.cpp"
