/***************************************************************************
 *   Copyright (c) 2013 Jürgen Riegel <juergen.riegel@web.de>              *
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
# include <QApplication>
# include <QClipboard>
# include <QLocale>
#endif

#include "DlgUnitsCalculatorImp.h"
#include "ui_DlgUnitsCalculator.h"
#include <Base/UnitsApi.h>

using namespace Gui::Dialog;

/* TRANSLATOR Gui::Dialog::DlgUnitsCalculator */

/**
 *  Constructs a DlgUnitsCalculator which is a child of 'parent', with the
 *  name 'name' and widget flags set to 'f'
 *
 *  The dialog will by default be modeless, unless you set 'modal' to
 *  true to construct a modal dialog.
 */
DlgUnitsCalculator::DlgUnitsCalculator( QWidget* parent, Qt::WindowFlags fl )
  : QDialog(parent, fl), ui(new Ui_DlgUnitCalculator)
{
    // create widgets
    ui->setupUi(this);
    this->setAttribute(Qt::WA_DeleteOnClose);

    connect(ui->ValueInput, SIGNAL(valueChanged(Base::Quantity)), this, SLOT(valueChanged(Base::Quantity)));
    connect(ui->ValueInput, SIGNAL(returnPressed()), this, SLOT(returnPressed()));
    connect(ui->UnitInput, SIGNAL(textChanged(QString)), this, SLOT(textChanged(QString)));
    connect(ui->UnitInput, SIGNAL(returnPressed()), this, SLOT(returnPressed()));

    connect(ui->pushButton_Close, SIGNAL(clicked()), this, SLOT(accept()));
    connect(ui->pushButton_Copy, SIGNAL(clicked()), this, SLOT(copy()));

    connect(ui->ValueInput, SIGNAL(parseError(QString)), this, SLOT(parseError(QString)));
    connect(ui->UnitInput, SIGNAL(parseError(QString)), this, SLOT(parseError(QString)));

    ui->ValueInput->setParamGrpPath(QByteArray("User parameter:BaseApp/History/UnitsCalculator"));
	// set a default that also illustrates how the dialog works
	ui->ValueInput->setText(QString::fromLatin1("1 cm"));
	ui->UnitInput->setText(QString::fromLatin1("in"));

    units << Base::Unit::Length << Base::Unit::Mass << Base::Unit::Angle << Base::Unit::Density
          << Base::Unit::Area << Base::Unit::Volume << Base::Unit::TimeSpan << Base::Unit::Frequency
          << Base::Unit::Velocity << Base::Unit::Acceleration << Base::Unit::Temperature
          << Base::Unit::ElectricCurrent << Base::Unit::ElectricPotential
          << Base::Unit::AmountOfSubstance << Base::Unit::LuminousIntensity << Base::Unit::Stress
          << Base::Unit::Pressure << Base::Unit::Force << Base::Unit::Work << Base::Unit::Power
          << Base::Unit::ThermalConductivity << Base::Unit::ThermalExpansionCoefficient
          << Base::Unit::SpecificHeat << Base::Unit::ThermalTransferCoefficient << Base::Unit::HeatFlux;
    for (QList<Base::Unit>::iterator it = units.begin(); it != units.end(); ++it) {
        ui->unitsBox->addItem(it->getTypeString());
    }

    ui->quantitySpinBox->setUnit(units.front());
}

/** Destroys the object and frees any allocated resources */
DlgUnitsCalculator::~DlgUnitsCalculator()
{
}

void DlgUnitsCalculator::accept()
{
    QDialog::accept();
}

void DlgUnitsCalculator::reject()
{
    QDialog::reject();
}

void DlgUnitsCalculator::textChanged(QString unit)
{
	valueChanged(actValue);
}

void DlgUnitsCalculator::valueChanged(const Base::Quantity& quant)
{
	// first check the unit, if it is invalid, getTypeString() outputs an empty string 
    if (Base::Unit(ui->UnitInput->text()).getTypeString().isEmpty()) {
		ui->ValueOutput->setText(tr("unknown unit: ") + ui->UnitInput->text());
		ui->pushButton_Copy->setEnabled(false);
	} else { // the unit is valid
		// we can only convert units of the same type, thus check
		if (Base::Unit(ui->UnitInput->text()).getTypeString() != quant.getUnit().getTypeString()) {
			ui->ValueOutput->setText(tr("unit mismatch"));
			ui->pushButton_Copy->setEnabled(false);
		} else { // the unit is valid and has the same type
			double convertValue = Base::Quantity::parse(QString::fromLatin1("1") + ui->UnitInput->text()).getValue();
			// we got now e.g. for "1 in" the value '25.4' because 1 in = 25.4 mm
			// the result is now just quant / convertValue because the input is always in a base unit
			// (an input of "1 cm" will immediately be converted to "10 mm" by Gui::InputField of the dialog)
			double value = quant.getValue() / convertValue;
			// determine how many decimals we will need to avoid an output like "0.00"
			// at first use scientific notation, if there is no "e", we can round it to the user-defined decimals,
			// but the user-defined decimals might be too low for cases like "10 um" in "in",
			// thus only if value > 0.005 because FC's default are 2 decimals
			QString val = QLocale::system().toString(value, 'g');
			if (!val.contains(QChar::fromLatin1('e')) && (value > 0.005))
				val = QLocale::system().toString(value, 'f', Base::UnitsApi::getDecimals());
			// create the output string
			QString out = QString::fromLatin1("%1 %2").arg(val, ui->UnitInput->text());
			ui->ValueOutput->setText(out);
			ui->pushButton_Copy->setEnabled(true);
		}
	}
	// store the input value
    actValue = quant;
}

void DlgUnitsCalculator::parseError(const QString& errorText)
{
    ui->pushButton_Copy->setEnabled(false);
    ui->ValueOutput->setText(errorText);
}

void DlgUnitsCalculator::copy(void)
{
    QClipboard *cb = QApplication::clipboard();
    cb->setText(ui->ValueOutput->text());
}

void DlgUnitsCalculator::returnPressed(void)
{
    if (ui->pushButton_Copy->isEnabled()) {
        ui->textEdit->append(ui->ValueInput->text() + QString::fromLatin1(" = ") + ui->ValueOutput->text());
        ui->ValueInput->pushToHistory();
    }
}

void DlgUnitsCalculator::on_unitsBox_activated(int index)
{
    ui->quantitySpinBox->setUnit(units[index]);
}

#include "moc_DlgUnitsCalculatorImp.cpp"
