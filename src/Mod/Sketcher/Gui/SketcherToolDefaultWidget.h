/***************************************************************************
 *   Copyright (c) 2022 Pierre-Louis Boyer <pierrelouis.boyer@gmail.com>   *
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


#ifndef SketcherGui_SketcherToolDefaultWidget_H
#define SketcherGui_SketcherToolDefaultWidget_H

#include <Gui/TaskView/TaskView.h>
#include <Gui/TaskView/TaskDialog.h>
#include <Gui/Selection.h>
#include <boost_signals2.hpp>

namespace App {
class Property;
}

namespace Gui {
class ViewProvider;
class PrefQuantitySpinBox;
class PrefCheckBox;
}

namespace SketcherGui {

class Ui_SketcherToolDefaultWidget;
class ViewProviderSketch;

class SketcherToolDefaultWidget : public QWidget
{
    Q_OBJECT

    enum class FontStyle
    {
        Normal,
        Bold,
        Italic,
    };

public:

    enum Parameter {
      First,
      Second,
      Third,
      Fourth,
      Fifth,
      Sixth,
      nParameters // Must Always be the last one
    };

    enum Checkbox {
        FirstBox,
        SecondBox,
        ThirdBox,
        FourthBox,
        nCheckbox // Must Always be the last one
    };

    SketcherToolDefaultWidget (QWidget *parent=0, ViewProviderSketch* sketchView=0);
    ~SketcherToolDefaultWidget();

    bool eventFilter(QObject* object, QEvent* event);
    //void keyPressEvent(QKeyEvent* event);

    void setParameter(int parameterindex, double val);
    double getParameter(int parameterindex);
    bool isParameterSet(int parameterindex);
    void updateVisualValue(int parameterindex, double val);

    void setParameterEnabled(int parameterindex, bool active = true);
    void setParameterFocus(int parameterindex);

    void setParameterVisible(int parameterindex, bool visible = true);

    void reset();

    void initNParameters(int nparameters);

    void setParameterLabel(int parameterindex, const QString & string);

    void setNoticeText(const QString& string);
    void setNoticeVisible(bool visible);

    void initNCheckboxes(int ncheckbox);
    void setCheckboxVisible(int checkboxindex, bool visible);
    void setCheckboxChecked(int checkboxindex, bool checked);
    void setCheckboxLabel(int checkboxindex, const QString& string);
    bool getCheckboxChecked(int checkboxindex);
    void setCheckboxPrefEntry(int checkboxindex, const std::string & prefEntry);

    template<typename F>
    boost::signals2::connection registerParameterValueChanged(F&& f) {
        return signalParameterValueChanged.connect(std::forward<F>(f));
    }

    template<typename F>
    boost::signals2::connection registerCheckboxCheckedChanged(F&& f) {
        return signalCheckboxCheckedChanged.connect(std::forward<F>(f));
    }

//Q_SIGNALS:
protected Q_SLOTS:
    void parameterOne_valueChanged(double val);
    void parameterTwo_valueChanged(double val);
    void parameterThree_valueChanged(double val);
    void parameterFour_valueChanged(double val);
    void parameterFive_valueChanged(double val);
    void parameterSix_valueChanged(double val);
    void checkBoxTS1_toggled(bool val);
    void checkBoxTS2_toggled(bool val);
    void checkBoxTS3_toggled(bool val);
    void checkBoxTS4_toggled(bool val);

protected:
    void changeEvent(QEvent *e);

private:
    QLabel * getParameterLabel(int parameterindex);
    Gui::PrefQuantitySpinBox * getParameterSpinBox(int parameterindex);
    Gui::PrefCheckBox* getCheckBox(int checkboxindex);

    void setParameterFontStyle(int parameterindex, FontStyle fontStyle);

    bool isCheckBoxPrefEntryEmpty(int checkboxindex);

private:
    std::unique_ptr<Ui_SketcherToolDefaultWidget> ui;
    ViewProviderSketch* sketchView;

    boost::signals2::signal<void (int parameterindex, double value)> signalParameterValueChanged;
    boost::signals2::signal<void(int checkboxindex, bool value)> signalCheckboxCheckedChanged;

    bool blockParameterSlots;

    std::vector<bool> isSet;

};


} //namespace SketcherGui

#endif // SketcherGui_SketcherToolDefaultWidget_H
