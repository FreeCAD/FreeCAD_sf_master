// SPDX-License-Identifier: LGPL-2.1-or-later
/****************************************************************************
 *                                                                          *
 *   Copyright (c) 2024 The FreeCAD Project Association AISBL               *
 *                                                                          *
 *   This file is part of FreeCAD.                                          *
 *                                                                          *
 *   FreeCAD is free software: you can redistribute it and/or modify it     *
 *   under the terms of the GNU Lesser General Public License as            *
 *   published by the Free Software Foundation, either version 2.1 of the   *
 *   License, or (at your option) any later version.                        *
 *                                                                          *
 *   FreeCAD is distributed in the hope that it will be useful, but         *
 *   WITHOUT ANY WARRANTY; without even the implied warranty of             *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU       *
 *   Lesser General Public License for more details.                        *
 *                                                                          *
 *   You should have received a copy of the GNU Lesser General Public       *
 *   License along with FreeCAD. If not, see                                *
 *   <https://www.gnu.org/licenses/>.                                       *
 *                                                                          *
 ***************************************************************************/

#include "PreCompiled.h"

#ifndef _PreComp_
#include <QApplication>
#include <QCheckBox>
#include <QFrame>
#include <QGridLayout>
#include <QLabel>
#include <QListView>
#include <QPushButton>
#include <QScrollArea>
#include <QWidget>
#include <QStackedWidget>
#endif

#include "StartView.h"
#include "FileCardDelegate.h"
#include "FileCardView.h"
#include "FirstStartWidget.h"
#include "FlowLayout.h"
#include "Gui/Workbench.h"
#include <Gui/Document.h>
#include <App/DocumentObject.h>
#include <App/Application.h>
#include <Base/Interpreter.h>
#include <Base/Tools.h>
#include <Gui/Application.h>
#include <Gui/Command.h>
#include <gsl/pointers>

using namespace StartGui;

TYPESYSTEM_SOURCE_ABSTRACT(StartGui::StartView, Gui::MDIView)  // NOLINT

namespace
{

struct NewButton
{
    QString heading;
    QString description;
    QString iconPath;
};

gsl::owner<QPushButton*> createNewButton(const NewButton& newButton)
{
    auto hGrp = App::GetApplication().GetParameterGroupByPath(
        "User parameter:BaseApp/Preferences/Mod/Start");
    const auto cardSpacing = static_cast<int>(hGrp->GetInt("FileCardSpacing", 25));       // NOLINT
    const auto newFileIconSize = static_cast<int>(hGrp->GetInt("NewFileIconSize", 48));   // NOLINT
    const auto cardLabelWith = static_cast<int>(hGrp->GetInt("FileCardLabelWith", 180));  // NOLINT

    auto button = gsl::owner<QPushButton*>(new QPushButton());
    auto mainLayout = gsl::owner<QHBoxLayout*>(new QHBoxLayout(button));
    auto iconLabel = gsl::owner<QLabel*>(new QLabel(button));
    mainLayout->addWidget(iconLabel);
    QIcon baseIcon(newButton.iconPath);
    iconLabel->setPixmap(baseIcon.pixmap(newFileIconSize, newFileIconSize));
    iconLabel->setPixmap(baseIcon.pixmap(newFileIconSize, newFileIconSize));

    auto textLayout = gsl::owner<QVBoxLayout*>(new QVBoxLayout);
    auto textLabelLine1 = gsl::owner<QLabel*>(new QLabel(button));
    textLabelLine1->setText(newButton.heading);
    textLabelLine1->setStyleSheet(QLatin1String("font-weight: bold;"));
    auto textLabelLine2 = gsl::owner<QLabel*>(new QLabel(button));
    textLabelLine2->setText(newButton.description);
    textLabelLine2->setWordWrap(true);
    textLayout->addWidget(textLabelLine1);
    textLayout->addWidget(textLabelLine2);
    textLayout->setSpacing(0);
    mainLayout->addItem(textLayout);

    mainLayout->addStretch();

    button->setMinimumHeight(newFileIconSize + cardSpacing);
    button->setMinimumWidth(newFileIconSize + cardLabelWith);
    return button;
}

}  // namespace

StartView::StartView(QWidget* parent)
    : Gui::MDIView(nullptr, parent)
    , _contents(new QStackedWidget(parent))
    , _newFileLabel {nullptr}
    , _examplesLabel {nullptr}
    , _recentFilesLabel {nullptr}
    , _showOnStartupCheckBox {nullptr}
{
    setObjectName(QLatin1String("StartView"));
    auto hGrp = App::GetApplication().GetParameterGroupByPath(
        "User parameter:BaseApp/Preferences/Mod/Start");
    auto cardSpacing = hGrp->GetInt("FileCardSpacing", 15);  // NOLINT

    // First start page
    auto firstStartScrollArea = gsl::owner<QScrollArea*>(new QScrollArea());
    auto firstStartRegion = gsl::owner<QHBoxLayout*>(new QHBoxLayout(firstStartScrollArea));
    firstStartRegion->addStretch();
    auto firstStartWidget = gsl::owner<FirstStartWidget*>(new FirstStartWidget(this));
    connect(firstStartWidget,
            &FirstStartWidget::dismissed,
            this,
            &StartView::firstStartWidgetDismissed);
    firstStartRegion->addWidget(firstStartWidget);
    firstStartRegion->addStretch();
    _contents->addWidget(firstStartScrollArea);

    // Documents page
    auto documentsWidget = gsl::owner<QWidget*>(new QWidget());
    _contents->addWidget(documentsWidget);
    auto documentsMainLayout = gsl::owner<QVBoxLayout*>(new QVBoxLayout());
    documentsWidget->setLayout(documentsMainLayout);
    auto documentsScrollArea = gsl::owner<QScrollArea*>(new QScrollArea());
    documentsScrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarPolicy::ScrollBarAsNeeded);
    documentsMainLayout->addWidget(documentsScrollArea);
    auto documentsScrollWidget = gsl::owner<QWidget*>(new QWidget(documentsScrollArea));
    documentsScrollArea->setWidget(documentsScrollWidget);
    documentsScrollArea->setWidgetResizable(true);
    auto documentsContentLayout = gsl::owner<QVBoxLayout*>(new QVBoxLayout(documentsScrollWidget));
    documentsContentLayout->setSizeConstraint(QLayout::SizeConstraint::SetMinAndMaxSize);

    _newFileLabel = gsl::owner<QLabel*>(new QLabel());
    documentsContentLayout->addWidget(_newFileLabel);

    auto createNewRow = gsl::owner<QWidget*>(new QWidget);
    auto flowLayout = gsl::owner<FlowLayout*>(new FlowLayout);

    // Reset margins of layout to provide consistent spacing
    flowLayout->setContentsMargins({});

    // This allows new file widgets to be targeted via QSS
    createNewRow->setObjectName(QStringLiteral("CreateNewRow"));
    createNewRow->setLayout(flowLayout);

    documentsContentLayout->addWidget(createNewRow);
    configureNewFileButtons(flowLayout);

    _recentFilesLabel = gsl::owner<QLabel*>(new QLabel());
    documentsContentLayout->addWidget(_recentFilesLabel);
    auto recentFilesListWidget = gsl::owner<FileCardView*>(new FileCardView(_contents));
    connect(recentFilesListWidget, &QListView::clicked, this, &StartView::fileCardSelected);
    documentsContentLayout->addWidget(recentFilesListWidget);

    _examplesLabel = gsl::owner<QLabel*>(new QLabel());
    documentsContentLayout->addWidget(_examplesLabel);
    auto examplesListWidget = gsl::owner<FileCardView*>(new FileCardView(_contents));
    connect(examplesListWidget, &QListView::clicked, this, &StartView::fileCardSelected);
    documentsContentLayout->addWidget(examplesListWidget);

    documentsContentLayout->setSpacing(static_cast<int>(cardSpacing));
    documentsContentLayout->addStretch();

    // Documents page footer
    auto footerLayout = gsl::owner<QHBoxLayout*>(new QHBoxLayout());
    documentsMainLayout->addLayout(footerLayout);

    _openFirstStart = gsl::owner<QPushButton*>(new QPushButton());
    _openFirstStart->setIcon(QIcon(QLatin1String(":/icons/preferences-general.svg")));
    connect(_openFirstStart, &QPushButton::clicked, this, &StartView::openFirstStartClicked);

    _showOnStartupCheckBox = gsl::owner<QCheckBox*>(new QCheckBox());
    bool showOnStartup = hGrp->GetBool("ShowOnStartup", true);
    _showOnStartupCheckBox->setCheckState(showOnStartup ? Qt::CheckState::Unchecked
                                                        : Qt::CheckState::Checked);
    connect(_showOnStartupCheckBox, &QCheckBox::toggled, this, &StartView::showOnStartupChanged);

    footerLayout->addWidget(_openFirstStart);
    footerLayout->addStretch();
    footerLayout->addWidget(_showOnStartupCheckBox);

    setCentralWidget(_contents);

    // Set startup widget according to the first start parameter
    auto firstStart = hGrp->GetBool("FirstStart2024", true);  // NOLINT
    _contents->setCurrentWidget(firstStart ? firstStartScrollArea : documentsWidget);

    configureExamplesListWidget(examplesListWidget);
    configureRecentFilesListWidget(recentFilesListWidget, _recentFilesLabel);

    retranslateUi();
}

void StartView::configureNewFileButtons(QLayout* layout) const
{
    auto newEmptyFile = createNewButton({tr("Empty file"),
                                         tr("Create a new empty FreeCAD file"),
                                         QLatin1String(":/icons/document-new.svg")});
    auto openFile = createNewButton({tr("Open File"),
                                     tr("Open an existing CAD file or 3D model"),
                                     QLatin1String(":/icons/document-open.svg")});
    auto partDesign = createNewButton({tr("Parametric Part"),
                                       tr("Create a part with the Part Design workbench"),
                                       QLatin1String(":/icons/PartDesignWorkbench.svg")});
    auto assembly = createNewButton({tr("Assembly"),
                                     tr("Create an assembly project"),
                                     QLatin1String(":/icons/AssemblyWorkbench.svg")});
    auto draft = createNewButton({tr("2D Draft"),
                                  tr("Create a 2D Draft with the Draft workbench"),
                                  QLatin1String(":/icons/DraftWorkbench.svg")});
    auto arch = createNewButton({tr("BIM/Architecture"),
                                 tr("Create an architectural project"),
                                 QLatin1String(":/icons/BIMWorkbench.svg")});

    auto hGrp = App::GetApplication().GetParameterGroupByPath(
        "User parameter:BaseApp/Preferences/Mod/Start");
    if (hGrp->GetBool("FileCardUseStyleSheet", true)) {
        QString style = fileCardStyle();
        newEmptyFile->setStyleSheet(style);
        openFile->setStyleSheet(style);
        partDesign->setStyleSheet(style);
        assembly->setStyleSheet(style);
        draft->setStyleSheet(style);
        arch->setStyleSheet(style);
    }

    // TODO: Ensure all of the required WBs are actually available
    layout->addWidget(partDesign);
    layout->addWidget(assembly);
    layout->addWidget(draft);
    layout->addWidget(arch);
    layout->addWidget(newEmptyFile);
    layout->addWidget(openFile);

    connect(newEmptyFile, &QPushButton::clicked, this, &StartView::newEmptyFile);
    connect(openFile, &QPushButton::clicked, this, &StartView::openExistingFile);
    connect(partDesign, &QPushButton::clicked, this, &StartView::newPartDesignFile);
    connect(assembly, &QPushButton::clicked, this, &StartView::newAssemblyFile);
    connect(draft, &QPushButton::clicked, this, &StartView::newDraftFile);
    connect(arch, &QPushButton::clicked, this, &StartView::newArchFile);
}

QString StartView::fileCardStyle() const
{
    if (!qApp->styleSheet().isEmpty()) {
        return {};
    }

    auto hGrp = App::GetApplication().GetParameterGroupByPath(
        "User parameter:BaseApp/Preferences/Mod/Start");

    auto getUserColor = [&hGrp](QColor color, const char* parameter) {
        uint32_t packed = App::Color::asPackedRGB<QColor>(color);
        packed = hGrp->GetUnsigned(parameter, packed);
        color = App::Color::fromPackedRGB<QColor>(packed);
        return color;
    };

    QColor background(221, 221, 221);  // NOLINT
    background = getUserColor(background, "FileCardBackgroundColor");

    QColor hovered(98, 160, 234);  // NOLINT
    hovered = getUserColor(hovered, "FileCardBorderColor");

    QColor pressed(38, 162, 105);  // NOLINT
    pressed = getUserColor(pressed, "FileCardSelectionColor");

    return QString::fromLatin1("QPushButton {"
                               " background-color: rgb(%1, %2, %3);"
                               " border-radius: 8px;"
                               "}"
                               "QPushButton:hover {"
                               " border: 2px solid rgb(%4, %5, %6);"
                               "}"
                               "QPushButton:pressed {"
                               " border: 2px solid rgb(%7, %8, %9);"
                               "}")
        .arg(background.red())
        .arg(background.green())
        .arg(background.blue())
        .arg(hovered.red())
        .arg(hovered.green())
        .arg(hovered.blue())
        .arg(pressed.red())
        .arg(pressed.green())
        .arg(pressed.blue());
}

void StartView::configureFileCardWidget(QListView* fileCardWidget)
{
    auto delegate = gsl::owner<FileCardDelegate*>(new FileCardDelegate(fileCardWidget));
    fileCardWidget->setItemDelegate(delegate);
    fileCardWidget->setMinimumWidth(fileCardWidget->parentWidget()->width());
    //    fileCardWidget->setGridSize(
    //        fileCardWidget->itemDelegate()->sizeHint(QStyleOptionViewItem(),
    //                                                 fileCardWidget->model()->index(0, 0)));
}


void StartView::configureRecentFilesListWidget(QListView* recentFilesListWidget,
                                               QLabel* recentFilesLabel)
{
    _recentFilesModel.loadRecentFiles();
    recentFilesListWidget->setModel(&_recentFilesModel);
    configureFileCardWidget(recentFilesListWidget);

    auto recentFilesGroup = App::GetApplication().GetParameterGroupByPath(
        "User parameter:BaseApp/Preferences/RecentFiles");
    auto numRecentFiles {recentFilesGroup->GetInt("RecentFiles", 0)};
    if (numRecentFiles == 0) {
        recentFilesListWidget->hide();
        recentFilesLabel->hide();
    }
    else {
        recentFilesListWidget->show();
        recentFilesLabel->show();
    }
}


void StartView::configureExamplesListWidget(QListView* examplesListWidget)
{
    _examplesModel.loadExamples();
    examplesListWidget->setModel(&_examplesModel);
    configureFileCardWidget(examplesListWidget);
}


void StartView::newEmptyFile() const
{
    Gui::Application::Instance->commandManager().runCommandByName("Std_New");
    postStart(PostStartBehavior::switchWorkbench);
}

void StartView::newPartDesignFile() const
{
    Gui::Application::Instance->commandManager().runCommandByName("Std_New");
    Gui::Application::Instance->activateWorkbench("PartDesignWorkbench");
    Gui::Application::Instance->commandManager().runCommandByName("PartDesign_Body");
    postStart(PostStartBehavior::doNotSwitchWorkbench);
}

void StartView::openExistingFile() const
{
    auto originalDocument = Gui::Application::Instance->activeDocument();
    Gui::Application::Instance->commandManager().runCommandByName("Std_Open");
    if (Gui::Application::Instance->activeDocument() != originalDocument) {
        // Only run this if the user chose a new document to open (that is, they didn't cancel the
        // open file dialog)
        postStart(PostStartBehavior::switchWorkbench);
    }
}

void StartView::newAssemblyFile() const
{
    Gui::Application::Instance->commandManager().runCommandByName("Std_New");
    Gui::Application::Instance->activateWorkbench("AssemblyWorkbench");
    Gui::Application::Instance->commandManager().runCommandByName("Assembly_CreateAssembly");
    Gui::Application::Instance->commandManager().runCommandByName("Std_Refresh");
    postStart(PostStartBehavior::doNotSwitchWorkbench);
}

void StartView::newDraftFile() const
{
    Gui::Application::Instance->commandManager().runCommandByName("Std_New");
    Gui::Application::Instance->activateWorkbench("DraftWorkbench");
    Gui::Application::Instance->commandManager().runCommandByName("Std_ViewTop");
    postStart(PostStartBehavior::doNotSwitchWorkbench);
}

void StartView::newArchFile() const
{
    Gui::Application::Instance->commandManager().runCommandByName("Std_New");
    try {
        Gui::Application::Instance->activateWorkbench("BIMWorkbench");
    }
    catch (...) {
        Gui::Application::Instance->activateWorkbench("ArchWorkbench");
    }
    postStart(PostStartBehavior::doNotSwitchWorkbench);
}

bool StartView::onHasMsg(const char* pMsg) const
{
    if (strcmp("AllowsOverlayOnHover", pMsg) == 0) {
        return false;
    }

    return MDIView::onHasMsg(pMsg);
}

void StartView::postStart(PostStartBehavior behavior) const
{
    auto hGrp = App::GetApplication().GetParameterGroupByPath(
        "User parameter:BaseApp/Preferences/Mod/Start");

    if (behavior == PostStartBehavior::switchWorkbench) {
        auto wb = hGrp->GetASCII("AutoloadModule", "");
        if (wb == "$LastModule") {
            wb = App::GetApplication()
                     .GetParameterGroupByPath("User parameter:BaseApp/Preferences/General")
                     ->GetASCII("LastModule", "");
        }
        if (!wb.empty()) {
            Gui::Application::Instance->activateWorkbench(wb.c_str());
        }
    }
    auto closeStart = hGrp->GetBool("closeStart", false);
    if (closeStart) {
        this->window()->close();
    }
}


void StartView::fileCardSelected(const QModelIndex& index)
{
    auto file = index.data(static_cast<int>(Start::DisplayedFilesModelRoles::path)).toString();
    std::string escapedstr = Base::Tools::escapedUnicodeFromUtf8(file.toStdString().c_str());
    escapedstr = Base::Tools::escapeEncodeFilename(escapedstr);
    auto command = std::string("FreeCAD.loadFile('") + escapedstr + "')";
    try {
        Base::Interpreter().runString(command.c_str());
        postStart(PostStartBehavior::doNotSwitchWorkbench);
    }
    catch (Base::PyException& e) {
        Base::Console().Error(e.getMessage().c_str());
    }
    catch (Base::Exception& e) {
        Base::Console().Error(e.getMessage().c_str());
    }
    catch (...) {
        Base::Console().Error("An unknown error occurred");
    }
}

void StartView::showOnStartupChanged(bool checked)
{
    auto hGrp = App::GetApplication().GetParameterGroupByPath(
        "User parameter:BaseApp/Preferences/Mod/Start");
    hGrp->SetBool(
        "ShowOnStartup",
        !checked);  // The sense of this option has been reversed: the checkbox actually says
                    // "*Don't* show on startup" now, but the option is preserved in its
                    // original sense, so is stored inverted.
}

void StartView::openFirstStartClicked()
{
    _contents->setCurrentIndex(0);
}

void StartView::firstStartWidgetDismissed()
{
    auto hGrp = App::GetApplication().GetParameterGroupByPath(
        "User parameter:BaseApp/Preferences/Mod/Start");
    hGrp->SetBool("FirstStart2024", false);
    _contents->setCurrentIndex(1);
}

void StartView::changeEvent(QEvent* event)
{
    if (event->type() == QEvent::LanguageChange) {
        this->retranslateUi();
    }
    Gui::MDIView::changeEvent(event);
}

void StartView::retranslateUi()
{
    QString title = QCoreApplication::translate("Workbench", "Start");
    setWindowTitle(title);

    const QLatin1String h1Start("<h1>");
    const QLatin1String h1End("</h1>");

    _newFileLabel->setText(h1Start + tr("New File") + h1End);
    _examplesLabel->setText(h1Start + tr("Examples") + h1End);
    _recentFilesLabel->setText(h1Start + tr("Recent Files") + h1End);

    QString application = QString::fromUtf8(App::Application::Config()["ExeName"].c_str());
    _openFirstStart->setText(tr("Open first start setup"));
    _showOnStartupCheckBox->setText(
        tr("Don't show this Start page again (start with blank screen)"));
}
