/***************************************************************************
 *   Copyright (c) 2023 David Carter <dcarter@david.carter.ca>             *
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
#endif

#include <limits>
#include <QString>
#include <QStringList>

#include <App/Application.h>
#include <Base/Interpreter.h>
#include <Gui/Command.h>
#include <Gui/WaitCursor.h>
#include <Gui/Application.h>
#include <Gui/PrefWidgets.h>
#include <Gui/SpinBox.h>

#include <QItemSelectionModel>

#include <Mod/Material/App/ModelManager.h>
#include "MaterialsEditor.h"
#include "ui_MaterialsEditor.h"


using namespace MatGui;

/* TRANSLATOR MatGui::MaterialsEditor */

MaterialsEditor::MaterialsEditor(QWidget* parent)
  : QDialog(parent), ui(new Ui_MaterialsEditor)
{
    ui->setupUi(this);

    createMaterialTree();
    createPropertyTree();
    createAppearanceTree();

    connect(ui->standardButtons, &QDialogButtonBox::accepted,
            this, &MaterialsEditor::accept);
    connect(ui->standardButtons, &QDialogButtonBox::rejected,
            this, &MaterialsEditor::reject);

    QItemSelectionModel* selectionModel = ui->treeMaterials->selectionModel();
    connect(selectionModel, &QItemSelectionModel::selectionChanged,
            this, &MaterialsEditor::onSelectMaterial);
}

/*
 *  Destroys the object and frees any allocated resources
 */
MaterialsEditor::~MaterialsEditor()
{
    // no need to delete child widgets, Qt does it all for us
}

void MaterialsEditor::tryPython()
{
    Base::Console().Log("MaterialsEditor::tryPython()\n");

    // from materialtools.cardutils import get_material_template
    // template_data = get_material_template(True)
    Gui::WaitCursor wc;
    App::GetApplication().setActiveTransaction(QT_TRANSLATE_NOOP("Command", "Python test"));

    Base::PyGILStateLocker lock;
    try {
        PyObject* module = PyImport_ImportModule("materialtools.cardutils");
        if (!module) {
            throw Py::Exception();
        }
        Py::Module utils(module, true);

        // Py::Tuple args(2);
        // args.setItem(0, Py::asObject(it->getPyObject()));
        // args.setItem(1, Py::Float(distance));
        auto ret = utils.callMemberFunction("get_material_template");
        // if (ret != nullptr)
        //     Base::Console().Log("MaterialsEditor::tryPython() - null return\n");
        // else
        //     Base::Console().Log("MaterialsEditor::tryPython() - data!\n");

        Base::Console().Log("MaterialsEditor::tryPython() - call complete\n");

    }
    catch (Py::Exception&) {
        Base::PyException e;
        e.ReportException();
    }

    App::GetApplication().closeActiveTransaction();
    Base::Console().Log("MaterialsEditor::tryPython() - finished\n");
}

void MaterialsEditor::accept()
{
    reject();
}

void MaterialsEditor::reject()
{
    QDialog::reject();
    // auto dw = qobject_cast<QDockWidget*>(parent());
    // if (dw) {
    //     dw->deleteLater();
    // }
}

// QIcon MaterialsEditor::errorIcon(const QIcon &icon) const {
//     auto pixmap = icon.pixmap();
// }

void MaterialsEditor::addCards(QStandardItem &parent, const std::string &top, const std::string &folder, const QIcon &icon)
{
    auto tree = ui->treeMaterials;
    for (const auto& mod : fs::directory_iterator(folder)) {
        if (fs::is_directory(mod)) {
            auto node = new QStandardItem(QString::fromStdString(mod.path().filename().string()));
            addExpanded(tree, &parent, node);
            node->setFlags(Qt::ItemIsEnabled | Qt::ItemIsDragEnabled | Qt::ItemIsDropEnabled);

            addCards(*node, top, mod.path().string(), icon);
        }
        else if (isCard(mod)) {
            auto card = new QStandardItem(icon, QString::fromStdString(mod.path().filename().string()));
            card->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled | Qt::ItemIsDragEnabled
                        | Qt::ItemIsDropEnabled);
            try {
                auto material = getMaterialManager().getMaterialByPath(mod.path().string());
                card->setData(QVariant(QString::fromStdString(material.getUUID())), Qt::UserRole);
            } catch (Materials::ModelNotFound &e) {
                Base::Console().Log("Model not found error\n");
            } catch (std::exception &e) {
                Base::Console().Log("Exception '%s'\n", e.what());
            }
            addExpanded(tree, &parent, card);
        }
    }
}

void MaterialsEditor::addExpanded(QTreeView *tree, QStandardItem *parent, QStandardItem *child)
{
    parent->appendRow(child);
    tree->setExpanded(child->index(), true);
}

void MaterialsEditor::addExpanded(QTreeView *tree, QStandardItemModel *parent, QStandardItem *child)
{
    parent->appendRow(child);
    tree->setExpanded(child->index(), true);
}

void MaterialsEditor::createPropertyTree()
{
    auto tree = ui->treePhysicalProperties;
    auto model = new QStandardItemModel();
    tree->setModel(model);

    QStringList headers;
    headers.append(QString::fromStdString("Property"));
    headers.append(QString::fromStdString("Value"));
    headers.append(QString::fromStdString("Type"));
    model->setHorizontalHeaderLabels(headers);

    tree->setColumnWidth(0, 250);
    tree->setColumnWidth(1, 250);
    tree->setColumnHidden(2, true);

    tree->setHeaderHidden(false);
    tree->setUniformRowHeights(true);
    tree->setItemDelegate(new MaterialDelegate(this));
}

void MaterialsEditor::createAppearanceTree()
{
    auto tree = ui->treeAppearance;
    auto model = new QStandardItemModel();
    tree->setModel(model);

    QStringList headers;
    headers.append(QString::fromStdString("Property"));
    headers.append(QString::fromStdString("Value"));
    headers.append(QString::fromStdString("Type"));
    model->setHorizontalHeaderLabels(headers);

    tree->setColumnWidth(0, 250);
    tree->setColumnWidth(1, 250);
    tree->setColumnHidden(2, true);

    tree->setHeaderHidden(false);
    tree->setUniformRowHeights(true);
    tree->setItemDelegate(new MaterialDelegate(this));
}

void MaterialsEditor::createMaterialTree()
{
    Materials::ModelManager modelManager;
    // Materials::MaterialManager materialManager;

    auto param =
        App::GetApplication().GetParameterGroupByPath("User parameter:BaseApp/Preferences/Mod/Material");
    if (!param)
        Base::Console().Log("Unable to find material parameter group\n");
    else
        Base::Console().Log("Found material parameter group\n");

    auto tree = ui->treeMaterials;
    auto model = new QStandardItemModel();
    tree->setModel(model);

    tree->setHeaderHidden(true);

    auto lib = new QStandardItem(QString::fromStdString("Favorites"));
    lib->setFlags(Qt::ItemIsEnabled | Qt::ItemIsDragEnabled | Qt::ItemIsDropEnabled);
    addExpanded(tree, model, lib);

    lib = new QStandardItem(QString::fromStdString("Recent"));
    lib->setFlags(Qt::ItemIsEnabled | Qt::ItemIsDragEnabled | Qt::ItemIsDropEnabled);
    addExpanded(tree, model, lib);

    auto libraries = Materials::MaterialManager::getMaterialLibraries();
    for (const auto &value : *libraries)
    {
        lib = new QStandardItem(QString::fromStdString(value->getName()));
        lib->setFlags(Qt::ItemIsEnabled | Qt::ItemIsDragEnabled | Qt::ItemIsDropEnabled);
        addExpanded(tree, model, lib);

        auto path = value->getDirectoryPath();
        addCards(*lib, path, path, QIcon(QString::fromStdString(value->getIconPath())));
    }
}

void MaterialsEditor::clearCardProperties(void)
{
    auto tree = ui->treePhysicalProperties;
    QStandardItemModel *model = static_cast<QStandardItemModel *>(tree->model());
    model->clear();
}

void MaterialsEditor::clearCardAppearance(void)
{
    auto tree = ui->treeAppearance;
    QStandardItemModel *model = static_cast<QStandardItemModel *>(tree->model());
    model->clear();
}

void MaterialsEditor::clearCard(void)
{
    // Update the general information
    ui->editName->setText(QString::fromStdString(""));
    ui->editAuthorLicense->setText(QString::fromStdString(""));
    ui->editParent->setText(QString::fromStdString(""));
    ui->editSourceURL->setText(QString::fromStdString(""));
    ui->editSourceReference->setText(QString::fromStdString(""));
    ui->editTags->setText(QString::fromStdString(""));
    ui->editDescription->setText(QString::fromStdString(""));

    clearCardProperties();
    clearCardAppearance();
}

void MaterialsEditor::updateCardAppearance(const Materials::Material &card)
{
    QTreeView *tree = ui->treeAppearance;
    QStandardItemModel *treeModel = static_cast<QStandardItemModel *>(tree->model());
    treeModel->clear();

    QStringList headers;
    headers.append(QString::fromStdString("Property"));
    headers.append(QString::fromStdString("Value"));
    headers.append(QString::fromStdString("Type"));
    treeModel->setHorizontalHeaderLabels(headers);

    tree->setColumnWidth(0, 250);
    tree->setColumnWidth(1, 250);
    tree->setColumnHidden(2, true);

    const std::vector<std::string> *models = card.getAppearanceModels();
    if (models) {
        for (auto it = models->begin(); it != models->end(); it++)
        {
            std::string uuid = *it;
            const Materials::Model *model = getModelManager().getModel(uuid);
            std::string name = model->getName();

            auto modelRoot = new QStandardItem(QString::fromStdString(name));
            modelRoot->setFlags(Qt::ItemIsEnabled | Qt::ItemIsDragEnabled | Qt::ItemIsDropEnabled);
            addExpanded(tree, treeModel, modelRoot);
            for (auto itp = model->begin(); itp != model->end(); itp++)
            {
                QList<QStandardItem*> items;

                std::string key = itp->first;
                auto propertyItem = new QStandardItem(QString::fromStdString(key));
                propertyItem->setToolTip(QString::fromStdString(itp->second.getDescription()));
                items.append(propertyItem);

                auto valueItem = new QStandardItem(QString::fromStdString(card.getAppearancePropertyValue(key)));
                valueItem->setToolTip(QString::fromStdString(itp->second.getDescription()));
                items.append(valueItem);

                auto typeItem = new QStandardItem(QString::fromStdString(itp->second.getPropertyType()));
                items.append(typeItem);

                // addExpanded(tree, modelRoot, propertyItem);
                modelRoot->appendRow(items);
                tree->setExpanded(modelRoot->index(), true);
            }
        }
    }
}

void MaterialsEditor::updateCardProperties(const Materials::Material &card)
{
    QTreeView *tree = ui->treePhysicalProperties;
    QStandardItemModel *treeModel = static_cast<QStandardItemModel *>(tree->model());
    treeModel->clear();

    QStringList headers;
    headers.append(QString::fromStdString("Property"));
    headers.append(QString::fromStdString("Value"));
    headers.append(QString::fromStdString("Type"));
    treeModel->setHorizontalHeaderLabels(headers);

    tree->setColumnWidth(0, 250);
    tree->setColumnWidth(1, 250);
    tree->setColumnHidden(2, true);

    const std::vector<std::string> *models = card.getModels();
    if (models) {
        for (auto it = models->begin(); it != models->end(); it++)
        {
            std::string uuid = *it;
            const Materials::Model *model = getModelManager().getModel(uuid);
            std::string name = model->getName();

            auto modelRoot = new QStandardItem(QString::fromStdString(name));
            modelRoot->setFlags(Qt::ItemIsEnabled | Qt::ItemIsDragEnabled | Qt::ItemIsDropEnabled);
            addExpanded(tree, treeModel, modelRoot);
            for (auto itp = model->begin(); itp != model->end(); itp++)
            {
                QList<QStandardItem*> items;

                std::string key = itp->first;
                Materials::ModelProperty modelProperty = itp->second;
                auto propertyItem = new QStandardItem(QString::fromStdString(key));
                propertyItem->setToolTip(QString::fromStdString(modelProperty.getDescription()));
                items.append(propertyItem);

                auto valueItem = new QStandardItem(QString::fromStdString(card.getPropertyValue(key)));
                valueItem->setToolTip(QString::fromStdString(modelProperty.getDescription()));
                items.append(valueItem);

                auto typeItem = new QStandardItem(QString::fromStdString(modelProperty.getPropertyType()));
                items.append(typeItem);

                // addExpanded(tree, modelRoot, propertyItem);
                modelRoot->appendRow(items);
                tree->setExpanded(modelRoot->index(), true);
            }
        }
    }
}

void MaterialsEditor::updateCard(const std::string &uuid)
{
    Materials::Material card = getMaterialManager().getMaterial(uuid);

    // Update the general information
    ui->editName->setText(QString::fromStdString(card.getName()));
    ui->editAuthorLicense->setText(QString::fromStdString(card.getAuthorAndLicense()));
    // ui->editParent->setText(QString::fromStdString(card.getName()));
    ui->editSourceURL->setText(QString::fromStdString(card.getURL()));
    ui->editSourceReference->setText(QString::fromStdString(card.getReference()));
    // ui->editTags->setText(QString::fromStdString(card.getName()));
    ui->editDescription->setText(QString::fromStdString(card.getDescription()));

    updateCardProperties(card);
    updateCardAppearance(card);
}

void MaterialsEditor::onSelectMaterial(const QItemSelection& selected, const QItemSelection& deselected)
{
    QStandardItemModel *model = static_cast<QStandardItemModel *>(ui->treeMaterials->model());
    QModelIndexList indexes = selected.indexes();
    for (auto it = indexes.begin(); it != indexes.end(); it++)
    {
        QStandardItem* item = model->itemFromIndex(*it);
        Base::Console().Log("%s\n", item->text().toStdString().c_str());
        if (item) {
            try
            {
                std::string uuid = item->data(Qt::UserRole).toString().toStdString();
                Base::Console().Log("\t%s\n", item->data(Qt::UserRole).toString().toStdString().c_str());
                updateCard(uuid);
            }
            catch(const std::exception& e)
            {
                Base::Console().Log("Error %s\n", e.what());
                clearCard();
                // std::cerr << e.what() << '\n';
            }

        }
    }
}

MaterialDelegate::MaterialDelegate(QObject* parent)
    : QStyledItemDelegate(parent)
{
}

QWidget* MaterialDelegate::createEditor(
    QWidget* parent, const QStyleOptionViewItem&, const QModelIndex& index) const
{
    if (index.column() != 1)
        return nullptr;

    const QStandardItemModel *treeModel = static_cast<const QStandardItemModel *>(index.model());

    // Check we're not the material model root. This is also used to access the entry columns
    auto item = treeModel->itemFromIndex(index);
    auto group = item->parent();
    if (!group)
        return nullptr;

    int row = index.row();

    QString propertyName = group->child(row, 0)->text();
    QString propertyType = QString::fromStdString("String");
    if (group->child(row, 2))
        propertyType = group->child(row, 2)->text();

    QString propertyValue = QString::fromStdString("");
    if (group->child(row, 1))
        propertyValue = group->child(row, 1)->text();

    QWidget* editor = createWidget(parent, propertyName, propertyType, propertyValue);

    return editor;
}

QWidget* MaterialDelegate::createWidget(QWidget* parent, const QString &propertyName, const QString &propertyType,
    const QString &propertyValue) const
{
                    // minimum=None, maximum=None, stepsize=None, precision=12)
    // auto ui = Gui::WidgetFactory();
    QWidget* widget = nullptr;

    std::string type = propertyType.toStdString();
    if (type == "String" || type == "URL" || type == "Vector")
    {
        widget = new Gui::PrefLineEdit(parent);

    } else if ((type == "Integer") || (type == "Int"))
    {
        Gui::UIntSpinBox *spinner = new Gui::UIntSpinBox(parent);
        spinner->setMinimum(0);
        spinner->setMaximum(UINT_MAX);
        spinner->setValue(propertyValue.toUInt());
        widget = spinner;
    } else if (type == "Float")
    {
        Gui::DoubleSpinBox *spinner = new Gui::DoubleSpinBox(parent);
        
        // the magnetic permeability is the parameter for which many decimals matter
        // the most however, even for this, 6 digits are sufficient
        spinner->setDecimals(6);

        // for almost all Float parameters of materials a step of 1 would be too large
        spinner->setSingleStep(0.1);

        spinner->setMinimum(std::numeric_limits<double>::min());
        spinner->setMaximum(std::numeric_limits<double>::max());
        spinner->setValue(propertyValue.toDouble());
        widget = spinner;
    } else if (type == "Boolean")
    {
        Gui::PrefComboBox *combo = new Gui::PrefComboBox(parent);
        combo->insertItem(0, QString::fromStdString(""));
        combo->insertItem(1, QString::fromStdString("False"));
        combo->insertItem(2, QString::fromStdString("True"));
        combo->setCurrentText(propertyValue);
        widget = combo;
    } else if (type == "Color")
    {
        Gui::PrefColorButton *button = new Gui::PrefColorButton();
        // if Value:
        //     value = string2tuple(Value)
        //     color = QtGui.QColor()
        //     color.setRgb(value[0], value[1], value[2], value[3])
        //     widget.setProperty("color", color)
        QColor color;
        button->setProperty("color", color);

        widget = button;
    } else
    {
        // Default editor
        widget = new QLineEdit(parent);
    }

    widget->setProperty("Type", propertyType);
    widget->setParent(parent);

    return widget;
}
#include "moc_MaterialsEditor.cpp"
