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
# include <QMessageBox>
#endif

#include <Gui/MainWindow.h>

#include <Mod/Material/App/MaterialLibrary.h>
#include "MaterialSave.h"
#include "ui_MaterialSave.h"


using namespace MatGui;

/* TRANSLATOR MatGui::MaterialsEditor */

MaterialSave::MaterialSave(QWidget* parent)
  : QDialog(parent), ui(new Ui_MaterialSave)
{
    ui->setupUi(this);

    setLibraries();
    createModelTree();
    showSelectedTree();

    connect(ui->standardButtons, &QDialogButtonBox::accepted,
            this, &MaterialSave::accept);
    connect(ui->standardButtons, &QDialogButtonBox::rejected,
            this, &MaterialSave::reject);

    connect(ui->comboLibrary, &QComboBox::currentTextChanged,
            this, &MaterialSave::currentTextChanged);
    connect(ui->buttonNewFolder, &QPushButton::clicked,
            this, &MaterialSave::onNewFolder);

    QItemSelectionModel* selectionModel = ui->treeMaterials->selectionModel();
    connect(selectionModel, &QItemSelectionModel::selectionChanged,
            this, &MaterialSave::onSelectModel);
}

/*
 *  Destroys the object and frees any allocated resources
 */
MaterialSave::~MaterialSave()
{
    // no need to delete child widgets, Qt does it all for us
}

void MaterialSave::accept()
{
    QDialog::accept();
}

void MaterialSave::reject()
{
    QDialog::reject();
}

void MaterialSave::setLibraries()
{
    std::list<Materials::MaterialLibrary*> *libraries = _manager.getMaterialLibraries();
    for (Materials::MaterialLibrary *library: *libraries)
    {
        if (!library->isReadOnly())
        {
            QVariant libraryVariant;
            libraryVariant.setValue(*library);
            ui->comboLibrary->addItem(QString::fromStdString(library->getName()), libraryVariant);
        }
    }
}

void MaterialSave::createModelTree()
{
    auto tree = ui->treeMaterials;
    auto model = new QStandardItemModel();
    tree->setModel(model);
    tree->setHeaderHidden(true);
}

void MaterialSave::addExpanded(QTreeView *tree, QStandardItem *parent, QStandardItem *child)
{
    parent->appendRow(child);
    tree->setExpanded(child->index(), true);
}

void MaterialSave::addExpanded(QTreeView *tree, QStandardItemModel *parent, QStandardItem *child)
{
    parent->appendRow(child);
    tree->setExpanded(child->index(), true);
}

void MaterialSave::addMaterials(QStandardItem &parent, const std::map<std::string, Materials::MaterialTreeNode*>* modelTree)
{
    auto tree = ui->treeMaterials;
    for (auto& mat : *modelTree) {
        Materials::MaterialTreeNode *nodePtr = mat.second;
        if (nodePtr->getType() == Materials::MaterialTreeNode::DataNode)
        {
            const Materials::Material *material = nodePtr->getData();
            std::string uuid = material->getUUID();

            auto card = new QStandardItem(QString::fromStdString(material->getName()));
            card->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled | Qt::ItemIsDragEnabled
                        | Qt::ItemIsDropEnabled);
            card->setData(QVariant(QString::fromStdString(uuid)), Qt::UserRole);

            addExpanded(tree, &parent, card);
        } else {
            auto node = new QStandardItem(QString::fromStdString(mat.first));
            addExpanded(tree, &parent, node);
            node->setFlags(Qt::ItemIsEnabled | Qt::ItemIsDragEnabled | Qt::ItemIsDropEnabled);
            const std::map<std::string, Materials::MaterialTreeNode*>* treeMap = nodePtr->getFolder();
            addMaterials(*node, treeMap);
        }
    }
}

void MaterialSave::showSelectedTree()
{
    auto tree = ui->treeMaterials;
    auto model = static_cast<QStandardItemModel *>(tree->model());
    model->clear();

    if (ui->comboLibrary->count() > 0)
    {
        auto variant = ui->comboLibrary->currentData();
        auto library = variant.value<Materials::MaterialLibrary>();

        auto lib = new QStandardItem(QString::fromStdString(library.getName()));
        lib->setFlags(Qt::ItemIsEnabled | Qt::ItemIsDragEnabled | Qt::ItemIsDropEnabled);
        addExpanded(tree, model, lib);

        std::map<std::string, Materials::MaterialTreeNode*>* modelTree = _manager.getMaterialTree(library);
        addMaterials(*lib, modelTree);
    } else {
        QMessageBox::warning(Gui::getMainWindow(), QObject::tr("No writeable library"),
            QObject::tr("No writeable library"));
    }

}

void MaterialSave::onSelectModel(const QItemSelection& selected, const QItemSelection& deselected)
{
    Q_UNUSED(selected);
    Q_UNUSED(deselected);

    // QStandardItemModel *model = static_cast<QStandardItemModel *>(ui->treeModels->model());
    // QModelIndexList indexes = selected.indexes();
    // for (auto it = indexes.begin(); it != indexes.end(); it++)
    // {
    //     QStandardItem* item = model->itemFromIndex(*it);
    //     Base::Console().Log("%s\n", item->text().toStdString().c_str());
    //     if (item) {
    //         try
    //         {
    //             _selected = item->data(Qt::UserRole).toString().toStdString();
    //             Base::Console().Log("\t%s\n", _selected.c_str());
    //             updateMaterialModel(_selected);
    //             ui->standardButtons->button(QDialogButtonBox::Ok)->setEnabled(true);
    //             ui->buttonFavorite->setEnabled(true);
    //         }
    //         catch(const std::exception& e)
    //         {
    //             _selected = "";
    //             Base::Console().Log("Error %s\n", e.what());
    //             clearMaterialModel();
    //             ui->standardButtons->button(QDialogButtonBox::Ok)->setEnabled(false);
    //             ui->buttonFavorite->setEnabled(false);
    //         }

    //     }
    // }
}

void MaterialSave::currentTextChanged(const QString &value)
{
    Q_UNUSED(value)

    showSelectedTree();
}

void MaterialSave::onNewFolder(bool checked)
{
    Q_UNUSED(checked)

    auto tree = ui->treeMaterials;
    auto model = static_cast<QStandardItemModel *>(tree->model());
    auto current = tree->currentIndex();
    if (!current.isValid())
    {
        current = model->index(0, 0);
    }
    auto item = model->itemFromIndex(current);
    if (item->hasChildren())
    {
        Base::Console().Log("Add new folder to '%s'\n", item->text().toStdString().c_str());
    }
}

#include "moc_MaterialSave.cpp"
