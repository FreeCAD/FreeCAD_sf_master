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

#ifndef MATERIAL_MATERIALVALUE_H
#define MATERIAL_MATERIALVALUE_H

#include <QVariant>

namespace Materials {

class MaterialsExport MaterialValue
{
public:
    enum ValueType {
        None = 0,
        String = 1,
        Boolean = 2,
        Int = 3,
        Float = 4,
        Quantity = 5,
        Distribution = 6,
        List = 7,
        Array2D = 8,
        Array3D = 9,
        Color = 10,
        Image = 11,
        File = 12,
        URL = 13
    };
    MaterialValue();
    MaterialValue(ValueType type);
    virtual ~MaterialValue();

    ValueType getType() { return _valueType; }

    const QVariant &getValue(void) const { return _value; }
    virtual const QVariant &getValueAt(const QVariant &value) const { return _value; }
    void setValue (const QVariant &value) { _value = value; }

protected:
    ValueType _valueType;
    QVariant _value;

    void setType(ValueType type) { _valueType = type; }
};

class MaterialsExport Material2DArray : public MaterialValue
{
public:
    Material2DArray();
    ~Material2DArray() override;

    void setDefault(MaterialValue value) { _default = value; }
    MaterialValue getDefault() const { return _default; }

    const std::vector<QVariant> &getRow(int row);
    void addRow(std::vector<QVariant> *row);
    std::vector<QVariant> *deleteRow(int row);

    void setValue(int row, int column,  const QVariant &value);
    const QVariant &getValue(int row, int column);

protected:
    std::vector<std::vector<QVariant> *> _rows;
    MaterialValue _default;
};

} // namespace Materials

Q_DECLARE_METATYPE(Materials::MaterialValue)
Q_DECLARE_METATYPE(Materials::Material2DArray)

#endif // MATERIAL_MATERIALVALUE_H
