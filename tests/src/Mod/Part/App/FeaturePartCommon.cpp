// SPDX-License-Identifier: LGPL-2.1-or-later

#include "gtest/gtest.h"

#include "Mod/Part/App/FeaturePartCommon.h"
#include <src/App/InitApplication.h>

#include "PartTestHelpers.h"

class FeaturePartCommonTest: public ::testing::Test, public PartTestHelpers::PartTestHelperClass
{
protected:
    static void SetUpTestSuite()
    {
        tests::initApplication();
    }


    void SetUp() override
    {
        createTestDoc();
        _common = dynamic_cast<Part::Common*>(_doc->addObject("Part::Common"));
    }

    void TearDown() override
    {}

    Part::Common* _common = nullptr;  // NOLINT Can't be private in a test framework
};

TEST_F(FeaturePartCommonTest, testIntersecting)
{
    // Arrange
    _common->Base.setValue(_boxes[0]);
    _common->Tool.setValue(_boxes[1]);

    // Act
    _common->execute();
    Part::TopoShape ts = _common->Shape.getValue();
    double volume = PartTestHelpers::getVolume(ts.getShape());
    Base::BoundBox3d bb = ts.getBoundBox();

    // Assert
    EXPECT_DOUBLE_EQ(volume, 3.0);
    // double check using bounds:
    EXPECT_DOUBLE_EQ(bb.MinX, 0.0);
    EXPECT_DOUBLE_EQ(bb.MinY, 1.0);
    EXPECT_DOUBLE_EQ(bb.MinZ, 0.0);
    EXPECT_DOUBLE_EQ(bb.MaxX, 1.0);
    EXPECT_DOUBLE_EQ(bb.MaxY, 2.0);
    EXPECT_DOUBLE_EQ(bb.MaxZ, 3.0);
}

TEST_F(FeaturePartCommonTest, testNonIntersecting)
{
    // Arrange
    _common->Base.setValue(_boxes[0]);
    _common->Tool.setValue(_boxes[2]);

    // Act
    _common->execute();
    Part::TopoShape ts = _common->Shape.getValue();
    double volume = PartTestHelpers::getVolume(ts.getShape());
    Base::BoundBox3d bb = ts.getBoundBox();

    // Assert
    EXPECT_FALSE(bb.IsValid());
    EXPECT_DOUBLE_EQ(volume, 0.0);
}

TEST_F(FeaturePartCommonTest, testTouching)
{
    // Arrange
    _common->Base.setValue(_boxes[0]);
    _common->Tool.setValue(_boxes[3]);

    // Act
    _common->execute();
    Part::TopoShape ts = _common->Shape.getValue();
    double volume = PartTestHelpers::getVolume(ts.getShape());
    Base::BoundBox3d bb = ts.getBoundBox();

    // Assert
    EXPECT_FALSE(bb.IsValid());
    EXPECT_DOUBLE_EQ(volume, 0.0);
}

TEST_F(FeaturePartCommonTest, testAlmostTouching)
{
    // Arrange
    _common->Base.setValue(_boxes[0]);
    _common->Tool.setValue(_boxes[4]);

    // Act
    _common->execute();
    Part::TopoShape ts = _common->Shape.getValue();
    double volume = PartTestHelpers::getVolume(ts.getShape());
    Base::BoundBox3d bb = ts.getBoundBox();

    // Assert
    EXPECT_FALSE(bb.IsValid());
    EXPECT_DOUBLE_EQ(volume, 0.0);
}

TEST_F(FeaturePartCommonTest, testBarelyIntersecting)
{
    // Arrange
    _common->Base.setValue(_boxes[0]);
    _common->Tool.setValue(_boxes[5]);  // NOLINT magic number

    // Act
    _common->execute();
    Part::TopoShape ts = _common->Shape.getValue();
    double volume = PartTestHelpers::getVolume(ts.getShape());
    double target = PartTestHelpers::minimalDistance * 3;  // 3 dimensions in a Volume
    Base::BoundBox3d bb = ts.getBoundBox();

    // Assert
    // Using FLOAT, not DOUBLE here so test library comparison is of reasonable precision 1e07
    // rather than 1e15 See
    // https://google.github.io/googletest/reference/assertions.html#floating-point
    EXPECT_FLOAT_EQ(volume, target);  // Should be approximately 0.00029999999999996696
    // double check using bounds:
    EXPECT_DOUBLE_EQ(bb.MinX, 0.0);
    EXPECT_DOUBLE_EQ(bb.MinY, 2.0 - PartTestHelpers::minimalDistance);
    EXPECT_DOUBLE_EQ(bb.MinZ, 0.0);
    EXPECT_DOUBLE_EQ(bb.MaxX, 1.0);
    EXPECT_DOUBLE_EQ(bb.MaxY, 2.0);
    EXPECT_DOUBLE_EQ(bb.MaxZ, 3.0);
}

TEST_F(FeaturePartCommonTest, testMustExecute)
{
    // Assert initially we don't need to execute
    EXPECT_FALSE(_common->mustExecute());
    // Act to change one property
    _common->Base.setValue(_boxes[0]);
    // Assert we still can't execute
    EXPECT_FALSE(_common->mustExecute());
    // Act to complete the properties we need
    _common->Tool.setValue(_boxes[1]);
    // Assert that we now must execute
    EXPECT_TRUE(_common->mustExecute());
    // Act to execute
    _doc->recompute();
    // Assert we don't need to execute anymore
    EXPECT_FALSE(_common->mustExecute());
}

TEST_F(FeaturePartCommonTest, testGetProviderName)
{
    // Act
    _common->execute();
    const char* name = _common->getViewProviderName();
    // Assert
    EXPECT_STREQ(name, "PartGui::ViewProviderBoolean");
}

TEST_F(FeaturePartCommonTest, testHistory)
{
    // Arrange
    _common->Base.setValue(_boxes[0]);
    _common->Tool.setValue(_boxes[1]);
    // Manually create the histories classically generated by FreeCAD for comparison
    using MapList = std::map<int, std::vector<int>>;
    using List = std::vector<int>;
    MapList compare1 = {{0, List {0}},
                        {1, List {5}},  // NOLINT magic number
                        {2, List()},
                        {3, List {2}},
                        {4, List {3}},
                        {5, List {1}}};  // NOLINT magic number
    MapList compare2 = {{0, List {0}},
                        {1, List {5}},  // NOLINT magic number
                        {2, List {4}},
                        {3, List()},
                        {4, List {3}},
                        {5, List {1}}};  // NOLINT magic number

    // Act and Assert no histories yet
    std::vector<Part::ShapeHistory> hist = _common->History.getValues();
    EXPECT_EQ(hist.size(), 0);

    // Act to generate histories
    _common->execute();
    hist = _common->History.getValues();
    // Assert
#ifndef FC_USE_TNP_FIX
    ASSERT_EQ(hist.size(), 2);
    EXPECT_EQ(hist[0].shapeMap, compare1);
    EXPECT_EQ(hist[1].shapeMap, compare2);

    // Act to reverse the histories
    _common->Base.setValue(_boxes[1]);
    _common->Tool.setValue(_boxes[0]);
    _common->execute();
    hist = _common->History.getValues();
    // Assert
    ASSERT_EQ(hist.size(), 2);
    EXPECT_EQ(hist[0].shapeMap, compare2);
    EXPECT_EQ(hist[1].shapeMap, compare1);
#else
    ASSERT_EQ(hist.size(),
              0);  // TODO: with TNP enabled, this becomes 0, matches the code.  Correct?
#endif
}

TEST_F(FeaturePartCommonTest, testMapping)
{

    // Arrange
    _common->Base.setValue(_boxes[0]);
    _common->Tool.setValue(_boxes[1]);
    // Act
    _common->execute();
    const Part::TopoShape& ts1 = _common->Shape.getShape();
    // Assert
#ifndef FC_USE_TNP_FIX
    EXPECT_EQ(ts1.getElementMap().size(), 0);
#else
    EXPECT_EQ(ts1.getElementMap().size(), 26);
#endif
}
