// SPDX-License-Identifier: LGPL-2.1-or-later

#include "gtest/gtest.h"

#include <App/Application.h>
#include <App/ElementMap.h>

// NOLINTBEGIN(readability-magic-numbers)


// this is a "holder" class used for simpler testing of ElementMap in the context of a class
class LessComplexPart
{
public:
    LessComplexPart(long tag, const std::string& nameStr, App::StringHasherRef hasher)
        : elementMapPtr(std::make_shared<Data::ElementMap>())
        , Tag(tag)
        , name(nameStr)
    {
        // object also have Vertexes etc and the face count varies; but that is not important
        // here since we are not testing a real model
        // the "MappedName" is left blank for now
        Data::IndexedName face1("Face", 1);
        Data::IndexedName face2("Face", 2);
        Data::IndexedName face3("Face", 3);
        Data::IndexedName face4("Face", 4);
        Data::IndexedName face5("Face", 5);
        Data::IndexedName face6("Face", 6);
        elementMapPtr->hasher = hasher;
        elementMapPtr->setElementName(face1, Data::MappedName(face1), Tag);
        elementMapPtr->setElementName(face2, Data::MappedName(face2), Tag);
        elementMapPtr->setElementName(face3, Data::MappedName(face3), Tag);
        elementMapPtr->setElementName(face4, Data::MappedName(face4), Tag);
        elementMapPtr->setElementName(face5, Data::MappedName(face5), Tag);
        elementMapPtr->setElementName(face6, Data::MappedName(face6), Tag);
    }

    Data::ElementMapPtr elementMapPtr;
    mutable long Tag;
    Data::MappedName name;
};

class ElementMapTest: public ::testing::Test
{
protected:
    static void SetUpTestSuite()
    {
        int argc = 1;
        char* argv[] = {"FreeCAD"};
        App::Application::Config()["ExeName"] = "FreeCAD";
        App::Application::init(argc, argv);
    }

    void SetUp() override
    {
        App::GetApplication().newDocument("test", "testUser");
        _sids = &_sid;
        _hasher = Base::Reference<App::StringHasher>(new App::StringHasher);
    }

    // void TearDown() override {}

    Data::ElementIDRefs _sid;
    QVector<App::StringIDRef>* _sids;
    App::StringHasherRef _hasher;
};

TEST_F(ElementMapTest, defaultConstruction)
{
    // Act
    Data::ElementMap elementMap = Data::ElementMap();

    // Assert
    EXPECT_EQ(elementMap.size(), 0);
}

TEST_F(ElementMapTest, setElementNameDefaults)
{
    // Arrange
    Data::ElementMap elementMap;
    Data::IndexedName element("Edge", 1);
    Data::MappedName mappedName("TEST");

    // Act
    auto resultName = elementMap.setElementName(element, mappedName, 0, _sids);
    auto mappedToElement = elementMap.find(element);

    // Assert
    EXPECT_EQ(resultName, mappedName);
    EXPECT_EQ(mappedToElement, mappedName);
}

TEST_F(ElementMapTest, setElementNameWithHashing)
{
    // Arrange
    Data::ElementMap elementMap;
    std::ostringstream ss;
    Data::IndexedName element("Edge", 1);
    Data::MappedName elementNameHolder(element);// Will get modified by the encoder
    const Data::MappedName expectedName(element);

    // Act
    elementMap.encodeElementName(
        element.getType()[0], elementNameHolder, ss, nullptr, 0, nullptr, 0);
    auto resultName = elementMap.setElementName(element, elementNameHolder, 0, _sids);
    auto mappedToElement = elementMap.find(element);

    // Assert
    EXPECT_EQ(resultName, expectedName);
    EXPECT_EQ(mappedToElement, expectedName);
}

TEST_F(ElementMapTest, mimicOnePart)
{
    // Arrange
    //   pattern: new doc, create Cube
    //   for a single part, there is no "naming algo" to speak of
    std::ostringstream ss;
    auto docName = "Unnamed";
    LessComplexPart cube(1L, "Box", _hasher);

    // Act
    auto children = cube.elementMapPtr->getAll();
    ss << docName << "#" << cube.name << "."
       << cube.elementMapPtr->find(Data::IndexedName("Face", 6));

    // Assert
    EXPECT_EQ(children.size(), 6);
    EXPECT_EQ(children[0].index.toString(), "Face1");
    EXPECT_EQ(children[0].name.toString(), "Face1");
    EXPECT_EQ(children[1].index.toString(), "Face2");
    EXPECT_EQ(children[1].name.toString(), "Face2");
    EXPECT_EQ(children[2].index.toString(), "Face3");
    EXPECT_EQ(children[2].name.toString(), "Face3");
    EXPECT_EQ(children[3].index.toString(), "Face4");
    EXPECT_EQ(children[3].name.toString(), "Face4");
    EXPECT_EQ(children[4].index.toString(), "Face5");
    EXPECT_EQ(children[4].name.toString(), "Face5");
    EXPECT_EQ(children[5].index.toString(), "Face6");
    EXPECT_EQ(children[5].name.toString(), "Face6");
    EXPECT_EQ(ss.str(), "Unnamed#Box.Face6");
}

TEST_F(ElementMapTest, mimicSimpleUnion)
{
    // Arrange
    //   pattern: new doc, create Cube, create Cylinder, Union of both (Cube first)
    std::ostringstream ss;
    std::ostringstream finalSs;
    char* docName = "Unnamed";

    LessComplexPart cube(1L, "Box", _hasher);
    LessComplexPart cylinder(2L, "Cylinder", _hasher);
    LessComplexPart unionPart(3L, "Fusion", _hasher);// Union (Fusion) operation via the Part Workbench

    // we are only going to simulate one face for testing purpose
    Data::IndexedName uface3("Face", 3);
    auto PartOp = "FUS";// Part::OpCodes::Fuse;

    // Act
    //   act: simulate a union/fuse operation
    auto parent = cube.elementMapPtr->getAll()[5];
    Data::MappedName postfixHolder(std::string(Data::POSTFIX_MOD) + "2");
    unionPart.elementMapPtr->encodeElementName(
        postfixHolder[0], postfixHolder, ss, nullptr, unionPart.Tag, nullptr, unionPart.Tag);
    auto postfixStr = postfixHolder.toString() + Data::ELEMENT_MAP_PREFIX + PartOp;

    //   act: with the fuse op, name against the cube's Face6
    Data::MappedName uface3Holder(parent.index);
    // we will invoke the encoder for face 3
    unionPart.elementMapPtr->encodeElementName(
        uface3Holder[0], uface3Holder, ss, nullptr, unionPart.Tag, postfixStr.c_str(), cube.Tag);
    unionPart.elementMapPtr->setElementName(uface3, uface3Holder, unionPart.Tag, nullptr, true);

    // act: generate a full toponame string for testing  purposes
    finalSs << docName << "#" << unionPart.name;
    finalSs << ".";
    finalSs << Data::ELEMENT_MAP_PREFIX + unionPart.elementMapPtr->find(uface3).toString();
    finalSs << ".";
    finalSs << uface3;

    // Assert
    EXPECT_EQ(postfixStr, ":M2;FUS");
    EXPECT_EQ(unionPart.elementMapPtr->find(uface3).toString(), "Face6;:M2;FUS;:H1:8,F");
    EXPECT_EQ(finalSs.str(), "Unnamed#Fusion.;Face6;:M2;FUS;:H1:8,F.Face3");

    // explanation of "Fusion.;Face6;:M2;FUS;:H2:3,F.Face3" toponame
    // Note: every postfix is prefixed by semicolon
    // Note: the start/middle/end are separated by periods
    //
    // "Fusion" means that we are on the "Fusion" object.
    // "." we are done with the first part
    // ";Face6" means default inheritance comes from face 6 of the parent (which is a cube)
    // ";:M2" means that a Workbench op has happened. "M" is the "Mod" directory in the source tree?
    // ";FUS" means that a Fusion operation has happened. Notice the lack of a colon.
    // ";:H2" means the subtending object (cylinder) has a tag of 2
    // ":3" means the writing position is 3; literally how far into the current postfix we are
    // ",F" means are of type "F" which is short for "Face" of Face3 of Fusion.
    // "." we are done with the second part
    // "Face3" is the localized name
}

TEST_F(ElementMapTest, mimicOperationAgainstSelf)
{
    // Arrange
    //   pattern: new doc, create Cube, Mystery Op with self as target
    std::ostringstream ss;
    LessComplexPart finalPart(99L, "MysteryOp", _hasher);
    // we are only going to simulate one face for testing purpose
    Data::IndexedName uface3("Face", 3);
    auto PartOp = "MYS";
    auto ownFace6 = finalPart.elementMapPtr->getAll()[5];
    Data::MappedName uface3Holder(ownFace6.index);
    auto workbenchId = std::string(Data::POSTFIX_MOD) + "9999";

    // Act
    //   act: with the mystery op, name against its own Face6 for some reason
    Data::MappedName postfixHolder(workbenchId);
    finalPart.elementMapPtr->encodeElementName(
        postfixHolder[0], postfixHolder, ss, nullptr, finalPart.Tag, nullptr, finalPart.Tag);
    auto postfixStr = postfixHolder.toString() + Data::ELEMENT_MAP_PREFIX + PartOp;
    // we will invoke the encoder for face 3
    finalPart.elementMapPtr->encodeElementName(
        uface3Holder[0], uface3Holder, ss, nullptr, finalPart.Tag, postfixStr.c_str(), finalPart.Tag);
    finalPart.elementMapPtr->setElementName(uface3, uface3Holder, finalPart.Tag, nullptr, false); // override not forced

    // Assert
    EXPECT_EQ(postfixStr, ":M9999;MYS");
    EXPECT_EQ(finalPart.elementMapPtr->find(uface3).toString(), "Face3"); // the override was not forced
    EXPECT_EQ(uface3Holder.toString(), "Face6;:M9999;MYS;:H63:b,F");
    // explaining ";Face6;:M2;MYS;:H2:3,F" name:
    //
    // ";Face6" means default inheritance comes from face 6 of the ownFace6 (which is itself)
    // ";:M9999" means that a Workbench op has happened. "M" is the "Mod" directory in the source tree?
    // ";MYS" means that a "Mystery" operation has happened. Notice the lack of a colon.
    // ";:H63" means the subtending object (cylinder) has a tag of 99 (63 in hex)
    // ":b" means the writing position is b (hex); literally how far into the current postfix we are
    // ",F" means are of type "F" which is short for "Face" of Face3 of Fusion.
}

TEST_F(ElementMapTest, hasChildElementMapTest)
{
    // Arrange
    auto child = (Data::ElementMap::MappedChildElements) {
        Data::IndexedName("face", 1),
        2,
        7,
        4L,
        Data::ElementMapPtr(),
        QByteArray("")
    };
    std::vector<Data::ElementMap::MappedChildElements> children = { child };
    LessComplexPart cubeFull(3L, "FullBox", _hasher);
    cubeFull.elementMapPtr->addChildElements(cubeFull.Tag, children);
    //
    LessComplexPart cubeWithoutChildren(2L, "EmptyBox", _hasher);

    // Act
    bool resultFull = cubeFull.elementMapPtr->hasChildElementMap();
    bool resultWhenEmpty = cubeWithoutChildren.elementMapPtr->hasChildElementMap();

    // Assert
    EXPECT_TRUE(resultFull);
    EXPECT_FALSE(resultWhenEmpty);
}

TEST_F(ElementMapTest, hashChildMapsTest)
{
    // Arrange
    LessComplexPart cube(1L, "Box", _hasher);
    auto childOneName = Data::IndexedName("Ping", 1);
    auto childOne = (Data::ElementMap::MappedChildElements) {
        childOneName,
        2,
        7,
        3L,
        Data::ElementMapPtr(),
        QByteArray("abcdefghij"), // postfix must be 10 or more bytes to invoke hasher
        _sid
    };
    std::vector<Data::ElementMap::MappedChildElements> children = { childOne };
    cube.elementMapPtr->addChildElements(cube.Tag, children);
    auto before = _hasher->getIDMap();

    // Act
    cube.elementMapPtr->hashChildMaps(cube.Tag);

    // Assert
    auto after = _hasher->getIDMap();
    EXPECT_EQ(before.size(), 0);
    EXPECT_EQ(after.size(), 1);
}

TEST_F(ElementMapTest, addAndGetChildElementsTest)
{
    // Arrange
    LessComplexPart cube(1L, "Box", _hasher);
    auto childOne = (Data::ElementMap::MappedChildElements) {
        Data::IndexedName("Ping", 1),
        2,
        7,
        3L,
        Data::ElementMapPtr(),
        QByteArray("abcdefghij"), // postfix must be 10 or more bytes to invoke hasher
        _sid
    };
    auto childTwo = (Data::ElementMap::MappedChildElements) {
        Data::IndexedName("Pong", 2),
        2,
        7,
        4L,
        Data::ElementMapPtr(),
        QByteArray("abc"),
        _sid
    };
    std::vector<Data::ElementMap::MappedChildElements> children = { childOne, childTwo };

    // Act
    cube.elementMapPtr->addChildElements(cube.Tag, children);
    auto result = cube.elementMapPtr->getChildElements();

    // Assert
    EXPECT_EQ(result.size(), 2);
    EXPECT_TRUE(std::any_of(result.begin(), result.end(), [](Data::ElementMap::MappedChildElements e){return e.indexedName.toString()=="Ping1";}));
    EXPECT_TRUE(std::any_of(result.begin(), result.end(), [](Data::ElementMap::MappedChildElements e){return e.indexedName.toString()=="Pong2";}));
}

// NOLINTEND(readability-magic-numbers)
