<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE TS>
<TS version="2.1" language="sr" sourcelanguage="en">
  <context>
    <name>Assembly_CreateAssembly</name>
    <message>
      <location filename="../../../CommandCreateAssembly.py" line="48"/>
      <source>Create Assembly</source>
      <translation>Направи склоп</translation>
    </message>
    <message>
      <location filename="../../../CommandCreateAssembly.py" line="53"/>
      <source>Create an assembly object in the current document, or in the current active assembly (if any). Limit of one root assembly per file.</source>
      <translation>Направи главни склоп у корену стабла документа, или подсклоп у тренутно активном склопу. Може постојати само један главни склоп.</translation>
    </message>
  </context>
  <context>
    <name>Assembly_CreateJointFixed</name>
    <message>
      <location filename="../../../CommandCreateJoint.py" line="76"/>
      <source>Create a Fixed Joint</source>
      <translation>Направи фиксни спој</translation>
    </message>
    <message>
      <location filename="../../../CommandCreateJoint.py" line="83"/>
      <source>1 - If an assembly is active : Create a joint permanently locking two parts together, preventing any movement or rotation.</source>
      <translation>1 - Ако је активан склоп: Направи спој који трајно закључа два дела заједно, спречавајуц́и било какво кретање или ротацију.</translation>
    </message>
    <message>
      <location filename="../../../CommandCreateJoint.py" line="89"/>
      <source>2 - If a part is active : Position sub parts by matching selected coordinate systems. The second part selected will move.</source>
      <translation>2 - Ако је активан део: Позиционира делове тако што ће направити подударним изабране координатне системе. Други изабрани део ће се померити.</translation>
    </message>
  </context>
  <context>
    <name>Assembly_CreateJointRevolute</name>
    <message>
      <location filename="../../../CommandCreateJoint.py" line="111"/>
      <source>Create Revolute Joint</source>
      <translation>Направи ротациони спој</translation>
    </message>
    <message>
      <location filename="../../../CommandCreateJoint.py" line="118"/>
      <source>Create a Revolute Joint: Allows rotation around a single axis between selected parts.</source>
      <translation>Направи ротациони спој (Кинематски пар V класе) између изабраних делова: Дозвољава ротацију око једне осе.</translation>
    </message>
  </context>
  <context>
    <name>Assembly_CreateJointCylindrical</name>
    <message>
      <location filename="../../../CommandCreateJoint.py" line="139"/>
      <source>Create Cylindrical Joint</source>
      <translation>Направи цилиндрични спој</translation>
    </message>
    <message>
      <location filename="../../../CommandCreateJoint.py" line="146"/>
      <source>Create a Cylindrical Joint: Enables rotation along one axis while permitting movement along the same axis between assembled parts.</source>
      <translation>Направи цилиндрични спој (Кинематски пар IV класе) измеђи изабраних делова: Дозвољава ротацију око једне осе и транслацију по истој оси.</translation>
    </message>
  </context>
  <context>
    <name>Assembly_CreateJointSlider</name>
    <message>
      <location filename="../../../CommandCreateJoint.py" line="165"/>
      <source>Create Slider Joint</source>
      <translation>Направи транслаторни спој</translation>
    </message>
    <message>
      <location filename="../../../CommandCreateJoint.py" line="172"/>
      <source>Create a Slider Joint: Allows linear movement along a single axis but restricts rotation between selected parts.</source>
      <translation>Направи транслаторни спој (Кинематски пар V класе) између изабраних делова: Дозвољава једну транслацију.</translation>
    </message>
  </context>
  <context>
    <name>Assembly_CreateJointBall</name>
    <message>
      <location filename="../../../CommandCreateJoint.py" line="191"/>
      <source>Create Ball Joint</source>
      <translation>Направи кугласти спој</translation>
    </message>
    <message>
      <location filename="../../../CommandCreateJoint.py" line="198"/>
      <source>Create a Ball Joint: Connects parts at a point, allowing unrestricted movement as long as the connection points remain in contact.</source>
      <translation>Направи кугласти спој (Кинематски пар III класе) између изабраних делова: Дозвољава ротацију око све три осе.</translation>
    </message>
  </context>
  <context>
    <name>Assembly_CreateJointDistance</name>
    <message>
      <location filename="../../../CommandCreateJoint.py" line="217"/>
      <source>Create Distance Joint</source>
      <translation>Направи равански спој</translation>
    </message>
    <message>
      <location filename="../../../CommandCreateJoint.py" line="224"/>
      <source>Create a Distance Joint: Fix the distance between the selected objects.</source>
      <translation>Направи равански спој (кинематски пар III класе) између изабаних делова: Дозвољава ротацију око једне осе и транслацију по две осе.</translation>
    </message>
    <message>
      <location filename="../../../CommandCreateJoint.py" line="230"/>
      <source>Create one of several different joints based on the selection.For example, a distance of 0 between a plane and a cylinder creates a tangent joint. A distance of 0 between planes will make them co-planar.</source>
      <translation>На основу избора направи један или неколико различитих спојева. На пример, растојање 0 између равни и цилиндра направиће их тангентним, растојање 0 између равни направиће их копланарним.</translation>
    </message>
  </context>
  <context>
    <name>Assembly_ToggleGrounded</name>
    <message>
      <location filename="../../../CommandCreateJoint.py" line="496"/>
      <source>Toggle grounded</source>
      <translation>Учврсти</translation>
    </message>
    <message>
      <location filename="../../../CommandCreateJoint.py" line="503"/>
      <source>Grounding a part permanently locks its position in the assembly, preventing any movement or rotation. You need at least one grounded part before starting to assemble.</source>
      <translation>Учвршћавање трајно фиксира позицију дела у склопу, спречавајући било какву транслацију или ротацију. Потребан је најмање један учвршћен део пре него што се почне формирати склоп.</translation>
    </message>
  </context>
  <context>
    <name>Assembly_ExportASMT</name>
    <message>
      <location filename="../../../CommandExportASMT.py" line="47"/>
      <source>Export ASMT File</source>
      <translation>Извези АСМТ датотеку</translation>
    </message>
    <message>
      <location filename="../../../CommandExportASMT.py" line="52"/>
      <source>Export currently active assembly as a ASMT file.</source>
      <translation>Извези тренутно активан склоп као АСМТ датотеку.</translation>
    </message>
  </context>
  <context>
    <name>Assembly_InsertLink</name>
    <message>
      <location filename="../../../CommandInsertLink.py" line="53"/>
      <source>Insert Component</source>
      <translation>Уметни компоненту</translation>
    </message>
    <message>
      <location filename="../../../CommandInsertLink.py" line="60"/>
      <source>Insert a component into the active assembly. This will create dynamic links to parts, bodies, primitives, and assemblies. To insert external components, make sure that the file is &lt;b&gt;open in the current session&lt;/b&gt;</source>
      <translation>Уметни компоненту у активни склоп. Ово ц́е створити динамичке везе ка деловима, телима, примитивима и склоповима. Да би уметнуо спољне компоненте, прво се увери да је датотека &lt;b&gt;отворена у тренутној сесији&lt;/b&gt;</translation>
    </message>
    <message>
      <location filename="../../../CommandInsertLink.py" line="62"/>
      <source>Insert by left clicking items in the list.</source>
      <translation>Уметнути левим кликом на ставке у листи.</translation>
    </message>
    <message>
      <location filename="../../../CommandInsertLink.py" line="66"/>
      <source>Remove by right clicking items in the list.</source>
      <translation>Уклонити десним кликом на ставке у листи.</translation>
    </message>
    <message>
      <location filename="../../../CommandInsertLink.py" line="71"/>
      <source>Press shift to add several instances of the component while clicking on the view.</source>
      <translation>Да би изабрао више компоненти одједанпут држи притиснуту типку shift.</translation>
    </message>
  </context>
  <context>
    <name>Assembly_SolveAssembly</name>
    <message>
      <location filename="../../../CommandSolveAssembly.py" line="51"/>
      <source>Solve Assembly</source>
      <translation>Реши склоп</translation>
    </message>
    <message>
      <location filename="../../../CommandSolveAssembly.py" line="58"/>
      <source>Solve the currently active assembly.</source>
      <translation>Реши тренутно активни склоп.</translation>
    </message>
  </context>
  <context>
    <name>QObject</name>
    <message>
      <location filename="../../../InitGui.py" line="74"/>
      <source>Assembly</source>
      <translation>Скупштина</translation>
    </message>
  </context>
  <context>
    <name>Workbench</name>
    <message>
      <location filename="../../../InitGui.py" line="108"/>
      <source>Assembly</source>
      <translation>Склопови</translation>
    </message>
    <message>
      <location filename="../../../InitGui.py" line="109"/>
      <source>Assembly Joints</source>
      <translation>Спојеви у склопу</translation>
    </message>
    <message>
      <location filename="../../../InitGui.py" line="112"/>
      <source>&amp;Assembly</source>
      <translation>&amp;Склопови</translation>
    </message>
  </context>
  <context>
    <name>Assembly</name>
    <message>
      <location filename="../../../JointObject.py" line="46"/>
      <source>Fixed</source>
      <translation>Учвршћен</translation>
    </message>
    <message>
      <location filename="../../../JointObject.py" line="47"/>
      <source>Revolute</source>
      <translation>Ротациони</translation>
    </message>
    <message>
      <location filename="../../../JointObject.py" line="48"/>
      <source>Cylindrical</source>
      <translation>Цилиндрични</translation>
    </message>
    <message>
      <location filename="../../../JointObject.py" line="49"/>
      <source>Slider</source>
      <translation>Транслациони</translation>
    </message>
    <message>
      <location filename="../../../JointObject.py" line="50"/>
      <source>Ball</source>
      <translation>Кугласти</translation>
    </message>
    <message>
      <location filename="../../../JointObject.py" line="51"/>
      <location filename="../../../JointObject.py" line="1530"/>
      <source>Distance</source>
      <translation>Растојање</translation>
    </message>
    <message>
      <location filename="../../../JointObject.py" line="52"/>
      <source>Parallel</source>
      <translation>Паралелно</translation>
    </message>
    <message>
      <location filename="../../../JointObject.py" line="53"/>
      <source>Perpendicular</source>
      <translation>Усправан</translation>
    </message>
    <message>
      <location filename="../../../JointObject.py" line="54"/>
      <location filename="../../../JointObject.py" line="1532"/>
      <source>Angle</source>
      <translation>Угао</translation>
    </message>
    <message>
      <location filename="../../../JointObject.py" line="55"/>
      <source>RackPinion</source>
      <translation>Зупчаста летва</translation>
    </message>
    <message>
      <location filename="../../../JointObject.py" line="56"/>
      <source>Screw</source>
      <translation>Завртањ</translation>
    </message>
    <message>
      <location filename="../../../JointObject.py" line="57"/>
      <source>Gears</source>
      <translation>Зупчаник</translation>
    </message>
    <message>
      <location filename="../../../JointObject.py" line="58"/>
      <source>Belt</source>
      <translation>Ремен</translation>
    </message>
    <message>
      <location filename="../../../JointObject.py" line="1365"/>
      <source>You need to select 2 elements from 2 separate parts.</source>
      <translation>Потребно је изабрати 2 елемента са 2 различита дела.</translation>
    </message>
    <message>
      <location filename="../../../JointObject.py" line="1534"/>
      <source>Radius 1</source>
      <translation type="unfinished">Radius 1</translation>
    </message>
    <message>
      <location filename="../../../JointObject.py" line="1536"/>
      <source>Pitch radius</source>
      <translation type="unfinished">Pitch radius</translation>
    </message>
    <message>
      <location filename="../../../Preferences.py" line="49"/>
      <source>Ask</source>
      <translation>Питај</translation>
    </message>
    <message>
      <location filename="../../../Preferences.py" line="50"/>
      <source>Always</source>
      <translation>Увек</translation>
    </message>
    <message>
      <location filename="../../../Preferences.py" line="51"/>
      <source>Never</source>
      <translation>Никада</translation>
    </message>
    <message>
      <location filename="../../../CommandCreateBom.py" line="46"/>
      <source>Index (auto)</source>
      <translation type="unfinished">Index (auto)</translation>
    </message>
    <message>
      <location filename="../../../CommandCreateBom.py" line="47"/>
      <source>Name (auto)</source>
      <translation type="unfinished">Name (auto)</translation>
    </message>
    <message>
      <location filename="../../../CommandCreateBom.py" line="48"/>
      <source>Description</source>
      <translation>Опис</translation>
    </message>
    <message>
      <location filename="../../../CommandCreateBom.py" line="49"/>
      <source>File Name (auto)</source>
      <translation type="unfinished">File Name (auto)</translation>
    </message>
    <message>
      <location filename="../../../CommandCreateBom.py" line="50"/>
      <source>Quantity (auto)</source>
      <translation type="unfinished">Quantity (auto)</translation>
    </message>
    <message>
      <location filename="../../../CommandCreateBom.py" line="197"/>
      <source>Default</source>
      <translation>Подразумевано</translation>
    </message>
    <message>
      <location filename="../../../CommandCreateBom.py" line="293"/>
      <source>Duplicate Name</source>
      <translation>Дуплирано име</translation>
    </message>
    <message>
      <location filename="../../../CommandCreateBom.py" line="294"/>
      <source>This name is already used. Please choose a different name.</source>
      <translation>Ово име је већ у употреби. Изабери друго име.</translation>
    </message>
    <message>
      <location filename="../../../CommandCreateBom.py" line="367"/>
      <source>Options:</source>
      <translation type="unfinished">Options:</translation>
    </message>
    <message>
      <location filename="../../../CommandCreateBom.py" line="374"/>
      <source>Sub-assemblies children : If checked, Sub assemblies children will be added to the bill of materials.</source>
      <translation type="unfinished">Sub-assemblies children : If checked, Sub assemblies children will be added to the bill of materials.</translation>
    </message>
    <message>
      <location filename="../../../CommandCreateBom.py" line="380"/>
      <source>Parts children : If checked, Parts children will be added to the bill of materials.</source>
      <translation type="unfinished">Parts children : If checked, Parts children will be added to the bill of materials.</translation>
    </message>
    <message>
      <location filename="../../../CommandCreateBom.py" line="386"/>
      <source>Only parts : If checked, only Part containers and sub-assemblies will be added to the bill of materials. Solids like PartDesign Bodies, fasteners or Part workbench primitives will be ignored.</source>
      <translation type="unfinished">Only parts : If checked, only Part containers and sub-assemblies will be added to the bill of materials. Solids like PartDesign Bodies, fasteners or Part workbench primitives will be ignored.</translation>
    </message>
    <message>
      <location filename="../../../CommandCreateBom.py" line="388"/>
      <source>Columns:</source>
      <translation>Колоне:</translation>
    </message>
    <message>
      <location filename="../../../CommandCreateBom.py" line="395"/>
      <source>Auto columns :  (Index, Quantity, Name...) are populated automatically. Any modification you make will be overridden. These columns cannot be renamed.</source>
      <translation type="unfinished">Auto columns :  (Index, Quantity, Name...) are populated automatically. Any modification you make will be overridden. These columns cannot be renamed.</translation>
    </message>
    <message>
      <location filename="../../../CommandCreateBom.py" line="401"/>
      <source>Custom columns : 'Description' and other custom columns you add by clicking on 'Add column' will not have their data overwritten. These columns can be renamed by double-clicking or pressing F2 (Renaming a column will currently lose its data).</source>
      <translation type="unfinished">Custom columns : 'Description' and other custom columns you add by clicking on 'Add column' will not have their data overwritten. These columns can be renamed by double-clicking or pressing F2 (Renaming a column will currently lose its data).</translation>
    </message>
    <message>
      <location filename="../../../CommandCreateBom.py" line="407"/>
      <source>Any column (custom or not) can be deleted by pressing Del.</source>
      <translation type="unfinished">Any column (custom or not) can be deleted by pressing Del.</translation>
    </message>
  </context>
  <context>
    <name>App::Property</name>
    <message>
      <location filename="../../../JointObject.py" line="192"/>
      <source>The type of the joint</source>
      <translation>Врста споја</translation>
    </message>
    <message>
      <location filename="../../../JointObject.py" line="213"/>
      <location filename="../../../JointObject.py" line="449"/>
      <source>The first object of the joint</source>
      <translation>Први објекат у споју</translation>
    </message>
    <message>
      <location filename="../../../JointObject.py" line="221"/>
      <source>The first part of the joint</source>
      <translation>Први део у споју</translation>
    </message>
    <message>
      <location filename="../../../JointObject.py" line="232"/>
      <source>This is the local coordinate system within object1 that will be used for the joint.</source>
      <translation>Ово је локални координатни систем унутар object1 који ће се користити у споју.</translation>
    </message>
    <message>
      <location filename="../../../JointObject.py" line="243"/>
      <source>This prevents Placement1 from recomputing, enabling custom positioning of the placement.</source>
      <translation>Ово спречава да се поново израчунава Placement1 омогућавајући сопствено позиционирање.</translation>
    </message>
    <message>
      <location filename="../../../JointObject.py" line="252"/>
      <location filename="../../../JointObject.py" line="468"/>
      <source>The second object of the joint</source>
      <translation>Други објекат у споју</translation>
    </message>
    <message>
      <location filename="../../../JointObject.py" line="260"/>
      <source>The second part of the joint</source>
      <translation>Други део у споју</translation>
    </message>
    <message>
      <location filename="../../../JointObject.py" line="271"/>
      <source>This is the local coordinate system within object2 that will be used for the joint.</source>
      <translation>Ово је локални координатни систем унутар object2 који ће се користити у споју.</translation>
    </message>
    <message>
      <location filename="../../../JointObject.py" line="282"/>
      <source>This prevents Placement2 from recomputing, enabling custom positioning of the placement.</source>
      <translation>Ово спречава да се поново израчунава Placement2 омогућавајући сопствено позиционирање.</translation>
    </message>
    <message>
      <location filename="../../../JointObject.py" line="294"/>
      <source>This is the distance of the joint. It is used only by the distance joint and by RackPinion (pitch radius), Screw and Gears and Belt(radius1)</source>
      <translation>Растојање споја. Користи се код раванског и навојног споја, а такође и код зупчастог, ременог (полупречник) и преносног споја са зупчастом летвом (полупречник корака)</translation>
    </message>
    <message>
      <location filename="../../../JointObject.py" line="305"/>
      <source>This is the second distance of the joint. It is used only by the gear joint to store the second radius.</source>
      <translation>Ово је друго растојање споја. Користи се само код зупчастог споја да одреди други полупречник.</translation>
    </message>
    <message>
      <location filename="../../../JointObject.py" line="316"/>
      <source>This is the rotation of the joint.</source>
      <translation>Ротација споја.</translation>
    </message>
    <message>
      <location filename="../../../JointObject.py" line="327"/>
      <source>This is the offset vector of the joint.</source>
      <translation>Вектор одмака споја.</translation>
    </message>
    <message>
      <location filename="../../../JointObject.py" line="338"/>
      <source>This indicates if the joint is active.</source>
      <translation>Ово показује да ли је спој активан.</translation>
    </message>
    <message>
      <location filename="../../../JointObject.py" line="350"/>
      <source>Enable the minimum length limit of the joint.</source>
      <translation type="unfinished">Enable the minimum length limit of the joint.</translation>
    </message>
    <message>
      <location filename="../../../JointObject.py" line="362"/>
      <source>Enable the maximum length limit of the joint.</source>
      <translation type="unfinished">Enable the maximum length limit of the joint.</translation>
    </message>
    <message>
      <location filename="../../../JointObject.py" line="374"/>
      <source>Enable the minimum angle limit of the joint.</source>
      <translation type="unfinished">Enable the minimum angle limit of the joint.</translation>
    </message>
    <message>
      <location filename="../../../JointObject.py" line="386"/>
      <source>Enable the minimum length of the joint.</source>
      <translation type="unfinished">Enable the minimum length of the joint.</translation>
    </message>
    <message>
      <location filename="../../../JointObject.py" line="398"/>
      <source>This is the minimum limit for the length between both coordinate systems (along their Z axis).</source>
      <translation>Ово је минимално дужинско ограничење између координатних система (уздуж њихових З оса).</translation>
    </message>
    <message>
      <location filename="../../../JointObject.py" line="409"/>
      <source>This is the maximum limit for the length between both coordinate systems (along their Z axis).</source>
      <translation>Ово је максимално дужинско ограничење између координатних система (уздуж њихових З оса).</translation>
    </message>
    <message>
      <location filename="../../../JointObject.py" line="420"/>
      <source>This is the minimum limit for the angle between both coordinate systems (between their X axis).</source>
      <translation>Ово је минимално угаоно ограничење између координатних система (између њихових X оса).</translation>
    </message>
    <message>
      <location filename="../../../JointObject.py" line="431"/>
      <source>This is the maximum limit for the angle between both coordinate systems (between their X axis).</source>
      <translation>Ово је максимално угаоно ограничење између координатних система (између њихових X оса).</translation>
    </message>
    <message>
      <location filename="../../../JointObject.py" line="1021"/>
      <source>The object to ground</source>
      <translation>Објекат који треба учврстити</translation>
    </message>
    <message>
      <location filename="../../../JointObject.py" line="1033"/>
      <source>This is where the part is grounded.</source>
      <translation>Овде је место где је део учвршћен.</translation>
    </message>
    <message>
      <location filename="../../../CommandCreateView.py" line="235"/>
      <source>The object moved by the move</source>
      <translation>Објекти који се померају приликом померања</translation>
    </message>
    <message>
      <location filename="../../../CommandCreateView.py" line="242"/>
      <source>The containing parts of objects moved by the move</source>
      <translation>Делови који садрже објекте који су се померали током померања</translation>
    </message>
    <message>
      <location filename="../../../CommandCreateView.py" line="252"/>
      <source>This is the movement of the move. The end placement is the result of the start placement * this placement.</source>
      <translation>Померање током померања. Крајњи положај је резултат почетног положаја * овог положаја.</translation>
    </message>
    <message>
      <location filename="../../../CommandCreateView.py" line="259"/>
      <source>The type of the move</source>
      <translation>Врста померања</translation>
    </message>
  </context>
  <context>
    <name>TaskAssemblyCreateJoint</name>
    <message>
      <location filename="../panels/TaskAssemblyCreateJoint.ui" line="14"/>
      <source>Create Joint</source>
      <translation>Направи спој</translation>
    </message>
    <message>
      <location filename="../panels/TaskAssemblyCreateJoint.ui" line="41"/>
      <source>Distance</source>
      <translation>Растојање</translation>
    </message>
    <message>
      <location filename="../panels/TaskAssemblyCreateJoint.ui" line="65"/>
      <source>Radius 2</source>
      <translation>Полупречник 2</translation>
    </message>
    <message>
      <location filename="../panels/TaskAssemblyCreateJoint.ui" line="89"/>
      <source>Offset</source>
      <translation>Одмак</translation>
    </message>
    <message>
      <location filename="../panels/TaskAssemblyCreateJoint.ui" line="113"/>
      <source>Rotation</source>
      <translation>Окретање</translation>
    </message>
    <message>
      <location filename="../panels/TaskAssemblyCreateJoint.ui" line="141"/>
      <source>Reverse the direction of the joint.</source>
      <translation>Обрни смер спојаа.</translation>
    </message>
    <message>
      <location filename="../panels/TaskAssemblyCreateJoint.ui" line="144"/>
      <source>Reverse</source>
      <translation>Обрнуто</translation>
    </message>
    <message>
      <location filename="../panels/TaskAssemblyCreateJoint.ui" line="155"/>
      <source>Limits</source>
      <translation>Ограничења</translation>
    </message>
    <message>
      <location filename="../panels/TaskAssemblyCreateJoint.ui" line="161"/>
      <source>Min length</source>
      <translation type="unfinished">Min length</translation>
    </message>
    <message>
      <location filename="../panels/TaskAssemblyCreateJoint.ui" line="184"/>
      <source>Max length</source>
      <translation type="unfinished">Max length</translation>
    </message>
    <message>
      <location filename="../panels/TaskAssemblyCreateJoint.ui" line="207"/>
      <source>Min angle</source>
      <translation type="unfinished">Min angle</translation>
    </message>
    <message>
      <location filename="../panels/TaskAssemblyCreateJoint.ui" line="236"/>
      <source>Max angle</source>
      <translation type="unfinished">Max angle</translation>
    </message>
    <message>
      <location filename="../panels/TaskAssemblyCreateJoint.ui" line="268"/>
      <source>Reverse rotation</source>
      <translation>Обрни ротацију</translation>
    </message>
  </context>
  <context>
    <name>TaskAssemblyInsertLink</name>
    <message>
      <location filename="../panels/TaskAssemblyInsertLink.ui" line="14"/>
      <source>Insert Component</source>
      <translation>Уметни компоненту</translation>
    </message>
    <message>
      <location filename="../panels/TaskAssemblyInsertLink.ui" line="20"/>
      <source>Search parts...</source>
      <translation>Претрага делова...</translation>
    </message>
    <message>
      <location filename="../panels/TaskAssemblyInsertLink.ui" line="32"/>
      <source>Don't find your part? </source>
      <translation>Не проналазиш део? </translation>
    </message>
    <message>
      <location filename="../panels/TaskAssemblyInsertLink.ui" line="39"/>
      <source>Open file</source>
      <translation>Отвори датотеку</translation>
    </message>
    <message>
      <location filename="../panels/TaskAssemblyInsertLink.ui" line="48"/>
      <source>If checked, the list will show only Parts.</source>
      <translation>Ako je čekirano lista će pokazivati samo delove.</translation>
    </message>
    <message>
      <location filename="../panels/TaskAssemblyInsertLink.ui" line="51"/>
      <source>Show only parts</source>
      <translation>Прикажи само делове</translation>
    </message>
  </context>
  <context>
    <name>AssemblyGui::DlgSettingsAssembly</name>
    <message>
      <location filename="../preferences/Assembly.ui" line="14"/>
      <source>General</source>
      <translation>Опште</translation>
    </message>
    <message>
      <location filename="../preferences/Assembly.ui" line="20"/>
      <source>Allow to leave edit mode when pressing Esc button</source>
      <translation>Напушти режим уређивања притиском на типку Есц</translation>
    </message>
    <message>
      <location filename="../preferences/Assembly.ui" line="23"/>
      <source>Esc leave edit mode</source>
      <translation>Есц напусти режим уређивања</translation>
    </message>
    <message>
      <location filename="../preferences/Assembly.ui" line="39"/>
      <source>Log the dragging steps of the solver. Useful if you want to report a bug.
The files are named "runPreDrag.asmt" and "dragging.log" and are located in the default directory of std::ofstream (on Windows it's the desktop)</source>
      <translation>Забележи кораке солвера. Ово је корисно ако желиш да пријавиш грешку.
Датотеке се зову „runPreDrag.asmt“ и „dragging.log“ и налазе се у подразумеваној фасцикли std::ofstream (у Windows-у је то радна површина)</translation>
    </message>
    <message>
      <location filename="../preferences/Assembly.ui" line="43"/>
      <source>Log dragging steps</source>
      <translation>Забележи кораке</translation>
    </message>
    <message>
      <location filename="../preferences/Assembly.ui" line="59"/>
      <source>Ground first part:</source>
      <translation>Учврсти први део:</translation>
    </message>
    <message>
      <location filename="../preferences/Assembly.ui" line="66"/>
      <source>When you insert the first part in the assembly, you can choose to ground the part automatically.</source>
      <translation>Да ли да први део који се умеће у склоп треба да буде учвршћен.</translation>
    </message>
  </context>
  <context>
    <name>AssemblyGui::ViewProviderAssembly</name>
    <message>
      <location filename="../../ViewProviderAssembly.cpp" line="150"/>
      <source>Delete associated joints</source>
      <translation>Обриши придружене спојеве</translation>
    </message>
    <message>
      <location filename="../../ViewProviderAssembly.cpp" line="162"/>
      <source>The object is associated to one or more joints.</source>
      <translation>Објекту су придружени један или више спојева.</translation>
    </message>
    <message>
      <location filename="../../ViewProviderAssembly.cpp" line="164"/>
      <source>Do you want to move the object and delete associated joints?</source>
      <translation>Да ли желиш померити објекат и обрисати придружене спојеве?</translation>
    </message>
    <message>
      <location filename="../../ViewProviderAssembly.cpp" line="780"/>
      <source>Move part</source>
      <translation>Помеи део</translation>
    </message>
  </context>
  <context>
    <name>Assembly_CreateJointRackPinion</name>
    <message>
      <location filename="../../../CommandCreateJoint.py" line="331"/>
      <source>Create Rack and Pinion Joint</source>
      <translation>Направи пренос са зупачастом летвом</translation>
    </message>
    <message>
      <location filename="../../../CommandCreateJoint.py" line="338"/>
      <source>Create a Rack and Pinion Joint: Links a part with a sliding joint with a part with a revolute joint.</source>
      <translation>Направи пренос са зупачастом летвом: Комбинација транслаторног и ротационог споја.</translation>
    </message>
    <message>
      <location filename="../../../CommandCreateJoint.py" line="343"/>
      <source>Select the same coordinate systems as the revolute and sliding joints. The pitch radius defines the movement ratio between the rack and the pinion.</source>
      <translation>Изабери исте координатне системе као код ротационих или транслаторних спојева. Полупречник корака одређује однос кретања између зупчаника и летве.</translation>
    </message>
  </context>
  <context>
    <name>Assembly_CreateJointScrew</name>
    <message>
      <location filename="../../../CommandCreateJoint.py" line="362"/>
      <source>Create Screw Joint</source>
      <translation>Направи навојни спој</translation>
    </message>
    <message>
      <location filename="../../../CommandCreateJoint.py" line="369"/>
      <source>Create a Screw Joint: Links a part with a sliding joint with a part with a revolute joint.</source>
      <translation>Направи навојни спој: Омогућава транслацију и ротацију једног дела унутар другог.</translation>
    </message>
    <message>
      <location filename="../../../CommandCreateJoint.py" line="374"/>
      <source>Select the same coordinate systems as the revolute and sliding joints. The pitch radius defines the movement ratio between the rotating screw and the sliding part.</source>
      <translation>Изабери исте координатне системе као код ротационих или транслаторних спојева. Полупречник корака одређује однос кретања између завртња који се ротира и дела који клизи.</translation>
    </message>
    <message>
      <location filename="../../../CommandCreateJoint.py" line="405"/>
      <location filename="../../../CommandCreateJoint.py" line="436"/>
      <source>Select the same coordinate systems as the revolute joints.</source>
      <translation>Изабери исте координатне системе као код ротационих спојева.</translation>
    </message>
  </context>
  <context>
    <name>Assembly_CreateJointGears</name>
    <message>
      <location filename="../../../CommandCreateJoint.py" line="393"/>
      <source>Create Gears Joint</source>
      <translation>Направи зупчасти пренос</translation>
    </message>
    <message>
      <location filename="../../../CommandCreateJoint.py" line="400"/>
      <source>Create a Gears Joint: Links two rotating gears together. They will have inverse rotation direction.</source>
      <translation>Направи зупчасти пренос: Спаја заједно два зупчаника који се ротирају. Зупчаници ће имати супротне смерове обртања.</translation>
    </message>
  </context>
  <context>
    <name>Assembly_CreateJointBelt</name>
    <message>
      <location filename="../../../CommandCreateJoint.py" line="424"/>
      <source>Create Belt Joint</source>
      <translation>Направи ремени пренос</translation>
    </message>
    <message>
      <location filename="../../../CommandCreateJoint.py" line="431"/>
      <source>Create a Belt Joint: Links two rotating objects together. They will have the same rotation direction.</source>
      <translation>Направи ремени пренос: Спаја заједно две ременице које се ротирају. Ременице ће имати исте смерове обртања.</translation>
    </message>
  </context>
  <context>
    <name>Assembly_CreateJointGearBelt</name>
    <message>
      <location filename="../../../CommandCreateJoint.py" line="456"/>
      <source>Create Gear/Belt Joint</source>
      <translation>Направи зупчасти/ремени пренос</translation>
    </message>
    <message>
      <location filename="../../../CommandCreateJoint.py" line="462"/>
      <source>Create a Gears/Belt Joint: Links two rotating gears together.</source>
      <translation>Направи зупачасти/ремени пренос: Споји заједно два зупчаника/ременице.</translation>
    </message>
    <message>
      <location filename="../../../CommandCreateJoint.py" line="467"/>
      <source>Select the same coordinate systems as the revolute joints.</source>
      <translation>Изабери исте координатне системе као код ротационих спојева.</translation>
    </message>
  </context>
  <context>
    <name>Assembly_CreateView</name>
    <message>
      <location filename="../../../CommandCreateView.py" line="55"/>
      <source>Create Exploded View</source>
      <translation>Направи растављени поглед</translation>
    </message>
    <message>
      <location filename="../../../CommandCreateView.py" line="62"/>
      <source>Create an exploded view of the current assembly.</source>
      <translation>Направи растављени поглед склопа.</translation>
    </message>
  </context>
  <context>
    <name>TaskAssemblyCreateView</name>
    <message>
      <location filename="../panels/TaskAssemblyCreateView.ui" line="14"/>
      <source>Create Exploded View</source>
      <translation>Направи растављени поглед</translation>
    </message>
    <message>
      <location filename="../panels/TaskAssemblyCreateView.ui" line="20"/>
      <source>If checked, Parts will be selected as a single solid.</source>
      <translation>Ако је чекирано, делови ће бити изабрани као једно тело.</translation>
    </message>
    <message>
      <location filename="../panels/TaskAssemblyCreateView.ui" line="23"/>
      <source>Parts as single solid</source>
      <translation>Део као пуно тело</translation>
    </message>
    <message>
      <location filename="../panels/TaskAssemblyCreateView.ui" line="42"/>
      <source>Align dragger</source>
      <translation>Поравнај вучни триедар</translation>
    </message>
    <message>
      <location filename="../panels/TaskAssemblyCreateView.ui" line="49"/>
      <source>Aligning dragger:
Select a feature.
Press ESC to cancel.</source>
      <translation>Вучни триедар за поравнавање:
Изабери.
Притисни ESC да откажеш.</translation>
    </message>
    <message>
      <location filename="../panels/TaskAssemblyCreateView.ui" line="58"/>
      <source>Explode radially</source>
      <translation>Растави радијално</translation>
    </message>
    <message>
      <location filename="../panels/TaskAssemblyCreateBom.ui" line="14"/>
      <source>Create Bill Of Materials</source>
      <translation>Направи саставницу</translation>
    </message>
    <message>
      <location filename="../panels/TaskAssemblyCreateBom.ui" line="20"/>
      <source>If checked, Sub assemblies children will be added to the bill of materials.</source>
      <translation type="unfinished">If checked, Sub assemblies children will be added to the bill of materials.</translation>
    </message>
    <message>
      <location filename="../panels/TaskAssemblyCreateBom.ui" line="23"/>
      <source>Sub-assemblies children</source>
      <translation type="unfinished">Sub-assemblies children</translation>
    </message>
    <message>
      <location filename="../panels/TaskAssemblyCreateBom.ui" line="39"/>
      <source>If checked, Parts children will be added to the bill of materials.</source>
      <translation type="unfinished">If checked, Parts children will be added to the bill of materials.</translation>
    </message>
    <message>
      <location filename="../panels/TaskAssemblyCreateBom.ui" line="42"/>
      <source>Parts children</source>
      <translation type="unfinished">Parts children</translation>
    </message>
    <message>
      <location filename="../panels/TaskAssemblyCreateBom.ui" line="58"/>
      <source>If checked, only Part containers and sub-assemblies will be added to the bill of materials. Solids like PartDesign Bodies, fasteners or Part workbench primitives will be ignored.</source>
      <translation type="unfinished">If checked, only Part containers and sub-assemblies will be added to the bill of materials. Solids like PartDesign Bodies, fasteners or Part workbench primitives will be ignored.</translation>
    </message>
    <message>
      <location filename="../panels/TaskAssemblyCreateBom.ui" line="61"/>
      <source>Only parts</source>
      <translation>Само контејнери Део</translation>
    </message>
    <message>
      <location filename="../panels/TaskAssemblyCreateBom.ui" line="77"/>
      <source>Columns</source>
      <translation>Колоне</translation>
    </message>
    <message>
      <location filename="../panels/TaskAssemblyCreateBom.ui" line="86"/>
      <source>Add column</source>
      <translation>Додај колону</translation>
    </message>
    <message>
      <location filename="../panels/TaskAssemblyCreateBom.ui" line="96"/>
      <source>Export</source>
      <translation>Извези</translation>
    </message>
    <message>
      <location filename="../panels/TaskAssemblyCreateBom.ui" line="109"/>
      <source>Help</source>
      <translation>Помоћ</translation>
    </message>
  </context>
  <context>
    <name>Assembly_CreateJointParallel</name>
    <message>
      <location filename="../../../CommandCreateJoint.py" line="249"/>
      <source>Create Parallel Joint</source>
      <translation>Направи паралелни спој</translation>
    </message>
    <message>
      <location filename="../../../CommandCreateJoint.py" line="256"/>
      <source>Create an Parallel Joint: Make the Z axis of selected coordinate systems parallel.</source>
      <translation>Направи паралелни спој: Направи З осе координатних система изабраних делова паралелним.</translation>
    </message>
  </context>
  <context>
    <name>Assembly_CreateJointPerpendicular</name>
    <message>
      <location filename="../../../CommandCreateJoint.py" line="277"/>
      <source>Create Perpendicular Joint</source>
      <translation>Направи управни спој</translation>
    </message>
    <message>
      <location filename="../../../CommandCreateJoint.py" line="284"/>
      <source>Create an Perpendicular Joint: Make the Z axis of selected coordinate systems perpendicular.</source>
      <translation>Направи управни спој: Направи З осе координатних система изабраних делова управним.</translation>
    </message>
  </context>
  <context>
    <name>Assembly_CreateJointAngle</name>
    <message>
      <location filename="../../../CommandCreateJoint.py" line="303"/>
      <source>Create Angle Joint</source>
      <translation>Направи угаони спој</translation>
    </message>
    <message>
      <location filename="../../../CommandCreateJoint.py" line="310"/>
      <source>Create an Angle Joint: Fix the angle between the Z axis of selected coordinate systems.</source>
      <translation>Направи угаони спој: Фиксира задани угао између З оса координатних система изабраних делова.</translation>
    </message>
  </context>
  <context>
    <name>Assembly_CreateBom</name>
    <message>
      <location filename="../../../CommandCreateBom.py" line="69"/>
      <source>Create Bill of Materials</source>
      <translation>Направи саставницу</translation>
    </message>
    <message>
      <location filename="../../../CommandCreateBom.py" line="76"/>
      <source>Create a bill of materials of the current assembly. If an assembly is active, it will be a BOM of this assembly. Else it will be a BOM of the whole document.</source>
      <translation>Направи саставницу склопа. Ако је неки склоп активан направиће се његова саставница. У супротном, направиће се саставница целог документа.</translation>
    </message>
    <message>
      <location filename="../../../CommandCreateBom.py" line="81"/>
      <source>The BOM object is a document object that stores the settings of your BOM. It is also a spreadsheet object so you can easily visualize the bom. If you don't need the BOM object to be saved as a document object, you can simply export and cancel the task.</source>
      <translation>Пошто је саставница објекат окружења Табеле у њему ју можете и прегледати. У објекту Саставница су сачувана и подешавања саставнице. Ако немате потребу да у Стаблу документа чувате саставницу, извезите ју и откажите задатак.</translation>
    </message>
    <message>
      <location filename="../../../CommandCreateBom.py" line="86"/>
      <source>The columns 'Index', 'Name', 'File Name' and 'Quantity' are automatically generated on recompute. The 'Description' and custom columns are not overwritten.</source>
      <translation type="unfinished">The columns 'Index', 'Name', 'File Name' and 'Quantity' are automatically generated on recompute. The 'Description' and custom columns are not overwritten.</translation>
    </message>
  </context>
</TS>
