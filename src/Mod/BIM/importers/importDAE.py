#***************************************************************************
#*   Copyright (c) 2011 Yorik van Havre <yorik@uncreated.net>              *
#*                                                                         *
#*   This program is free software; you can redistribute it and/or modify  *
#*   it under the terms of the GNU Lesser General Public License (LGPL)    *
#*   as published by the Free Software Foundation; either version 2 of     *
#*   the License, or (at your option) any later version.                   *
#*   for detail see the LICENCE text file.                                 *
#*                                                                         *
#*   This program is distributed in the hope that it will be useful,       *
#*   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
#*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
#*   GNU Library General Public License for more details.                  *
#*                                                                         *
#*   You should have received a copy of the GNU Library General Public     *
#*   License along with this program; if not, write to the Free Software   *
#*   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  *
#*   USA                                                                   *
#*                                                                         *
#***************************************************************************

import FreeCAD, Mesh, os, numpy, MeshPart, Arch, Draft
from draftutils import params

if FreeCAD.GuiUp:
    from draftutils.translate import translate
else:
    # \cond
    def translate(context,text):
        return text
    # \endcond

## @package importDAE
#  \ingroup ARCH
#  \brief DAE (Collada) file format importer and exporter
#
#  This module provides tools to import and export Collada (.dae) files.

__title__  = "FreeCAD Collada importer"
__author__ = "Yorik van Havre"
__url__    = "https://www.freecad.org"

DEBUG = True

try:
    # Python 2 forward compatibility
    range = xrange
except NameError:
    pass

def checkCollada():

    "checks if collada if available"

    global collada
    COLLADA = None
    try:
        import collada
    except ImportError:
        FreeCAD.Console.PrintError(translate('BIM', 'pycollada not found, collada support is disabled.') + '\n')
        return False
    else:
        return True


def triangulate(shape):

    "triangulates the given face"

    mesher = params.get_param_arch("ColladaMesher")
    tessellation = params.get_param_arch("ColladaTessellation")
    grading = params.get_param_arch("ColladaGrading")
    segsperedge = params.get_param_arch("ColladaSegsPerEdge")
    segsperradius = params.get_param_arch("ColladaSegsPerRadius")
    secondorder = params.get_param_arch("ColladaSecondOrder")
    optimize = params.get_param_arch("ColladaOptimize")
    allowquads = params.get_param_arch("ColladaAllowQuads")
    if mesher == 0:
        return shape.tessellate(tessellation)
    elif mesher == 1:
        return MeshPart.meshFromShape(Shape=shape,MaxLength=tessellation).Topology
    else:
        return MeshPart.meshFromShape(Shape=shape,GrowthRate=grading,SegPerEdge=segsperedge,
               SegPerRadius=segsperradius,SecondOrder=secondorder,Optimize=optimize,
               AllowQuad=allowquads).Topology


def open(filename):

    "called when freecad wants to open a file"

    if not checkCollada():
        return
    docname = os.path.splitext(os.path.basename(filename))[0]
    doc = FreeCAD.newDocument(docname)
    doc.Label = docname
    FreeCAD.ActiveDocument = doc
    read(filename)
    return doc


def insert(filename,docname):

    "called when freecad wants to import a file"

    if not checkCollada():
        return
    try:
        doc = FreeCAD.getDocument(docname)
    except NameError:
        doc = FreeCAD.newDocument(docname)
    FreeCAD.ActiveDocument = doc
    read(filename)
    return doc


def read(filename):

    "reads a DAE file"

    global col
    col = collada.Collada(filename, ignore=[collada.common.DaeUnsupportedError])
    # Read the unitmeter info from dae file and compute unit to convert to mm
    unitmeter = col.assetInfo.unitmeter or 1
    unit = unitmeter / 0.001
    #for geom in col.geometries:
    #for geom in col.scene.objects('geometry'):
    for node in col.scene.nodes:
        for child in node.children:
            if not isinstance(child, collada.scene.GeometryNode):
                continue
            geom: collada.scenes.GeometryNode = child.geometry
            mat_symbols: list[str] = [m.symbol for m in child.materials]
            for prim in geom.primitives:
                meshdata = []
                for tri in prim:
                    # tri.vertices is a numpy array.
                    meshdata.append((tri.vertices * unit).tolist())
                mesh = Mesh.Mesh(meshdata)
                try:
                    name = geom.name
                except AttributeError:
                    name = geom.id
                obj = FreeCAD.ActiveDocument.addObject('Mesh::Feature', name)
                obj.Label = name
                obj.Mesh = mesh
                if FreeCAD.GuiUp:
                    try:
                        mat_index = mat_symbols.index(prim.material)
                        material = child.materials[mat_index].target
                        color = material.effect.diffuse
                        obj.ViewObject.ShapeColor = color
                    except ValueError:
                        # Material not found.
                        pass
                    except TypeError:
                        # color is not a tuple but a texture.
                        pass

    # Print the errors that occurred during reading.
    if col.errors:
        FreeCAD.Console.PrintWarning(translate('BIM', 'File was read but some errors occured:') + '\n')
    for e in col.errors:
        FreeCAD.Console.PrintWarning(str(e) + '\n')


def export(exportList,filename,tessellation=1,colors=None):

    """export(exportList,filename,tessellation=1,colors=None) -- exports FreeCAD contents to a DAE file.
    colors is an optional dictionary of objName:shapeColorTuple or objName:diffuseColorList elements
    to be used in non-GUI mode if you want to be able to export colors. Tessellation is used when breaking
    curved surfaces into triangles."""

    if not checkCollada(): return
    scale = params.get_param_arch("ColladaScalingFactor")
    scale = scale * 0.001 # from millimeters (FreeCAD) to meters (Collada)
    defaultcolor = Draft.get_rgba_tuple(params.get_param_view("DefaultShapeColor"))[:3]
    colmesh = collada.Collada()
    colmesh.assetInfo.upaxis = collada.asset.UP_AXIS.Z_UP
    # authoring info
    cont = collada.asset.Contributor()
    try:
        author = FreeCAD.ActiveDocument.CreatedBy
    except UnicodeEncodeError:
        author = FreeCAD.ActiveDocument.CreatedBy.encode("utf8")
    author = author.replace("<","")
    author = author.replace(">","")
    cont.author = author
    ver = FreeCAD.Version()
    appli = "FreeCAD v" + ver[0] + "." + ver[1] + " build" + ver[2] + "\n"
    cont.authoring_tool = appli
    #print(author, appli)
    colmesh.assetInfo.contributors.append(cont)
    colmesh.assetInfo.unitname = "meter"
    colmesh.assetInfo.unitmeter = 1.0
    defaultmat = None
    objind = 0
    scenenodes = []
    objectslist = Draft.get_group_contents(exportList, walls=True,
                                           addgroups=True)
    objectslist = Arch.pruneIncluded(objectslist, strict=True)
    for obj in objectslist:
        findex = numpy.array([])
        m = None
        if obj.isDerivedFrom("Part::Feature"):
            print("exporting object ",obj.Name, obj.Shape)
            new_shape = obj.Shape.copy()
            new_shape.Placement = obj.getGlobalPlacement()
            m = Mesh.Mesh(triangulate(new_shape))
        elif obj.isDerivedFrom("Mesh::Feature"):
            print("exporting object ",obj.Name, obj.Mesh)
            m = obj.Mesh
        elif obj.isDerivedFrom("App::Part"):
            for child in obj.OutList:
                objectslist.append(child)
            continue
        else:
            continue
        if m:
            Topology = m.Topology
            Facets = m.Facets

            # vertex indices
            vindex = numpy.empty(len(Topology[0]) * 3)
            for i in range(len(Topology[0])):
                v = Topology[0][i]
                vindex[list(range(i*3, i*3+3))] = (v.x*scale,v.y*scale,v.z*scale)

            # normals
            nindex = numpy.empty(len(Facets) * 3)
            for i in range(len(Facets)):
                n = Facets[i].Normal
                nindex[list(range(i*3, i*3+3))] = (n.x,n.y,n.z)

            # face indices
            findex = numpy.empty(len(Topology[1]) * 6, numpy.int64)
            for i in range(len(Topology[1])):
                f = Topology[1][i]
                findex[list(range(i*6, i*6+6))] = (f[0],i,f[1],i,f[2],i)

        print(len(vindex), " vert indices, ", len(nindex), " norm indices, ", len(findex), " face indices.")
        vert_src = collada.source.FloatSource("cubeverts-array"+str(objind), vindex, ('X', 'Y', 'Z'))
        normal_src = collada.source.FloatSource("cubenormals-array"+str(objind), nindex, ('X', 'Y', 'Z'))
        geom = collada.geometry.Geometry(colmesh, "geometry"+str(objind), obj.Name, [vert_src, normal_src])
        input_list = collada.source.InputList()
        input_list.addInput(0, 'VERTEX', "#cubeverts-array"+str(objind))
        input_list.addInput(1, 'NORMAL', "#cubenormals-array"+str(objind))
        matnode = None
        matref = "materialref"
        if hasattr(obj,"Material"):
            if obj.Material:
                if hasattr(obj.Material,"Material"):
                    if "DiffuseColor" in obj.Material.Material:
                        kd = tuple([float(k) for k in obj.Material.Material["DiffuseColor"].strip("()").split(",")])
                        effect = collada.material.Effect("effect_"+obj.Material.Name, [], "phong", diffuse=kd, specular=(1,1,1))
                        mat = collada.material.Material("mat_"+obj.Material.Name, obj.Material.Name, effect)
                        colmesh.effects.append(effect)
                        colmesh.materials.append(mat)
                        matref = "ref_"+obj.Material.Name
                        matnode = collada.scene.MaterialNode(matref, mat, inputs=[])
        if not matnode:
            if colors:
                if obj.Name in colors:
                    color = colors[obj.Name]
                    if color:
                        if isinstance(color[0],tuple):
                            # this is a diffusecolor. For now, use the first color - #TODO: Support per-face colors
                            color = color[0]
                        #print("found color for obj",obj.Name,":",color)
                        kd = color[:3]
                        effect = collada.material.Effect("effect_"+obj.Name, [], "phong", diffuse=kd, specular=(1,1,1))
                        mat = collada.material.Material("mat_"+obj.Name, obj.Name, effect)
                        colmesh.effects.append(effect)
                        colmesh.materials.append(mat)
                        matref = "ref_"+obj.Name
                        matnode = collada.scene.MaterialNode(matref, mat, inputs=[])
            elif FreeCAD.GuiUp:
                if hasattr(obj.ViewObject,"ShapeColor"):
                    kd = obj.ViewObject.ShapeColor[:3]
                    effect = collada.material.Effect("effect_"+obj.Name, [], "phong", diffuse=kd, specular=(1,1,1))
                    mat = collada.material.Material("mat_"+obj.Name, obj.Name, effect)
                    colmesh.effects.append(effect)
                    colmesh.materials.append(mat)
                    matref = "ref_"+obj.Name
                    matnode = collada.scene.MaterialNode(matref, mat, inputs=[])
        if not matnode:
            if not defaultmat:
                effect = collada.material.Effect("effect_default", [], "phong", diffuse=defaultcolor, specular=(1,1,1))
                defaultmat = collada.material.Material("mat_default", "default_material", effect)
                colmesh.effects.append(effect)
                colmesh.materials.append(defaultmat)
            matnode = collada.scene.MaterialNode(matref, defaultmat, inputs=[])
        triset = geom.createTriangleSet(findex, input_list, matref)
        geom.primitives.append(triset)
        colmesh.geometries.append(geom)
        geomnode = collada.scene.GeometryNode(geom, [matnode])
        node = collada.scene.Node("node"+str(objind), children=[geomnode])
        scenenodes.append(node)
        objind += 1
    myscene = collada.scene.Scene("myscene", scenenodes)
    colmesh.scenes.append(myscene)
    colmesh.scene = myscene
    colmesh.write(filename)
    FreeCAD.Console.PrintMessage(translate("BIM", f'file "{filename}" successfully created.' + "\n"))
