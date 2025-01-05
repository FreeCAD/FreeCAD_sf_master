# -*- coding: utf-8 -*-
# ***************************************************************************
# *   Copyright (c) 2018 Kresimir Tusek <kresimir.tusek@gmail.com>          *
# *   Copyright (c) 2019-2021 Schildkroet                                   *
# *                                                                         *
# *   This file is part of the FreeCAD CAx development system.              *
# *                                                                         *
# *   This library is free software; you can redistribute it and/or         *
# *   modify it under the terms of the GNU Library General Public           *
# *   License as published by the Free Software Foundation; either          *
# *   version 2 of the License, or (at your option) any later version.      *
# *                                                                         *
# *   This library  is distributed in the hope that it will be useful,      *
# *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
# *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
# *   GNU Library General Public License for more details.                  *
# *                                                                         *
# *   You should have received a copy of the GNU Library General Public     *
# *   License along with this library; see the file COPYING.LIB. If not,    *
# *   write to the Free Software Foundation, Inc., 59 Temple Place,         *
# *   Suite 330, Boston, MA  02111-1307, USA                                *
# *                                                                         *
# ***************************************************************************

import Path
import Path.Op.Base as PathOp
import PathScripts.PathUtils as PathUtils
import FreeCAD
import time
import json
import math
import area
from functools import cmp_to_key
from PySide.QtCore import QT_TRANSLATE_NOOP

if FreeCAD.GuiUp:
    from pivy import coin
    import FreeCADGui

__doc__ = "Class and implementation of the Adaptive CAM operation."

# lazily loaded modules
from lazy_loader.lazy_loader import LazyLoader

Part = LazyLoader("Part", globals(), "Part")
TechDraw = LazyLoader("TechDraw", globals(), "TechDraw")
FeatureExtensions = LazyLoader("Path.Op.FeatureExtension", globals(), "Path.Op.FeatureExtension")
DraftGeomUtils = LazyLoader("DraftGeomUtils", globals(), "DraftGeomUtils")


if False:
    Path.Log.setLevel(Path.Log.Level.DEBUG, Path.Log.thisModule())
    Path.Log.trackModule(Path.Log.thisModule())
else:
    Path.Log.setLevel(Path.Log.Level.INFO, Path.Log.thisModule())


translate = FreeCAD.Qt.translate


def convertTo2d(pathArray):
    output = []
    for path in pathArray:
        pth2 = []
        for edge in path:
            for pt in edge:
                pth2.append([pt[0], pt[1]])
        output.append(pth2)
    return output


sceneGraph = None
scenePathNodes = []  # for scene cleanup afterwards
topZ = 10

# Constants to avoid magic numbers in the code
_ADAPTIVE_MIN_STEPDOWN = 0.1


def sceneDrawPath(path, color=(0, 0, 1)):
    coPoint = coin.SoCoordinate3()

    pts = []
    for pt in path:
        pts.append([pt[0], pt[1], topZ])

    coPoint.point.setValues(0, len(pts), pts)
    ma = coin.SoBaseColor()
    ma.rgb = color
    li = coin.SoLineSet()
    li.numVertices.setValue(len(pts))
    pathNode = coin.SoSeparator()
    pathNode.addChild(coPoint)
    pathNode.addChild(ma)
    pathNode.addChild(li)
    sceneGraph.addChild(pathNode)
    scenePathNodes.append(pathNode)  # for scene cleanup afterwards


def sceneClean():
    for n in scenePathNodes:
        sceneGraph.removeChild(n)

    del scenePathNodes[:]


def discretize(edge, flipDirection=False):
    pts = edge.discretize(Deflection=0.002)
    if flipDirection:
        pts.reverse()

    return pts


def CalcHelixConePoint(height, cur_z, radius, angle):
    x = ((height - cur_z) / height) * radius * math.cos(math.radians(angle) * cur_z)
    y = ((height - cur_z) / height) * radius * math.sin(math.radians(angle) * cur_z)
    z = cur_z

    return {"X": x, "Y": y, "Z": z}


def GenerateGCode(op, obj, adaptiveResults, helixDiameter):
    if not adaptiveResults or not adaptiveResults[0]["AdaptivePaths"]:
        return

    # minLiftDistance = op.tool.Diameter
    helixRadius = 0
    for region in adaptiveResults:
        p1 = region["HelixCenterPoint"]
        p2 = region["StartPoint"]
        helixRadius = max(math.dist(p1[:2], p2[:2]), helixRadius)

    stepDown = max(obj.StepDown.Value, _ADAPTIVE_MIN_STEPDOWN)

    length = 2 * math.pi * helixRadius

    obj.HelixAngle = min(89, max(obj.HelixAngle.Value, 1))
    obj.HelixConeAngle = max(obj.HelixConeAngle, 0)

    helixAngleRad = math.radians(obj.HelixAngle)
    depthPerOneCircle = length * math.tan(helixAngleRad)

    stepUp = max(obj.LiftDistance.Value, 0)

    # TODO: finish_step is of limited utility with how regions are now broken
    # up based on the model geometry- the "finish" step gets applied to each
    # region separately, which results in excessive "finish" steps being taken
    # where they really need not be. Leaving stock in Z generally makes more
    # sense, but both technically have their uses, so leaving this here as
    # option. Implementing flat area detection would make better use of both.
    finish_step = min(obj.FinishDepth.Value, stepDown) if hasattr(obj, "FinishDepth") else 0.0

    # Track Z position to determine when changing height is necessary prior to a move
    lz = obj.StartDepth.Value

    for region in adaptiveResults:
        passStartDepth = region["TopDepth"]

        depth_params = PathUtils.depth_params(
            clearance_height=obj.ClearanceHeight.Value,
            safe_height=obj.SafeHeight.Value,
            start_depth=region["TopDepth"],
            step_down=stepDown,
            z_finish_step=finish_step,
            final_depth=region["BottomDepth"],
            user_depths=None,
        )

        for passEndDepth in depth_params.data:
            startAngle = math.atan2(
                region["StartPoint"][1] - region["HelixCenterPoint"][1],
                region["StartPoint"][0] - region["HelixCenterPoint"][0],
            )

            passDepth = passStartDepth - passEndDepth

            p1 = region["HelixCenterPoint"]
            p2 = region["StartPoint"]
            helixRadius = math.dist(p1[:2], p2[:2])

            # Helix ramp
            if helixRadius > 0.01:
                r = helixRadius - 0.01

                maxfi = passDepth / depthPerOneCircle * 2 * math.pi
                fi = 0
                offsetFi = -maxfi + startAngle - math.pi / 16

                helixStart = [
                    region["HelixCenterPoint"][0] + r * math.cos(offsetFi),
                    region["HelixCenterPoint"][1] + r * math.sin(offsetFi),
                ]

                op.commandlist.append(Path.Command("(Helix to depth: %f)" % passEndDepth))

                if obj.UseHelixArcs is False:
                    # rapid move to start point
                    op.commandlist.append(Path.Command("G0", {"Z": obj.ClearanceHeight.Value}))
                    op.commandlist.append(
                        Path.Command(
                            "G0",
                            {
                                "X": helixStart[0],
                                "Y": helixStart[1],
                                "Z": obj.ClearanceHeight.Value,
                            },
                        )
                    )

                    # rapid move to safe height
                    op.commandlist.append(
                        Path.Command(
                            "G0",
                            {
                                "X": helixStart[0],
                                "Y": helixStart[1],
                                "Z": obj.SafeHeight.Value,
                            },
                        )
                    )

                    # move to start depth
                    op.commandlist.append(
                        Path.Command(
                            "G1",
                            {
                                "X": helixStart[0],
                                "Y": helixStart[1],
                                "Z": passStartDepth,
                                "F": op.vertFeed,
                            },
                        )
                    )

                    if obj.HelixConeAngle == 0:
                        while fi < maxfi:
                            x = region["HelixCenterPoint"][0] + r * math.cos(fi + offsetFi)
                            y = region["HelixCenterPoint"][1] + r * math.sin(fi + offsetFi)
                            z = passStartDepth - fi / maxfi * (passStartDepth - passEndDepth)
                            op.commandlist.append(
                                Path.Command("G1", {"X": x, "Y": y, "Z": z, "F": op.vertFeed})
                            )
                            fi = fi + math.pi / 16

                        # one more circle at target depth to make sure center is cleared
                        maxfi = maxfi + 2 * math.pi
                        while fi < maxfi:
                            x = region["HelixCenterPoint"][0] + r * math.cos(fi + offsetFi)
                            y = region["HelixCenterPoint"][1] + r * math.sin(fi + offsetFi)
                            z = passEndDepth
                            op.commandlist.append(
                                Path.Command("G1", {"X": x, "Y": y, "Z": z, "F": op.horizFeed})
                            )
                            fi = fi + math.pi / 16

                    else:
                        # Cone
                        _HelixAngle = 360 - (obj.HelixAngle.Value * 4)

                        if obj.HelixConeAngle > 6:
                            obj.HelixConeAngle = 6

                        helixRadius *= 0.9

                        # Calculate everything
                        helix_height = passStartDepth - passEndDepth
                        r_extra = helix_height * math.tan(math.radians(obj.HelixConeAngle))
                        HelixTopRadius = helixRadius + r_extra
                        helix_full_height = HelixTopRadius * (
                            math.cos(math.radians(obj.HelixConeAngle))
                            / math.sin(math.radians(obj.HelixConeAngle))
                        )

                        # Start height
                        z = passStartDepth
                        i = 0

                        # Default step down
                        z_step = 0.05

                        # Bigger angle, smaller step down
                        if _HelixAngle > 120:
                            z_step = 0.025
                        if _HelixAngle > 240:
                            z_step = 0.015

                        p = None
                        # Calculate conical helix
                        while z >= passEndDepth:
                            if z < passEndDepth:
                                z = passEndDepth

                            p = CalcHelixConePoint(
                                helix_full_height, i, HelixTopRadius, _HelixAngle
                            )
                            op.commandlist.append(
                                Path.Command(
                                    "G1",
                                    {
                                        "X": p["X"] + region["HelixCenterPoint"][0],
                                        "Y": p["Y"] + region["HelixCenterPoint"][1],
                                        "Z": z,
                                        "F": op.vertFeed,
                                    },
                                )
                            )
                            z = z - z_step
                            i = i + z_step

                        # Calculate some stuff for arcs at bottom
                        p["X"] = p["X"] + region["HelixCenterPoint"][0]
                        p["Y"] = p["Y"] + region["HelixCenterPoint"][1]
                        x_m = region["HelixCenterPoint"][0] - p["X"] + region["HelixCenterPoint"][0]
                        y_m = region["HelixCenterPoint"][1] - p["Y"] + region["HelixCenterPoint"][1]
                        i_off = (x_m - p["X"]) / 2
                        j_off = (y_m - p["Y"]) / 2

                        # One more circle at target depth to make sure center is cleared
                        op.commandlist.append(
                            Path.Command(
                                "G3",
                                {
                                    "X": x_m,
                                    "Y": y_m,
                                    "Z": passEndDepth,
                                    "I": i_off,
                                    "J": j_off,
                                    "F": op.horizFeed,
                                },
                            )
                        )
                        op.commandlist.append(
                            Path.Command(
                                "G3",
                                {
                                    "X": p["X"],
                                    "Y": p["Y"],
                                    "Z": passEndDepth,
                                    "I": -i_off,
                                    "J": -j_off,
                                    "F": op.horizFeed,
                                },
                            )
                        )

                else:
                    # Use arcs for helix - no conical shape support
                    helixStart = [
                        region["HelixCenterPoint"][0] + r,
                        region["HelixCenterPoint"][1],
                    ]

                    # rapid move to start point
                    op.commandlist.append(Path.Command("G0", {"Z": obj.ClearanceHeight.Value}))
                    op.commandlist.append(
                        Path.Command(
                            "G0",
                            {
                                "X": helixStart[0],
                                "Y": helixStart[1],
                                "Z": obj.ClearanceHeight.Value,
                            },
                        )
                    )

                    # rapid move to safe height
                    op.commandlist.append(
                        Path.Command(
                            "G0",
                            {
                                "X": helixStart[0],
                                "Y": helixStart[1],
                                "Z": obj.SafeHeight.Value,
                            },
                        )
                    )

                    # move to start depth
                    op.commandlist.append(
                        Path.Command(
                            "G1",
                            {
                                "X": helixStart[0],
                                "Y": helixStart[1],
                                "Z": passStartDepth,
                                "F": op.vertFeed,
                            },
                        )
                    )

                    x = region["HelixCenterPoint"][0] + r
                    y = region["HelixCenterPoint"][1]

                    curDep = passStartDepth
                    while curDep > (passEndDepth + depthPerOneCircle):
                        op.commandlist.append(
                            Path.Command(
                                "G2",
                                {
                                    "X": x - (2 * r),
                                    "Y": y,
                                    "Z": curDep - (depthPerOneCircle / 2),
                                    "I": -r,
                                    "F": op.vertFeed,
                                },
                            )
                        )
                        op.commandlist.append(
                            Path.Command(
                                "G2",
                                {
                                    "X": x,
                                    "Y": y,
                                    "Z": curDep - depthPerOneCircle,
                                    "I": r,
                                    "F": op.vertFeed,
                                },
                            )
                        )
                        curDep = curDep - depthPerOneCircle

                    lastStep = curDep - passEndDepth
                    if lastStep > (depthPerOneCircle / 2):
                        op.commandlist.append(
                            Path.Command(
                                "G2",
                                {
                                    "X": x - (2 * r),
                                    "Y": y,
                                    "Z": curDep - (lastStep / 2),
                                    "I": -r,
                                    "F": op.vertFeed,
                                },
                            )
                        )
                        op.commandlist.append(
                            Path.Command(
                                "G2",
                                {
                                    "X": x,
                                    "Y": y,
                                    "Z": passEndDepth,
                                    "I": r,
                                    "F": op.vertFeed,
                                },
                            )
                        )
                    else:
                        op.commandlist.append(
                            Path.Command(
                                "G2",
                                {
                                    "X": x - (2 * r),
                                    "Y": y,
                                    "Z": passEndDepth,
                                    "I": -r,
                                    "F": op.vertFeed,
                                },
                            )
                        )
                        op.commandlist.append(
                            Path.Command(
                                "G1",
                                {"X": x, "Y": y, "Z": passEndDepth, "F": op.vertFeed},
                            )
                        )

                    # one more circle at target depth to make sure center is cleared
                    op.commandlist.append(
                        Path.Command(
                            "G2",
                            {
                                "X": x - (2 * r),
                                "Y": y,
                                "Z": passEndDepth,
                                "I": -r,
                                "F": op.horizFeed,
                            },
                        )
                    )
                    op.commandlist.append(
                        Path.Command(
                            "G2",
                            {
                                "X": x,
                                "Y": y,
                                "Z": passEndDepth,
                                "I": r,
                                "F": op.horizFeed,
                            },
                        )
                    )

            else:  # no helix entry
                # rapid move to clearance height
                op.commandlist.append(Path.Command("G0", {"Z": obj.ClearanceHeight.Value}))
                op.commandlist.append(
                    Path.Command(
                        "G0",
                        {
                            "X": region["StartPoint"][0],
                            "Y": region["StartPoint"][1],
                            "Z": obj.ClearanceHeight.Value,
                        },
                    )
                )
                # straight plunge to target depth
                op.commandlist.append(
                    Path.Command(
                        "G1",
                        {
                            "X": region["StartPoint"][0],
                            "Y": region["StartPoint"][1],
                            "Z": passEndDepth,
                            "F": op.vertFeed,
                        },
                    )
                )

            lz = passEndDepth
            z = obj.ClearanceHeight.Value
            op.commandlist.append(Path.Command("(Adaptive - depth: %f)" % passEndDepth))

            # add adaptive paths
            for pth in region["AdaptivePaths"]:
                motionType = pth[0]  # [0] contains motion type

                for pt in pth[1]:  # [1] contains list of points
                    x = pt[0]
                    y = pt[1]

                    if motionType == area.AdaptiveMotionType.Cutting:
                        z = passEndDepth
                        if z != lz:
                            op.commandlist.append(Path.Command("G1", {"Z": z, "F": op.vertFeed}))

                        op.commandlist.append(
                            Path.Command("G1", {"X": x, "Y": y, "F": op.horizFeed})
                        )

                    elif motionType == area.AdaptiveMotionType.LinkClear:
                        z = passEndDepth + stepUp
                        if z != lz:
                            op.commandlist.append(Path.Command("G0", {"Z": z}))

                        op.commandlist.append(Path.Command("G0", {"X": x, "Y": y}))

                    elif motionType == area.AdaptiveMotionType.LinkNotClear:
                        z = obj.ClearanceHeight.Value
                        if z != lz:
                            op.commandlist.append(Path.Command("G0", {"Z": z}))

                        op.commandlist.append(Path.Command("G0", {"X": x, "Y": y}))

                    lz = z

            # return to safe height in this Z pass
            z = obj.ClearanceHeight.Value
            if z != lz:
                op.commandlist.append(Path.Command("G0", {"Z": z}))

            lz = z

            passStartDepth = passEndDepth

            # return to safe height in this Z pass
            z = obj.ClearanceHeight.Value
            if z != lz:
                op.commandlist.append(Path.Command("G0", {"Z": z}))

            lz = z

    z = obj.ClearanceHeight.Value
    if z != lz:
        op.commandlist.append(Path.Command("G0", {"Z": z}))


# functions for user-area-limited and automagic behavior. Maybe a parameter.
def Execute(op, obj):
    global sceneGraph
    global topZ

    if FreeCAD.GuiUp:
        sceneGraph = FreeCADGui.ActiveDocument.ActiveView.getSceneGraph()

    Path.Log.info("*** Adaptive toolpath processing started...\n")

    # hide old toolpaths during recalculation
    obj.Path = Path.Path("(Calculating...)")

    if FreeCAD.GuiUp:
        # store old visibility state
        job = op.getJob(obj)
        oldObjVisibility = obj.ViewObject.Visibility
        oldJobVisibility = job.ViewObject.Visibility

        obj.ViewObject.Visibility = False
        job.ViewObject.Visibility = False

        FreeCADGui.updateGui()

    try:
        helixDiameter = obj.HelixDiameterLimit.Value
        topZ = op.stock.Shape.BoundBox.ZMax
        obj.Stopped = False
        obj.StopProcessing = False
        obj.Tolerance = max(0.001, obj.Tolerance)

        # NOTE: Reminder that stock is formatted differently than inside/outside!
        stockPaths = {d: convertTo2d(op.stockPathArray[d]) for d in op.stockPathArray}

        outsideOpType = area.AdaptiveOperationType.ClearingOutside
        insideOpType = area.AdaptiveOperationType.ClearingInside

        # List every REGION separately- we can then calculate a toolpath based
        # on the region. One or more stepdowns may use that same toolpath by
        # keeping a reference to the region without requiring we calculate the
        # toolpath once per step down OR forcing all stepdowns of a region into
        # a single list.
        regionOps = list()
        outsidePathArray2dDepthTuples = list()
        insidePathArray2dDepthTuples = list()
        # NOTE: Make sure the depth lists are sorted for use in order-by-depth
        # and order-by-region algorithms below
        # NOTE: Pretty sure sorting is already guaranteed by how these are
        # created, but best to not assume that
        for rdict in op.outsidePathArray:
            regionOps.append(
                {
                    "opType": outsideOpType,
                    "path2d": convertTo2d(rdict["edges"]),
                    "id": rdict["id"],
                    "children": rdict["children"],
                    # FIXME: Kinda gross- just use this to match up with the
                    # appropriate stockpaths entry...
                    "startdepth": rdict["depths"][0],
                }
            )
            outsidePathArray2dDepthTuples.append(
                (sorted(rdict["depths"], reverse=True), regionOps[-1])
            )
        for rdict in op.insidePathArray:
            regionOps.append(
                {
                    "opType": insideOpType,
                    "path2d": convertTo2d(rdict["edges"]),
                    "id": rdict["id"],
                    "children": rdict["children"],
                    # FIXME: Kinda gross- just use this to match up with the
                    # appropriate stockpaths entry...
                    "startdepth": rdict["depths"][0],
                }
            )
            insidePathArray2dDepthTuples.append(
                (sorted(rdict["depths"], reverse=True), regionOps[-1])
            )

        keepToolDownRatio = 3.0
        if hasattr(obj, "KeepToolDownRatio"):
            keepToolDownRatio = obj.KeepToolDownRatio.Value

        # These fields are used to determine if toolpaths should be recalculated
        outsideInputStateObject = {
            "tool": op.tool.Diameter.Value,
            "tolerance": obj.Tolerance,
            "geometry": [k["path2d"] for k in regionOps if k["opType"] == outsideOpType],
            "stockGeometry": stockPaths,
            "stepover": obj.StepOver,
            "effectiveHelixDiameter": helixDiameter,
            "operationType": "Clearing",
            "side": "Outside",
            "forceInsideOut": obj.ForceInsideOut,
            "finishingProfile": obj.FinishingProfile,
            "keepToolDownRatio": keepToolDownRatio,
            "xyStockToLeave": obj.XYStockToLeave.Value,
            "zStockToLeave": obj.ZStockToLeave.Value,
            "orderCutsByRegion": obj.OrderCutsByRegion,
        }

        insideInputStateObject = {
            "tool": op.tool.Diameter.Value,
            "tolerance": obj.Tolerance,
            "geometry": [k["path2d"] for k in regionOps if k["opType"] == insideOpType],
            "stockGeometry": stockPaths,
            "stepover": obj.StepOver,
            "effectiveHelixDiameter": helixDiameter,
            "operationType": "Clearing",
            "side": "Inside",
            "forceInsideOut": obj.ForceInsideOut,
            "finishingProfile": obj.FinishingProfile,
            "keepToolDownRatio": keepToolDownRatio,
            "xyStockToLeave": obj.XYStockToLeave.Value,
            "zStockToLeave": obj.ZStockToLeave.Value,
            "orderCutsByRegion": obj.OrderCutsByRegion,
        }

        inputStateObject = [outsideInputStateObject, insideInputStateObject]

        inputStateChanged = False
        adaptiveResults = None

        # If we have a valid... path? Something. Generated, make that
        # tentatively the output
        if obj.AdaptiveOutputState:
            adaptiveResults = obj.AdaptiveOutputState

        # If ANYTHING in our input-cutting parameters, cutting regions,
        # etc.- changes, force recalculating
        if json.dumps(obj.AdaptiveInputState) != json.dumps(inputStateObject):
            inputStateChanged = True
            adaptiveResults = None

        # progress callback fn, if return true it will stop processing
        def progressFn(tpaths):
            if FreeCAD.GuiUp:
                for (
                    path
                ) in tpaths:  # path[0] contains the MotionType, #path[1] contains list of points
                    if path[0] == area.AdaptiveMotionType.Cutting:
                        sceneDrawPath(path[1], (0, 0, 1))

                    else:
                        sceneDrawPath(path[1], (1, 0, 1))

                FreeCADGui.updateGui()

            return obj.StopProcessing

        start = time.time()

        if inputStateChanged or adaptiveResults is None:
            # NOTE: Seem to need to create a new a2d for each area when we're
            # stepping down depths like this. If we don't, it will keep history
            # from the last region we did.

            # TODO: QThread/QRunnable trigger Python's global interpretor lock
            # (GIL). To calculate toolpaths in parallel, making a C++ shim that
            # takes in the array of regions/stock paths and parallelizes in
            # C++-land is probably the way to do it.

            # Create a toolpath for each region to avoid re-calculating for
            # identical stepdowns
            for rdict in regionOps:
                path2d = rdict["path2d"]
                opType = rdict["opType"]

                a2d = area.Adaptive2d()
                a2d.stepOverFactor = 0.01 * obj.StepOver
                a2d.toolDiameter = op.tool.Diameter.Value
                a2d.helixRampDiameter = helixDiameter
                a2d.keepToolDownDistRatio = keepToolDownRatio
                # NOTE: Z stock is handled in our stepdowns
                a2d.stockToLeave = obj.XYStockToLeave.Value
                a2d.tolerance = obj.Tolerance
                a2d.forceInsideOut = obj.ForceInsideOut
                a2d.finishingProfile = obj.FinishingProfile
                a2d.opType = opType

                rdict["toolpaths"] = a2d.Execute(
                    stockPaths[rdict["startdepth"]], path2d, progressFn
                )

            # Sort regions to cut by either depth or area.
            # TODO: Bonus points for ordering to minimize rapids
            cutlist = list()
            # Region IDs that have been cut already
            cutids = list()
            # Create sorted list of unique depths
            # NOTE: reverse because we cut top-down!
            depths = list()
            alltuples = outsidePathArray2dDepthTuples + insidePathArray2dDepthTuples
            for t in alltuples:
                depths += [d for d in t[0]]
            depths = sorted(list(set(depths)), reverse=True)
            if obj.OrderCutsByRegion:
                # Helper function to recurse down children
                def addToCutList(regionId):
                    for k in alltuples:
                        if k[1]["id"] == regionId:
                            # Should never happen
                            if k in cutlist:
                                return
                            cutlist.append(k)
                            for childId in k[1]["children"]:
                                addToCutList(childId)
                            break

                for k in alltuples:
                    if k not in cutlist:
                        addToCutList(k[1]["id"])
            else:
                for d in depths:
                    cutlist += [([d], o[1]) for o in outsidePathArray2dDepthTuples if d in o[0]]
                    cutlist += [([d], i[1]) for i in insidePathArray2dDepthTuples if d in i[0]]

            # need to convert results to python object to be JSON serializable
            stepdown = max(obj.StepDown.Value, _ADAPTIVE_MIN_STEPDOWN)
            adaptiveResults = list()
            for depths, region in cutlist:
                for result in region["toolpaths"]:
                    adaptiveResults.append(
                        {
                            "HelixCenterPoint": result.HelixCenterPoint,
                            "StartPoint": result.StartPoint,
                            "AdaptivePaths": result.AdaptivePaths,
                            "ReturnMotionType": result.ReturnMotionType,
                            "TopDepth": depths[0] + stepdown,
                            "BottomDepth": depths[-1],
                        }
                    )

        # GENERATE
        GenerateGCode(op, obj, adaptiveResults, helixDiameter)

        if not obj.StopProcessing:
            Path.Log.info("*** Done. Elapsed time: %f sec\n\n" % (time.time() - start))
            obj.AdaptiveOutputState = adaptiveResults
            obj.AdaptiveInputState = inputStateObject

        else:
            Path.Log.info("*** Processing cancelled (after: %f sec).\n\n" % (time.time() - start))

    finally:
        if FreeCAD.GuiUp:
            obj.ViewObject.Visibility = oldObjVisibility
            job.ViewObject.Visibility = oldJobVisibility
            sceneClean()


def projectFacesToXY(faces, minEdgeLength=1e-10):
    """projectFacesToXY(faces)
    Calculates the projection of the provided list of faces onto the XY plane.
    The returned value is a single shape that may contain multiple faces if
    there were disjoint projections. Each individual face clean, and won't have
    internal edges, triangulated geometry, etc., and will be at Z=0 on the XY
    plane
    """
    projdir = FreeCAD.Vector(0, 0, 1)
    outfaces = []
    for f in faces:
        # Vertical cones and spheres will still have a projection on the XY
        # plane. Cylinders and flat faces will not.
        if Path.Geom.isVertical(f) and type(f.Surface) not in [Part.Cone, Part.Sphere]:
            continue

        # NOTE: Wires/edges get clipped if we have an "exact fit" bounding box
        projface = Path.Geom.makeBoundBoxFace(f.BoundBox, offset=1, zHeight=0)

        # NOTE: Cylinders, cones, and spheres are messy:
        # - Internal representation of non-truncted cones and spheres includes
        # the "tip" with a ~0-area closed edge
        # - Projecting edges doesn't naively work due to the way seams are handled
        # - There may be holes at either end that may or may not line up
        if type(f.Surface) in [Part.Cone, Part.Cylinder, Part.Sphere]:
            oface = Part.makeFace(TechDraw.findShapeOutline(f, 1, projdir))
            wires = DraftGeomUtils.findWires(
                [e for e in f.Edges if not e.isSeam(f) and e.Length > minEdgeLength]
            )
            projwires = [projface.makeParallelProjection(w, projdir).Wires[0] for w in wires]
            if len(projwires) > 1:
                faces = [Part.makeFace(x) for x in projwires]
                overlap = faces[0].common(faces[1:])
                outfaces.append(oface.cut(overlap))
            else:
                outfaces.append(oface)
        # For other cases, projecting the wires to a plane should suffice
        else:
            facewires = list()
            for w in f.Wires:
                if w.isClosed():
                    projwire = projface.makeParallelProjection(w, projdir).Wires[0]
                    if projwire.isClosed():
                        facewires.append(projwire)
            if facewires:
                outfaces.append(Part.makeFace(facewires))
    if outfaces:
        fusion = outfaces[0].fuse(outfaces[1:])
        # removeSplitter fixes occasional concatenate issues for some face orders
        return DraftGeomUtils.concatenate(fusion.removeSplitter())
    else:
        return Part.Shape()


def _get_solid_projection(shp, z):
    """_get_solid_projection(shp, z)
    Calculates a shape obtained by slicing shp at the height z, then projecting
    the solids above that height onto a region of proj_face, and creating a
    simplified face
    """
    bb = shp.BoundBox

    # Find all faces above the machining depth. This is used to mask future
    # interior cuts, and the outer wire is used as the external wire
    bb_cut_top = Part.makeBox(
        bb.XLength,
        bb.YLength,
        max(bb.ZLength, bb.ZLength - z),
        FreeCAD.Vector(bb.XMin, bb.YMin, z),
    )
    above_solids = shp.common(bb_cut_top).Solids

    faces = list()
    for s in above_solids:
        faces += s.Faces

    return projectFacesToXY(faces)


def _working_edge_helper_roughing(op, obj, depths):
    # Final calculated regions- list of dicts with entries:
    # "region" - actual shape
    # "depths" - list of depths this region applies to
    inside_regions = list()
    outside_regions = list()

    # Multiple input solids can be selected- make a single part out of them,
    # will process each solid separately as appropriate
    shps = op.model[0].Shape.fuse([k.Shape for k in op.model[1:]])

    projdir = FreeCAD.Vector(0, 0, 1)

    # Take outline of entire model as our baseline machining region. No need to
    # do this repeatedly inside the loop.
    modelOutlineFaces = [
        Part.makeFace(TechDraw.findShapeOutline(s, 1, projdir)) for s in shps.Solids
    ]

    lastdepth = obj.StartDepth.Value

    for depth in depths:
        # If we have no stock to machine, just skip all the rest of the math
        if depth >= op.stock.Shape.BoundBox.ZMax:
            lastdepth = depth
            continue

        # NOTE: To "leave" stock along Z without actually checking any face
        # depths, we simply slice the model "lower" than our actual cut depth by
        # the Z stock to leave, which ensures we stay at least that far from the
        # actual faces
        stockface = _get_solid_projection(op.stock.Shape, depth - obj.ZStockToLeave.Value)
        above_refined = _get_solid_projection(shps, depth - obj.ZStockToLeave.Value)

        # Outside is based on the outer wire of the above_faces
        # Insides are based on the remaining "below" regions, masked by the
        # "above"- if something is above an area, we can't machine it in 2.5D

        # OUTSIDE REGIONS
        # Outside: Take the outer wire of the above faces
        # NOTE: Exactly one entry per depth (not necessarily one depth entry per
        # stepdown, however), which is a LIST of the wires we're staying outside
        # NOTE: Do this FIRST- if any inside regions share enough of an edge
        # with an outside region for a tool to get through, we want to skip them
        # for the current stepdown
        aboveModelFaces = [
            Part.makeFace(TechDraw.findShapeOutline(f, 1, projdir)) for f in above_refined.Faces
        ]
        if aboveModelFaces:
            aboveModelFaces = aboveModelFaces[0].fuse(aboveModelFaces[1:])
        else:
            aboveModelFaces = Part.Shape()
        # If this region exists in our list, it has to be the last entry, due to
        # proceeding in order and having only one per depth. If it's already
        # there, replace with the new, deeper depth, else add new
        # NOTE: Check for NULL regions to not barf on regions between the top of
        # the model and the top of the stock, which are "outside" of nothing
        if (
            outside_regions
            and not outside_regions[-1]["region"].isNull()
            and not aboveModelFaces.isNull()
            and not aboveModelFaces.cut(outside_regions[-1]["region"]).Wires
        ):
            outside_regions[-1]["depths"].append(depth)
        else:
            outside_regions.append({"region": aboveModelFaces, "depths": [depth]})

        # NOTE: If you don't care about controlling depth vs region ordering,
        # you can actually just do everything with "outside" processing, if you
        # don't remove internal holes from the regions above

        # INSIDE REGIONS
        # NOTE: Nothing inside if there's no model above us
        if not aboveModelFaces.isNull():
            # Remove any overlapping areas already machined from the outside.
            outsideface = stockface.cut(outside_regions[-1]["region"].Faces)
            if not outsideface.isNull():
                below_faces = [f.cut(outsideface) for f in modelOutlineFaces]
            else:
                # NOTE: Doesn't matter here, but ensure we're making a new list so
                # we don't clobber modelOutlineFaces
                below_faces = [f for f in modelOutlineFaces]

            # This isn't really necessary unless the user inputs bad data- eg, a
            # min depth above the top of the model. In which case we still want to
            # clear the stock
            below_faces = [b for b in below_faces if b.Area > obj.Tolerance]

            if below_faces:
                below_fusion = below_faces[0].fuse(below_faces[1:])
                # Remove the overhangs from the desired region to cut
                below_cut = below_fusion.cut(above_refined)
                if below_cut.Area:
                    # removeSplitter fixes occasional concatenate issues for
                    # some face orders
                    final_cut = DraftGeomUtils.concatenate(below_cut.removeSplitter())
                else:
                    final_cut = Part.Shape()
            else:
                # Make a dummy shape if we don't have anything actually below
                final_cut = Part.Shape()

            # Split up into individual faces if any are disjoint, then update
            # inside_regions- either by adding a new entry OR by updating the depth
            # of an existing entry
            for f in final_cut.Faces:
                addNew = True
                # Brute-force search all existing regions to see if any are the same
                newtop = lastdepth
                for rdict in inside_regions:
                    # FIXME: Smarter way to do this than a full cut operation?
                    df = rdict["region"]
                    if not df.cut(f).Wires:
                        rdict["depths"].append(depth)
                        addNew = False
                        break
                if addNew:
                    inside_regions.append({"region": f, "depths": [depth]})

        # Update the last depth step
        lastdepth = depth
    # end for depth

    return inside_regions, outside_regions


def _working_edge_helper_manual(op, obj, depths):
    # Final calculated regions- list of dicts with entries:
    # "region" - actual shape
    # "depths" - list of depths this region applies to
    inside_regions = list()
    outside_regions = list()

    # User selections, with extensions
    selected_regions = list()
    selected_edges = list()

    # Get extensions and identify faces to avoid
    extensions = FeatureExtensions.getExtensions(obj)
    avoidFeatures = [e for e in extensions if e.avoid]

    # Similarly, expand selected regions with extensions
    for ext in extensions:
        if not ext.avoid:
            if wire := ext.getWire():
                selected_regions += [f for f in ext.getExtensionFaces(wire)]

    # Easy boolean to decide if we're machining the inside or outside region
    isOutside = obj.Side == "Outside"

    for base, subs in obj.Base:
        for sub in subs:
            element = base.Shape.getElement(sub)
            if sub.startswith("Face") and sub not in avoidFeatures:
                shape = Part.Face(element.OuterWire) if obj.UseOutline else element
                selected_regions.append(shape)
            elif sub.startswith("Edge"):
                selected_edges.append(element)

    # Multiple input solids can be selected- make a single part out of them,
    # will process each solid separately as appropriate
    shps = op.model[0].Shape.fuse([k.Shape for k in op.model[1:]])

    # Make a face to project onto
    # NOTE: Use 0 as the height, since that's what TechDraw.findShapeOutline
    # uses, which we use to find the machining boundary, and the actual depth
    # is tracked separately.
    # NOTE: Project to the PART bounding box- with some padding- not the stock,
    # since the stock may be smaller than the part
    projface = Path.Geom.makeBoundBoxFace(shps.BoundBox, offset=1, zHeight=0)
    projdir = FreeCAD.Vector(0, 0, 1)

    edgefaces = list()
    if selected_edges:
        edgeWires = DraftGeomUtils.findWires(selected_edges)
        for ew in edgeWires:
            edgefaces += [Part.makeFace(projface.makeParallelProjection(ew, projdir).Wires[0])]

    selected_refined = projectFacesToXY(selected_regions + edgefaces)

    lastdepth = obj.StartDepth.Value

    for depth in depths:
        # If our depth is above the top of the stock, there's nothing to machine
        if depth >= op.stock.Shape.BoundBox.ZMax:
            lastdepth = depth
            continue

        # NOTE: See note in _working_edge_helper_roughing- tl;dr slice stock
        # lower than cut depth to effectively leave (at least) obj.ZStockToLeave
        above_refined = _get_solid_projection(shps, depth - obj.ZStockToLeave.Value)

        # Create appropriate tuples and add to list
        if isOutside:
            # Outside is based on the outer wire of the above_faces
            # Insides are based on the remaining "below" regions, masked by the
            # "above"- if something is above an area, we can't machine it in 2.5D

            # Outside: Take the outer wire of the above faces, added to selected
            # edges and regions
            # NOTE: Exactly one entry per depth (not necessarily one depth entry per
            # stepdown, however), which is a LIST of the wires we're staying outside
            # NOTE: Do this FIRST- if any inside regions share enough of an edge
            # with an outside region for a tool to get through, we want to skip them
            # for the current stepdown
            regions = [
                Part.makeFace(TechDraw.findShapeOutline(f, 1, projdir))
                for f in above_refined.Faces + selected_refined.Faces
            ]
            if regions:
                final_merge = regions[0].fuse(regions[1:])
            else:
                final_merge = selected_refined
            # Without removeSplitter(), concatenate will sometimes fail when
            # trying to merge faces that are (eg) connected A-B and B-C,
            # seemingly when trying to merge A-C
            regions = DraftGeomUtils.concatenate(final_merge.removeSplitter())

            # If this region exists in our list, it has to be the last entry, due to
            # proceeding in order and having only one per depth. If it's already
            # there, replace with the new, deeper depth, else add new
            match = (
                outside_regions
                and not regions.isNull()
                and not regions.cut(outside_regions[-1]["region"]).Wires
            )

            if match:
                outside_regions[-1]["depths"].append(depth)
            else:
                outside_regions.append({"region": regions, "depths": [depth]})

        # Inside
        # For every area selected by the user, project to a plane
        else:
            if not above_refined.isNull() and above_refined.Area > obj.Tolerance:
                final_cut = selected_refined.cut(above_refined)
            else:
                final_cut = selected_refined

            # Split up into individual faces if any are disjoint, then update
            # inside_regions- either by adding a new entry OR by updating the depth
            # of an existing entry
            for f in final_cut.Faces:
                addNew = True
                # Brute-force search all existing regions to see if any are the same
                newtop = lastdepth
                for rdict in inside_regions:
                    # FIXME: Smarter way to do this than a full cut operation?
                    df = rdict["region"]
                    if not df.cut(f).Wires:
                        rdict["depths"].append(depth)
                        addNew = False
                        break
                if addNew:
                    inside_regions.append({"region": f, "depths": [depth]})

        # Update the last depth step
        lastdepth = depth
    # end for depth

    return inside_regions, outside_regions


def _get_working_edges(op, obj):
    """_get_working_edges(op, obj)...
    Compile all working edges from the Base Geometry selection (obj.Base)
    for the current operation (or the entire model if no selections).
    Additional modifications to selected region(face), such as extensions,
    should be placed within this function.
    This version will return two lists- one for outside edges and one for inside
    edges. Each list will be a dict with "region" and "depths" entries- the
    former being discretized geometry of the region, the latter being a list of
    every depth the geometry is machined on
    """

    stock_bb = op.stock.Shape.BoundBox

    # Find depth steps, throwing out all depths above anywhere we might cut
    # NOTE: Finish stepdown = 0 here- it's actually applied when gcode is
    # generated; doing so here would cause it to be applied twice.
    depth_params = PathUtils.depth_params(
        clearance_height=obj.ClearanceHeight.Value,
        safe_height=obj.SafeHeight.Value,
        start_depth=obj.StartDepth.Value,
        step_down=max(obj.StepDown.Value, _ADAPTIVE_MIN_STEPDOWN),
        z_finish_step=0.0,
        final_depth=obj.FinalDepth.Value,
        user_depths=None,
    )

    depths = [d for d in depth_params.data if d < stock_bb.ZMax]

    # Get the stock outline at each stepdown. Used to calculate toolpaths and
    # for calcuating cut regions in some instances
    # NOTE: See note in _working_edge_helper_roughing- tl;dr slice stock lower
    # than cut depth to effectively leave (at least) obj.ZStockToLeave
    # NOTE: Stock is handled DIFFERENTLY than inside and outside regions!
    # Combining different depths just adds code to look up the correct outline
    # when computing inside/outside regions, for no real benefit.
    stockProjectionDict = {
        d: _get_solid_projection(op.stock.Shape, d - obj.ZStockToLeave.Value) for d in depths
    }

    # If user specified edges, calculate the machining regions based on that
    # input. Otherwise, process entire model
    # Output are lists of dicts with "region" and "depths" entries. Depths are
    # a list of Z depths that the region applies to
    # Inside regions are a single face; outside regions consist of ALL geometry
    # to be avoided at those depths.
    if obj.Base:
        inside_regions, outside_regions = _working_edge_helper_manual(op, obj, depths)
    else:
        inside_regions, outside_regions = _working_edge_helper_roughing(op, obj, depths)

    # Find all children of each region. A child of region X is any region Y such
    # that Y is a subset of X AND Y starts within one stepdown of X (ie, direct
    # children only).
    # NOTE: Inside and outside regions are inverses of each other, so above
    # refers to the area to be machined!

    # Assign an ID number to track each region
    idnumber = 0
    for r in inside_regions + outside_regions:
        r["id"] = idnumber
        r["children"] = list()
        idnumber += 1

    # NOTE: Inside and outside regions are inverses of each other
    # NOTE: Outside regions can't have parents
    for rx in inside_regions:
        for ry in [k for k in inside_regions if k != rx]:
            dist = min(rx["depths"]) - max(ry["depths"])
            # Ignore regions at our level or above, or more than one step down
            if dist <= 0 or dist > depth_params.step_down:
                continue
            if not ry["region"].cut(rx["region"]).Wires:
                rx["children"].append(ry["id"])
        # See which outside region this is a child of- basically inverse of above
        for ry in [k for k in outside_regions]:
            dist = min(ry["depths"]) - max(rx["depths"])
            # Ignore regions at our level or above, or more than one step down
            if dist <= 0 or dist > depth_params.step_down:
                continue
            # child if cut does NOT affect the area- no overlap
            # Also a child if the outer region is NULL (includes everything)
            if ry["region"].isNull() or rx["region"].Area == rx["region"].cut(ry["region"]).Area:
                ry["children"].append(rx["id"])

    # Further split regions as necessary for when the stock changes- a region as
    # reported here is where a toolpath will be generated, and can be projected
    # along all of the depths associated with it. By doing this, we can minimize
    # the number of toolpaths that need to be generated AND avoid more complex
    # logic in depth-first vs region-first sorting of regions.
    # NOTE: For internal regions, stock is "the same" if the region cut with
    # the stock results in the same region.
    # NOTE: For external regions, stock is "the same" if the stock cut by the
    # region results in the same region
    def _regionChildSplitterHelper(regions, areInsideRegions):
        nonlocal stockProjectionDict
        nonlocal idnumber
        for r in regions:
            depths = sorted(r["depths"], reverse=True)
            if areInsideRegions:
                rcut = r["region"].cut(stockProjectionDict[depths[0]])
            else:
                # NOTE: We may end up with empty "outside" regions in the space
                # between the top of the stock and the top of the model- want
                # to machine the entire stock in that case
                if r["region"].isNull():
                    rcut = stockProjectionDict[depths[0]]
                else:
                    rcut = stockProjectionDict[depths[0]].cut(r["region"])
            parentdepths = depths[0:1]
            # If the region cut with the stock at a new depth is different than
            # the original cut, we need to split this region
            # The new region gets all of the children, and becomes a child of
            # the existing region.
            for d in depths[1:]:
                if (
                    areInsideRegions and r["region"].cut(stockProjectionDict[d]).cut(rcut).Wires
                ) or stockProjectionDict[d].cut(r["region"]).cut(rcut).Wires:
                    newregion = {
                        "id": idnumber,
                        "depths": [k for k in depths if k not in parentdepths],
                        "region": r["region"],
                        "children": r["children"],
                    }
                    # Update parent with the new region as a child, along with all
                    # the depths it was unchanged on
                    r["children"] = [idnumber]
                    r["depths"] = parentdepths

                    # Add the new region to the end of the list and stop processing
                    # this region
                    # When the new region is processed at the end, we'll effectively
                    # recurse and handle splitting that new region if required
                    regions.append(newregion)
                    idnumber += 1
                    continue
                # If we didn't split at this depth, the parent will keep "control"
                # of this depth
                parentdepths.append(d)

    _regionChildSplitterHelper(inside_regions, True)
    _regionChildSplitterHelper(outside_regions, False)

    # Create discretized regions
    inside_discretized = list()
    for rdict in inside_regions:
        discretizedEdges = [[discretize(w)] for w in rdict["region"].Wires]
        inside_discretized.append(
            {
                "edges": discretizedEdges,
                "depths": rdict["depths"],
                "id": rdict["id"],
                "children": rdict["children"],
            }
        )

    outside_discretized = list()
    for rdict in outside_regions:
        discretizedEdges = list()
        for a in rdict["region"].Faces:
            for w in a.Wires:
                discretizedEdges.append([discretize(w)])
        outside_discretized.append(
            {
                "edges": discretizedEdges,
                "depths": rdict["depths"],
                "id": rdict["id"],
                "children": rdict["children"],
            }
        )

    # NOTE: REMINDER: This is notably different from machining regions- just
    # a dict with depth: region entries, single depth for easy lookup
    stock_discretized = {}
    for d in stockProjectionDict:
        discretizedEdges = list()
        for a in stockProjectionDict[d].Faces:
            for w in a.Wires:
                discretizedEdges.append([discretize(w)])
        stock_discretized[d] = discretizedEdges

    # Return found inside and outside regions/depths. Up to the caller to decide
    # which ones it cares about.
    # NOTE: REMINDER: Stock is notably different from machining regions- just
    # a dict with depth: region entries, single depth for easy lookup
    return inside_discretized, outside_discretized, stock_discretized


class PathAdaptive(PathOp.ObjectOp):
    def opFeatures(self, obj):
        """opFeatures(obj) ... returns the OR'ed list of features used and supported by the operation.
        The default implementation returns "FeatureTool | FeatureDepths | FeatureHeights | FeatureStartPoint"
        Should be overwritten by subclasses."""
        return (
            PathOp.FeatureTool
            | PathOp.FeatureBaseEdges
            | PathOp.FeatureDepths
            | PathOp.FeatureFinishDepth
            | PathOp.FeatureStepDown
            | PathOp.FeatureHeights
            | PathOp.FeatureBaseGeometry
            | PathOp.FeatureCoolant
            | PathOp.FeatureLocations
        )

    @classmethod
    def propertyEnumerations(self, dataType="data"):
        """helixOpPropertyEnumerations(dataType="data")... return property enumeration lists of specified dataType.
        Args:
            dataType = 'data', 'raw', 'translated'
        Notes:
        'data' is list of internal string literals used in code
        'raw' is list of (translated_text, data_string) tuples
        'translated' is list of translated string literals
        """

        # Enumeration lists for App::PropertyEnumeration properties
        enums = {
            "Side": [
                (translate("CAM_Adaptive", "Outside"), "Outside"),
                (translate("CAM_Adaptive", "Inside"), "Inside"),
            ],  # this is the direction that the profile runs
            "OperationType": [
                (translate("CAM_Adaptive", "Clearing"), "Clearing"),
                (translate("CAM_Adaptive", "Profiling"), "Profiling"),
            ],  # side of profile that cutter is on in relation to direction of profile
        }

        if dataType == "raw":
            return enums

        data = list()
        idx = 0 if dataType == "translated" else 1

        Path.Log.debug(enums)

        for k, v in enumerate(enums):
            data.append((v, [tup[idx] for tup in enums[v]]))
        Path.Log.debug(data)

        return data

    def initOperation(self, obj):
        """initOperation(obj) ... implement to create additional properties.
        Should be overwritten by subclasses."""
        obj.addProperty(
            "App::PropertyEnumeration",
            "Side",
            "Adaptive",
            QT_TRANSLATE_NOOP(
                "App::Property",
                "Side of selected faces that tool should cut",
            ),
        )
        obj.addProperty(
            "App::PropertyEnumeration",
            "OperationType",
            "Adaptive",
            QT_TRANSLATE_NOOP(
                "App::Property",
                "Type of adaptive operation",
            ),
        )
        obj.addProperty(
            "App::PropertyFloat",
            "Tolerance",
            "Adaptive",
            QT_TRANSLATE_NOOP(
                "App::Property",
                "Influences calculation performance vs stability and accuracy.\n\nLarger values (further to the right) will calculate faster; smaller values (further to the left) will result in more accurate toolpaths.",
            ),
        )
        obj.addProperty(
            "App::PropertyPercent",
            "StepOver",
            "Adaptive",
            QT_TRANSLATE_NOOP(
                "App::Property",
                "Percent of cutter diameter to step over on each pass",
            ),
        )
        obj.addProperty(
            "App::PropertyDistance",
            "LiftDistance",
            "Adaptive",
            QT_TRANSLATE_NOOP(
                "App::Property",
                "Lift distance for rapid moves",
            ),
        )
        obj.addProperty(
            "App::PropertyDistance",
            "KeepToolDownRatio",
            "Adaptive",
            QT_TRANSLATE_NOOP(
                "App::Property",
                "Max length of keep tool down path compared to direct distance between points",
            ),
        )
        # FIXME: Changing this from StockToLeave is good for code clarity, bad in
        # that it breaks all existing toolpaths. Users are already going to need
        # to change existing toolpaths due to handing boundaries differently, so
        # maybe worth the change?
        obj.addProperty(
            "App::PropertyDistance",
            "XYStockToLeave",
            "Adaptive",
            QT_TRANSLATE_NOOP(
                "App::Property",
                "How much stock to leave in the XY plane (i.e. for finishing operation)",
            ),
        )
        obj.addProperty(
            "App::PropertyDistance",
            "ZStockToLeave",
            "Adaptive",
            QT_TRANSLATE_NOOP(
                "App::Property",
                "How much stock to leave along the Z axis (i.e. for finishing operation)",
            ),
        )
        obj.addProperty(
            "App::PropertyBool",
            "ForceInsideOut",
            "Adaptive",
            QT_TRANSLATE_NOOP(
                "App::Property",
                "Force plunging into material inside and clearing towards the edges",
            ),
        )
        obj.addProperty(
            "App::PropertyBool",
            "FinishingProfile",
            "Adaptive",
            QT_TRANSLATE_NOOP(
                "App::Property",
                "To take a finishing profile path at the end",
            ),
        )
        obj.addProperty(
            "App::PropertyBool",
            "Stopped",
            "Adaptive",
            QT_TRANSLATE_NOOP("App::Property", "Stop processing"),
        )
        obj.setEditorMode("Stopped", 2)  # hide this property

        obj.addProperty(
            "App::PropertyBool",
            "StopProcessing",
            "Adaptive",
            QT_TRANSLATE_NOOP(
                "App::Property",
                "Stop processing",
            ),
        )
        obj.setEditorMode("StopProcessing", 2)  # hide this property

        obj.addProperty(
            "App::PropertyBool",
            "UseHelixArcs",
            "Adaptive",
            QT_TRANSLATE_NOOP(
                "App::Property",
                "Use Arcs (G2) for helix ramp",
            ),
        )

        obj.addProperty(
            "App::PropertyPythonObject",
            "AdaptiveInputState",
            "Adaptive",
            QT_TRANSLATE_NOOP(
                "App::Property",
                "Internal input state",
            ),
        )
        obj.addProperty(
            "App::PropertyPythonObject",
            "AdaptiveOutputState",
            "Adaptive",
            QT_TRANSLATE_NOOP(
                "App::Property",
                "Internal output state",
            ),
        )
        obj.setEditorMode("AdaptiveInputState", 2)  # hide this property
        obj.setEditorMode("AdaptiveOutputState", 2)  # hide this property
        obj.addProperty(
            "App::PropertyAngle",
            "HelixAngle",
            "Adaptive",
            QT_TRANSLATE_NOOP(
                "App::Property",
                "Helix ramp entry angle (degrees)",
            ),
        )
        obj.addProperty(
            "App::PropertyAngle",
            "HelixConeAngle",
            "Adaptive",
            QT_TRANSLATE_NOOP(
                "App::Property",
                "Helix cone angle (degrees)",
            ),
        )
        obj.addProperty(
            "App::PropertyLength",
            "HelixDiameterLimit",
            "Adaptive",
            QT_TRANSLATE_NOOP(
                "App::Property",
                "Limit helix entry diameter, if limit larger than tool diameter or 0, tool diameter is used",
            ),
        )

        obj.addProperty(
            "App::PropertyBool",
            "UseOutline",
            "Adaptive",
            QT_TRANSLATE_NOOP(
                "App::Property",
                "Uses the outline of the base geometry.",
            ),
        )

        obj.addProperty(
            "App::PropertyBool",
            "OrderCutsByRegion",
            "Adaptive",
            QT_TRANSLATE_NOOP(
                "App::Property",
                "Orders cuts by region instead of depth.",
            ),
        )

        obj.addProperty(
            "Part::PropertyPartShape",
            "removalshape",
            "Path",
            QT_TRANSLATE_NOOP("App::Property", ""),
        )

        for n in self.propertyEnumerations():
            setattr(obj, n[0], n[1])

        obj.setEditorMode("removalshape", 2)  # hide

        FeatureExtensions.initialize_properties(obj)

    def opSetDefaultValues(self, obj, job):
        obj.Side = "Inside"
        obj.OperationType = "Clearing"
        obj.Tolerance = 0.1
        obj.StepOver = 20
        obj.LiftDistance = 0
        # obj.ProcessHoles = True
        obj.ForceInsideOut = False
        obj.FinishingProfile = True
        obj.Stopped = False
        obj.StopProcessing = False
        obj.HelixAngle = 5
        obj.HelixConeAngle = 0
        obj.HelixDiameterLimit = 0.0
        obj.AdaptiveInputState = ""
        obj.AdaptiveOutputState = ""
        obj.XYStockToLeave = 0
        obj.ZStockToLeave = 0
        obj.KeepToolDownRatio = 3.0
        obj.UseHelixArcs = False
        obj.UseOutline = False
        obj.OrderCutsByRegion = False
        FeatureExtensions.set_default_property_values(obj, job)

    def opExecute(self, obj):
        """opExecute(obj) ... called whenever the receiver needs to be recalculated.
        See documentation of execute() for a list of base functionality provided.
        Should be overwritten by subclasses."""

        # Contains both geometry to machine and the applicable depths
        # NOTE: Reminder that stock is formatted differently than inside/outside!
        inside, outside, stock = _get_working_edges(self, obj)

        self.insidePathArray = inside
        self.outsidePathArray = outside
        self.stockPathArray = stock

        Execute(self, obj)

    def opOnDocumentRestored(self, obj):
        if not hasattr(obj, "HelixConeAngle"):
            obj.addProperty(
                "App::PropertyAngle",
                "HelixConeAngle",
                "Adaptive",
                "Helix cone angle (degrees)",
            )

        if not hasattr(obj, "UseOutline"):
            obj.addProperty(
                "App::PropertyBool",
                "UseOutline",
                "Adaptive",
                "Uses the outline of the base geometry.",
            )
        # FIXME: I don't really understand why this is here for some attributes
        # and not others, but the build whines about this attribute not existing
        # if we don't... these are things that aren't in files created by older
        # versions of FreeCAD?
        # EDIT: Now seems fine to have commented out????
        """
        if not hasattr(obj, "OrderCutsByRegion"):
            obj.addProperty(
                "App::PropertyBool",
                "OrderCutsByRegion",
                "Adaptive",
                "Orders cuts by region instead of depth.",
            )
        """

        if not hasattr(obj, "removalshape"):
            obj.addProperty("Part::PropertyPartShape", "removalshape", "Path", "")
        obj.setEditorMode("removalshape", 2)  # hide

        FeatureExtensions.initialize_properties(obj)


# Eclass


def SetupProperties():
    setup = [
        "Side",
        "OperationType",
        "Tolerance",
        "StepOver",
        "LiftDistance",
        "KeepToolDownRatio",
        "XYStockToLeave",
        "ZStockToLeave",
        "ForceInsideOut",
        "FinishingProfile",
        "Stopped",
        "StopProcessing",
        "UseHelixArcs",
        "AdaptiveInputState",
        "AdaptiveOutputState",
        "HelixAngle",
        "HelixConeAngle",
        "HelixDiameterLimit",
        "UseOutline",
        "OrderCutsByRegion",
    ]
    return setup


def Create(name, obj=None, parentJob=None):
    """Create(name) ... Creates and returns a Adaptive operation."""
    if obj is None:
        obj = FreeCAD.ActiveDocument.addObject("Path::FeaturePython", name)
    obj.Proxy = PathAdaptive(obj, name, parentJob)
    return obj
