# -*- coding: utf-8 -*-
# ***************************************************************************
# *   Copyright (c) 2022 sliptonic <shopinthewoods@gmail.com>               *
# *                                                                         *
# *   This program is free software; you can redistribute it and/or modify  *
# *   it under the terms of the GNU Lesser General Public License (LGPL)    *
# *   as published by the Free Software Foundation; either version 2 of     *
# *   the License, or (at your option) any later version.                   *
# *   for detail see the LICENCE text file.                                 *
# *                                                                         *
# *   This program is distributed in the hope that it will be useful,       *
# *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
# *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
# *   GNU Library General Public License for more details.                  *
# *                                                                         *
# *   You should have received a copy of the GNU Library General Public     *
# *   License along with this program; if not, write to the Free Software   *
# *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  *
# *   USA                                                                   *
# *                                                                         *
# ***************************************************************************

import FreeCAD
import Path
import PathScripts.PathGeom as PathGeom
import math

__title__ = "PathLanguage - classes for an internal language/representaion for Path"
__author__ = "sliptonic (Brad Collette)"
__url__ = "https://www.freecadweb.org"
__doc__ = "Functions to extract and convert between Path.Command and Part.Edge and utility functions to reason about them."

CmdMoveStraight = PathGeom.CmdMoveStraight + PathGeom.CmdMoveRapid

class Instruction (object):
    '''An Instruction is a pure python replacement of Path.Command which also tracks its begin position.'''

    def __init__(self, begin, cmd, param=None):
        self.begin = begin
        if type(cmd) == Path.Command:
            self.cmd = Path.Name
            self.param = Path.Parameters
        else:
            self.cmd = cmd
            if param is None:
                self.param = {}
            else:
                self.param = param

    def anglesOfTangents(self):
        return (0, 0)

    def setPositionBegin(self, begin):
        self.begin = begin

    def positionBegin(self):
        '''positionBegin() ... returns a Vector of the begin position'''
        return self.begin

    def positionEnd(self):
        '''positionEnd() ... returns a Vector of the end position'''
        return FreeCAD.Vector(self.x(self.begin.x), self.y(self.begin.y), self.z(self.begin.z))

    def pathLength(self):
        '''pathLength() ... returns the lenght in mm'''
        return 0

    def isMove(self):
        return False

    def x(self, default=0):
        return self.param.get('X', default)

    def y(self, default=0):
        return self.param.get('Y', default)

    def z(self, default=0):
        return self.param.get('Z', default)

    def i(self, default=0):
        return self.param.get('I', default)

    def j(self, default=0):
        return self.param.get('J', default)

    def k(self, default=0):
        return self.param.get('K', default)

    def xyBegin(self):
        '''xyBegin() ... internal convenience function'''
        return FreeCAD.Vector(self.begin.x, self.begin.y, 0)
    def xyEnd(self):
        '''xyEnd() ... internal convenience function'''
        return FreeCAD.Vector(self.x(self.begin.x), self.y(self.begin.y), 0)

    def __repr__(self):
        return f"{self.cmd}{self.param}"

    def str(self, digits=2):
        if digits == 0:
            s = [f"{k}: {int(v)}" for k, v in self.param.items()]
        else:
            fmt = f"{{}}: {{:.{digits}}}"
            s = [fmt.format(k, v) for k, v in self.param.items()]
        return f"{self.cmd}{{{', '.join(s)}}}"

class MoveStraight (Instruction):

    def anglesOfTangents(self):
        '''anglesOfTangents() ... return a tuple with the tangent angles at begin and end position'''
        begin = self.xyBegin()
        end = self.xyEnd()
        if end == begin:
            return (0, 0)
        a = PathGeom.getAngle(end - begin)
        return (a, a)

    def isMove(self):
        return True

    def pathLength(self):
        return (self.positionEnd() - self.positionBegin()).Length

class MoveArc (Instruction):

    def anglesOfTangents(self):
        '''anglesOfTangents() ... return a tuple with the tangent angles at begin and end position'''
        begin = self.xyBegin()
        end = self.xyEnd()
        center = self.xyCenter()
        # calculate angle of the hypotenuse at begin and end
        s0 = PathGeom.getAngle(begin - center)
        s1 = PathGeom.getAngle(end - center)
        # the tangents are perpendicular to the hypotenuse with the sign determined by the
        # direction of the arc
        return (PathGeom.normalizeAngle(s0 + self.arcDirection()), PathGeom.normalizeAngle(s1 + self.arcDirection()))

    def isMove(self):
        return True

    def isArc(self):
        return True

    def isCW(self):
        return self.arcDirection() < 0

    def isCCW(self):
        return self.arcDirection() > 0

    def arcAngle(self):
        '''arcAngle() ... return the angle of the arc opening'''
        begin = self.xyBegin()
        end = self.xyEnd()
        center = self.xyCenter()
        s0 = PathGeom.getAngle(begin - center)
        s1 = PathGeom.getAngle(end - center)

        if self.isCW():
            while s0 < s1:
                s0 = s0 + 2 * math.pi
            return s0 - s1

        # CCW
        while s1 < s0:
            s1 = s1 + 2 * math.pi
        return s1 - s0

    def arcRadius(self):
        '''arcRadius() ... return the radius'''
        return (self.xyBegin() - self.xyCenter()).Length

    def pathLength(self):
        return self.arcAngle() * self.arcRadius()

    def xyCenter(self):
        return FreeCAD.Vector(self.begin.x + self.i(), self.begin.y + self.j(), 0)

class MoveArcCW (MoveArc):
    def arcDirection(self):
        return -math.pi/2

class MoveArcCCW (MoveArc):
    def arcDirection(self):
        return math.pi/2

class Maneuver (object):
    '''A series of instructions and moves'''

    def __init__(self, begin=None, instr=None):
        self.instr = instr if instr else []
        self.setPositionBegin(begin if begin else FreeCAD.Vector(0, 0, 0))

    def setPositionBegin(self, begin):
        self.begin = begin
        for i in self.instr:
            i.setPositionBegin(begin)
            begin = i.positionEnd()

    def positionBegin(self):
        return self.begin

    def getMoves(self):
        return [instr for instr in self.instr if instr.isMove()]

    def __repr__(self):
        if self.instr:
            return '\n'.join([str(i) for i in self.instr])
        return ''

    @classmethod
    def InstructionFromCommand(cls, cmd, begin=None):
        if not begin:
            begin = FreeCAD.Vector(0, 0, 0)

        if cmd.Name in CmdMoveStraight:
            return MoveStraight(begin, cmd.Name, cmd.Parameters)
        if cmd.Name in PathGeom.CmdMoveCW:
            return MoveArcCW(begin, cmd.Name, cmd.Parameters)
        if cmd.Name in PathGeom.CmdMoveCCW:
            return MoveArcCCW(begin, cmd.Name, cmd.Parameters)
        return Instruction(begin, cmd.Name, cmd.Parameters)

    @classmethod
    def FromPath(cls, path, begin=None):
        maneuver = Maneuver(begin)
        instr = []
        begin = maneuver.positionBegin()
        for cmd in path.Commands:
            i = cls.InstructionFromCommand(cmd, begin)
            instr.append(i)
            begin = i.positionEnd()
        maneuver.instr = instr
        return maneuver

    @classmethod
    def FromGCode(cls, gcode, begin=None):
        return cls.FromPath(Path.Path(gcode), begin)

