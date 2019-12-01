# ------------------------------------------------------------------------------
#   BSD 2-Clause License
#   
#   Copyright (c) 2019, Thomas Larsson
#   All rights reserved.
#   
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions are met:
#   
#   1. Redistributions of source code must retain the above copyright notice, this
#      list of conditions and the following disclaimer.
#   
#   2. Redistributions in binary form must reproduce the above copyright notice,
#      this list of conditions and the following disclaimer in the documentation
#      and/or other materials provided with the distribution.
#   
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
#   FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
#   DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
#   SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
#   OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# ------------------------------------------------------------------------------



import bpy
from math import pi
from . import utils
from .utils import *

#
#    simplifyFCurves(context, rig, useVisible, useMarkers):
#

def simplifyFCurves(context, rig, useVisible, useMarkers):
    scn = context.scene
    act = getAction(rig)
    if not act:
        return
    (fcurves, minTime, maxTime) = getActionFCurves(act, useVisible, useMarkers, scn)
    if not fcurves:
        return

    for fcu in fcurves:
        simplifyFCurve(fcu, rig.animation_data.action, scn.McpErrorLoc, scn.McpErrorRot, minTime, maxTime)
    setInterpolation(rig)
    print("Curves simplified")
    return

#
#   getActionFCurves(act, useVisible, useMarkers, scn):
#

def getActionFCurves(act, useVisible, useMarkers, scn):
    if useVisible:
        fcurves = []
        for fcu in act.fcurves:
            if not fcu.hide:
                fcurves.append(fcu)
                #print(fcu.data_path, fcu.array_index)
    else:
        fcurves = act.fcurves

    if useMarkers:
        (minTime, maxTime) = getMarkedTime(scn)
        if minTime == None:
            print("Need two selected markers")
            return ([], 0, 0)
    else:
        (minTime, maxTime) = ('All', 0)
    return (fcurves, minTime, maxTime)

#
#   splitFCurvePoints(fcu, minTime, maxTime):
#

def splitFCurvePoints(fcu, minTime, maxTime):
    if minTime == 'All':
        points = fcu.keyframe_points
        before = []
        after = []
    else:
        points = []
        before = []
        after = []
        for pt in fcu.keyframe_points:
            t = pt.co[0]
            if t < minTime:
                before.append(pt.co)
            elif t > maxTime:
                after.append(pt.co)
            else:
                points.append(pt)
    return (points, before, after)

#
#    simplifyFCurve(fcu, act, maxErrLoc, maxErrRot, minTime, maxTime):
#

def simplifyFCurve(fcu, act, maxErrLoc, maxErrRot, minTime, maxTime):
    #print("WARNING: F-curve simplification turned off")
    #return
    words = fcu.data_path.split('.')
    if words[-1] == 'location':
        maxErr = maxErrLoc
    elif words[-1] == 'rotation_quaternion':
        maxErr = maxErrRot * 1.0/180
    elif words[-1] == 'rotation_euler':
        maxErr = maxErrRot * pi/180
    else:
        raise MocapError("Unknown FCurve type %s" % words[-1])

    (points, before, after) = splitFCurvePoints(fcu, minTime, maxTime)

    nPoints = len(points)
    nBefore = len(before)
    nAfter = len(after)
    if nPoints <= 2:
        return
    keeps = []
    new = [0, nPoints-1]
    while new:
        keeps += new
        keeps.sort()
        new = iterateFCurves(points, keeps, maxErr)
    newVerts = []
    for n in keeps:
        newVerts.append(points[n].co.copy())
    nNewPoints = len(newVerts)

    oldOffset = nBefore+nPoints
    newOffset = nBefore+nNewPoints
    for n in range(nAfter):
        fcu.keyframe_points[n+newOffset].co = fcu.keyframe_points[n+oldOffset].co.copy()
    n = nBefore+nPoints+nAfter
    n1 = nBefore+nNewPoints+nAfter
    while n > n1:
        n -= 1
        kp = fcu.keyframe_points[n]
        fcu.keyframe_points.remove(kp)
    for n in range(nNewPoints):
        fcu.keyframe_points[n+nBefore].co = newVerts[n]
    return

#
#    iterateFCurves(points, keeps, maxErr):
#

def iterateFCurves(points, keeps, maxErr):
    new = []
    for edge in range(len(keeps)-1):
        n0 = keeps[edge]
        n1 = keeps[edge+1]
        (x0, y0) = points[n0].co
        (x1, y1) = points[n1].co
        if x1 > x0:
            dxdn = (x1-x0)/(n1-n0)
            dydx = (y1-y0)/(x1-x0)
            err = 0
            for n in range(n0+1, n1):
                (x, y) = points[n].co
                xn = n0 + dxdn*(n-n0)
                yn = y0 + dydx*(xn-x0)
                if abs(y-yn) > err:
                    err = abs(y-yn)
                    worst = n
            if err > maxErr:
                new.append(worst)
    return new

#
#   rescaleFCurves(context, rig, factor):
#

def rescaleFCurves(context, rig, factor):
    act = getAction(context.object)
    if not act:
        return
    for fcu in act.fcurves:
        rescaleFCurve(fcu, factor)
    print("Curves rescaled")
    return

#
#   rescaleFCurve(fcu, factor):
#

def rescaleFCurve(fcu, factor):
    n = len(fcu.keyframe_points)
    if n < 2:
        return
    (t0,v0) = fcu.keyframe_points[0].co
    (tn,vn) = fcu.keyframe_points[n-1].co
    limitData = getFCurveLimits(fcu)
    (mode, upper, lower, diff) = limitData

    tm = t0
    vm = v0
    inserts = []
    for pk in fcu.keyframe_points:
        (tk,vk) = pk.co
        tn = factor*(tk-t0) + t0
        if upper:
            if (vk > upper) and (vm < lower):
                inserts.append((tm, vm, tn, vk))
            elif (vm > upper) and (vk < lower):
                inserts.append((tm, vm, tn,vk))
        pk.co = (tn,vk)
        tm = tn
        vm = vk

    addFCurveInserts(fcu, inserts, limitData)
    return

#
#   getFCurveLimits(fcu):
#

def getFCurveLimits(fcu):
    words = fcu.data_path.split('.')
    mode = words[-1]
    if mode == 'rotation_euler':
        upper = 0.8*pi
        lower = -0.8*pi
        diff = pi
    elif mode == 'rotation_quaternion':
        upper = 0.8
        lower = -0.8
        diff = 2
    else:
        upper = 0
        lower = 0
        diff = 0
    #print(words[1], mode, upper, lower)
    return (mode, upper, lower, diff)

#
#   addFCurveInserts(fcu, inserts, limitData):
#

def addFCurveInserts(fcu, inserts, limitData):
    (mode, upper, lower, diff) = limitData
    for (tm,vm,tn,vn) in inserts:
        tp = int((tm+tn)/2 - 0.1)
        tq = tp + 1
        vp = (vm+vn)/2
        if vm > upper:
            vp += diff/2
            vq = vp - diff
        elif vm < lower:
            vp -= diff/2
            vq = vp + diff
        if tp > tm:
            fcu.keyframe_points.insert(frame=tp, value=vp)
        if tq < tn:
            fcu.keyframe_points.insert(frame=tq, value=vq)
    return


########################################################################
#
#   class MCP_OT_SimplifyFCurves(bpy.types.Operator):
#

class MCP_OT_SimplifyFCurves(bpy.types.Operator):
    bl_idname = "mcp.simplify_fcurves"
    bl_label = "Simplify FCurves"
    bl_options = {'UNDO'}

    def execute(self, context):
        try:
            scn = context.scene
            simplifyFCurves(context, context.object, scn.McpSimplifyVisible, scn.McpSimplifyMarkers)
        except MocapError:
            bpy.ops.mcp.error('INVOKE_DEFAULT')
        return{'FINISHED'}

class MCP_OT_RescaleFCurves(bpy.types.Operator):
    bl_idname = "mcp.rescale_fcurves"
    bl_label = "Rescale FCurves"
    bl_options = {'UNDO'}

    def execute(self, context):
        try:
            scn = context.scene
            rescaleFCurves(context, context.object, scn.McpRescaleFactor)
        except MocapError:
            bpy.ops.mcp.error('INVOKE_DEFAULT')
        return{'FINISHED'}

#----------------------------------------------------------
#   Initialize
#----------------------------------------------------------

classes = [
    MCP_OT_SimplifyFCurves,
    MCP_OT_RescaleFCurves,
]

def initialize():
    for cls in classes:
        bpy.utils.register_class(cls)


def uninitialize():
    for cls in classes:
        bpy.utils.unregister_class(cls)
