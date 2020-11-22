# ------------------------------------------------------------------------------
#   BSD 2-Clause License
#
#   Copyright (c) 2019-2020, Thomas Larsson
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
from math import pi, sqrt
from mathutils import *

from .utils import *
from .simplify import FCurvesGetter

#
#   fCurveIdentity(fcu):
#

def fCurveIdentity(fcu):
    words = fcu.data_path.split('"')
    if len(words) < 2:
        return (None, None)
    name = words[1]
    words = fcu.data_path.split('.')
    mode = words[-1]
    return (name, mode)

#
#   Loop F-curves
#

class MCP_OT_LoopFCurves(BvhPropsOperator, IsArmature, FCurvesGetter):
    bl_idname = "mcp.loop_fcurves"
    bl_label = "Loop F-Curves"
    bl_description = "Make the beginning and end of the selected time range connect smoothly. Use before repeating."
    bl_options = {'UNDO'}

    blendRange : IntProperty(
        name="Blend Range",
        min=1,
        default=5)

    loopInPlace : BoolProperty(
        name="Loop In Place",
        description="Remove Location F-curves",
        default=False)

    deleteOutside : BoolProperty(
        name="Delete Outside Keyframes",
        description="Delete all keyframes outside the looped region",
        default = False)

    def draw(self, context):
        self.layout.prop(self, "blendRange")
        self.layout.prop(self, "loopInPlace")
        self.layout.prop(self, "deleteOutside")
        FCurvesGetter.draw(self, context)
        self.layout.separator()

    def run(self, context):
        startProgress("Loop F-curves")
        from .action import getObjectAction
        scn = context.scene
        rig = context.object
        act = getObjectAction(rig)
        if not act:
            return
        self.useMarkers = True
        (fcurves, minTime, maxTime) = self.getActionFCurves(act, rig, scn)
        if not fcurves:
            return

        frames = getActiveFrames(rig, minTime, maxTime)
        nFrames = len(frames)
        self.normalizeRotCurves(scn, rig, fcurves, frames)

        hasLocation = {}
        for n,fcu in enumerate(fcurves):
            (name, mode) = fCurveIdentity(fcu)
            if isRotation(mode):
                self.loopFCurve(fcu, minTime, maxTime, scn)

        if self.loopInPlace:
            iknames = [pb.name for pb in self.getIkBoneList(rig)]
            ikbones = {}
            for fcu in fcurves:
                (name, mode) = fCurveIdentity(fcu)
                if isLocation(mode) and name in iknames:
                    ikbones[name] = rig.pose.bones[name]

            for pb in ikbones.values():
                print("IK bone %s" % pb.name)
                scn.frame_set(minTime)
                head0 = pb.head.copy()
                scn.frame_set(maxTime)
                head1 = pb.head.copy()
                offs = (head1-head0)/(maxTime-minTime)

                restMat = pb.bone.matrix_local.to_3x3()
                restInv = restMat.inverted()

                heads = {}
                for n,frame in enumerate(frames):
                    scn.frame_set(frame)
                    showProgress(n, frame, nFrames)
                    heads[frame] = pb.head.copy()

                for n,frame in enumerate(frames):
                    showProgress(n, frame, nFrames)
                    scn.frame_set(frame)
                    head = heads[frame] - (frame-minTime)*offs
                    diff = head - pb.bone.head_local
                    pb.location = restInv @ diff
                    pb.keyframe_insert("location", group=pb.name)

        if self.deleteOutside:
            for fcu in fcurves:
                kpts = list(fcu.keyframe_points)
                kpts.reverse()
                for kp in kpts:
                    t = kp.co[0]
                    if t < minTime or t > maxTime:
                        fcu.keyframe_points.remove(kp)

        raise MocapMessage("F-curves looped")


    def loopFCurve(self, fcu, t0, tn, scn):
        from .simplify import getFCurveLimits
        delta = self.blendRange

        v0 = fcu.evaluate(t0)
        vn = fcu.evaluate(tn)
        fcu.keyframe_points.insert(frame=t0, value=v0)
        fcu.keyframe_points.insert(frame=tn, value=vn)
        (mode, upper, lower, diff) = getFCurveLimits(fcu)
        if mode == 'location':
            dv = vn-v0
        else:
            dv = 0.0

        newpoints = []
        for dt in range(delta):
            eps = 0.5*(1-dt/delta)

            t1 = t0+dt
            v1 = fcu.evaluate(t1)
            tm = tn+dt
            vm = fcu.evaluate(tm) - dv
            if (v1 > upper) and (vm < lower):
                vm += diff
            elif (v1 < lower) and (vm > upper):
                vm -= diff
            pt1 = (t1, (eps*vm + (1-eps)*v1))

            t1 = t0-dt
            v1 = fcu.evaluate(t1) + dv
            tm = tn-dt
            vm = fcu.evaluate(tm)
            if (v1 > upper) and (vm < lower):
                v1 -= diff
            elif (v1 < lower) and (vm > upper):
                v1 += diff
            ptm = (tm, eps*v1 + (1-eps)*vm)

            newpoints.extend([pt1,ptm])

        newpoints.sort()
        for (t,v) in newpoints:
            fcu.keyframe_points.insert(frame=t, value=v)


    def normalizeRotCurves(self, scn, rig, fcurves, frames):
        hasQuat = {}
        for fcu in fcurves:
            (name, mode) = fCurveIdentity(fcu)
            if mode == 'rotation_quaternion':
                hasQuat[name] = rig.pose.bones[name]

        nFrames = len(frames)
        for n,frame in enumerate(frames):
            scn.frame_set(frame)
            showProgress(n, frame, nFrames)
            for (name, pb) in hasQuat.items():
                pb.rotation_quaternion.normalize()
                pb.keyframe_insert("rotation_quaternion", group=name)


    def getIkBoneList(self, rig):
        hips = getTrgBone('hips', rig)
        if hips is None:
            if isMhxRig(rig):
                hips = rig.pose.bones["root"]
            elif isRigify(rig):
                hips = rig.pose.bones["hips"]
            elif isRigify2(rig):
                hips = rig.pose.bones["torso"]
            else:
                for bone in rig.data.bones:
                    if bone.parent is None:
                        hips = bone
                        break
        blist = [hips]
        for bname in ['hand.ik.L', 'hand.ik.R', 'foot.ik.L', 'foot.ik.R']:
            try:
                blist.append(rig.pose.bones[bname])
            except KeyError:
                pass
        return blist



#
#   repeatFCurves(context, nRepeats):
#

class MCP_OT_RepeatFCurves(BvhPropsOperator, IsArmature, FCurvesGetter):
    bl_idname = "mcp.repeat_fcurves"
    bl_label = "Repeat Animation"
    bl_description = "Repeat the part of the animation between selected markers n times"
    bl_options = {'UNDO'}

    repeatNumber : IntProperty(
        name="Repeat Number",
        min=1,
        default=1)

    def draw(self, context):
        self.layout.prop(self, "repeatNumber")
        FCurvesGetter.draw(self, context)
        self.layout.separator()

    def run(self, context):
        from .action import getObjectAction
        startProgress("Repeat F-curves %d times" % self.repeatNumber)
        act = getObjectAction(context.object)
        if not act:
            return
        self.useMarkers = True
        (fcurves, minTime, maxTime) = self.getActionFCurves(act, context.object, context.scene)
        if not fcurves:
            return

        dt0 = maxTime-minTime
        for fcu in fcurves:
            (name, mode) = fCurveIdentity(fcu)
            dy0 = fcu.evaluate(maxTime) - fcu.evaluate(minTime)
            points = []
            for kp in fcu.keyframe_points:
                t = kp.co[0]
                if t >= minTime and t < maxTime:
                    points.append((t, kp.co[1]))
            for n in range(1, self.repeatNumber):
                dt = n*dt0
                dy = n*dy0
                for (t,y) in points:
                    fcu.keyframe_points.insert(t+dt, y+dy, options={'FAST'})

        raise MocapMessage("F-curves repeated %d times" % self.repeatNumber)


#
#   stitchActions(context):
#

def getActionItems(self, context):
    return [(act.name, act.name, act.name) for act in bpy.data.actions]


class MCP_OT_StitchActions(BvhPropsOperator, IsArmature):
    bl_idname = "mcp.stitch_actions"
    bl_label = "Stitch Actions"
    bl_description = "Stitch two action together seamlessly"
    bl_options = {'UNDO'}

    blendRange : IntProperty(
        name="Blend Range",
        min=1,
        default=5)

    firstAction : EnumProperty(
        items = getActionItems,
        name = "First Action")

    secondAction : EnumProperty(
        items = getActionItems,
        name = "Second Action")

    firstEndFrame : IntProperty(
        name="First End Frame",
        default=1)

    secondStartFrame : IntProperty(
        name="Second Start Frame",
        default=1)

    actionTarget : EnumProperty(
        items = [('Stitch new', 'Stitch new', 'Stitch new'),
                 ('Prepend second', 'Prepend second', 'Prepend second')],
        name = "Action Target")

    outputActionName : StringProperty(
        name="Output Action Name",
        maxlen=24,
        default="Stitched")


    def run(self, context):
        from .retarget import getLocks, correctMatrixForLocks

        startProgress("Stitch actions")
        scn = context.scene
        rig = context.object
        act1 = bpy.data.actions[self.firstAction]
        act2 = bpy.data.actions[self.secondAction]
        frame1 = self.firstEndFrame
        frame2 = self.secondStartFrame
        delta = self.blendRange
        factor = 1.0/delta
        shift = frame1 - frame2 - delta

        if rig.animation_data:
            rig.animation_data.action = None

        first1,last1 = self.getActionExtent(act1)
        first2,last2 = self.getActionExtent(act2)
        frames1 = range(first1, frame1)
        frames2 = range(frame2, last2+1)
        frames = range(first1, last2+shift+1)
        bmats1,_ = getBaseMatrices(act1, frames1, rig, True)
        bmats2,useLoc = getBaseMatrices(act2, frames2, rig, True)

        deletes = []
        for bname in bmats2.keys():
            try:
                bmats1[bname]
            except KeyError:
                deletes.append(bname)
        for bname in deletes:
            del bmats2[bname]

        orders = {}
        locks = {}
        for bname in bmats2.keys():
            pb = rig.pose.bones[bname]
            orders[bname],locks[bname] = getLocks(pb, context)

        nFrames = len(frames)
        for n,frame in enumerate(frames):
            scn.frame_set(frame)
            showProgress(n, frame, nFrames)

            if frame <= frame1-delta:
                n1 = frame - first1
                for bname,mats in bmats1.items():
                    pb = rig.pose.bones[bname]
                    mat = mats[n1]
                    if useLoc[bname]:
                        insertLocation(pb, mat)
                    insertRotation(pb, mat)

            elif frame >= frame1:
                n2 = frame - frame1
                for bname,mats in bmats2.items():
                    pb = rig.pose.bones[bname]
                    mat = mats[n2]
                    if useLoc[bname]:
                        insertLocation(pb, mat)
                    insertRotation(pb, mat)

            else:
                n1 = frame - first1
                n2 = frame - frame1 + delta
                eps = factor*n2
                for bname,mats2 in bmats2.items():
                    pb = rig.pose.bones[bname]
                    mats1 = bmats1[bname]
                    mat1 = mats1[n1]
                    mat2 = mats2[n2]
                    mat = (1-eps)*mat1 + eps*mat2
                    mat = correctMatrixForLocks(mat, orders[bname], locks[bname], pb, scn.McpUseLimits)
                    if useLoc[bname]:
                        insertLocation(pb, mat)
                    insertRotation(pb, mat)

        setInterpolation(rig)
        act = rig.animation_data.action
        act.name = self.outputActionName
        raise MocapMessage("Actions stitched")


    def getActionExtent(self, act):
        first = 10000
        last = -10000
        for fcu in act.fcurves:
            t0 = int(fcu.keyframe_points[0].co[0])
            t1 = int(fcu.keyframe_points[-1].co[0])
            if t0 < first:
                first = t0
            if t1 > last:
                last = t1
        return first,last


#
#   shiftBoneFCurves(rig, context):
#   class MCP_OT_ShiftBoneFCurves(HideOperator):
#

def getBaseMatrices(act, frames, rig, useAll):
    locFcurves = {}
    quatFcurves = {}
    eulerFcurves = {}
    for fcu in act.fcurves:
        (bname, mode) = fCurveIdentity(fcu)
        if bname in rig.pose.bones.keys():
            pb = rig.pose.bones[bname]
        else:
            continue
        if useAll or pb.bone.select:
            if mode == "location":
                try:
                    fcurves = locFcurves[bname]
                except KeyError:
                    fcurves = locFcurves[bname] = [None,None,None]
            elif mode == "rotation_euler":
                try:
                    fcurves = eulerFcurves[bname]
                except KeyError:
                    fcurves = eulerFcurves[bname] = [None,None,None]
            elif mode == "rotation_quaternion":
                try:
                    fcurves = quatFcurves[bname]
                except KeyError:
                    fcurves = quatFcurves[bname] = [None,None,None,None]
            else:
                continue

            fcurves[fcu.array_index] = fcu

    basemats = {}
    useLoc = {}
    for bname,fcurves in eulerFcurves.items():
        useLoc[bname] = False
        order = rig.pose.bones[bname].rotation_mode
        fcu0,fcu1,fcu2 = fcurves
        rmats = basemats[bname] = []
        for frame in frames:
            euler = Euler((fcu0.evaluate(frame), fcu1.evaluate(frame), fcu2.evaluate(frame)), order)
            rmats.append(euler.to_matrix().to_4x4())

    for bname,fcurves in quatFcurves.items():
        useLoc[bname] = False
        fcu0,fcu1,fcu2,fcu3 = fcurves
        rmats = basemats[bname] = []
        for frame in frames:
            quat = Quaternion((fcu0.evaluate(frame), fcu1.evaluate(frame), fcu2.evaluate(frame), fcu3.evaluate(frame)))
            rmats.append(quat.to_matrix().to_4x4())

    for bname,fcurves in locFcurves.items():
        useLoc[bname] = True
        fcu0,fcu1,fcu2 = fcurves
        tmats = []
        for frame in frames:
            loc = (fcu0.evaluate(frame), fcu1.evaluate(frame), fcu2.evaluate(frame))
            tmats.append(Matrix.Translation(loc))
        try:
            rmats = basemats[bname]
        except KeyError:
            basemats[bname] = tmats
            rmats = None
        if rmats:
            mats = []
            for n,rmat in enumerate(rmats):
                tmat = tmats[n]
                mats.append( tmat @ rmat )
            basemats[bname] = mats

    return basemats, useLoc


def printmat(mat):
    print("   (%.4f %.4f %.4f %.4f)" % tuple(mat.to_quaternion()))


class MCP_OT_ShiftBoneFCurves(HideOperator, IsArmature):
    bl_idname = "mcp.shift_animation"
    bl_label = "Shift Animation"
    bl_description = "Shift the animation globally for selected boens"
    bl_options = {'UNDO'}

    def run(self, context):
        from .action import getObjectAction
        from .retarget import getLocks, correctMatrixForLocks

        startProgress("Shift animation")
        scn = context.scene
        rig = context.object
        frames = [scn.frame_current] + getActiveFrames(rig)
        nFrames = len(frames)
        act = getObjectAction(rig)
        if not act:
            return
        basemats, useLoc = getBaseMatrices(act, frames, rig, False)

        deltaMat = {}
        orders = {}
        locks = {}
        for bname,bmats in basemats.items():
            pb = rig.pose.bones[bname]
            bmat = bmats[0]
            deltaMat[pb.name] = pb.matrix_basis @ bmat.inverted()
            orders[pb.name], locks[pb.name] = getLocks(pb, context)

        objects = detachRig(rig)      # speed-up, exclude modifier from meshes
        for n,frame in enumerate(frames[1:]):
            scn.frame_set(frame)
            showProgress(n, frame, nFrames)
            for bname,bmats in basemats.items():
                pb = rig.pose.bones[bname]
                mat = deltaMat[pb.name] @ bmats[n+1]
                mat = correctMatrixForLocks(mat, orders[bname], locks[bname], pb, scn.McpUseLimits)
                if useLoc[bname]:
                    insertLocation(pb, mat)
                insertRotation(pb, mat)

        attachRig(rig, objects)   # include it again
        raise MocapMessage("Animation shifted")


class MCP_OT_FixateBoneFCurves(HideOperator, IsArmature):
    bl_idname = "mcp.fixate_bone"
    bl_label = "Fixate Bone Location"
    bl_description = "Keep bone location fixed (local coordinates)"
    bl_options = {'UNDO'}

    fixX : BoolProperty(
        name="X",
        description="Fix Local X Location",
        default=True)

    fixY : BoolProperty(
        name="Y",
        description="Fix Local Y Location",
        default=True)

    fixZ : BoolProperty(
        name="Z",
        description="Fix Local Z Location",
        default=True)


    def invoke(self, context, event):
        wm = context.window_manager
        return wm.invoke_props_dialog(self)


    def draw(self, context):
        row = self.layout.row()
        row.prop(self, "fixX")
        row.prop(self, "fixY")
        row.prop(self, "fixZ")


    def run(self, context):
        from .action import getObjectAction
        startProgress("Fixate bone locations")
        rig = context.object
        scn = context.scene
        act = getObjectAction(rig)
        if not act:
            return
        frame = scn.frame_current
        minTime,maxTime = getMarkedTime(scn)
        if minTime is None:
            minTime = -1e6
        if maxTime is None:
            maxTime = 1e6
        fixArray = [False,False,False]
        if self.fixX:
            fixArray[0] = True
        if self.fixY:
            fixArray[1] = True
        if self.fixZ:
            fixArray[2] = True

        for fcu in act.fcurves:
            (bname, mode) = fCurveIdentity(fcu)
            pb = rig.pose.bones[bname]
            if pb.bone.select and isLocation(mode) and fixArray[fcu.array_index]:
                value = fcu.evaluate(frame)
                for kp in fcu.keyframe_points:
                    if kp.co[0] >= minTime and kp.co[0] <= maxTime:
                        kp.co[1] = value
        raise MocapMessage("Bone locations fixated")

#----------------------------------------------------------
#   Get active frames
#----------------------------------------------------------

def getActiveFrames0(ob):
    active = {}
    if ob.animation_data is None:
        return active
    action = ob.animation_data.action
    if action is None:
        return active
    for fcu in action.fcurves:
        for kp in fcu.keyframe_points:
            active[kp.co[0]] = True
    return active


def getActiveFrames(ob, minTime=None, maxTime=None):
    active = getActiveFrames0(ob)
    frames = list(active.keys())
    frames.sort()
    if minTime is not None:
        while frames[0] < minTime:
            frames = frames[1:]
    if maxTime is not None:
        frames.reverse()
        while frames[0] > maxTime:
            frames = frames[1:]
        frames.reverse()
    return frames


def getActiveFramesBetweenMarkers(ob, scn):
    minTime,maxTime = getMarkedTime(scn)
    if minTime is None:
        return getActiveFrames(ob)
    active = getActiveFrames0(ob)
    frames = []
    for time in active.keys():
        if time >= minTime and time <= maxTime:
            frames.append(time)
    frames.sort()
    return frames


def getMarkedTime(scn):
    markers = []
    for mrk in scn.timeline_markers:
        if mrk.select:
            markers.append(mrk.frame)
    markers.sort()
    if len(markers) >= 2:
        return (markers[0], markers[-1])
    else:
        return (None, None)

#----------------------------------------------------------
#   Initialize
#----------------------------------------------------------

classes = [
    MCP_OT_LoopFCurves,
    MCP_OT_RepeatFCurves,
    MCP_OT_StitchActions,
    MCP_OT_ShiftBoneFCurves,
    MCP_OT_FixateBoneFCurves,
]

def initialize():
    for cls in classes:
        bpy.utils.register_class(cls)


def uninitialize():
    for cls in classes:
        bpy.utils.unregister_class(cls)
