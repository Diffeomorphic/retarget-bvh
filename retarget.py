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

#
#   M_b = global bone matrix, relative world (PoseBone.matrix)
#   L_b = local bone matrix, relative parent and rest (PoseBone.matrix_local)
#   R_b = bone rest matrix, relative armature (Bone.matrix_local)
#   T_b = global T-pose marix, relative world
#
#   M_b = M_p R_p^-1 R_b L_b
#   M_b = A_b M'_b
#   T_b = A_b T'_b
#   A_b = T_b T'^-1_b
#   B_b = R^-1_b R_p
#
#   L_b = R^-1_b R_p M^-1_p A_b M'_b
#   L_b = B_b M^-1_p A_b M'_b
#


import bpy
import mathutils
import time
import os
from collections import OrderedDict
from mathutils import *
from bpy.props import *
from bpy_extras.io_utils import ImportHelper

from .utils import *
from .target import Target
from .simplify import Simplifier, TimeScaler
from .load import BvhFile, MultiFile, BvhLoader, BvhRenamer
from .fkik import Bender


class CAnimation:

    def __init__(self, srcRig, trgRig, info, context):
        self.srcRig = srcRig
        self.trgRig = trgRig
        self.scene = context.scene
        self.boneAnims = OrderedDict()

        scn = context.scene
        for (trgName, srcName) in info.bones:
            if not scn.McpIncludeFingers and srcName[0:2] == "f_":
                continue
            elif (trgName in trgRig.pose.bones.keys() and
                srcName in srcRig.pose.bones.keys()):
                trgBone = trgRig.pose.bones[trgName]
                srcBone = srcRig.pose.bones[srcName]
            else:
                #print("  -", trgName, srcName)
                continue
            parent = self.getTargetParent(trgName, trgBone)
            self.boneAnims[trgName] = CBoneAnim(srcBone, trgBone, parent, self, context)


    def getTargetParent(self, trgName, trgBone):
        parName = trgBone.McpParent
        while (parName and parName not in self.boneAnims.keys()):
            print("Skipping", parName)
            parBone = self.trgRig.pose.bones[parName]
            parName = parBone.McpParent
        if parName:
            return self.boneAnims[parName]
        else:
            return None


    def printResult(self, scn, frame):
        scn.frame_set(frame)
        for name in ["LeftHip"]:
            banim = self.boneAnims[name]
            banim.printResult(frame)


    def putInTPoses(self, context):
        from .t_pose import putInTPose, putInRestPose
        scn = context.scene
        scn.frame_set(0)
        putInRestPose(self.srcRig, True)
        putInTPose(self.srcRig, scn.McpSourceTPose, context)
        putInRestPose(self.trgRig, True)
        putInTPose(self.trgRig, scn.McpTargetTPose, context)
        for banim in self.boneAnims.values():
            banim.getTPoseMatrix()


    def retarget(self, frames, context, offset, nFrames):
        objects = hideObjects(context, self.srcRig)
        scn = context.scene
        try:
            for n,frame in enumerate(frames):
                scn.frame_set(frame)
                showProgress(n+offset, frames[n], nFrames)
                for banim in self.boneAnims.values():
                    banim.retarget(frame)
        finally:
            unhideObjects(objects)


class CBoneAnim:

    def __init__(self, srcBone, trgBone, parent, anim, context):
        self.name = srcBone.name
        self.srcMatrices = {}
        self.trgMatrices = {}
        self.srcMatrix = None
        self.trgMatrix = None
        self.srcBone = srcBone
        self.trgBone = trgBone
        self.parent = parent
        self.order,self.locks = getLocks(trgBone, context)
        self.aMatrix = None
        if self.parent:
            self.bMatrix = trgBone.bone.matrix_local.inverted() @ self.parent.trgBone.bone.matrix_local
        else:
            self.bMatrix = trgBone.bone.matrix_local.inverted()
        self.useLimits = anim.scene.McpUseLimits


    def __repr__(self):
        return (
            "<CBoneAnim %s" % self.name +
            "  src %s" % self.srcBone.name +
            "  trg %s\n" % self.trgBone.name +
            "  A %s\n" % self.aMatrix +
            "  B %s\n" % self.bMatrix)


    def printResult(self, frame):
        print(
            "Retarget %s => %s\n" % (self.srcBone.name, self.trgBone.name) +
            "S %s\n" % self.srcBone.matrix +
            "T %s\n" % self.trgBone.matrix +
            "R %s\n" % self.trgBone.matrix @ self.srcBone.matrix.inverted()
            )


    def insertKeyFrame(self, mat, frame):
        pb = self.trgBone
        insertRotation(pb, mat, frame)
        if not self.parent:
            insertLocation(pb, mat, frame)


    def getTPoseMatrix(self):
        trgrot = self.trgBone.matrix.decompose()[1]
        trgmat = trgrot.to_matrix().to_4x4()
        srcrot = self.srcBone.matrix.decompose()[1]
        srcmat = srcrot.to_matrix().to_4x4()
        self.aMatrix = srcmat.inverted() @ trgmat


    def retarget(self, frame):
        self.srcMatrix = self.srcBone.matrix.copy()
        self.trgMatrix = self.srcMatrix @ self.aMatrix
        self.trgMatrix.col[3] = self.srcMatrix.col[3]
        if self.parent:
            mat1 = self.parent.trgMatrix.inverted() @ self.trgMatrix
        else:
            mat1 = self.trgMatrix
        mat2 = self.bMatrix @ mat1
        mat3 = correctMatrixForLocks(mat2, self.order, self.locks, self.trgBone, self.useLimits)
        self.insertKeyFrame(mat3, frame)

        self.srcMatrices[frame] = self.srcMatrix
        mat1 = self.bMatrix.inverted() @ mat3
        if self.parent:
            self.trgMatrix = self.parent.trgMatrix @ mat1
        else:
            self.trgMatrix = mat1
        self.trgMatrices[frame] = self.trgMatrix

        return

        if self.name == "upper_arm.L":
            print()
            print(self)
            print("S ", self.srcMatrix)
            print("T ", self.trgMatrix)
            print(self.parent.name)
            print("TP", self.parent.trgMatrix)
            print("M1", mat1)
            print("M2", mat2)
            print("MB2", self.trgBone.matrix)


def getLocks(pb, context):
    scn = context.scene
    locks = []
    order = 'XYZ'
    if scn.McpClearLocks:
        pb.lock_rotation[0] = pb.lock_rotation[2] = False
        for cns in pb.constraints:
            if cns.type == 'LIMIT_ROTATION':
                cns.use_limit_x = cns.use_limit_z = 0

    if pb.lock_rotation[1]:
        locks.append(1)
        order = 'YZX'
        if pb.lock_rotation[0]:
            order = 'YXZ'
            locks.append(0)
        if pb.lock_rotation[2]:
            locks.append(2)
    elif pb.lock_rotation[2]:
        locks.append(2)
        order = 'ZYX'
        if pb.lock_rotation[0]:
            order = 'ZXY'
            locks.append(0)
    elif pb.lock_rotation[0]:
        locks.append(0)
        order = 'XYZ'

    if pb.rotation_mode != 'QUATERNION':
        order = pb.rotation_mode

    return order,locks


def correctMatrixForLocks(mat, order, locks, pb, useLimits):
    head = Vector(mat.col[3])

    if locks:
        euler = mat.to_3x3().to_euler(order)
        for n in locks:
            euler[n] = 0
        mat = euler.to_matrix().to_4x4()

    if not useLimits:
        mat.col[3] = head
        return mat

    for cns in pb.constraints:
        if (cns.type == 'LIMIT_ROTATION' and
            cns.owner_space == 'LOCAL' and
            not cns.mute and
            cns.influence > 0.5):
            euler = mat.to_3x3().to_euler(order)
            if cns.use_limit_x:
                euler.x = min(cns.max_x, max(cns.min_x, euler.x))
            if cns.use_limit_y:
                euler.y = min(cns.max_y, max(cns.min_y, euler.y))
            if cns.use_limit_z:
                euler.z = min(cns.max_z, max(cns.min_z, euler.z))
            mat = euler.to_matrix().to_4x4()

    mat.col[3] = head
    return mat


def hideObjects(context, rig):
    if bpy.app.version >= (2,80,0):
        return None
    objects = []
    for ob in context.view_layer.objects:
        if ob != rig:
            objects.append((ob, list(ob.layers)))
            ob.layers = 20*[False]
    return objects


def unhideObjects(objects):
    if bpy.app.version >= (2,80,0):
        return
    for (ob,layers) in objects:
        ob.layers = layers


class Retargeter:

    def prequel(self, context):
        data = changeTargetData(context.object, context.scene)
        return (time.perf_counter(), data)


    def sequel(self, context, stuff):
        time1,data = stuff
        restoreTargetData(data)
        time2 = time.perf_counter()
        print("Retargeting finished in %.3f s" % (time2-time1))


    def retargetAnimation(self, context, srcRig, trgRig):
        from .source import setSourceArmature
        from .target import findTargetArmature
        from .fkik import setRigToFK
        from .loop import getActiveFrames

        startProgress("Retargeting %s => %s" % (srcRig.name, trgRig.name))
        if srcRig.type != 'ARMATURE':
            return None,0
        scn = context.scene
        frames = getActiveFrames(srcRig)
        nFrames = len(frames)
        setActiveObject(context, trgRig)
        if trgRig.animation_data:
            trgRig.animation_data.action = None
        setRigToFK(trgRig)

        if frames:
            scn.frame_current = frames[0]
        else:
            raise MocapError("No frames found.")
        oldData = changeTargetData(trgRig, scn)

        setSourceArmature(srcRig, scn)
        print("Retarget %s --> %s" % (srcRig.name, trgRig.name))

        info = findTargetArmature(context, trgRig, self.useAutoTarget)
        anim = CAnimation(srcRig, trgRig, info, context)
        anim.putInTPoses(context)

        frameBlock = frames[0:100]
        index = 0
        try:
            while frameBlock:
                anim.retarget(frameBlock, context, index, nFrames)
                index += 100
                frameBlock = frames[index:index+100]

            scn.frame_current = frames[0]
        finally:
            restoreTargetData(oldData)

        #anim.printResult(scn, 1)

        setInterpolation(trgRig)
        act = trgRig.animation_data.action
        act.name = trgRig.name[:4] + srcRig.name[2:]
        act.use_fake_user = True
        endProgress("Retargeted %s --> %s" % (srcRig.name, trgRig.name))
        return act,nFrames

#
#   changeTargetData(rig, scn):
#   restoreTargetData(data):
#

def changeTargetData(rig, scn):
    tempProps = [
        ("MhaRotationLimits", 0.0),
        ("MhaArmIk_L", 0.0),
        ("MhaArmIk_R", 0.0),
        ("MhaLegIk_L", 0.0),
        ("MhaLegIk_R", 0.0),
        ("MhaSpineIk", 0),
        ("MhaSpineInvert", 0),
        ("MhaElbowPlant_L", 0),
        ("MhaElbowPlant_R", 0),
        ]

    props = []
    for (key, value) in tempProps:
        try:
            props.append((key, rig[key]))
            rig[key] = value
        except KeyError:
            pass

    permProps = [
        ("MhaElbowFollowsShoulder", 0),
        ("MhaElbowFollowsWrist", 0),
        ("MhaKneeFollowsHip", 0),
        ("MhaKneeFollowsFoot", 0),
        ("MhaArmHinge", 0),
        ("MhaLegHinge", 0),
        ]

    for (key, value) in permProps:
        try:
            rig[key+"_L"]
            rig[key+"_L"] = value
            rig[key+"_R"] = value
        except KeyError:
            pass

    layers = list(rig.data.layers)
    if rig.MhAlpha8:
        rig.data.layers = MhxLayers
    elif isRigify(rig):
        rig.data.layers = RigifyLayers

    locks = []
    for pb in rig.pose.bones:
        constraints = []
        if not scn.McpUseLimits:
            for cns in pb.constraints:
                if cns.type == 'LIMIT_DISTANCE':
                    cns.mute = True
                elif cns.type[0:5] == 'LIMIT':
                    constraints.append( (cns, cns.mute) )
                    cns.mute = True
        locks.append( (pb, constraints) )

    norotBones = []
    return (rig, props, layers, locks, norotBones)


def restoreTargetData(data):
    (rig, props, layers, locks, norotBones) = data
    rig.data.layers = layers

    for (key,value) in props:
        rig[key] = value

    for b in norotBones:
        b.use_inherit_rotation = True

    for lock in locks:
        (pb, constraints) = lock
        for (cns, mute) in constraints:
            cns.mute = mute

########################################################################
#
#   Buttons
#

def ensureInited(scn):
    from .source import ensureSourceInited
    from .target import ensureTargetInited
    ensureSourceInited(scn)
    ensureTargetInited(scn)


def getOtherRig(context, rig):
    for ob in context.selected_objects:
        if ob != rig and ob.type == 'ARMATURE':
            return ob
    return None


class MCP_OT_RetargetRenamedToActive(HidePropsOperator, IsArmature, Target, Retargeter):
    bl_idname = "mcp.retarget_renamed_to_active"
    bl_label = "Retarget Renamed To Active"
    bl_description = "Retarget animation from the renamed source armature (selected) to the target (active) armature."
    bl_options = {'UNDO'}

    def run(self, context):
        from .load import checkObjectProblems
        checkObjectProblems(context)
        trgRig = context.object
        srcRig = getOtherRig(context, trgRig)
        if srcRig is None:
            raise MocapError("No source armature found")
        self.retargetAnimation(context, srcRig, trgRig)
        trgRig.select_set(True)
        context.view_layer.objects.active = trgRig

    def invoke(self, context, event):
        ensureInited(context.scene)
        return HidePropsOperator.invoke(self, context, event)


class MCP_OT_RetargetSelectedToActive(HidePropsOperator, IsArmature, BvhRenamer, Retargeter):
    bl_idname = "mcp.retarget_selected_to_active"
    bl_label = "Retarget Selected To Active"
    bl_description = "Retarget animation to the active (target) armature from the other selected (source) armatures"
    bl_options = {'UNDO'}

    def run(self, context):
        from .load import checkObjectProblems
        checkObjectProblems(context)
        trgRig = context.object
        srcRig = getOtherRig(context, trgRig)
        if srcRig is None:
            raise MocapError("No source armature found")
        bpy.ops.object.select_all(action='DESELECT')
        context.view_layer.objects.active = srcRig
        srcRig.select_set(True)
        bpy.ops.object.duplicate()
        tmpRig = context.object
        context.view_layer.objects.active = trgRig
        try:
            self.renameAndRescaleBvh(context, tmpRig, trgRig)
            bpy.ops.object.mode_set(mode='OBJECT')
            self.retargetAnimation(context, tmpRig, trgRig)
        finally:
            bpy.ops.object.mode_set(mode='OBJECT')
            bpy.ops.object.select_all(action='DESELECT')
            tmpRig.select_set(True)
            bpy.ops.object.delete()
            trgRig.select_set(True)
            context.view_layer.objects.active = trgRig

    def invoke(self, context, event):
        ensureInited(context.scene)
        return HidePropsOperator.invoke(self, context, event)


class MCP_OT_LoadAndRetarget(HideOperator, IsArmature, MultiFile, BvhFile, BvhLoader, BvhRenamer, Retargeter, TimeScaler, Simplifier, Bender):
    bl_idname = "mcp.load_and_retarget"
    bl_label = "Load And Retarget"
    bl_description = "Load animation from bvh file to the active armature"
    bl_options = {'UNDO'}

    useNLA : BoolProperty(
        name = "Create NLA Strips",
        description = "Create a NLA strip for each loaded action",
        default = False)

    def draw(self, context):
        BvhLoader.draw(self, context)
        BvhRenamer.draw(self, context)
        self.layout.prop(self, "useBendPositive")
        self.layout.separator()
        TimeScaler.draw(self, context)
        Simplifier.draw(self, context)
        self.layout.prop(self, "useNLA")


    def run(self, context):
        from .load import checkObjectProblems
        checkObjectProblems(context)
        rig = context.object
        infos = []
        for filepath in self.getFilePaths():
            print("---------------")
            info = self.retarget(context, filepath)
            infos.append(info)
        print("---------------")
        if self.useNLA:
            for act,size in infos:
                track = rig.animation_data.nla_tracks.new()
                track.name = act.name
                track.is_solo = True
                track.strips.new(act.name, 1, act)
            rig.animation_data.action = None
        rig.select_set(True)
        context.view_layer.objects.active = rig
        raise MocapMessage("BVH file(s) retargeted")


    def retarget(self, context, filepath):
        from .load import deleteSourceRig

        print("\n---------------\nLoad and retarget %s" % filepath)
        scn = context.scene
        trgRig = context.object
        srcRig = self.readBvhFile(context, filepath, scn, False)
        info = (None, 0)
        try:
            self.renameAndRescaleBvh(context, srcRig, trgRig)
            info = self.retargetAnimation(context, srcRig, trgRig)
            scn = context.scene
            if self.useBendPositive:
                self.useKnees = self.useElbows = True
                self.limbsBendPositive(trgRig, (0,1e6))
            if self.useSimplify:
                self.simplifyFCurves(context, trgRig)
            if self.useTimeScale:
                self.timescaleFCurves(trgRig)
        finally:
            deleteSourceRig(context, srcRig, 'Y_')
        return info


    def invoke(self, context, event):
        ensureInited(context.scene)
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}


class MCP_OT_ClearTempProps(BvhOperator):
    bl_idname = "mcp.clear_temp_props"
    bl_label = "Clear Temporary Properties"
    bl_description = "Clear properties used by BVH Retargeter. Animation editing may fail after this."
    bl_options = {'UNDO'}

    def run(self, context):
        rig = context.object
        for key in list(rig.keys()):
            if key[0:3] == "Mcp":
                del rig[key]
        for pb in rig.pose.bones:
            for key in list(pb.keys()):
                if key[0:3] == "Mcp":
                    del pb[key]

#----------------------------------------------------------
#   Initialize
#----------------------------------------------------------

classes = [
    MCP_OT_RetargetRenamedToActive,
    MCP_OT_RetargetSelectedToActive,
    MCP_OT_LoadAndRetarget,
    MCP_OT_ClearTempProps,
]

def initialize():
    bpy.types.Scene.McpUseLimits = BoolProperty(
        name="Use Limits",
        description="Restrict angles to Limit Rotation constraints",
        default=True)

    bpy.types.Scene.McpClearLocks = BoolProperty(
        name="Unlock Rotation",
        description="Clear X and Z rotation locks",
        default=False)

    bpy.types.Scene.McpIncludeFingers = BoolProperty(
        name = "Include Fingers",
        description = "Include finger bones",
        default = False)

    bpy.types.Object.MhAlpha8 = BoolProperty(default=False)

    for cls in classes:
        bpy.utils.register_class(cls)


def uninitialize():
    for cls in classes:
        bpy.utils.unregister_class(cls)
