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
from bpy.props import *
from bpy_extras.io_utils import ImportHelper, ExportHelper

import os
from math import sqrt, pi
from mathutils import Quaternion, Matrix
from .utils import *

#------------------------------------------------------------------
#   Classes
#------------------------------------------------------------------

class JsonFile:
    filename_ext = ".json"
    filter_glob : StringProperty(default="*.json", options={'HIDDEN'})
    filepath : StringProperty(name="File Path", description="Filepath to json file", maxlen=1024, default="")


class Rigger:
    autoRig : BoolProperty(
        name = "Auto Rig",
        description = "Find rig automatically",
        default = True)

    def draw(self, context):
        self.layout.prop(self, "autoRig")
        if not self.autoRig:
            scn = context.scene
            rig = context.object
            if self.isSourceRig:
                self.layout.prop(scn, "McpSourceRig")
                self.layout.prop(scn, "McpSourceTPose")
            else:
                self.layout.prop(scn, "McpTargetRig")
                self.layout.prop(scn, "McpTargetTPose")

    def initRig(self, context):
        from .target import findTargetArmature
        from .source import findSourceArmature
        from .fkik import setRigifyFKIK, setRigify2FKIK

        rig = context.object
        pose = [(pb, pb.matrix_basis.copy()) for pb in rig.pose.bones]

        if self.isSourceRig:
            findSourceArmature(context, rig, self.autoRig)
        else:
            findTargetArmature(context, rig, self.autoRig)

        for pb,mat in pose:
            pb.matrix_basis = mat

        if isRigify(rig):
            setRigifyFKIK(rig, 0.0)
        elif isRigify2(rig):
            setRigify2FKIK(rig, 1.0)

        return rig

#------------------------------------------------------------------
#   Define current pose as rest pose
#------------------------------------------------------------------

class MCP_OT_RestCurrentPose(BvhOperator, IsArmature):
    bl_idname = "mcp.rest_current_pose"
    bl_label = "Current Pose => Rest Pose"
    bl_description = "Change rest pose to current pose"
    bl_options = {'UNDO'}

    def run(self, context):
        rig = context.object
        children = []
        for ob in context.view_layer.objects:
            if ob.type != 'MESH':
                continue

            setActiveObject(context, ob)
            if ob != context.object:
                raise MocapError("Context switch did not take:\nob = %s\nc.ob = %s\nc.aob = %s" %
                    (ob, context.object, context.active_object))

            if (ob.McpArmatureName == rig.name and
                ob.McpArmatureModifier != ""):
                mod = ob.modifiers[ob.McpArmatureModifier]
                ob.modifiers.remove(mod)
                ob.data.shape_keys.key_blocks[ob.McpArmatureModifier].value = 1.0
                children.append(ob)
            else:
                for mod in ob.modifiers:
                    if (mod.type == 'ARMATURE' and
                        mod.object == rig):
                        children.append(ob)
                        bpy.ops.object.modifier_apply(apply_as='SHAPE', modifier=mod.name)
                        ob.data.shape_keys.key_blocks[mod.name].value = 1.0
                        ob.McpArmatureName = rig.name
                        ob.McpArmatureModifier = mod.name
                        break

        setActiveObject(context, rig)
        bpy.ops.object.mode_set(mode='POSE')
        try:
            bpy.ops.pose.armature_apply()
        except RuntimeError as err:
            raise MocapError("Error when applying armature:   \n%s" % err)

        for pb in rig.pose.bones:
            pb.McpQuat = (1,0,0,0)

        bpy.ops.object.mode_set(mode='OBJECT')
        for ob in children:
            name = ob.McpArmatureModifier
            setActiveObject(context, ob)
            mod = ob.modifiers.new(name, 'ARMATURE')
            mod.object = rig
            mod.use_vertex_groups = True
            bpy.ops.object.modifier_move_up(modifier=name)
            if False and ob.data.shape_keys:
                skey = ob.data.shape_keys.key_blocks[name]
                skey.value = 1.0

        setActiveObject(context, rig)
        raise MocapMessage("Applied pose as rest pose")

#------------------------------------------------------------------
#   Automatic T-Pose
#------------------------------------------------------------------

TPose = {
    "shoulder.L" : (0, 0, -90*D, 'XYZ'),
    "upper_arm.L" : (0, 0, -90*D, 'XYZ'),
    "forearm.L" :   (0, 0, -90*D, 'XYZ'),
    "hand.L" :      (0, 0, -90*D, 'XYZ'),

    "shoulder.R" : (0, 0, 90*D, 'XYZ'),
    "upper_arm.R" : (0, 0, 90*D, 'XYZ'),
    "forearm.R" :   (0, 0, 90*D, 'XYZ'),
    "hand.R" :      (0, 0, 90*D, 'XYZ'),

    "thigh.L" :     (-90*D, 0, 0, 'XYZ'),
    "shin.L" :      (-90*D, 0, 0, 'XYZ'),
    #"foot.L" :      (None, 0, 0, 'XYZ'),
    #"toe.L" :       (pi, 0, 0, 'XYZ'),

    "thigh.R" :     (-90*D, 0, 0, 'XYZ'),
    "shin.R" :      (-90*D, 0, 0, 'XYZ'),
    #"foot.R" :      (None, 0, 0, 'XYZ'),
    #"toe.R" :       (pi, 0, 0, 'XYZ'),

    "f_carpal1.L": (0, 0, -105*D, 'XYZ'),
    "f_carpal2.L": (0, 0, -90*D, 'XYZ'),
    "f_carpal3.L": (0, 0, -75*D, 'XYZ'),
    "f_carpal4.L": (0, 0, -60*D, 'XYZ'),

    "f_thumb.01.L": (0, 0, -120*D, 'XYZ'),
    "f_thumb.02.L": (0, 0, -120*D, 'XYZ'),
    "f_thumb.03.L": (0, 0, -120*D, 'XYZ'),
    "f_index.01.L": (0, 0, -105*D, 'XYZ'),
    "f_index.02.L": (0, 0, -105*D, 'XYZ'),
    "f_index.03.L": (0, 0, -105*D, 'XYZ'),
    "f_middle.01.L": (0, 0, -90*D, 'XYZ'),
    "f_middle.02.L": (0, 0, -90*D, 'XYZ'),
    "f_middle.03.L": (0, 0, -90*D, 'XYZ'),
    "f_ring.01.L": (0, 0, -75*D, 'XYZ'),
    "f_ring.02.L": (0, 0, -75*D, 'XYZ'),
    "f_ring.03.L": (0, 0, -75*D, 'XYZ'),
    "f_pinky.01.L": (0, 0, -60*D, 'XYZ'),
    "f_pinky.02.L": (0, 0, -60*D, 'XYZ'),
    "f_pinky.03.L": (0, 0, -60*D, 'XYZ'),

    "f_carpal1.R": (0, 0, 105*D, 'XYZ'),
    "f_carpal2.R": (0, 0, 90*D, 'XYZ'),
    "f_carpal3.R": (0, 0, 75*D, 'XYZ'),
    "f_carpal4.R": (0, 0, 60*D, 'XYZ'),

    "f_thumb.01.R": (0, 0, 120*D, 'XYZ'),
    "f_thumb.02.R": (0, 0, 120*D, 'XYZ'),
    "f_thumb.03.R": (0, 0, 120*D, 'XYZ'),
    "f_index.01.R": (0, 0, 105*D, 'XYZ'),
    "f_index.02.R": (0, 0, 105*D, 'XYZ'),
    "f_index.03.R": (0, 0, 105*D, 'XYZ'),
    "f_middle.01.R": (0, 0, 90*D, 'XYZ'),
    "f_middle.02.R": (0, 0, 90*D, 'XYZ'),
    "f_middle.03.R": (0, 0, 90*D, 'XYZ'),
    "f_ring.01.R": (0, 0, 75*D, 'XYZ'),
    "f_ring.02.R": (0, 0, 75*D, 'XYZ'),
    "f_ring.03.R": (0, 0, 75*D, 'XYZ'),
    "f_pinky.01.R": (0, 0, 60*D, 'XYZ'),
    "f_pinky.02.R": (0, 0, 60*D, 'XYZ'),
    "f_pinky.03.R": (0, 0, 60*D, 'XYZ'),

}

def autoTPose(rig, context):
    print("Auto T-pose", rig.name)
    scn = context.scene
    putInRestPose(rig, True)
    for pb in rig.pose.bones:
        if pb.McpBone[0:2] == "f_" and not scn.McpIncludeFingers:
            continue
        if pb.McpBone in TPose.keys():
            ex,ey,ez,order = TPose[pb.McpBone]
        else:
            continue

        euler = pb.matrix.to_euler(order)
        if ex is None:
            ex = euler.x
        if ey is None:
            ey = euler.y
        if ez is None:
            ez = euler.z
        euler = Euler((ex,ey,ez), order)
        mat = euler.to_matrix().to_4x4()
        mat.col[3] = pb.matrix.col[3]

        loc = pb.bone.matrix_local
        if pb.parent:
            mat = pb.parent.matrix.inverted() @ mat
            loc = pb.parent.bone.matrix_local.inverted() @ loc
        mat =  loc.inverted() @ mat
        euler = mat.to_euler('YZX')
        euler.y = 0
        pb.matrix_basis = euler.to_matrix().to_4x4()
        updateScene()
        setKeys(pb)

#------------------------------------------------------------------
#   Put in rest and T pose
#------------------------------------------------------------------

def putInRestPose(rig, useSetKeys):
    for pb in rig.pose.bones:
        pb.matrix_basis = Matrix()
        if useSetKeys:
            setKeys(pb)
    updateScene()


def putInRightPose(rig, tpose, context):
    if tpose != "Default":
        tinfo = getTPoseInfo(tpose)
        if tinfo:
            tinfo.addTPose(rig)
            putInTPose(rig, tpose, context)
            return True
    else:
        putInRestPose(rig, True)
    return False


def getStoredTPose(rig, useSetKeys):
    for pb in rig.pose.bones:
        quat = Quaternion(pb.McpQuat)
        pb.matrix_basis = quat.to_matrix().to_4x4()
        if useSetKeys:
            setKeys(pb)
    updateScene()


def setKeys(pb):
    if pb.rotation_mode == "QUATERNION":
        pb.keyframe_insert("rotation_quaternion", group=pb.name)
    elif pb.rotation_mode == "AXIS_ANGLE":
        pb.keyframe_insert("rotation_axis_angle", group=pb.name)
    else:
        pb.keyframe_insert("rotation_euler", group=pb.name)


def putInTPose(rig, name, context):
    scn = context.scene
    if False and rig.McpTPoseDefined:
        getStoredTPose(rig, True)
    elif name == "Default":
        autoTPose(rig, context)
        print("Put %s in automatic T-pose" % (rig.name))
    else:
        info = getTPoseInfo(name)
        if info is None:
            raise MocapError("T-pose %s not found" % name)
        info.addTPose(rig)
        getStoredTPose(rig, True)
        print("Put %s in T-pose %s" % (rig.name, name))
    updateScene()


class MCP_OT_PutInSrcTPose(BvhPropsOperator, IsArmature, Rigger):
    bl_idname = "mcp.put_in_src_t_pose"
    bl_label = "Put In T-pose (Source)"
    bl_description = "Put the character into T-pose"
    bl_options = {'UNDO'}

    isSourceRig = True

    def run(self, context):
        rig = self.initRig(context)
        putInTPose(rig, context.scene.McpSourceTPose, context)
        print("Pose set to source T-pose")

    def invoke(self, context, event):
        from .source import ensureSourceInited
        ensureSourceInited(context.scene)
        return BvhPropsOperator.invoke(self, context, event)


class MCP_OT_PutInTrgTPose(BvhPropsOperator, IsArmature, Rigger):
    bl_idname = "mcp.put_in_trg_t_pose"
    bl_label = "Put In T-pose (Target)"
    bl_description = "Put the character into T-pose"
    bl_options = {'UNDO'}

    isSourceRig = False

    def run(self, context):
        rig = self.initRig(context)
        putInTPose(rig, context.scene.McpTargetTPose, context)
        print("Pose set to target T-pose")

    def invoke(self, context, event):
        from .target import ensureTargetInited
        ensureTargetInited(context.scene)
        return BvhPropsOperator.invoke(self, context, event)

#------------------------------------------------------------------
#   Define and undefine T-Pose
#------------------------------------------------------------------

class MCP_OT_DefineTPose(BvhOperator, IsArmature):
    bl_idname = "mcp.define_t_pose"
    bl_label = "Define T-pose"
    bl_description = "Define T-pose as current pose"
    bl_options = {'UNDO'}

    def run(self, context):
        rig = context.object
        for pb in rig.pose.bones:
            pb.McpQuat = pb.matrix_basis.to_quaternion()
        rig.McpTPoseDefined = True
        print("T-pose defined as current pose")


class MCP_OT_UndefineTPose(BvhOperator, IsArmature):
    bl_idname = "mcp.undefine_t_pose"
    bl_label = "Undefine T-pose"
    bl_description = "Remove definition of T-pose"
    bl_options = {'UNDO'}

    def run(self, context):
        rig = context.object
        rig.McpTPoseDefined = False
        quat = Quaternion()
        for pb in rig.pose.bones:
            pb.McpQuat = quat
        print("Undefined T-pose")

#------------------------------------------------------------------
#   Load T-pose from file
#------------------------------------------------------------------

def getBoneName(rig, name):
    if rig.McpIsSourceRig:
        return name
    else:
        pb = getTrgBone(name, rig)
        if pb:
            return pb.name
        else:
            return ""


class MCP_OT_LoadTPose(BvhOperator, IsArmature, ExportHelper, JsonFile):
    bl_idname = "mcp.load_t_pose"
    bl_label = "Load T-Pose"
    bl_description = "Load T-pose from file"
    bl_options = {'UNDO'}

    isSourceRig = True

    def run(self, context):
        from .io_json import loadJson
        rig = context.object
        print("Loading %s" % self.filepath)
        struct = loadJson(self.filepath)
        rig.McpTPoseFile = self.filepath
        if "t-pose" in struct.keys():
            self.setTPose(rig, struct["t-pose"])
        else:
            raise MocapError("File does not define a T-pose:\n%s" % self.filepath)

    def setTPose(self, rig, struct):
        putInRestPose(rig, True)
        for bname,value in struct.items():
            if bname in rig.pose.bones.keys():
                pb = rig.pose.bones[bname]
                euler = Euler(Vector(value)*D)
                quat = pb.McpQuat = euler.to_quaternion()
                pb.matrix_basis = quat.to_matrix().to_4x4()
                setKeys(pb)

    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}

#------------------------------------------------------------------
#   Save current pose to file
#------------------------------------------------------------------

class MCP_OT_SaveTPose(BvhOperator, IsArmature, ExportHelper, JsonFile):
    bl_idname = "mcp.save_t_pose"
    bl_label = "Save T-Pose"
    bl_description = "Save current pose as .json file"
    bl_options = {'UNDO'}

    onlyMcpBones : BoolProperty(
        name = "Only Mcp Bones",
        default = False,
    )

    def draw(self, context):
        self.layout.prop(self, "onlyMcpBones")

    def run(self, context):
        from collections import OrderedDict
        from .io_json import saveJson
        rig = context.object
        tstruct = OrderedDict()
        struct = OrderedDict()
        fname = os.path.splitext(os.path.basename(self.filepath))[0]
        words = [word.capitalize() for word in fname.split("_")]
        struct["name"] = " ".join(words)
        struct["t-pose"] = tstruct
        for pb in rig.pose.bones:
            bmat = pb.matrix
            rmat = pb.bone.matrix_local
            if pb.parent:
                bmat = pb.parent.matrix.inverted() @ bmat
                rmat = pb.parent.bone.matrix_local.inverted() @ rmat
            mat = rmat.inverted() @ bmat
            q = mat.to_quaternion()
            magn = math.sqrt( (q.w-1)*(q.w-1) + q.x*q.x + q.y*q.y + q.z*q.z )
            if magn > -1e-4:
                if pb.McpBone or not self.onlyMcpBones:
                    euler = Vector(mat.to_euler())/D
                    tstruct[pb.name] = [int(round(ex)) for ex in euler]

        if os.path.splitext(self.filepath)[-1] != ".json":
            filepath = self.filepath + ".json"
        else:
            filepath = self.filepath
        filepath = os.path.join(os.path.dirname(__file__), filepath)
        print("Saving %s" % filepath)
        saveJson(struct, filepath)
        print("Saved current pose")

    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}

#----------------------------------------------------------
#   Global T-pose
#----------------------------------------------------------

from .source import CRigInfo

class CTPoseInfo(CRigInfo):
    verboseString = "Read T-pose file"

_tposeInfos = {}
_activeTPoseInfo = None

def getTPoseInfo(name):
    global _tposeInfos
    if name in _tposeInfos.keys():
        return _tposeInfos[name]
    else:
        return None


def initTPoses(scn):
    global _tposeInfos

    _tposeInfos = { "Default" : CTPoseInfo(scn) }
    keys = []
    folder = os.path.join(os.path.dirname(__file__), "t_poses")
    for fname in os.listdir(folder):
        filepath = os.path.join(folder, fname)
        if os.path.splitext(fname)[-1] == ".json":
            info = CTPoseInfo(scn)
            info.readFile(filepath)
            _tposeInfos[info.name] = info
            keys.append(info.name)
    enums = []
    keys.sort()
    keys = ["Default"] + keys
    for key in keys:
        enums.append((key,key,key))

    bpy.types.Scene.McpSourceTPose = EnumProperty(
        items = enums,
        name = "TPose Source",
        default = 'Default')
    scn.McpSourceTPose = 'Default'

    bpy.types.Scene.McpTargetTPose = EnumProperty(
        items = enums,
        name = "TPose Target",
        default = 'Default')
    scn.McpTargetTPose = 'Default'

    print("T-poses initialized")

#----------------------------------------------------------
#   Initialize
#----------------------------------------------------------

classes = [
    MCP_OT_RestCurrentPose,
    MCP_OT_PutInSrcTPose,
    MCP_OT_PutInTrgTPose,
    #MCP_OT_DefineTPose,
    #MCP_OT_UndefineTPose,
    MCP_OT_LoadTPose,
    MCP_OT_SaveTPose,
]

def initialize():
    bpy.types.Object.McpTPoseDefined = BoolProperty(default = False)
    bpy.types.Object.McpTPoseFile = StringProperty(default = "")
    bpy.types.Object.McpArmatureName = StringProperty(default = "")
    bpy.types.Object.McpArmatureModifier = StringProperty(default = "")
    bpy.types.PoseBone.McpQuat = FloatVectorProperty(size=4, default=(1,0,0,0))
    bpy.types.Object.McpIsSourceRig = BoolProperty(default=False)

    for cls in classes:
        bpy.utils.register_class(cls)


def uninitialize():
    for cls in classes:
        bpy.utils.unregister_class(cls)
