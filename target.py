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
from bpy_extras.io_utils import ExportHelper
import math
import os

from .utils import *
from .armature import CArmature
from .source import CRigInfo

#----------------------------------------------------------
#   Target classes
#----------------------------------------------------------

class CTargetInfo(CArmature, CRigInfo):
    verboseString = "Read target file"

    def __init__(self, scn, name="Automatic"):
        CArmature.__init__(self, scn)
        CRigInfo.__init__(self, scn, name)


class Target:
    useAutoTarget : BoolProperty(
        name = "Auto Target",
        description = "Find target rig automatically",
        default = True)

    def draw(self, context):
        self.layout.prop(self, "useAutoTarget")
        if not self.useAutoTarget:
            scn = context.scene
            self.layout.prop(scn, "McpTargetRig")
            self.layout.prop(scn, "McpTargetTPose")
        self.layout.separator()

    def findTarget(self, context, rig):
        return findTargetArmature(context, rig, self.useAutoTarget)

#
#   Global variables
#

_targetInfos = {}

def getTargetInfo(rigname):
    global _targetInfos
    return _targetInfos[rigname]

def loadTargets():
    global _targetInfos
    _targetInfos = {}

def isTargetInited(scn):
    return ( _targetInfos != {} )

def ensureTargetInited(scn):
    if not isTargetInited(scn):
        initTargets(scn)

#
#   findTargetArmature(context, rig, auto):
#

def findTargetArmature(context, rig, auto):
    from .t_pose import autoTPose, putInRestPose, getTPoseInfo, putInRightPose
    global _targetInfos

    scn = context.scene
    ensureTargetInited(scn)

    if auto:
        scn.McpTargetRig, scn.McpTargetTPose = guessArmatureFromList(rig, scn, _targetInfos)

    if scn.McpTargetRig == "Automatic":
        info = CTargetInfo(scn)
        tposed = info.identifyRig(rig, context, scn.McpTargetTPose)
        if not tposed:
            autoTPose(rig, context)
        _targetInfos["Automatic"] = info
        info.display("Target")
    else:
        info = _targetInfos[scn.McpTargetRig]
        info.addManualBones(rig)
        tinfo = getTPoseInfo(scn.McpTargetTPose)
        if tinfo:
            tinfo.addTPose(rig)
        else:
            scn.McpTargetTPose = "Default"

    rig.McpArmature = info.name
    print("Using target armature %s." % rig.McpArmature)
    return info


def guessArmatureFromList(rig, scn, infos):
    print("Identifying rig")
    for name,info in infos.items():
        if name == "Automatic":
            continue
        elif matchAllBones(rig, info, scn):
            if info.t_pose_file:
                return name, info.t_pose_file
            else:
                return name, "Default"
    else:
        return "Automatic", "Default"


def matchAllBones(rig, info, scn):
    if not hasAllBones(info.fingerprint, rig):
        return False
    if hasSomeBones(info.illegal, rig):
        return False
    for bname,mhx in info.bones:
        if bname in info.optional:
            continue
        if (mhx[0:2] == "f_" and not scn.McpIncludeFingers):
            continue
        elif bname not in rig.data.bones.keys():
            if scn.McpVerbose:
                print("Missing bone:", bname)
            return False
    return True

###############################################################################
#
#    Target initialization
#
###############################################################################

def readTargetFiles(scn, subdir):
    global _targetInfos
    keys = []
    path = os.path.join(os.path.dirname(__file__), subdir)
    for fname in os.listdir(path):
        filepath = os.path.join(path, fname)
        (name, ext) = os.path.splitext(fname)
        if ext == ".json" and os.path.isfile(filepath):
            info = CTargetInfo(scn, "Manual")
            info.readFile(filepath)
            _targetInfos[info.name] = info
            keys.append(info.name)
    keys.sort()
    return keys


def initTargets(scn):
    global _targetInfos
    from .t_pose import initTPoses
    initTPoses(scn)
    _targetInfos = { "Automatic" : CTargetInfo(scn, "Automatic") }
    tkeys = readTargetFiles(scn, "known_rigs")
    keys = ["Automatic"] + tkeys
    enums = [(key,key,key) for key in keys]

    bpy.types.Scene.McpTargetRig = EnumProperty(
        items = enums,
        name = "Target rig",
        default = 'Automatic')
    print("Defined McpTargetRig")


class MCP_OT_IdentifyTargetRig(BvhOperator, IsArmature):
    bl_idname = "mcp.identify_target_rig"
    bl_label = "Identify Target Rig"
    bl_description = "Identify the target rig type of the active armature."
    bl_options = {'UNDO'}

    def prequel(self, context):
        from .retarget import changeTargetData
        return changeTargetData(context.object, context.scene)

    def run(self, context):
        scn = context.scene
        scn.McpTargetRig = "Automatic"
        findTargetArmature(context, context.object, True)
        print("Identified rig %s" % scn.McpTargetRig)

    def sequel(self, context, data):
        from .retarget import restoreTargetData
        restoreTargetData(data)

#----------------------------------------------------------
#   List Rig
#----------------------------------------------------------

from .source import ListRig

class MCP_OT_ListTargetRig(BvhOperator, ListRig):
    bl_idname = "mcp.list_target_rig"
    bl_label = "List Target Rig"
    bl_description = "List the bone associations of the active target rig"
    bl_options = {'UNDO'}

    @classmethod
    def poll(self, context):
        return context.scene.McpTargetRig

    def sfindKeys(self, mhx, bones):
        keys = []
        for (bone, mhx1) in bones:
            if mhx1 == mhx:
                keys.append(bone)
        return keys

    def getBones(self, context):
        from .t_pose import getTPoseInfo
        scn = context.scene
        info = getTargetInfo(scn.McpTargetRig)
        tinfo = getTPoseInfo(scn.McpTargetTPose)
        if info and tinfo:
            return info.bones, tinfo.t_pose
        elif info:
            return info.bones, {}
        else:
            return [], {}

class MCP_OT_VerifyTargetRig(BvhOperator):
    bl_idname = "mcp.verify_target_rig"
    bl_label = "Verify Target Rig"
    bl_description = "Verify the target rig type of the active armature"
    bl_options = {'UNDO'}

    @classmethod
    def poll(self, context):
        ob = context.object
        return (context.scene.McpTargetRig and ob and ob.type == 'ARMATURE')

    def run(self, context):
        rigtype = context.scene.McpTargetRig
        info = _targetInfos[rigtype]
        info.testRig(rigtype, context.object, context.scene)
        raise MocapMessage("Target armature %s verified" % rigtype)

#----------------------------------------------------------
#   Initialize
#----------------------------------------------------------

classes = [
    MCP_OT_IdentifyTargetRig,
    MCP_OT_ListTargetRig,
    MCP_OT_VerifyTargetRig,
]

def initialize():
    bpy.types.Scene.McpTargetRig = EnumProperty(
        items = [("Automatic", "Automatic", "Automatic")],
        name = "Target Rig",
        default = "Automatic")

    bpy.types.Scene.McpTargetTPose = EnumProperty(
        items = [("Default", "Default", "Default")],
        name = "TPose Target",
        default = "Default")

    bpy.types.Object.McpReverseHip = BoolProperty(
        name = "Reverse Hip",
        description = "The rig has a reverse hip",
        default = False)

    bpy.types.PoseBone.McpBone = StringProperty(
        name = "Canonical Bone Name",
        description = "Canonical bone corresponding to this bone",
        default = "")

    bpy.types.PoseBone.McpParent = StringProperty(
        name = "Parent",
        description = "Parent of this bone for retargeting purposes",
        default = "")


    for cls in classes:
        bpy.utils.register_class(cls)


def uninitialize():
    for cls in classes:
        bpy.utils.unregister_class(cls)
