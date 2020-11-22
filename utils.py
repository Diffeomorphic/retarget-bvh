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
import math
from mathutils import *

D = math.pi/180
R = 180/math.pi

#-------------------------------------------------------------
#   Blender 2.8 compatibility
#-------------------------------------------------------------

def setActiveObject(context, ob):
    vly = context.view_layer
    vly.objects.active = ob
    vly.update()


def updateScene():
    deps = bpy.context.evaluated_depsgraph_get()
    deps.update()
    scn = bpy.context.scene
    scn.frame_current = scn.frame_current

#
#  quadDict():
#

def quadDict():
    return {
        0: {},
        1: {},
        2: {},
        3: {},
    }

MhxLayers = 8*[True] + 8*[False] + 8*[True] + 8*[False]
RigifyLayers = 27*[True] + 5*[False]

#
#   Identify rig type
#

def hasAllBones(blist, rig):
    for bname in blist:
        if bname not in rig.pose.bones.keys():
            return False
    return True

def hasSomeBones(blist, rig):
    for bname in blist:
        if bname in rig.pose.bones.keys():
            return bname
    return None

def isMhxRig(rig):
    return hasAllBones(["foot.rev.L"], rig)

def isMakeHuman(rig):
    return hasAllBones(["risorius03.R"], rig)

def isMhx7Rig(rig):
    return hasAllBones(["FootRev_L"], rig)

def isRigify(rig):
    return hasAllBones(["MCH-spine.flex"], rig)

def isRigify2(rig):
    return hasAllBones(["MCH-forearm_ik.L"], rig)

#
#   nameOrNone(string):
#

def nameOrNone(string):
    if string == "None":
        return None
    else:
        return string


def canonicalName(string):
    return string.lower().replace(' ','_').replace('-','_')


#
#   getRoll(bone):
#

def getRoll(bone):
    return getRollMat(bone.matrix_local)


def getRollMat(mat):
    quat = mat.to_3x3().to_quaternion()
    if abs(quat.w) < 1e-4:
        roll = pi
    else:
        roll = -2*math.atan(quat.y/quat.w)
    return roll


#
#   getTrgBone(b):
#

def getTrgBone(bname, rig):
    for pb in rig.pose.bones:
        if pb.McpBone == bname:
            return pb
    return None

#
#   isRotation(mode):
#   isLocation(mode):
#

def isRotation(mode):
    return (mode[0:3] == 'rot')

def isLocation(mode):
    return (mode[0:3] == 'loc')

#
#    Insert location and rotation
#

def insertLocation(pb, mat, frame=None):
    if frame is None:
        frame = bpy.context.scene.frame_current
    pb.location = mat.to_translation()
    pb.keyframe_insert("location", group=pb.name)


def insertRotation(pb, mat, frame=None):
    if frame is None:
        frame = bpy.context.scene.frame_current
    if pb.rotation_mode == 'QUATERNION':
        pb.rotation_quaternion = mat.to_quaternion()
        pb.keyframe_insert("rotation_quaternion", frame=frame, group=pb.name)
    elif pb.rotation_mode == "AXIS_ANGLE":
        pb.rotation_axis_angle = mat.to_axis_angle()
        pb.keyframe_insert("rotation_axis_angle", frame=frame, group=pb.name)
    else:
        pb.rotation_euler = mat.to_euler(pb.rotation_mode)
        pb.keyframe_insert("rotation_euler", frame=frame, group=pb.name)

#
#    setInterpolation(rig):
#

def setInterpolation(rig):
    if not rig.animation_data:
        return
    act = rig.animation_data.action
    if not act:
        return
    for fcu in act.fcurves:
        for pt in fcu.keyframe_points:
            pt.interpolation = 'LINEAR'
        fcu.extrapolation = 'CONSTANT'
    return

#-------------------------------------------------------------
#   Progress
#-------------------------------------------------------------

def startProgress(string):
    print(string + " (0%)")
    wm = bpy.context.window_manager
    wm.progress_begin(0, 100)

def endProgress(string):
    print(string + " (100%)")
    wm = bpy.context.window_manager
    wm.progress_end()

def showProgress(n, frame, nFrames, step=20):
    pct = (100.0*n)/nFrames
    if n % step == 0:
        print("%d (%.1f " % (int(frame), pct) + "%)")
    wm = bpy.context.window_manager
    wm.progress_update(int(pct))

#-------------------------------------------------------------
#   Error handling
#-------------------------------------------------------------

def clearErrorMessage():
    global theMessage, theErrorLines
    theMessage = ""
    theErrorLines = []

clearErrorMessage()

def getErrorMessage():
    """getErrorMessage()

    Returns:
    The error message from previous operator invokation if it raised
    an error, or the empty string if the operator exited without errors.
    """
    global theMessage
    return theMessage


def getSilentMode():
    global theSilentMode
    return theSilentMode

def setSilentMode(value):
    """setSilentMode(value)

    In silent mode, operators fail silently if they encounters an error.
    This is useful for scripting.

    value: True turns silent mode on, False turns it off.
    """
    global theSilentMode
    theSilentMode = value

setSilentMode(False)


class MocapError(Exception):
    def __init__(self, value):
        global theErrorLines, theMessage
        theMessage = value
        theErrorLines = (
            theMessage.split("\n") +
            ["" +
             "For corrective actions see:",
             "http://diffeomorphic.blogspot.com/p/bvh-retargeter.html"]
            )
        print("*** BVH Retargeter Error ***")
        for line in theErrorLines:
            print(line)

    def __str__(self):
        return repr(theMessage)


class MocapMessage(Exception):
    def __init__(self, value):
        global theErrorLines, theMessage
        theMessage = value
        theErrorLines = theMessage.split("\n")
        print(theMessage)


class MocapPopup(bpy.types.Operator):
    def execute(self, context):
        return {'RUNNING_MODAL'}

    def invoke(self, context, event):
        wm = context.window_manager
        wm.progress_end()
        return wm.invoke_props_dialog(self)

    def draw(self, context):
        global theErrorLines
        for line in theErrorLines:
            self.layout.label(text=line)


class ErrorOperator(MocapPopup):
    bl_idname = "mcp.error"
    bl_label = "BVH Retargeter Error"


class MessageOperator(MocapPopup):
    bl_idname = "mcp.message"
    bl_label = "BVH Retargeter"

#-------------------------------------------------------------
#   Poll
#-------------------------------------------------------------

class IsMesh:
    @classmethod
    def poll(self, context):
        ob = context.object
        return (ob and ob.type == 'MESH')


class IsArmature:
    @classmethod
    def poll(self, context):
        ob = context.object
        return (ob and ob.type == 'ARMATURE')


class IsMhx:
    @classmethod
    def poll(self, context):
        ob = context.object
        return (ob and ob.type == 'ARMATURE' and isMhxRig(ob))

#-------------------------------------------------------------
#   Execute
#-------------------------------------------------------------

class BvhOperator(bpy.types.Operator):
    def execute(self, context):
        clearErrorMessage()
        data = self.prequel(context)
        try:
            self.run(context)
        except MocapError:
            if getSilentMode():
                print(theMessage)
            else:
                bpy.ops.mcp.error('INVOKE_DEFAULT')
        except MocapMessage:
            if getSilentMode():
                print(theMessage)
            else:
                bpy.ops.mcp.message('INVOKE_DEFAULT')
        except KeyboardInterrupt:
            global theErrorLines
            theErrorLines = ["Keyboard interrupt"]
            bpy.ops.mcp.error('INVOKE_DEFAULT')
        finally:
            self.sequel(context, data)
        return{'FINISHED'}

    def prequel(self, context):
        return None

    def sequel(self, context, data):
        pass

    def run(self, context):
        pass


class BvhPropsOperator(BvhOperator):
    def invoke(self, context, event):
        clearErrorMessage()
        wm = context.window_manager
        return wm.invoke_props_dialog(self)

#-------------------------------------------------------------
#   HideOperator class
#-------------------------------------------------------------

class HideOperator(BvhOperator):
    def prequel(self, context):
        BvhOperator.prequel(self, context)
        self.layerColls = []
        rig = context.object
        self.hideLayerColls(rig, context.view_layer.layer_collection)
        return None


    def hideLayerColls(self, rig, layer):
        if layer.exclude:
            return True
        ok = True
        for ob in layer.collection.objects:
            if ob == rig:
                ok = False
        for child in layer.children:
            ok = (self.hideLayerColls(rig, child) and ok)
        if ok:
            self.layerColls.append(layer)
            layer.exclude = True
            print("HIDE", layer)
        return ok


    def sequel(self, context, _data):
        BvhOperator.prequel(self, context)
        for layer in self.layerColls:
            layer.exclude = False


class HidePropsOperator(HideOperator):
    def invoke(self, context, event):
        clearErrorMessage()
        wm = context.window_manager
        return wm.invoke_props_dialog(self)
