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
from math import pi, sqrt
from mathutils import *
from .utils import *

_Markers = []
_EditLoc = None
_EditRot = None

#----------------------------------------------------------------
#   Markers
#----------------------------------------------------------------

def saveMarkers(scn):
    global _Markers
    _Markers = [(mrk.camera, mrk.frame, mrk.name, mrk.select) for mrk in scn.timeline_markers]
    scn.timeline_markers.clear()


def restoreMarkers(scn):
    global _Markers
    scn.timeline_markers.clear()
    for (camera, frame, name, select) in _Markers:
        mrk = scn.timeline_markers.new(name)
        mrk.camera = camera
        mrk.frame = frame
        mrk.select = select


def setMarker(scn, frame):
    mrk = scn.timeline_markers.new("F_%d" % int(frame))
    mrk.frame = frame


def removeMarker(scn, frame):
    marker = None
    for mrk in scn.timeline_markers:
        if mrk.frame == frame:
            scn.timeline_markers.remove(mrk)
            return
    raise MocapError("No keys at frame %d" % int(frame))


########################################################################
#
#   startEdit(context):
#   class MCP_OT_StartEdit(HideOperator):
#

def getUndoAction(rig):
    if rig.McpUndoAction in bpy.data.actions.keys():
        return bpy.data.actions[rig.McpUndoAction]
    else:
        return None


def startEdit(context):
    from .action import getObjectAction
    from .loop import fCurveIdentity
    global _EditLoc, _EditRot

    rig = context.object
    scn = context.scene
    if getUndoAction(rig):
        raise MocapError("Action already being edited. Undo or confirm edit first")
    act = getObjectAction(rig)
    if not act:
        raise MocapError("Object %s has no action" % rig.name)
    aname = act.name
    act.name = '#'+aname
    nact = bpy.data.actions.new(aname)
    rig.animation_data.action = nact
    rig.McpUndoAction = act.name
    rig.McpActionName = aname

    saveMarkers(scn)
    _EditLoc = quadDict()
    _EditRot = quadDict()

    for fcu in act.fcurves:
        (name, mode) = fCurveIdentity(fcu)
        nfcu = nact.fcurves.new(fcu.data_path, index=fcu.array_index, action_group=name)
        n = len(fcu.keyframe_points)
        nfcu.keyframe_points.add(count=n)
        for i in range(n):
            nfcu.keyframe_points[i].co = fcu.keyframe_points[i].co
    setInterpolation(rig)
    print("Action editing started")
    return nact


class MCP_OT_StartEdit(HideOperator, IsArmature):
    bl_idname = "mcp.start_edit"
    bl_label = "Start Edit"
    bl_description = "Start local F-curve editing"
    bl_options = {'UNDO'}

    @classmethod
    def poll(self, context):
        return (context.object.McpUndoAction == "")

    def run(self, context):
        startEdit(context)
        setKeyMap(context, "mcp.insert_locrot", True)


def setKeyMap(context, idname, doAdd):
    return
    km = context.window_manager.keyconfigs.active.keymaps['3D View']
    if doAdd:
        km.keymap_items.new(idname, 'SPACE', 'PRESS')
    else:
        try:
            item = km.keymap_items[idname]
        except KeyError:
            return
        km.keymap_items.remove(item)

#
#   undoEdit(context):
#   class MCP_OT_UndoEdit(HideOperator):
#

def undoEdit(context):
    from .action import deleteAction
    global _EditLoc, _EditRot

    rig = context.object
    restoreMarkers(context.scene)
    oact = getUndoAction(rig)
    if not oact:
        clearUndoAction(rig)
        raise MocapError("No action to undo")
    clearUndoAction(rig)
    act = rig.animation_data.action
    act.name = "#Delete"
    oact.name = rig.McpActionName
    rig.animation_data.action = oact
    deleteAction(act)
    print("Action changes undone")
    return


class MCP_OT_UndoEdit(HideOperator):
    bl_idname = "mcp.undo_edit"
    bl_label = "Undo Edit"
    bl_description = "Quit local editing and discard changes"
    bl_options = {'UNDO'}

    answer : StringProperty()

    @classmethod
    def poll(self, context):
        return (context.object.McpUndoAction != "")

    def run(self, context):
        setKeyMap(context, "mcp.insert_locrot", False)
        undoEdit(context)

    def invoke(self, context, event):
        wm = context.window_manager
        return wm.invoke_props_dialog(self, width=200, height=20)

    def draw(self, context):
        self.layout.label(text="Really undo editing?")

#
#   getActionPair(context):
#

def clearUndoAction(rig):
    global _EditLoc, _EditRot
    rig.McpUndoAction = ""
    _EditLoc = None
    _EditRot = None
    rig.McpActionName = ""


def getActionPair(context):
    from .action import getObjectAction
    global _EditLoc, _EditRot

    rig = context.object
    oact = getUndoAction(rig)
    if not oact:
        clearUndoAction(rig)
        raise MocapError("No action is currently being edited")
        return None
    if not _EditLoc:
        _EditLoc = quadDict()
    if not _EditRot:
        _EditRot = quadDict()
    act = getObjectAction(rig)
    if act:
        return (act, oact)
    else:
        return None

#
#   confirmEdit(context):
#   class MCP_OT_ConfirmEdit(HideOperator):
#

def confirmEdit(context):
    from .action import deleteAction
    from .loop import fCurveIdentity
    global _EditLoc, _EditRot

    rig = context.object
    pair = getActionPair(context)
    if not pair:
        return
    (act, oact) = pair

    for fcu in act.fcurves:
        ofcu = findFCurve(fcu.data_path, fcu.array_index, oact.fcurves)
        if not ofcu:
            continue
        (name,mode) =  fCurveIdentity(fcu)
        if isRotation(mode):
            try:
                edit = _EditRot[fcu.array_index][name]
            except KeyError:
                continue
            displaceFCurve(fcu, ofcu, edit)
        elif  isLocation(mode):
            try:
                edit = _EditLoc[fcu.array_index][name]
            except KeyError:
                continue
            displaceFCurve(fcu, ofcu, edit)

    rig.McpUndoAction = ""
    rig.McpActionName = ""
    restoreMarkers(context.scene)
    _EditLoc = None
    _EditRot = None
    deleteAction(oact)
    print("Action changed")
    return


class MCP_OT_ConfirmEdit(HideOperator):
    bl_idname = "mcp.confirm_edit"
    bl_label = "Confirm Edit"
    bl_description = "Quit local editing and keep changes"
    bl_options = {'UNDO'}

    @classmethod
    def poll(self, context):
        return (context.object.McpUndoAction != "")

    def run(self, context):
        setKeyMap(context, "mcp.insert_locrot", False)
        confirmEdit(context)

#
#   setEditDict(editDict, frame, name, channel, index):
#   insertKey(context, useLoc, useRot):
#   class MCP_OT_InsertLoc(BvhOperator):
#   class MCP_OT_InsertRot(BvhOperator):
#   class MCP_OT_InsertLocRot(BvhOperator):
#

def setEditDict(editDict, frame, name, channel, n):
    for index in range(n):
        try:
            edit = editDict[index][name]
        except KeyError:
            edit = editDict[index][name] = {}
        edit[frame] = channel[index]
    return


def removeEditDict(editDict, frame, name, n):
    for index in range(n):
        try:
            del editDict[index][name][frame]
        except KeyError:
            editDict[index][name] = {}


def insertKey(context, useLoc, useRot, delete):
    from .loop import fCurveIdentity
    global _EditLoc, _EditRot

    rig = context.object
    pair = getActionPair(context)
    if not pair:
        raise MocapError("No action is currently being edited")
    (act, oact) = pair

    scn = context.scene
    frame = scn.frame_current
    if delete:
        removeMarker(scn, frame)
    else:
        setMarker(scn, frame)

    for pb in rig.pose.bones:
        if not pb.bone.select:
            continue

        if delete:
            removeEditDict(_EditLoc, frame, pb.name, 3)
            removeEditDict(_EditRot, frame, pb.name, 4)
        else:
            if useLoc:
                setEditDict(_EditLoc, frame, pb.name, pb.location, 3)
            if useRot:
                if pb.rotation_mode == 'QUATERNION':
                    setEditDict(_EditRot, frame, pb.name, pb.rotation_quaternion, 4)
                else:
                    setEditDict(_EditRot, frame, pb.name, pb.rotation_euler, 3)

        for fcu in act.fcurves:
            ofcu = findFCurve(fcu.data_path, fcu.array_index, oact.fcurves)
            if not ofcu:
                continue
            (name,mode) = fCurveIdentity(fcu)
            if name == pb.name:
                if isRotation(mode) and useRot:
                    displaceFCurve(fcu, ofcu, _EditRot[fcu.array_index][name])
                if isLocation(mode) and useLoc:
                    displaceFCurve(fcu, ofcu, _EditLoc[fcu.array_index][name])


class MCP_OT_InsertKey(BvhOperator):
    bl_idname = "mcp.insert_key"
    bl_label = "Key"
    bl_description = "Insert or delete a key"
    bl_options = {'UNDO'}

    loc : BoolProperty("Loc", default=False)
    rot : BoolProperty("Rot", default=False)
    delete : BoolProperty("Del", default=False)

    @classmethod
    def poll(self, context):
        return (context.object.McpUndoAction != "")

    def run(self, context):
        insertKey(context, self.properties.loc, self.properties.rot, self.properties.delete)


def move2marker(context, left, last):
    scn = context.scene
    frames = [mrk.frame for mrk in scn.timeline_markers]
    frames.sort()
    if frames == []:
        return
    if last:
        if left:
            scn.frame_current = frames[0]
        else:
            scn.frame_current = frames[-1]
    else:
        if left:
            frames.reverse()
            for frame in frames:
                if frame < scn.frame_current:
                    scn.frame_current = frame
                    break
        else:
            for frame in frames:
                if frame > scn.frame_current:
                    scn.frame_current = frame
                    break


class MCP_OT_MoveToMarker(BvhOperator):
    bl_idname = "mcp.move_to_marker"
    bl_label = "Move"
    bl_description = "Move to time marker"
    bl_options = {'UNDO'}

    left : BoolProperty("Loc", default=False)
    last : BoolProperty("Rot", default=False)

    @classmethod
    def poll(self, context):
        return (context.object.McpUndoAction != "")

    def run(self, context):
        move2marker(context, self.properties.left, self.properties.last)

#
#   findFCurve(path, index, fcurves):
#

def findFCurve(path, index, fcurves):
    for fcu in fcurves:
        if (fcu.data_path == path and
            fcu.array_index == index):
            return fcu
    print('F-curve "%s" not found.' % path)
    return None

#
#   displaceFCurve(fcu, ofcu, edits):
#   setupCatmullRom(kpoints, modified):
#   evalCatmullRom(t, fcn):
#

def displaceFCurve(fcu, ofcu, edits):
    modified = []
    editList = list(edits.items())
    editList.sort()
    for (t,y) in editList:
        dy = y - ofcu.evaluate(t)
        modified.append((t,dy))

    if len(modified) >= 1:
        kp = fcu.keyframe_points[0].co
        t0 = int(kp[0])
        (t1,y1) = modified[0]
        kp = fcu.keyframe_points[-1].co
        tn = int(kp[0])
        (tn_1,yn_1) = modified[-1]
        modified = [(t0, y1)] + modified
        modified.append( (tn, yn_1) )
        fcn = setupCatmullRom(modified)
        for kp in fcu.keyframe_points:
            t = kp.co[0]
            y = ofcu.evaluate(t)
            dy = evalCatmullRom(t, fcn)
            kp.co[1] = y+dy
    return


def setupCatmullRom(points):
    points.sort()
    n = len(points)-1
    fcn = []
    tension = 0.5

    # First interval
    (t0,y0) = points[0]
    (t1,y1) = points[1]
    (t2,y2) = points[2]
    if t1-t0 < 0.5:
        t0 = t1-1
    d = y0
    a = y1
    c = 3*d + tension*(y1-y0)
    b = 3*a - tension*(y2-y0)
    tfac = 1.0/(t1-t0)
    fcn.append((t0, t1, tfac, (a,b,c,d)))

    # Inner intervals
    for i in range(1,n-1):
        (t_1,y_1) = points[i-1]
        (t0,y0) = points[i]
        (t1,y1) = points[i+1]
        (t2,y2) = points[i+2]
        d = y0
        a = y1
        c = 3*d + tension*(y1-y_1)
        b = 3*a - tension*(y2-y0)
        tfac = 1.0/(t1-t0)
        fcn.append((t0, t1, tfac, (a,b,c,d)))

    # Last interval
    (t_1,y_1) = points[n-2]
    (t0,y0) = points[n-1]
    (t1,y1) = points[n]
    if t1-t0 < 0.5:
        t1 = t0+1
    d = y0
    a = y1
    c = 3*d + tension*(y1-y_1)
    b = 3*a - tension*(y1-y0)
    tfac = 1.0/(t1-t0)
    fcn.append((t0, t1, tfac, (a,b,c,d)))

    return fcn

def evalCatmullRom(t, fcn):
    (t0, t1, tfac, params) = fcn[0]
    if t < t0:
        return evalCRInterval(t, t0, t1, tfac, params)
    for (t0, t1, tfac, params) in fcn:
        if t >= t0 and t < t1:
            return evalCRInterval(t, t0, t1, tfac, params)
    return evalCRInterval(t, t0, t1, tfac, params)

def evalCRInterval(t, t0, t1, tfac, params):
    (a,b,c,d) = params
    x = tfac*(t-t0)
    x1 = 1-x
    f = x*x*(a*x + b*x1) + x1*x1*(c*x+d*x1)
    return f

#----------------------------------------------------------
#   Initialize
#----------------------------------------------------------

classes = [
    MCP_OT_StartEdit,
    MCP_OT_UndoEdit,
    MCP_OT_ConfirmEdit,
    MCP_OT_InsertKey,
    MCP_OT_MoveToMarker,
]

def initialize():
    bpy.types.Object.McpUndoAction = StringProperty(
        default="")

    bpy.types.Object.McpActionName = StringProperty(
        default="")

    for cls in classes:
        bpy.utils.register_class(cls)


def uninitialize():
    for cls in classes:
        bpy.utils.unregister_class(cls)
