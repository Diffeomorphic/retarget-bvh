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

import bpy, os, mathutils, math, time
from bpy_extras.io_utils import ImportHelper
from math import sin, cos
from mathutils import *
from bpy.props import *

from .utils import *
from .source import Source
from .target import Target
from .simplify import TimeScaler

class BvhFile:
    filename_ext = ".bvh"
    filter_glob : StringProperty(default="*.bvh", options={'HIDDEN'})
    filepath : StringProperty(name="File Path", description="Filepath used for importing the BVH file", maxlen=1024, default="")


class MultiFile(ImportHelper):
    files : CollectionProperty(
        name = "File Path",
        type = bpy.types.OperatorFileListElement)

    directory : StringProperty(
        subtype='DIR_PATH')

    def getFilePaths(self):
        if self.files:
            filepaths = []
            for file_elem in self.files:
                filepath = os.path.join(self.directory, file_elem.name)
                filepaths.append(filepath)
            return filepaths
        else:
            return [self.filepath]

###################################################################################
#    BVH importer.
#    The importer that comes with Blender had memory leaks which led to instability.
#    It also creates a weird skeleton from CMU data, with hands theat start at the wrist
#    and ends at the elbow.
#

#
#    class CNode:
#

class CNode:
    def __init__(self, words, parent):
        name = words[1]
        for word in words[2:]:
            name += ' '+word

        self.name = name
        self.parent = parent
        self.children = []
        self.head = Vector((0,0,0))
        self.offset = Vector((0,0,0))
        if parent:
            parent.children.append(self)
        self.channels = []
        self.matrix = None
        self.inverse = None
        return

    def __repr__(self):
        return "CNode %s" % (self.name)

    def display(self, pad):
        vec = self.offset
        if vec.length < Epsilon:
            c = '*'
        else:
            c = ' '
        print("%s%s%10s (%8.3f %8.3f %8.3f)" % (c, pad, self.name, vec[0], vec[1], vec[2]))
        for child in self.children:
            child.display(pad+"  ")
        return


    def build(self, amt, orig, parent):
        self.head = orig + self.offset
        if not self.children:
            return self.head

        zero = (self.offset.length < Epsilon)
        eb = amt.edit_bones.new(self.name)
        if parent:
            eb.parent = parent
        eb.head = self.head
        tails = Vector((0,0,0))
        for child in self.children:
            tails += child.build(amt, self.head, eb)
        tail = tails/len(self.children)
        if (tail-self.head).length == 0:
            print("Zero-length bone: %s" % eb.name)
            vec = self.head - parent.head
            tail = self.head + vec*0.1
        eb.tail = tail
        (loc, rot, scale) = eb.matrix.decompose()
        self.matrix = rot.to_matrix()
        self.inverse = self.matrix.copy()
        self.inverse.invert()
        if zero:
            return eb.tail
        else:
            return eb.head

#
#    readBvhFile(context, filepath, scn, scan):
#    Custom importer
#

Location = 1
Rotation = 2
Hierarchy = 1
Motion = 2
Frames = 3

Epsilon = 1e-5

class BvhLoader:
    startFrame : IntProperty(
        name = "Start Frame",
        description = "Starting frame for the animation",
        default = 1)

    endFrame : IntProperty(
        name = "Last Frame",
        description = "Last frame for the animation",
        default = 250)

    scale : FloatProperty(
        name="Scale",
        description="Scale the BVH by this value",
        min=0.0001, max=1000000.0,
        soft_min=0.001, soft_max=100.0,
        precision = 3,
        default=1.0)

    x : EnumProperty(
        items = [(x,x,x) for x in ["0", "90", "180", "270"]],
        name = "X",
        description = "X Euler Angle",
        default = "90")

    y : EnumProperty(
        items = [(y,y,y) for y in ["0", "90", "180", "270"]],
        name = "Y",
        description = "Y Euler Angle",
        default = "0")

    z : EnumProperty(
        items = [(z,z,z) for z in ["0", "90", "180", "270"]],
        name = "Z",
        description = "Z Euler Angle",
        default = "0")

    ssFactor : IntProperty(
        name="Subsample Factor",
        description="Sample only every n:th frame",
        min=1, default=1)

    useDefaultSS : BoolProperty(
        name="Use default subsample",
        description = "Subsample based on difference in frame rates between BVH file and Blender",
        default=True)

    def draw(self, context):
        self.layout.prop(self, "startFrame")
        self.layout.prop(self, "endFrame")
        self.layout.separator()
        self.layout.label(text="Source Rig Orientation:")
        row = self.layout.row()
        row.label(text="X:")
        row.prop(self, "x", expand=True)
        row = self.layout.row()
        row.label(text="Y:")
        row.prop(self, "y", expand=True)
        row = self.layout.row()
        row.label(text="Z:")
        row.prop(self, "z", expand=True)
        self.layout.separator()
        self.layout.prop(self, "useDefaultSS")
        if not self.useDefaultSS:
            self.layout.prop(self, "ssFactor")
        self.layout.separator()


    def readBvhFile(self, context, filepath, scn, scan):
        frameno = 1
        euler = Euler((int(self.x)*D, int(self.y)*D, int(self.z)*D))
        flipMatrix = euler.to_matrix()
        ssFactor = self.ssFactor

        fileName = os.path.realpath(os.path.expanduser(filepath))
        (shortName, ext) = os.path.splitext(fileName)
        if ext.lower() != ".bvh":
            raise MocapError("Not a bvh file: " + fileName)
        startProgress( "Loading BVH file "+ fileName )

        time1 = time.perf_counter()
        level = 0
        nErrors = 0
        coll = context.scene.collection
        rig = None

        fp = open(fileName, "rU")
        print( "Reading skeleton" )
        lineNo = 0
        for line in fp:
            words= line.split()
            lineNo += 1
            if len(words) == 0:
                continue
            key = words[0].upper()
            if key == 'HIERARCHY':
                status = Hierarchy
                ended = False
            elif key == 'MOTION':
                if level != 0:
                    raise MocapError("Tokenizer out of kilter %d" % level)
                if scan:
                    return root
                amt = bpy.data.armatures.new("BvhAmt")
                rig = bpy.data.objects.new("BvhRig", amt)
                coll.objects.link(rig)
                setActiveObject(context, rig)
                updateScene()
                bpy.ops.object.mode_set(mode='EDIT')
                bpy.ops.object.mode_set(mode='EDIT')
                root.build(amt, Vector((0,0,0)), None)
                #root.display('')
                bpy.ops.object.mode_set(mode='OBJECT')
                status = Motion
                print("Reading motion")
            elif status == Hierarchy:
                if key == 'ROOT':
                    node = CNode(words, None)
                    root = node
                    nodes = [root]
                elif key == 'JOINT':
                    node = CNode(words, node)
                    nodes.append(node)
                    ended = False
                elif key == 'OFFSET':
                    (x,y,z) = (float(words[1]), float(words[2]), float(words[3]))
                    node.offset = self.scale * flipMatrix @ Vector((x,y,z))
                elif key == 'END':
                    node = CNode(words, node)
                    ended = True
                elif key == 'CHANNELS':
                    oldmode = None
                    for word in words[2:]:
                        (index, mode, sign) = channelYup(word)
                        if mode != oldmode:
                            indices = []
                            node.channels.append((mode, indices))
                            oldmode = mode
                        indices.append((index, sign))
                elif key == '{':
                    level += 1
                elif key == '}':
                    if not ended:
                        node = CNode(["End", "Site"], node)
                        node.offset = self.scale * flipMatrix @ Vector((0,1,0))
                        node = node.parent
                        ended = True
                    level -= 1
                    node = node.parent
                else:
                    raise MocapError("Did not expect %s" % words[0])
            elif status == Motion:
                if key == 'FRAMES:':
                    nFrames = int(words[1])
                elif key == 'FRAME' and words[1].upper() == 'TIME:':
                    frameTime = float(words[2])
                    frameFactor = int(1.0/(scn.render.fps*frameTime) + 0.49)
                    if self.useDefaultSS:
                        ssFactor = frameFactor if frameFactor > 0 else 1
                    self.startFrame *= ssFactor
                    self.endFrame *= ssFactor
                    status = Frames
                    frame = 0
                    frameno = 1

                    bpy.ops.object.mode_set(mode='POSE')
                    pbones = rig.pose.bones
                    for pb in pbones:
                        pb.rotation_mode = 'QUATERNION'
            elif status == Frames:
                if (frame >= self.startFrame and
                    frame <= self.endFrame and
                    frame % ssFactor == 0 and
                    frame < nFrames):
                    self.addFrame(words, frameno, nodes, pbones, flipMatrix)
                    showProgress(frameno, frame, nFrames, step=200)
                    frameno += 1
                frame += 1

        fp.close()
        if not rig:
            raise MocapError("Bvh file \n%s\n is corrupt: No rig defined" % filepath)
        setInterpolation(rig)
        time2 = time.perf_counter()
        endProgress("Bvh file %s loaded in %.3f s" % (filepath, time2-time1))
        if frameno == 1:
            print("Warning: No frames in range %d -- %d." % (self.startFrame, self.endFrame))
        renameBvhRig(rig, filepath)
        rig.McpIsSourceRig = True
        return rig


    def addFrame(self, words, frame, nodes, pbones, flipMatrix):
        m = 0
        first = True
        flipInv = flipMatrix.inverted()
        for node in nodes:
            bname = node.name
            if bname not in pbones.keys():
                for (mode, indices) in node.channels:
                    m += len(indices)
            else:
                pb = pbones[bname]
                for (mode, indices) in node.channels:
                    if mode == Location:
                        vec = Vector((0,0,0))
                        for (index, sign) in indices:
                            vec[index] = sign*float(words[m])
                            m += 1
                        if first:
                            pb.location = node.inverse @ (self.scale * flipMatrix @ vec) - node.head
                            pb.keyframe_insert('location', frame=frame, group=bname)
                        first = False
                    elif mode == Rotation:
                        mats = []
                        for (axis, sign) in indices:
                            angle = sign*float(words[m])*D
                            mats.append(Matrix.Rotation(angle, 3, axis))
                            m += 1
                        mat = (node.inverse @ flipMatrix) @ mats[0] @ mats[1] @ mats[2] @ (flipInv @ node.matrix)
                        insertRotation(pb, mat, frame)

#
#    channelYup(word):
#    channelZup(word):
#

def channelYup(word):
    if word == 'Xrotation':
        return ('X', Rotation, +1)
    elif word == 'Yrotation':
        return ('Y', Rotation, +1)
    elif word == 'Zrotation':
        return ('Z', Rotation, +1)
    elif word == 'Xposition':
        return (0, Location, +1)
    elif word == 'Yposition':
        return (1, Location, +1)
    elif word == 'Zposition':
        return (2, Location, +1)

def channelZup(word):
    if word == 'Xrotation':
        return ('X', Rotation, +1)
    elif word == 'Yrotation':
        return ('Z', Rotation, +1)
    elif word == 'Zrotation':
        return ('Y', Rotation, -1)
    elif word == 'Xposition':
        return (0, Location, +1)
    elif word == 'Yposition':
        return (2, Location, +1)
    elif word == 'Zposition':
        return (1, Location, -1)

#
#   end BVH importer
#
###################################################################################


###################################################################################

#
#    class CEditBone():
#

class CEditBone():
    def __init__(self, bone):
        self.name = bone.name
        self.head = bone.head.copy()
        self.tail = bone.tail.copy()
        self.roll = bone.roll
        if bone.parent:
            self.parent = bone.parent.name
        else:
            self.parent = None
        if self.parent:
            self.use_connect = bone.use_connect
        else:
            self.use_connect = False
        #self.matrix = bone.matrix.copy().rotation_part()
        (loc, rot, scale) = bone.matrix.decompose()
        self.matrix = rot.to_matrix()
        self.inverse = self.matrix.copy()
        self.inverse.invert()

    def __repr__(self):
        return ("%s p %s\n  h %s\n  t %s\n" % (self.name, self.parent, self.head, self.tail))

#
#    renameBones(srcRig, context):
#

def renameBones(srcRig, context):
    from .source import getSourceBoneName

    srcBones = []
    trgBones = {}

    setActiveObject(context, srcRig)
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.object.mode_set(mode='EDIT')
    #print("Ren", bpy.context.object, srcRig.mode)
    ebones = srcRig.data.edit_bones
    for bone in ebones:
        srcBones.append( CEditBone(bone) )

    setbones = []
    adata = srcRig.animation_data
    if adata is None:
        action = None
    else:
        action = adata.action
    for srcBone in srcBones:
        srcName = srcBone.name
        trgName = getSourceBoneName(srcName)
        if isinstance(trgName, tuple):
            print("BUG. Target name is tuple:", trgName)
            trgName = trgName[0]
        eb = ebones[srcName]
        if trgName:
            if action and srcName in action.groups.keys():
                grp = action.groups[srcName]
                grp.name = trgName
            eb.name = trgName
            trgBones[trgName] = CEditBone(eb)
            setbones.append((eb, trgName))
        else:
            eb.name = '_' + srcName

    for (eb, name) in setbones:
        eb.name = name
    #createExtraBones(ebones, trgBones)
    bpy.ops.object.mode_set(mode='OBJECT')

#
#    renameBvhRig(srcRig, filepath):
#

def renameBvhRig(srcRig, filepath):
    base = os.path.basename(filepath)
    (filename, ext) = os.path.splitext(base)
    print("File", filename, len(filename))
    if len(filename) > 12:
        words = filename.split('_')
        if len(words) == 1:
            words = filename.split('-')
        name = 'Y_'
        if len(words) > 1:
            words = words[1:]
        for word in words:
            name += word
    else:
        name = 'Y_' + filename
    print("Name", name)

    srcRig.name = name
    adata = srcRig.animation_data
    if adata:
        adata.action.name = name
    return

#
#    deleteSourceRig(context, rig, prefix):
#

def deleteSourceRig(context, rig, prefix):
    ob = context.object
    setActiveObject(context, rig)
    bpy.ops.object.mode_set(mode='OBJECT')
    setActiveObject(context, ob)
    deleteObject(context, rig)
    if bpy.data.actions:
        for act in bpy.data.actions:
            if act.name[0:2] == prefix:
                act.use_fake_user = False
                if act.users == 0:
                    bpy.data.actions.remove(act)


def deleteObject(context, ob):
    if context.object:
        bpy.ops.object.mode_set(mode='OBJECT')
    bpy.ops.object.select_all(action='DESELECT')
    ob.select_set(True)
    for coll in bpy.data.collections:
        if ob in coll.objects.values():
            coll.objects.unlink(ob)
    bpy.ops.object.delete(use_global=False)
    del ob

#----------------------------------------------------------
#   Renamer
#----------------------------------------------------------

class BvhRenamer(Source, Target):
    useAutoScale : BoolProperty(
        name="Auto Scale",
        description="Rescale skeleton to match target",
        default=True)

    def draw(self, context):
        self.layout.prop(context.scene, "McpIncludeFingers")
        self.layout.separator()
        Source.draw(self, context)
        Target.draw(self, context)
        self.layout.prop(self, "useAutoScale")
        if not self.useAutoScale:
            self.layout.prop(self, "scale")
        self.layout.separator()


    def rescaleRig(self, trgRig, srcRig):
        if not self.useAutoScale:
            return
        upleg1 = getTrgBone("thigh.L", trgRig)
        upleg2 = getTrgBone("thigh_twist.L", trgRig)
        if upleg2:
            trgScale = upleg1.length + upleg2.length
        else:
            trgScale = upleg1.length
        srcScale = srcRig.data.bones["thigh.L"].length
        scale = trgScale/srcScale
        print("Rescale %s with factor %f" % (srcRig.name, scale))
        self.scale = scale

        bpy.ops.object.mode_set(mode='EDIT')
        ebones = srcRig.data.edit_bones
        for eb in ebones:
            oldlen = eb.length
            eb.head *= scale
            eb.tail *= scale
        bpy.ops.object.mode_set(mode='OBJECT')
        adata = srcRig.animation_data
        if adata is None:
            return
        for fcu in adata.action.fcurves:
            words = fcu.data_path.split('.')
            if words[-1] == 'location':
                for kp in fcu.keyframe_points:
                    kp.co[1] *= scale


    def renameAndRescaleBvh(self, context, srcRig, trgRig):
        if srcRig.McpRenamed:
            raise MocapError("%s already renamed and rescaled." % srcRig.name)

        from .t_pose import putInTPose

        scn = context.scene
        scn.frame_current = 0
        setActiveObject(context, srcRig)
        #(srcRig, srcBones, action) =  renameBvhRig(rig, filepath)
        self.findTarget(context, trgRig)
        self.findSource(context, srcRig)
        renameBones(srcRig, context)
        putInTPose(srcRig, scn.McpSourceTPose, context)
        setInterpolation(srcRig)
        self.rescaleRig(trgRig, srcRig)
        srcRig.McpRenamed = True

#----------------------------------------------------------
#   Object Problems
#----------------------------------------------------------

def checkObjectProblems(context):
    problems = ""
    epsilon = 1e-2
    rig = context.object

    eu = rig.rotation_euler
    if abs(eu.x) + abs(eu.y) + abs(eu.z) > epsilon:
        problems += "object rotation\n"

    vec = rig.scale - Vector((1,1,1))
    if vec.length > epsilon:
        problems += "object scaling\n"

    if problems:
        msg = ("BVH Retargeter cannot use this rig because it has:\n" +
               problems +
               "Apply object transformations before using BVH Retargeter")
        raise MocapError(msg)

########################################################################
#
#   class MCP_OT_LoadBvh(BvhOperator, MultiFile, BvhFile):
#

class MCP_OT_LoadBvh(HideOperator, MultiFile, BvhFile, BvhLoader):
    bl_idname = "mcp.load_bvh"
    bl_label = "Load BVH File"
    bl_description = "Load an armature from a bvh file"
    bl_options = {'UNDO'}

    def draw(self, context):
        self.layout.prop(self, "scale")
        BvhLoader.draw(self, context)

    def run(self, context):
        for filepath in self.getFilePaths():
            rig = self.readBvhFile(context, filepath, context.scene, False)
            bpy.ops.object.mode_set(mode='OBJECT')
            rig.select_set(True)
            context.view_layer.objects.active = rig
        print("BVH file(s) loaded")

    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}

#
#   class MCP_OT_RenameActiveToSelected(BvhOperator):
#

class MCP_OT_RenameActiveToSelected(BvhPropsOperator, IsArmature, TimeScaler, BvhRenamer):
    bl_idname = "mcp.rename_active_to_selected"
    bl_label = "Rename Selected From Active"
    bl_description = "Rename bones of selected (source) armatures and scale it to fit the active (target) armature"
    bl_options = {'UNDO'}

    def draw(self, context):
        BvhRenamer.draw(self, context)
        TimeScaler.draw(self, context)

    def run(self, context):
        scn = context.scene
        trgRig = context.object
        for srcRig in context.selected_objects:
            if srcRig != trgRig and srcRig.type == 'ARMATURE':
                self.renameAndRescaleBvh(context, srcRig, trgRig)
                if self.useTimeScale:
                    self.timescaleFCurves(srcRig)
                bpy.ops.object.mode_set(mode='OBJECT')
                print("%s renamed" % srcRig.name)
        context.view_layer.objects.active = trgRig

    def invoke(self, context, event):
        from .retarget import ensureInited
        ensureInited(context.scene)
        return BvhPropsOperator.invoke(self, context, event)

#
#   class MCP_OT_LoadAndRenameBvh(HideOperator, ImportHelper, BvhFile):
#

class MCP_OT_LoadAndRenameBvh(HideOperator, IsArmature, ImportHelper, BvhFile, BvhLoader, BvhRenamer, TimeScaler):
    bl_idname = "mcp.load_and_rename_bvh"
    bl_label = "Load And Rename BVH File"
    bl_description = "Load armature from bvh file and rename bones"
    bl_options = {'UNDO'}

    def draw(self, context):
        BvhLoader.draw(self, context)
        BvhRenamer.draw(self, context)
        TimeScaler.draw(self, context)

    def prequel(self, context):
        from .retarget import changeTargetData
        return changeTargetData(context.object, context.scene)

    def run(self, context):
        checkObjectProblems(context)
        scn = context.scene
        trgRig = context.object
        srcRig = self.readBvhFile(context, self.properties.filepath, scn, False)
        self.renameAndRescaleBvh(context, srcRig, trgRig)
        if self.useTimeScale:
            self.timescaleFCurves(srcRig)
        bpy.ops.object.mode_set(mode='OBJECT')
        srcRig.select_set(True)
        trgRig.select_set(True)
        context.view_layer.objects.active = trgRig
        print("%s loaded and renamed" % srcRig.name)

    def sequel(self, context, data):
        from .retarget import restoreTargetData
        restoreTargetData(data)

#----------------------------------------------------------
#   Initialize
#----------------------------------------------------------

classes = [
    MCP_OT_LoadBvh,
    MCP_OT_RenameActiveToSelected,
    MCP_OT_LoadAndRenameBvh,
]

def initialize():

    bpy.types.Object.McpRenamed = BoolProperty(default = False)

    for cls in classes:
        bpy.utils.register_class(cls)


def uninitialize():
    for cls in classes:
        bpy.utils.unregister_class(cls)
