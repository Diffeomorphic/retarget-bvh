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
from mathutils import Vector, Matrix
from bpy.props import *
from .utils import *
from .target import Target

#-------------------------------------------------------------
#   Limbs bend positive
#-------------------------------------------------------------

class Bender:
    useElbows : BoolProperty(
        name="Elbows",
        description="Keep elbow bending positive",
        default=True)

    useKnees : BoolProperty(
        name="Knees",
        description="Keep knee bending positive",
        default=True)

    useBendPositive : BoolProperty(
        name="Bend Positive",
        description="Ensure that elbow and knee bending is positive",
        default=True)

    def draw(self, context):
        self.layout.prop(self, "useElbows")
        self.layout.prop(self, "useKnees")

    def limbsBendPositive(self, rig, frames):
        limbs = {}
        if self.useElbows:
            pb = getTrgBone("forearm.L", rig)
            self.minimizeFCurve(pb, rig, 0, frames)
            pb = getTrgBone("forearm.R", rig)
            self.minimizeFCurve(pb, rig, 0, frames)
        if self.useKnees:
            pb = getTrgBone("shin.L", rig)
            self.minimizeFCurve(pb, rig, 0, frames)
            pb = getTrgBone("shin.R", rig)
            self.minimizeFCurve(pb, rig, 0, frames)


    def minimizeFCurve(self, pb, rig, index, frames):
        from .floor import findBoneFCurve
        if pb is None:
            return
        fcu = findBoneFCurve(pb, rig, index)
        if fcu is None:
            return
        y0 = fcu.evaluate(0)
        t0 = frames[0]
        t1 = frames[-1]
        for kp in fcu.keyframe_points:
            t = kp.co[0]
            if t >= t0 and t <= t1:
                y = kp.co[1]
                if y < y0:
                    kp.co[1] = y0


class MCP_OT_LimbsBendPositive(BvhPropsOperator, IsArmature, Bender, Target):
    bl_idname = "mcp.limbs_bend_positive"
    bl_label = "Bend Limbs Positive"
    bl_description = "Ensure that limbs' X rotation is positive."
    bl_options = {'UNDO'}

    def prequel(self, context):
        rig = context.object
        return (rig, list(rig.data.layers))

    def run(self, context):
        from .loop import getActiveFramesBetweenMarkers

        scn = context.scene
        rig = context.object
        self.findTarget(context, rig)
        frames = getActiveFramesBetweenMarkers(rig, scn)
        self.limbsBendPositive(rig, frames)
        print("Limbs bent positive")

    def sequel(self, context, data):
        rig,layers = data
        rig.data.layers = layers

#-------------------------------------------------------------
#
#-------------------------------------------------------------

def getPoseMatrix(gmat, pb):
    restInv = pb.bone.matrix_local.inverted()
    if pb.parent:
        parInv = pb.parent.matrix.inverted()
        parRest = pb.parent.bone.matrix_local
        return restInv @ parRest @ parInv @ gmat
    else:
        return restInv @ gmat


def getGlobalMatrix(mat, pb):
    gmat = pb.bone.matrix_local @ mat
    if pb.parent:
        parMat = pb.parent.matrix
        parRest = pb.parent.bone.matrix_local
        return parMat @ parRest.inverted() @ gmat
    else:
        return gmat


def matchPoseTranslation(pb, src):
    pmat = getPoseMatrix(src.matrix, pb)
    insertLocation(pb, pmat)


def matchPoseRotation(pb, src):
    pmat = getPoseMatrix(src.matrix, pb)
    insertRotation(pb, pmat)


def matchPoseTwist(pb, src):
    pmat0 = src.matrix_basis
    euler = pmat0.to_3x3().to_euler('YZX')
    euler.z = 0
    pmat = euler.to_matrix().to_4x4()
    pmat.col[3] = pmat0.col[3]
    insertRotation(pb, pmat)


def printMatrix(string,mat):
    print(string)
    for i in range(4):
        print("    %.4g %.4g %.4g %.4g" % tuple(mat[i]))


def matchIkLeg(legIk, toeFk, mBall, mToe, mHeel):
    rmat = toeFk.matrix.to_3x3()
    tHead = Vector(toeFk.matrix.col[3][:3])
    ty = rmat.col[1]
    tail = tHead + ty * toeFk.bone.length

    zBall = mBall.matrix.col[3][2]
    zToe = mToe.matrix.col[3][2]
    zHeel = mHeel.matrix.col[3][2]

    x = Vector(rmat.col[0])
    y = Vector(rmat.col[1])
    z = Vector(rmat.col[2])

    if zHeel > zBall and zHeel > zToe:
        # 1. foot.ik is flat
        if abs(y[2]) > abs(z[2]):
            y = -z
        y[2] = 0
    else:
        # 2. foot.ik starts at heel
        hHead = Vector(mHeel.matrix.col[3][:3])
        y = tail - hHead

    y.normalize()
    x -= x.dot(y)*y
    x.normalize()
    if abs(x[2]) < 0.7:
        x[2] = 0
        x.normalize()
    z = x.cross(y)
    head = tail - y * legIk.bone.length

    # Create matrix
    gmat = Matrix()
    gmat.col[0][:3] = x
    gmat.col[1][:3] = y
    gmat.col[2][:3] = z
    gmat.col[3][:3] = head
    pmat = getPoseMatrix(gmat, legIk)

    insertLocation(legIk, pmat)
    insertRotation(legIk, pmat)


def matchPoleTarget(pb, above, below):
    x = Vector(above.matrix.col[1][:3])
    y = Vector(below.matrix.col[1][:3])
    p0 = Vector(below.matrix.col[3][:3])
    n = x.cross(y)
    if abs(n.length) > 1e-4:
        z = x - y
        n.normalize()
        z -= z.dot(n)*n
        z.normalize()
        p = p0 + 6*pb.length*z
    else:
        p = p0
    gmat = Matrix.Translation(p)
    pmat = getPoseMatrix(gmat, pb)
    insertLocation(pb, pmat)


def matchPoseReverse(pb, src):
    gmat = src.matrix
    tail = gmat.col[3] + src.length * gmat.col[1]
    rmat = Matrix((gmat.col[0], -gmat.col[1], -gmat.col[2], tail))
    rmat.transpose()
    pmat = getPoseMatrix(rmat, pb)
    pb.matrix_basis = pmat
    insertRotation(pb, pmat)


def matchPoseScale(pb, src):
    pmat = getPoseMatrix(src.matrix, pb)
    pb.scale = pmat.to_scale()
    pb.keyframe_insert("scale", group=pb.name)


def snapFkArm(rig, snapIk, snapFk, frame):

    (uparmFk, loarmFk, handFk) = snapFk
    (uparmIk, loarmIk, elbow, elbowPt, handIk) = snapIk

    matchPoseRotation(uparmFk, uparmIk)
    matchPoseRotation(loarmFk, loarmIk)
    matchPoseRotation(handFk, handIk)


def snapIkArm(rig, snapIk, snapFk, frame):

    (uparmIk, loarmIk, elbow, elbowPt, handIk) = snapIk
    (uparmFk, loarmFk, handFk) = snapFk

    matchPoseTranslation(handIk, handFk)
    matchPoseRotation(handIk, handFk)
    updateScene()

    matchPoleTarget(elbowPt, uparmFk, loarmFk)

    #matchPoseRotation(uparmIk, uparmFk)
    #matchPoseRotation(loarmIk, loarmFk)


def snapFkLeg(rig, snapIk, snapFk, frame, legIkToAnkle):

    (uplegIk, lolegIk, kneePt, ankleIk, legIk, footRev, toeRev, mBall, mToe, mHeel) = snapIk
    (uplegFk, lolegFk, footFk, toeFk) = snapFk

    matchPoseRotation(uplegFk, uplegIk)
    matchPoseRotation(lolegFk, lolegIk)
    if not legIkToAnkle:
        matchPoseReverse(footFk, footRev)
        matchPoseReverse(toeFk, toeRev)


def snapIkLeg(rig, snapIk, snapFk, frame, legIkToAnkle):

    (uplegIk, lolegIk, kneePt, ankleIk, legIk, footRev, toeRev, mBall, mToe, mHeel) = snapIk
    (uplegFk, lolegFk, footFk, toeFk) = snapFk

    if legIkToAnkle:
        matchPoseTranslation(ankleIk, footFk)
    else:
        matchIkLeg(legIk, toeFk, mBall, mToe, mHeel)

    matchPoseTwist(lolegIk, lolegFk)
    updateScene()

    matchPoseReverse(toeRev, toeFk)
    updateScene()
    matchPoseReverse(footRev, footFk)
    updateScene()

    matchPoleTarget(kneePt, uplegFk, lolegFk)

    if not legIkToAnkle:
        matchPoseTranslation(ankleIk, footFk)


SnapBonesAlpha8 = {
    "Arm"   : ["upper_arm", "forearm", "hand"],
    "ArmFK" : ["upper_arm.fk", "forearm.fk", "hand.fk"],
    "ArmIK" : ["upper_arm.ik", "forearm.ik", None, "elbow.pt.ik", "hand.ik"],
    "Leg"   : ["thigh", "shin", "foot", "toe"],
    "LegFK" : ["thigh.fk", "shin.fk", "foot.fk", "toe.fk"],
    "LegIK" : ["thigh.ik", "shin.ik", "knee.pt.ik", "ankle.ik", "foot.ik", "foot.rev", "toe.rev", "ball.marker", "toe.marker", "heel.marker"],
}

def getSnapBones(rig, key, suffix):
    try:
        rig.pose.bones["thigh.fk.L"]
        names = SnapBonesAlpha8[key]
        suffix = '.' + suffix[1:]
    except KeyError:
        names = None
    if not names:
        raise MocapError("Not an mhx armature")

    pbones = []
    constraints = []
    for name in names:
        if name:
            pb = rig.pose.bones[name+suffix]
            pbones.append(pb)
            for cns in pb.constraints:
                if cns.type == 'LIMIT_ROTATION' and not cns.mute:
                    constraints.append(cns)
        else:
            pbones.append(None)
    return tuple(pbones),constraints


def muteConstraints(constraints, value):
    for cns in constraints:
        cns.mute = value


class Transferer(Bender, Target):
    useArms : BoolProperty(
        name="Include Arms",
        description="Include arms in FK/IK snapping",
        default=False)

    useLegs : BoolProperty(
        name="Include Legs",
        description="Include legs in FK/IK snapping",
        default=True)

    def draw(self, context):
        self.layout.prop(self, "useArms")
        self.layout.prop(self, "useLegs")
        Target.draw(self, context)


    def clearAnimation(self, rig, context, act, type, snapBones):
        scn = context.scene
        self.findTarget(context, rig)

        ikBones = []
        if self.useArms:
            for bname in snapBones["Arm" + type]:
                if bname is not None:
                    ikBones += [bname+".L", bname+".R"]
        if self.useLegs:
            for bname in snapBones["Leg" + type]:
                if bname is not None:
                    ikBones += [bname+".L", bname+".R"]

        ikFCurves = []
        for fcu in act.fcurves:
            words = fcu.data_path.split('"')
            if (words[0] == "pose.bones[" and
                words[1] in ikBones):
                ikFCurves.append(fcu)

        if ikFCurves == []:
            raise MocapError("%s bones have no animation" % type)

        for fcu in ikFCurves:
            act.fcurves.remove(fcu)


    def transferMhxToFk(self, rig, context):
        from .loop import getActiveFramesBetweenMarkers

        scn = context.scene
        self.findTarget(context, rig)

        lArmSnapIk,lArmCnsIk = getSnapBones(rig, "ArmIK", "_L")
        lArmSnapFk,lArmCnsFk = getSnapBones(rig, "ArmFK", "_L")
        rArmSnapIk,rArmCnsIk = getSnapBones(rig, "ArmIK", "_R")
        rArmSnapFk,rArmCnsFk = getSnapBones(rig, "ArmFK", "_R")
        lLegSnapIk,lLegCnsIk = getSnapBones(rig, "LegIK", "_L")
        lLegSnapFk,lLegCnsFk = getSnapBones(rig, "LegFK", "_L")
        rLegSnapIk,rLegCnsIk = getSnapBones(rig, "LegIK", "_R")
        rLegSnapFk,rLegCnsFk = getSnapBones(rig, "LegFK", "_R")

        #muteAllConstraints(rig, True)

        oldLayers = list(rig.data.layers)
        setMhxIk(rig, self.useArms, self.useLegs, 1.0)
        rig.data.layers = MhxLayers

        lLegIkToAnkle = rig["MhaLegIkToAnkle_L"]
        rLegIkToAnkle = rig["MhaLegIkToAnkle_R"]

        frames = getActiveFramesBetweenMarkers(rig, scn)
        nFrames = len(frames)
        self.useKnees = self.useElbows = True
        self.limbsBendPositive(rig, frames)

        for n,frame in enumerate(frames):
            showProgress(n, frame, nFrames)
            scn.frame_set(frame)
            updateScene()
            if self.useArms:
                snapFkArm(rig, lArmSnapIk, lArmSnapFk, frame)
                snapFkArm(rig, rArmSnapIk, rArmSnapFk, frame)
            if self.useLegs:
                snapFkLeg(rig, lLegSnapIk, lLegSnapFk, frame, lLegIkToAnkle)
                snapFkLeg(rig, rLegSnapIk, rLegSnapFk, frame, rLegIkToAnkle)

        rig.data.layers = oldLayers
        setMhxIk(rig, self.useArms, self.useLegs, 0.0)
        setInterpolation(rig)
        #muteAllConstraints(rig, False)


    def transferMhxToIk(self, rig, context):
        from .loop import getActiveFramesBetweenMarkers

        scn = context.scene
        self.findTarget(context, rig)

        lArmSnapIk,lArmCnsIk = getSnapBones(rig, "ArmIK", "_L")
        lArmSnapFk,lArmCnsFk = getSnapBones(rig, "ArmFK", "_L")
        rArmSnapIk,rArmCnsIk = getSnapBones(rig, "ArmIK", "_R")
        rArmSnapFk,rArmCnsFk = getSnapBones(rig, "ArmFK", "_R")
        lLegSnapIk,lLegCnsIk = getSnapBones(rig, "LegIK", "_L")
        lLegSnapFk,lLegCnsFk = getSnapBones(rig, "LegFK", "_L")
        rLegSnapIk,rLegCnsIk = getSnapBones(rig, "LegIK", "_R")
        rLegSnapFk,rLegCnsFk = getSnapBones(rig, "LegFK", "_R")

        #muteAllConstraints(rig, True)

        oldLayers = list(rig.data.layers)
        setMhxIk(rig, self.useArms, self.useLegs, 0.0)
        rig.data.layers = MhxLayers

        lLegIkToAnkle = rig["MhaLegIkToAnkle_L"]
        rLegIkToAnkle = rig["MhaLegIkToAnkle_R"]

        frames = getActiveFramesBetweenMarkers(rig, scn)
        #frames = range(scn.frame_start, scn.frame_end+1)
        nFrames = len(frames)
        for n,frame in enumerate(frames):
            showProgress(n, frame, nFrames)
            scn.frame_set(frame)
            updateScene()
            if self.useArms:
                snapIkArm(rig, lArmSnapIk, lArmSnapFk, frame)
                snapIkArm(rig, rArmSnapIk, rArmSnapFk, frame)
            if self.useLegs:
                snapIkLeg(rig, lLegSnapIk, lLegSnapFk, frame, lLegIkToAnkle)
                snapIkLeg(rig, rLegSnapIk, rLegSnapFk, frame, rLegIkToAnkle)

        rig.data.layers = oldLayers
        setMhxIk(rig, self.useArms, self.useLegs, 1.0)
        setInterpolation(rig)
        #muteAllConstraints(rig, False)



def muteAllConstraints(rig, value):
    lArmSnapIk,lArmCnsIk = getSnapBones(rig, "ArmIK", "_L")
    lArmSnapFk,lArmCnsFk = getSnapBones(rig, "ArmFK", "_L")
    rArmSnapIk,rArmCnsIk = getSnapBones(rig, "ArmIK", "_R")
    rArmSnapFk,rArmCnsFk = getSnapBones(rig, "ArmFK", "_R")
    lLegSnapIk,lLegCnsIk = getSnapBones(rig, "LegIK", "_L")
    lLegSnapFk,lLegCnsFk = getSnapBones(rig, "LegFK", "_L")
    rLegSnapIk,rLegCnsIk = getSnapBones(rig, "LegIK", "_R")
    rLegSnapFk,rLegCnsFk = getSnapBones(rig, "LegFK", "_R")

    muteConstraints(lArmCnsIk, value)
    muteConstraints(lArmCnsFk, value)
    muteConstraints(rArmCnsIk, value)
    muteConstraints(rArmCnsFk, value)
    muteConstraints(lLegCnsIk, value)
    muteConstraints(lLegCnsFk, value)
    muteConstraints(rLegCnsIk, value)
    muteConstraints(rLegCnsFk, value)

#------------------------------------------------------------------------
#
#------------------------------------------------------------------------

def setLocation(bname, rig):
    pb = rig.pose.bones[bname]
    pb.keyframe_insert("location", group=pb.name)


def setRotation(bname, rig):
    pb = rig.pose.bones[bname]
    if pb.rotation_mode == 'QUATERNION':
        pb.keyframe_insert("rotation_quaternion", group=pb.name)
    else:
        pb.keyframe_insert("rotation_euler", group=pb.name)


def setLocRot(bname, rig):
    pb = rig.pose.bones[bname]
    pb.keyframe_insert("location", group=pb.name)
    pb = rig.pose.bones[bname]
    if pb.rotation_mode == 'QUATERNION':
        pb.keyframe_insert("rotation_quaternion", group=pb.name)
    else:
        pb.keyframe_insert("rotation_euler", group=pb.name)


def setMhxIk(rig, useArms, useLegs, value):
    if isMhxRig(rig):
        ikLayers = []
        fkLayers = []
        if useArms:
            rig["MhaArmIk_L"] = value
            rig["MhaArmIk_R"] = value
            ikLayers += [2,18]
            fkLayers += [3,19]
        if useLegs:
            rig["MhaLegIk_L"] = value
            rig["MhaLegIk_R"] = value
            ikLayers += [4,20]
            fkLayers += [5,21]

        if value:
            first = ikLayers
            second = fkLayers
        else:
            first = fkLayers
            second = ikLayers
        for n in first:
            rig.data.layers[n] = True
        for n in second:
            rig.data.layers[n] = False


def setRigifyFKIK(rig, value):
    rig.pose.bones["hand.ik.L"]["ikfk_switch"] = value
    rig.pose.bones["hand.ik.R"]["ikfk_switch"] = value
    rig.pose.bones["foot.ik.L"]["ikfk_switch"] = value
    rig.pose.bones["foot.ik.R"]["ikfk_switch"] = value
    on = (value < 0.5)
    for n in [6, 9, 12, 15]:
        rig.data.layers[n] = on
    for n in [7, 10, 13, 16]:
        rig.data.layers[n] = not on


def setRigify2FKIK(rig, value):
    rig.pose.bones["upper_arm_parent.L"]["IK_FK"] = value
    rig.pose.bones["upper_arm_parent.R"]["IK_FK"] = value
    rig.pose.bones["thigh_parent.L"]["IK_FK"] = value
    rig.pose.bones["thigh_parent.R"]["IK_FK"] = value
    on = (value > 0.5)
    for n in [8, 11, 14, 17]:
        rig.data.layers[n] = on
    for n in [7, 10, 13, 16]:
        rig.data.layers[n] = not on
    torso = rig.pose.bones["torso"]
    torso["head_follow"] = 1.0
    torso["neck_follow"] = 1.0


def setRigToFK(rig):
    setMhxIk(rig, True, True, 0.0)
    if isRigify(rig):
        setRigifyFKIK(rig, 0.0)
    elif isRigify2(rig):
        setRigify2FKIK(rig, 1.0)

#------------------------------------------------------------------------
#   Buttons
#------------------------------------------------------------------------

def disableGlobalUndo(context):
    return None
    undo = context.user_preferences.edit.use_global_undo
    context.user_preferences.edit.use_global_undo = False
    return undo


def restoreGlobalUndo(context, undo):
    return
    context.user_preferences.edit.use_global_undo = undo


class MCP_OT_TransferToFk(BvhPropsOperator, IsMhx, Transferer):
    bl_idname = "mcp.transfer_to_fk"
    bl_label = "Transfer IK => FK"
    bl_description = "Transfer IK animation to FK bones"
    bl_options = {'UNDO'}

    def prequel(self, context):
        return disableGlobalUndo(context)

    def run(self, context):
        startProgress("Transfer to FK")
        rig = context.object
        scn = context.scene
        if isMhxRig(rig):
            self.transferMhxToFk(rig, context)
        else:
            raise MocapError("Can not transfer to FK with this rig")
        raise MocapMessage("Transfer to FK completed")

    def sequel(self, context, undo):
        return restoreGlobalUndo(context, undo)



class MCP_OT_TransferToIk(BvhPropsOperator, IsMhx, Transferer):
    bl_idname = "mcp.transfer_to_ik"
    bl_label = "Transfer FK => IK"
    bl_description = "Transfer FK animation to IK bones"
    bl_options = {'UNDO'}

    def prequel(self, context):
        return disableGlobalUndo(context)

    def run(self, context):
        startProgress("Transfer to IK")
        rig = context.object
        scn = context.scene
        if isMhxRig(rig):
            self.transferMhxToIk(rig, context)
        elif isRigify(rig):
            self.transferRigifyToIk(rig, context, ".")
        elif isRigify2(rig):
            self.transferRigifyToIk(rig, context, "_")
        else:
            raise MocapError("Can not transfer to IK with this rig")
        raise MocapMessage("Transfer to IK completed")

    def sequel(self, context, undo):
        return restoreGlobalUndo(context, undo)


class MCP_OT_ClearAnimation(HideOperator, IsMhx, Transferer):
    bl_idname = "mcp.clear_animation"
    bl_label = "Clear Animation"
    bl_description = "Clear Animation For FK or IK Bones"
    bl_options = {'UNDO'}

    type : StringProperty()

    def prequel(self, context):
        return disableGlobalUndo(context)

    def run(self, context):
        startProgress("Clear animation")
        rig = context.object
        scn = context.scene
        if not rig.animation_data:
            raise MocapError("Rig has no animation data")
        act = rig.animation_data.action
        if not act:
            raise MocapError("Rig has no action")

        if isMhxRig(rig):
            self.clearAnimation(rig, context, act, self.type, SnapBonesAlpha8)
            if self.type == "FK":
                value = 1.0
            else:
                value = 0.0
            setMhxIk(rig, self.useArms, self.useLegs, value)
        elif isRigify(rig):
            self.clearAnimation(rig, context, act, self.type, SnapBonesRigify)
        else:
            raise MocapError("Can not clear %s animation with this rig" % self.type)
        raise MocapMessage("Animation cleared")

    def sequel(self, context, undo):
        return restoreGlobalUndo(context, undo)

#------------------------------------------------------------------------
#   Debug
#------------------------------------------------------------------------

def printHand(context):
        rig = context.object
        '''
        handFk = rig.pose.bones["hand.fk.L"]
        handIk = rig.pose.bones["hand.ik.L"]
        print(handFk)
        print(handFk.matrix)
        print(handIk)
        print(handIk.matrix)
        '''
        footIk = rig.pose.bones["foot.ik.L"]
        print(footIk)
        print(footIk.matrix)


class MCP_OT_PrintHands(BvhOperator, IsArmature):
    bl_idname = "mcp.print_hands"
    bl_label = "Print Hands"
    bl_options = {'UNDO'}

    def run(self, context):
        printHand(context)


class MCP_OT_Test(BvhOperator, IsArmature):
    bl_idname = "mcp.test"
    bl_label = "Test"
    bl_options = {'UNDO'}

    def run(self, context):
        rig = context.object
        print("TRIK", rig)
        print(rig.data.rigify_rig_ui)

#----------------------------------------------------------
#   Initialize
#----------------------------------------------------------

classes = [
    MCP_OT_LimbsBendPositive,
    MCP_OT_TransferToFk,
    MCP_OT_TransferToIk,
    MCP_OT_ClearAnimation,
    MCP_OT_PrintHands,
    MCP_OT_Test,
]

def initialize():
    for cls in classes:
        bpy.utils.register_class(cls)


def uninitialize():
    for cls in classes:
        bpy.utils.unregister_class(cls)

