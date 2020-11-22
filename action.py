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
from bpy.props import EnumProperty, StringProperty
from .utils import *

#
#   ActionGroup
#

class ActionGroup(bpy.types.PropertyGroup):
    name : StringProperty()
    select : BoolProperty()
    fake : BoolProperty()
    users : IntProperty()


class ActionList:
    filter : StringProperty(
        name="Filter",
        description="Filter action names with this",
        default="")

    actions : CollectionProperty(type = ActionGroup)
    
    def draw(self, context):
        split = self.layout.split(factor = 0.5)
        split.label(text="Action")
        split.label(text="Select")
        split.label(text="Users")
        for act in self.actions:
            if self.filter in act.name:
                split = self.layout.split(factor = 0.6)
                split.label(text=act.name)
                split.prop(act, "select", text="")
                split.label(text = str(act.users))
        self.layout.separator()
        self.layout.prop(self, "filter")


    def invoke(self, context, event):
        animdata = context.object.animation_data
        self.actions.clear()
        for act in bpy.data.actions:
            item = self.actions.add()
            item.name = act.name
            item.select = self.selected(animdata, act)
            item.fake = act.use_fake_user
            item.users = act.users

        return BvhPropsOperator.invoke(self, context, event)
        

    def getActions(self, context):        
        acts = []
        for agrp in self.actions:
            if agrp.name in bpy.data.actions.keys():
                act = bpy.data.actions[agrp.name]
                acts.append((act, agrp.select))
        return acts                

#
#   Buttons:
#

class MCP_OT_DeleteAction(BvhOperator, IsArmature, ActionList):
    bl_idname = "mcp.delete_action"
    bl_label = "Delete Actions"
    bl_description = "Delete the action selected in the action list"
    bl_options = {'UNDO'}

    def selected(self, animdata, act):
        return False
        
    def run(self, context):
        self.failed = []
        for act,select in self.getActions(context):
            if select:
                self.deleteAction(act)
        if self.failed:       
            msg = ("Could not delete all actions.\n%s" % [act.name for act in self.failed])     
            raise MocapError(msg)

    def deleteAction(self, act):            
        act.use_fake_user = False
        if act.users == 0:
            bpy.data.actions.remove(act)
        else:
            self.failed.append(act)


def deleteAction(act):
    act.use_fake_user = False
    if act.users == 0:
        bpy.data.actions.remove(act)
    else:
        print("Action %s has %d users" % (act.name, act.users))


class MCP_OT_DeleteAllActions(BvhPropsOperator):
    bl_idname = "mcp.delete_all_actions"
    bl_label = "Delete All Actions"
    bl_description = "Delete all action"
    bl_options = {'UNDO'}

    def draw(self, context):
        self.layout.label(text="Really delete all actions?")
        
    def run(self, context):
        for act in bpy.data.actions:
            deleteAction(act)


class MCP_OT_DeleteHash(BvhOperator):

    bl_idname = "mcp.delete_hash"
    bl_label = "Delete Temporary Actions"
    bl_description = (
        "Delete all actions whose name start with '#'. " +
        "Such actions are created temporarily by BVH Retargeter. " +
        "They should be deleted automatically but may be left over."
    )
    bl_options = {'UNDO'}

    def run(self, context):
        for act in bpy.data.actions:
            if act.name[0] == '#':
                deleteAction(act)



class MCP_OT_SetCurrentAction(BvhOperator, IsArmature, ActionList):
    bl_idname = "mcp.set_current_action"
    bl_label = "Set Current Action"
    bl_description = "Set the action selected in the action list as the current action"
    bl_options = {'UNDO'}

    def selected(self, animdata, act):
        return (animdata and animdata.action == act)

    def run(self, context):
        for act,select in self.getActions(context):        
            if select:
                context.object.animation_data.action = act
                print("Action set to %s" % act)


class MCP_OT_SetAllFakeUser(BvhPropsOperator, IsArmature):
    bl_idname = "mcp.set_all_fake_user"
    bl_label = "Set All Fake Users"
    bl_description = "Add or remove fake users from all actions"
    bl_options = {'UNDO'}

    fake : BoolProperty(
        name = "Add Fake User",
        description = "Add or remove fake user",
        default = False)

    def draw(self, context):
        self.layout.prop(self, "fake")
    
    def run(self, context):
        for act in bpy.data.actions:
            act.use_fake_user = self.fake


class MCP_OT_SetFakeUser(BvhOperator, IsArmature, ActionList):
    bl_idname = "mcp.set_fake_user"
    bl_label = "Set Fake User"
    bl_description = "Make selected actions fake and others unfake"
    bl_options = {'UNDO'}

    def selected(self, animdata, act):
        return act.use_fake_user

    def run(self, context):
        for act,select in self.getActions(context):
            act.use_fake_user = select        


def getObjectAction(rig):
    if rig and rig.animation_data:
        return rig.animation_data.action
    else:
        return None

#----------------------------------------------------------
#   Initialize
#----------------------------------------------------------

classes = [
    ActionGroup,
    
    MCP_OT_DeleteAction,
    MCP_OT_DeleteAllActions,
    MCP_OT_DeleteHash,
    MCP_OT_SetCurrentAction,
    MCP_OT_SetFakeUser,
    MCP_OT_SetAllFakeUser,
]

def initialize():
    for cls in classes:
        bpy.utils.register_class(cls)


def uninitialize():
    for cls in classes:
        bpy.utils.unregister_class(cls)
