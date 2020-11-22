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

bl_info = {
    "name": "BVH Retargeter",
    "author": "Thomas Larsson",
    "version": (2,0,1),
    "blender": (2,83,0),
    "location": "View3D > Tools > Retarget BVH",
    "description": "Mocap retargeting tool",
    "warning": "",
    'wiki_url': "https://diffeomorphic.blogspot.com/p/bvh-retargeter.html",
    "tracker_url": "https://bitbucket.org/Diffeomorphic/retarget_bvh/issues?status=new&status=open",
    "category": "Animation"}


# To support reload properly, try to access a package var, if it's there, reload everything
if "bpy" in locals():
    print("Reloading BVH Retargeter")
    import imp
    imp.reload(utils)
    imp.reload(io_json)
    imp.reload(armature)
    imp.reload(source)
    imp.reload(target)
    imp.reload(t_pose)
    imp.reload(simplify)
    imp.reload(load)
    imp.reload(fkik)
    imp.reload(retarget)
    imp.reload(action)
    imp.reload(loop)
    imp.reload(edit)
    imp.reload(floor)
    imp.reload(panels)
else:
    print("Loading BVH Retargeter")
    import bpy
    from . import utils
    from . import io_json
    from . import armature
    from . import source
    from . import target
    from . import t_pose
    from . import simplify
    from . import load
    from . import fkik
    from . import retarget
    from . import action
    from . import loop
    from . import edit
    from . import floor
    from . import panels

#----------------------------------------------------------
#   Import documented functions available for external scripting
#----------------------------------------------------------

from .utils import getErrorMessage, setSilentMode
from .retarget import ensureInited

#----------------------------------------------------------
#   Initialize
#----------------------------------------------------------

def register():

    action.initialize()
    edit.initialize()
    fkik.initialize()
    floor.initialize()
    load.initialize()
    loop.initialize()
    retarget.initialize()
    simplify.initialize()
    source.initialize()
    t_pose.initialize()
    target.initialize()
    panels.initialize()


def unregister():
    action.uninitialize()
    edit.uninitialize()
    fkik.uninitialize()
    floor.uninitialize()
    load.uninitialize()
    loop.uninitialize()
    retarget.uninitialize()
    simplify.uninitialize()
    source.uninitialize()
    t_pose.uninitialize()
    target.uninitialize()
    panels.uninitialize()


if __name__ == "__main__":
    register()

print("BVH Retargeter loaded")

