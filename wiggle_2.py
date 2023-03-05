bl_info = {
    "name": "Wiggle 2",
    "author": "Steve Miller",
    "version": (2, 0, 0),
    "blender": (3, 00, 0),
    "location": "3d Viewport > Animation Panel",
    "description": "Simulate spring-like physics on Bone transforms",
    "warning": "",
    "wiki_url": "",
    "category": "Animation",
}

### TO DO #####
# [x] Gravity
# [x] Rope on zero stiff
# [x] Animated
# [x] Skip frames
# [x] Scene mask
# [x] Indirect wiggle relationships

#implement a constant physics step

# Move object
# Efficient list
# Collision
# Mass
# Pinning

import bpy, math
from mathutils import Vector, Matrix, Euler, Quaternion, geometry
from bpy.app.handlers import persistent

#return m2 in m1 space
def relative_matrix(m1,m2):
    return (m2.inverted() @ m1).inverted()

def flatten(mat):
    dim = len(mat)
    return [mat[j][i] for i in range(dim) 
                      for j in range(dim)]

def update_prop(self,context,prop): 
    print(type(self) == bpy.types.Scene)
    if type(self) == bpy.types.PoseBone: 
        for b in context.selected_pose_bones:
            b[prop] = self[prop]
    if prop == 'wiggle_enable':
        print('should reset')
        bpy.ops.wiggle.reset()
        
def get_parent(b):
    p = b.parent
    if p:
        if p.wiggle_enable:
            return p
        else:
            return get_parent(p)
    else:
        return None
    
def update_matrix(b):
    mat = b.matrix
    p = get_parent(b)
    if p:
        mat = p.wiggle.matrix @ relative_matrix(p.matrix, b.matrix) 
        mat3 = Matrix.LocRotScale(mat.decompose()[0], mat.decompose()[1],b.matrix.decompose()[2]) 
    else:
        mat3 = mat
        
    tvec = relative_matrix(mat3, Matrix.Translation(b.wiggle.position)).translation
    rxz = tvec.to_track_quat('Y','Z')
    rot = rxz.to_matrix().to_4x4()
    
    if not p:
        sy = (b.head - b.wiggle.position).length/b.bone.length
    else:
        sy = (p.wiggle.matrix @ relative_matrix(p.matrix, b.matrix).translation - b.wiggle.position).length/b.bone.length
    scale = Matrix.Scale(sy,4,Vector((0,1,0)))

    if p:
        m4 = p.matrix @ relative_matrix(p.matrix, b.matrix)
        m5 = Matrix.LocRotScale(m4.decompose()[0], m4.decompose()[1],b.matrix.decompose()[2]) 
        b.matrix = m5 @ rot @ scale
    else:
        b.matrix = mat3 @ rot @ scale
    b.wiggle.matrix = flatten(mat3 @ rot @ scale)

def wiggle(b):
    mat = b.matrix
    p = get_parent(b)
    if p:
        mat = p.wiggle.matrix @ relative_matrix(p.matrix, b.matrix)
        mat = Matrix.LocRotScale(mat.decompose()[0], mat.decompose()[1],b.matrix.decompose()[2])
            
    dt = bpy.context.scene.wiggle.dt
    if dt:
        multiplier = (bpy.context.scene.render.fps/60)
        #damp
        damp = max(min(1-b.wiggle_damp*dt, 1),0) 
        b.wiggle.position += b.wiggle.velocity*damp
    
        #gravity
        Fg = bpy.context.scene.gravity * b.wiggle_gravity
        b.wiggle.position += Fg*dt*dt
        
        #stiff animated 
        target = mat @ Vector((0,b.bone.length,0))
        target = mat.translation + (target - mat.translation).normalized()*b.bone.length
        s = target - b.wiggle.position #spring offset
        Fs = b.wiggle_stiff * s * multiplier #spring force
        b.wiggle.position += Fs*dt*dt
        
    update_matrix(b)

def stretch(b,i):
#    target = b.wiggle.matrix.translation + (b.wiggle.position - b.wiggle.matrix.translation).normalized()*b.bone.length
#    s = (target - b.wiggle.position) * (1-b.wiggle_stretch)
    if b == bpy.context.active_pose_bone:
        bpy.context.scene.cursor.location = b.wiggle.matrix.translation
    p = get_parent(b) 
    if p:
        target = p.wiggle.position + (b.wiggle.position - p.wiggle.position).normalized()*b.bone.length
        s = (target - b.wiggle.position)*(1-b.wiggle_stretch)
        if p == b.parent and i:
            if b.wiggle_mass==p.wiggle_mass:
                fac = 0.5
            else:
                fac = b.wiggle_mass/(p.wiggle_mass + b.wiggle_mass)
            p.wiggle.position -= s*fac
            b.wiggle.position += s*(1-fac)
        else:
            b.wiggle.position += s
        update_matrix(p)
    else:
        target = b.head + (b.wiggle.position - b.head).normalized()*b.bone.length
        s = (target - b.wiggle. position) * (1-b.wiggle_stretch)
        b.wiggle.position += s
    update_matrix(b)
            
    
@persistent
def wiggle_pre(scene):
    if not scene.wiggle_enable: return
    ob = bpy.context.active_object
    if not ob.type == 'ARMATURE': return
    for b in ob.pose.bones:
        if not b.wiggle_enable: continue
        b.location = Vector((0,0,0))
        b.rotation_quaternion = Quaternion((1,0,0,0))
        b.rotation_euler = Vector((0,0,0))
        b.scale = Vector((1,1,1))
#        bpy.context.view_layer.update()

@persistent                
def wiggle_post(scene):
    if not scene.wiggle_enable: return

    lastframe = scene.wiggle.lastframe
    if scene.frame_current >= lastframe:
        frames_elapsed = scene.frame_current - lastframe
    else:
        e1 = (scene.frame_end - lastframe) + (scene.frame_current - scene.frame_start) + 1
        e2 = lastframe - scene.frame_current
        frames_elapsed = min(e1,e2)
    if frames_elapsed > 4: frames_elapsed = 1 #handle large jumps?
    scene.wiggle.dt = 1/scene.render.fps * frames_elapsed
    scene.wiggle.lastframe = scene.frame_current
    
    #for now do all bones on selected object: (fix with list structure later)
    for ob in bpy.context.selected_objects:
        if not ob.type == 'ARMATURE': continue
        
        for b in ob.pose.bones:
            if b.wiggle_enable:
                wiggle(b)
        count = 2
        for i in range(scene.wiggle.iterations):
            for b in ob.pose.bones:
                if b.wiggle_enable:
                    stretch(b,scene.wiggle.iterations-1-i)
#        for b in ob.pose.bones:
#            if b.wiggle_enable:
#                update_matrix(b)
        if frames_elapsed:
            for b in ob.pose.bones:
                if b.wiggle_enable:
                    b.wiggle.velocity = (b.wiggle.position - b.wiggle.position_last)/max(frames_elapsed,1)
                    b.wiggle.position_last = b.wiggle.position
            
class WiggleReset(bpy.types.Operator):
    """Reset wiggle physics to rest state"""
    bl_idname = "wiggle.reset"
    bl_label = "Reset Wiggle State"
    
    @classmethod
    def poll(cls,context):
        return context.mode in ['OBJECT', 'POSE']
    
    def execute(self,context):
        for ob in context.selected_objects:
            if not ob.type == 'ARMATURE': continue
            for b in ob.pose.bones:
                if not b.wiggle_enable: continue
                rest_mat = b.bone.matrix_local
                if b.parent:
                    rest_mat = b.parent.matrix @ relative_matrix(b.parent.bone.matrix_local, b.bone.matrix_local)
                b.wiggle.position = b.wiggle.position_last = rest_mat @ Vector((0,b.bone.length,0))
                b.wiggle.velocity = Vector((0,0,0))
                b.wiggle.matrix = flatten(rest_mat)
                b.matrix = rest_mat
                context.scene.wiggle.lastframe = context.scene.frame_current
                context.scene.frame_set(context.scene.frame_current)
        print('reset')
        return {'FINISHED'}
    
class WIGGLE_PT_Settings(bpy.types.Panel):
    bl_category = 'Animation'
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_label = 'Wiggle 2'
    
    @classmethod
    def poll(cls, context):
        return context.active_object
    
    def draw(self,context):
        layout = self.layout
        layout.operator('wiggle.reset')
        layout.prop(context.scene.wiggle, 'iterations')
        row = layout.row()
        row.prop(context.scene, 'wiggle_enable', icon = 'SCENE_DATA',icon_only=True)
        if not context.scene.wiggle_enable or context.active_object.mode != 'POSE' or not context.active_pose_bone:
            return
        b = context.active_pose_bone
        row.prop(b, 'wiggle_enable', icon = 'BONE_DATA',icon_only=True)
        row.label(text=b.name)
        if not b.wiggle_enable:
            return
        layout.prop(b, 'wiggle_mass')
        layout.prop(b, 'wiggle_stiff')
        layout.prop(b, 'wiggle_stretch')
        layout.prop(b, 'wiggle_damp')
        layout.prop(b, 'wiggle_gravity')

#store properties for a bone. custom properties for user editable. property group for internal calculations
class WiggleBone(bpy.types.PropertyGroup):
    matrix: bpy.props.FloatVectorProperty(name = 'Matrix', size=16, subtype = 'MATRIX')
    position: bpy.props.FloatVectorProperty(subtype='TRANSLATION')
    position_last: bpy.props.FloatVectorProperty(subtype='TRANSLATION')
    velocity: bpy.props.FloatVectorProperty(subtype='VELOCITY')

class WiggleScene(bpy.types.PropertyGroup):
    dt: bpy.props.FloatProperty()
    lastframe: bpy.props.IntProperty()
    iterations: bpy.props.IntProperty(name='Chain Quality', min=1, default=1, max=4)

def register():
    #user variables
    bpy.types.Scene.wiggle_enable = bpy.props.BoolProperty(
        name = 'Enable',
        description = 'Enable jiggle on this scene',
        default = False,
        override={'LIBRARY_OVERRIDABLE'},
        update=lambda s, c: update_prop(s, c, 'wiggle_enable')
    )
    bpy.types.PoseBone.wiggle_enable = bpy.props.BoolProperty(
        name = 'Enable',
        description = 'Enable jiggle on this bone',
        default = False,
        override={'LIBRARY_OVERRIDABLE'},
        update=lambda s, c: update_prop(s, c, 'wiggle_enable')
    )
    bpy.types.PoseBone.wiggle_mass = bpy.props.FloatProperty(
        name = 'Mass',
        description = 'Mass of bone',
        min = 0.01,
        default = 1,
        override={'LIBRARY_OVERRIDABLE'},
        update=lambda s, c: update_prop(s, c, 'wiggle_mass')
    )
    bpy.types.PoseBone.wiggle_stiff = bpy.props.FloatProperty(
        name = 'Stiff',
        description = 'Stiffness coefficient',
        min = 0,
        default = 20,
        override={'LIBRARY_OVERRIDABLE'},
        update=lambda s, c: update_prop(s, c, 'wiggle_stiff')
    )
    bpy.types.PoseBone.wiggle_stretch = bpy.props.FloatProperty(
        name = 'Stretch',
        description = 'Stretch factor',
        min = 0,
        default = 0,
        max=1,
        override={'LIBRARY_OVERRIDABLE'},
        update=lambda s, c: update_prop(s, c, 'wiggle_stretch')
    )
    bpy.types.PoseBone.wiggle_damp = bpy.props.FloatProperty(
        name = 'Damp',
        description = 'Dampening coefficient',
        min = 0,
        default = 1,
        override={'LIBRARY_OVERRIDABLE'},
        update=lambda s, c: update_prop(s, c, 'wiggle_damp')
    )
    bpy.types.PoseBone.wiggle_gravity = bpy.props.FloatProperty(
        name = 'Gravity',
        description = 'Gravity influence on bone',
        default = 1,
        override={'LIBRARY_OVERRIDABLE'},
        update=lambda s, c: update_prop(s, c, 'wiggle_gravity')
    )
    
    #internal variables
    bpy.utils.register_class(WiggleBone)
    bpy.types.PoseBone.wiggle = bpy.props.PointerProperty(type=WiggleBone, override={'LIBRARY_OVERRIDABLE'})
    
    bpy.utils.register_class(WiggleScene)
    bpy.types.Scene.wiggle = bpy.props.PointerProperty(type=WiggleScene, override={'LIBRARY_OVERRIDABLE'})
    
    bpy.utils.register_class(WiggleReset)
    bpy.utils.register_class(WIGGLE_PT_Settings)
    
    bpy.app.handlers.frame_change_pre.clear()
    bpy.app.handlers.frame_change_post.clear()
    bpy.app.handlers.render_pre.clear()
    bpy.app.handlers.render_post.clear()
    
    bpy.app.handlers.frame_change_pre.append(wiggle_pre)
    bpy.app.handlers.frame_change_post.append(wiggle_post)

def unregister():
    bpy.utils.unregister_class(WiggleBone)
    bpy.utils.unregister_class(WiggleScene)
    bpy.utils.unregister_class(WiggleReset)
    bpy.utils.unregister_class(WIGGLE_PT_Settings)
    
if __name__ == "__main__":
    register()
