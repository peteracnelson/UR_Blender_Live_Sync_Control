import sys, subprocess
import bpy, socket, math, time, threading, struct
import mathutils
from mathutils import Matrix, Vector, Euler

# ==============================================================================
# [1] DEPENDENCY CHECK & AUTO-INSTALLER
# ==============================================================================
def ensure_dependencies():
    """Checks and auto-installs required external Python modules."""
    required_modules = [] 
    for module in required_modules:
        try:
            __import__(module)
        except ImportError:
            print(f"[Auto-Install] Installing missing module: {module}...")
            subprocess.check_call([sys.executable, "-m", "pip", "install", module])

ensure_dependencies()

# ==============================================================================
# [2] CONFIGURATION
# ==============================================================================
# Scene Objects
TRACK_TARGET_NAME = "pen_Bone"            
ROBOT_CTRL_NAME   = "mocap_cleaned"       
OBJ_NAME          = "TCP_Empty"           
BOX_NAME          = "bounding_box"        

# Network Settings
DEF_IP = '192.168.0.100'
PORT_SCRIPT = 30003
PORT_DASH = 29999

# Kinematics & Safety
DEF_MAX_SPEED = 0.15   
DEF_ROT_SMOOTH = 0.10  
DEF_LOGIC_ENABLE = 'YES'

# Physical Robot Home Position (Safe parking spot: X, Y, Z in meters)
ROBOT_HOME_POS  = [-0.2983, 0.1314, 0.304]  

# Visual Tool Offset (Z-axis height in Blender)
# IMPORTANT: If you change to a longer or shorter pen, you MUST update this value 
# to match the new tool tip's Z-height in your Blender scene.
TOOL_OFFSET = 0.219
 
# ServoJ Constants
INTERNAL_T = 0.03        
INTERNAL_LH = 0.03       
INTERNAL_GAIN = 300      

# ==============================================================================
# [3] GLOBAL STATE
# ==============================================================================
class Global:
    is_live = False
    stop_requested = False 
    lock = threading.Lock()
    pose = None
    thread_exec = None
    stop_evt = None
    status = "Standby (Ready)"

# ==============================================================================
# [4] MATH & LOGIC UTILS
# ==============================================================================
def get_obj_bounds(obj):
    if obj.type != 'MESH': return Vector((-1,-1,-1)), Vector((1,1,1))
    coords = [v.co for v in obj.data.vertices]
    if not coords: return Vector((-1,-1,-1)), Vector((1,1,1))
    min_v = Vector((min(v.x for v in coords), min(v.y for v in coords), min(v.z for v in coords)))
    max_v = Vector((max(v.x for v in coords), max(v.y for v in coords), max(v.z for v in coords)))
    return min_v, max_v

def is_inside_box(point, box_obj):
    if not point or not box_obj: return False
    local_point = box_obj.matrix_world.inverted() @ point
    min_b, max_b = get_obj_bounds(box_obj)
    return (min_b.x <= local_point.x <= max_b.x and 
            min_b.y <= local_point.y <= max_b.y and 
            min_b.z <= local_point.z <= max_b.z)

def get_target_transform(track_obj):
    mat = track_obj.matrix_world
    return mat.translation, mat.to_quaternion()

def update_robot_logic(scene, force_return=False):
    wm = bpy.context.window_manager
    ctrl = bpy.data.objects.get(ROBOT_CTRL_NAME) 
    track = bpy.data.objects.get(TRACK_TARGET_NAME) 
    box = bpy.data.objects.get(BOX_NAME)
    
    if not (ctrl and track and box): return 0.0

    raw_loc, raw_rot = get_target_transform(track)
    
    if force_return or not is_inside_box(track.matrix_world.translation, box):
        # 🌟 合併 X, Y (來自實體) 與 Z (來自工具偏移)
        target_loc = Vector((ROBOT_HOME_POS[0], ROBOT_HOME_POS[1], TOOL_OFFSET))
        target_rot = Euler((math.pi, 0, 0)).to_quaternion()
    else:
        target_loc = raw_loc 
        target_rot = raw_rot

    curr_loc = ctrl.matrix_world.translation
    curr_rot = ctrl.matrix_world.to_quaternion()
    
    vec_to_target = target_loc - curr_loc
    dist = vec_to_target.length
    max_step = wm.ur_max_speed * 0.033 
    
    final_loc = curr_loc
    if dist > 0.0001:
        final_loc = curr_loc + vec_to_target.normalized() * max_step if dist > max_step else target_loc
    
    if curr_rot.dot(target_rot) < 0.0:
        target_rot.negate()
    final_rot = curr_rot.slerp(target_rot, wm.ur_rot_smooth)

    ctrl.matrix_world = mathutils.Matrix.LocRotScale(final_loc, final_rot, ctrl.matrix_world.to_scale())
    return dist

# ==============================================================================
# [5] NETWORK & THREADING
# ==============================================================================
def send_cmd(ip, port, cmd, wait=False):
    if not cmd.endswith('\n'): cmd += '\n'
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(1.0)
            s.connect((ip, port))
            s.sendall(cmd.encode('ascii'))
            if wait: return s.recv(1024).decode('ascii').strip()
    except Exception as e:
        print(f"[UR Network] Send Error: {e}")

def get_ur_pose(obj):
    mat = obj.matrix_world
    loc, rot = mat.to_translation(), mat.to_quaternion()
    if rot.w < 0: rot.negate()
    angle, axis = rot.angle, rot.axis
    return loc, rot, [loc.x, loc.y, loc.z, axis.x * angle, axis.y * angle, axis.z * angle]

def execution_thread(ip, stop_evt):
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(2.0)
        sock.connect((ip, PORT_SCRIPT))
    except Exception:
        Global.status = "Connection Failed"
        return

    Global.status = "Connected"
    
    with Global.lock: start_target = Global.pose
    if start_target:
        Global.status = "Moving to First Frame"
        p = start_target
        cmd = f"movel(p[{p[0]:.4f},{p[1]:.4f},{p[2]:.4f},{p[3]:.4f},{p[4]:.4f},{p[5]:.4f}], a=0.2, v=0.2, t=3.0)\n"
        try:
            sock.sendall(cmd.encode('ascii'))
            time.sleep(3.5) 
        except: pass
    
    Global.status = "Live Syncing"
    
    while not stop_evt.is_set():
        if Global.stop_requested: break 
            
        loop_start = time.time()
        with Global.lock: target_pose = Global.pose

        if target_pose:
            p_str = f"p[{target_pose[0]:.4f},{target_pose[1]:.4f},{target_pose[2]:.4f},{target_pose[3]:.4f},{target_pose[4]:.4f},{target_pose[5]:.4f}]"
            cmd = f"servoj(get_inverse_kin({p_str}, get_actual_joint_positions()), t={INTERNAL_T}, lookahead_time={INTERNAL_LH}, gain={INTERNAL_GAIN})\n"
            try: sock.sendall(cmd.encode('ascii'))
            except: break
        
        time.sleep(max(0, (1.0/50.0) - (time.time() - loop_start)))

    if Global.stop_requested:
        Global.status = "Auto Returning Home"
        try:
            home_cmd = f"movel(p[{ROBOT_HOME_POS[0]},{ROBOT_HOME_POS[1]},{ROBOT_HOME_POS[2]},3.1415,0,0], a=0.2, v=0.2, t=3.0)\n"
            sock.sendall(home_cmd.encode('ascii'))
            time.sleep(3.5)
        except: pass

    try:
        sock.sendall(b"stopj(2.0)\n")
        sock.close()
    except: pass
    Global.status = "Standby (Ready)"

# ==============================================================================
# [6] UI & OPERATORS
# ==============================================================================
class UR_OT_Reset(bpy.types.Operator):
    bl_idname, bl_label = "ur.reset_defaults", "Reset Settings"
    bl_description = "Reset connection and speed settings to default"
    def execute(self, context):
        wm = context.window_manager
        wm.ur_ip, wm.ur_max_speed, wm.ur_rot_smooth, wm.ur_bbox_enable = DEF_IP, DEF_MAX_SPEED, DEF_ROT_SMOOTH, DEF_LOGIC_ENABLE
        self.report({'INFO'}, "Settings Reset")
        return {'FINISHED'}

class UR_OT_ZeroPosition(bpy.types.Operator):
    bl_idname, bl_label = "ur.zero_position", "Zero Position"
    bl_description = f"Reset {OBJ_NAME} XYZ location to 0"
    
    def execute(self, context):
        tcp = bpy.data.objects.get(OBJ_NAME)
        if tcp:
            tcp.location = (0, 0, 0)
            self.report({'INFO'}, f"[{OBJ_NAME}] XYZ reset to 0")
        else:
            self.report({'WARNING'}, f"[{OBJ_NAME}] not found!")
        return {'FINISHED'}

class UR_OT_LiveSync(bpy.types.Operator):
    bl_idname, bl_label = "ur.f2_live", "Live Sync"
    _timer = None

    def modal(self, context, event):
        if not Global.is_live: return self.cancel(context)
        if event.type == 'TIMER':
            wm = context.window_manager
            
            if Global.stop_requested:
                dist = update_robot_logic(context.scene, force_return=True)
                context.workspace.status_text_set(f"Status: {Global.status} | Dist: {dist:.4f}")
                if dist < 0.001 and (not Global.thread_exec or not Global.thread_exec.is_alive()): 
                    return self.cancel(context)
            elif wm.ur_bbox_enable == 'YES': 
                update_robot_logic(context.scene, force_return=False)
            
            context.view_layer.update()
            tcp = bpy.data.objects.get(OBJ_NAME)
            if tcp:
                _, _, pose = get_ur_pose(tcp)
                with Global.lock: Global.pose = pose
            context.area.tag_redraw()
        return {'PASS_THROUGH'}

    def execute(self, context):
        if Global.is_live: 
            if not Global.stop_requested:
                Global.stop_requested, Global.status = True, "Auto Returning Home"
            return {'RUNNING_MODAL'} 

        wm = context.window_manager
        ctrl = bpy.data.objects.get(ROBOT_CTRL_NAME)
        
        if ctrl:
            # 🌟 合併 X, Y (來自實體) 與 Z (來自工具偏移)
            visual_home = Vector((ROBOT_HOME_POS[0], ROBOT_HOME_POS[1], TOOL_OFFSET))
            ctrl.matrix_world = mathutils.Matrix.LocRotScale(visual_home, Euler((math.pi, 0, 0)).to_quaternion(), ctrl.matrix_world.to_scale())
            context.view_layer.update() 
        
        if wm.ur_bbox_enable == 'YES': update_robot_logic(context.scene)
        context.view_layer.update()
        
        tcp = bpy.data.objects.get(OBJ_NAME)
        if tcp:
            _, _, pose = get_ur_pose(tcp)
            Global.pose = pose

        send_cmd(wm.ur_ip, PORT_DASH, "stop")
        Global.is_live, Global.stop_requested, Global.status = True, False, "Connecting..."
        Global.stop_evt = threading.Event()
        
        Global.thread_exec = threading.Thread(target=execution_thread, args=(wm.ur_ip, Global.stop_evt), daemon=True)
        Global.thread_exec.start()
        
        self._timer = wm.event_timer_add(0.033, window=context.window)
        wm.modal_handler_add(self)
        return {'RUNNING_MODAL'}

    def cancel(self, context):
        Global.is_live, Global.stop_requested, Global.status = False, False, "Standby (Ready)"
        context.workspace.status_text_set(None)
        if self._timer: context.window_manager.event_timer_remove(self._timer)
        if Global.stop_evt: Global.stop_evt.set()
        if Global.thread_exec: Global.thread_exec.join(1.0)
        return {'FINISHED'}

class UR_PT_Panel(bpy.types.Panel):
    bl_label, bl_idname, bl_space_type, bl_region_type, bl_category = "UR Live Control", "UR_PT_Panel", 'VIEW_3D', 'UI', "UR Control"

    def draw(self, context):
        wm, layout = context.window_manager, self.layout
        
        b = layout.box()
        b.label(text="Connection", icon='PREFERENCES')
        b.prop(wm, "ur_ip", text="Robot IP")
        b.row().label(text=f"Status: {Global.status}", icon='INFO')
        
        layout.separator()
        b = layout.box()
        b.label(text="Bounding Box", icon='GRAPH')
        b.prop(wm, "ur_bbox_enable", text="Follow Logic")
        if wm.ur_bbox_enable == 'YES':
            col = b.column(align=True)
            col.prop(wm, "ur_max_speed", text="Max Speed")
            col.prop(wm, "ur_rot_smooth", text="Rot Smooth")

        layout.separator()
        layout.operator("ur.reset_defaults", icon='LOOP_BACK')
        layout.operator("ur.zero_position", icon='PIVOT_CURSOR')
        layout.separator()
        
        col = layout.column(align=True)
        col.scale_y = 1.5
        if not Global.is_live:
            col.operator("ur.f2_live", text="START LIVE SYNC", icon='PLAY')
        elif Global.stop_requested:
            col.label(text="Returning Home...", icon='TIME')
        else:
            col.alert = True
            col.operator("ur.f2_live", text="STOP & RETURN HOME", icon='PAUSE')

# ==============================================================================
# [7] REGISTRATION
# ==============================================================================
classes = (UR_OT_Reset, UR_OT_ZeroPosition, UR_OT_LiveSync, UR_PT_Panel)
addon_keymaps = []

def register():
    for cls in classes: bpy.utils.register_class(cls)
    wm = bpy.types.WindowManager
    wm.ur_ip = bpy.props.StringProperty(name="IP", default=DEF_IP)
    wm.ur_bbox_enable = bpy.props.EnumProperty(name="Logic", items=[('NO',"Disabled",""),('YES',"Enabled","")], default=DEF_LOGIC_ENABLE)
    wm.ur_max_speed = bpy.props.FloatProperty(name="Max Speed", default=DEF_MAX_SPEED, min=0.01, max=1.0)
    wm.ur_rot_smooth = bpy.props.FloatProperty(name="Rot Smooth", default=DEF_ROT_SMOOTH, min=0.01, max=1.0)
    
    kc = bpy.context.window_manager.keyconfigs.addon
    if kc:
        km = kc.keymaps.new(name='3D View', space_type='VIEW_3D')
        addon_keymaps.append((km, km.keymap_items.new("ur.f2_live", 'F2', 'PRESS')))

def unregister():
    for km, kmi in addon_keymaps: km.keymap_items.remove(kmi)
    addon_keymaps.clear()
    for cls in classes: bpy.utils.unregister_class(cls)

if __name__ == "__main__": register()
