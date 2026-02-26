import bpy, socket, math, time, threading, struct
import mathutils
from mathutils import Matrix, Vector, Euler

# ==============================================================================
# CONFIGURATION (Settings)
# ==============================================================================

# [1] Scene Object Names
# ------------------------------------------------------------------------------
# Logic Flow: brush_Bone -> Script -> MCP_Empty -> (GeoNodes/Parent) -> TCP_Empty -> Robot

TRACK_TARGET_NAME = "brush_Bone.002"  # [Input] Original bone tracked by Script
ROBOT_CTRL_NAME   = "mocap_cleaned"       # [Driver] Script moves this object
OBJ_NAME          = "TCP_Empty"       # [Output] Network thread reads this object to send to robot arm
BOX_NAME          = "bounding_box"    # Bounding box to limit movement range
HOME_NAME         = "Home_Position"  # The location of robot while the script was running

# [2] Network Defaults
# ------------------------------------------------------------------------------
DEF_IP = '192.168.0.100'
PORT_SCRIPT = 30003
PORT_DASH = 29999

# [3] Safety & Logic Defaults
# ------------------------------------------------------------------------------
DEF_MAX_SPEED = 0.15   
DEF_ROT_SMOOTH = 0.10  
DEF_LOGIC_ENABLE = 'YES'
HOME_POS = [-0.2983, 0.1314, 0.219] 
 
# [4] Internal Control Constants
# ------------------------------------------------------------------------------
INTERNAL_T = 0.03        
INTERNAL_LH = 0.03       
INTERNAL_GAIN = 300      

HOME_CMD = f"movel(p[{HOME_POS[0]},{HOME_POS[1]},{HOME_POS[2]},3.1415,0,0], a=0.5, v=0.3)\n"

# ==============================================================================
# GLOBAL STATE
# ==============================================================================
class Global:
    is_live = False
    stop_requested = False 
    lock = threading.Lock()
    pose = None
    thread_exec = None
    stop_evt = None

# ==============================================================================
# LOGIC UTILS
# ==============================================================================
def get_obj_bounds(obj):
    if obj.type != 'MESH': return Vector((-1,-1,-1)), Vector((1,1,1))
    coords = [v.co for v in obj.data.vertices]
    if not coords: return Vector((-1,-1,-1)), Vector((1,1,1))
    min_v = Vector((min(v.x for v in coords), min(v.y for v in coords), min(v.z for v in coords)))
    max_v = Vector((max(v.x for v in coords), max(v.y for v in coords), max(v.z for v in coords)))
    return min_v, max_v

def is_inside_box(point, box_obj):
    if point is None or box_obj is None: return False
    box_inv = box_obj.matrix_world.inverted()
    local_point = box_inv @ point
    min_bound, max_bound = get_obj_bounds(box_obj)
    return min_bound.x <= local_point.x <= max_bound.x and \
           min_bound.y <= local_point.y <= max_bound.y and \
           min_bound.z <= local_point.z <= max_bound.z

def get_target_transform(track_obj):
    # Directly read target world coordinates and rotation, no Offset processing
    target_matrix = track_obj.matrix_world
    target_loc = target_matrix.translation
    target_rot = target_matrix.to_quaternion()
    return target_loc, target_rot

def update_robot_logic(scene, force_return=False):
    wm = bpy.context.window_manager
    
    robot_ctrl = bpy.data.objects.get(ROBOT_CTRL_NAME) # MCP_Empty
    track_obj = bpy.data.objects.get(TRACK_TARGET_NAME) # brush_Bone
    home_obj = bpy.data.objects.get(HOME_NAME)
    box = bpy.data.objects.get(BOX_NAME)
    
    if not (robot_ctrl and track_obj and box): return 0.0

    # 1. Get original armature coordinates
    raw_track_loc, track_rot = get_target_transform(track_obj)
    
    # 2. Logic decision (Is inside Box)
    magnet_loc = None
    magnet_rot = None
    
    if force_return:
        if home_obj:
            magnet_loc = home_obj.matrix_world.translation
            magnet_rot = home_obj.matrix_world.to_quaternion()
        else:
            magnet_loc = Vector(HOME_POS)
            magnet_rot = Euler((math.pi, 0, 0)).to_quaternion()
    else:
        is_in = is_inside_box(track_obj.matrix_world.translation, box)
        
        if is_in:
            magnet_loc = raw_track_loc # Follow directly, no scaling
            magnet_rot = track_rot
        else:
            if home_obj:
                magnet_loc = home_obj.matrix_world.translation
                magnet_rot = home_obj.matrix_world.to_quaternion()
            else:
                magnet_loc = Vector(HOME_POS)
                magnet_rot = Euler((math.pi, 0, 0)).to_quaternion()

    # 3. Smoothly move MCP_Empty
    current_loc = robot_ctrl.matrix_world.translation
    current_rot = robot_ctrl.matrix_world.to_quaternion()
    
    vec_to_magnet = magnet_loc - current_loc
    dist_to_magnet = vec_to_magnet.length
    
    max_step = wm.ur_max_speed * 0.033 
    
    final_loc = current_loc
    if dist_to_magnet > 0.0001:
        if dist_to_magnet > max_step:
            move_vec = vec_to_magnet.normalized() * max_step
            final_loc = current_loc + move_vec
        else:
            final_loc = magnet_loc
    
    final_rot = current_rot.slerp(magnet_rot, wm.ur_rot_smooth)

    # 4. Write to MCP_Empty
    robot_ctrl.matrix_world.translation = final_loc
    robot_ctrl.rotation_euler = final_rot.to_euler()
    
    return dist_to_magnet

# ==============================================================================
# NETWORK (DEBUG VERSION)
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
        print(f"[UR Send Error] {e}")

def get_ur_pose(obj):
    # Read TCP_Empty data
    mat = obj.matrix_world
    loc, rot = mat.to_translation(), mat.to_quaternion()
    if rot.w < 0: rot.negate()
    angle, axis = rot.angle, rot.axis
    return loc, rot, [loc.x, loc.y, loc.z, axis.x * angle, axis.y * angle, axis.z * angle]

def execution_thread_func(ip, stop_evt):
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(2.0)
        sock.connect((ip, PORT_SCRIPT))
    except Exception as e:
        print(f"[UR Exec Error] Connection failed: {e}")
        return

    print("[UR] Execution Thread Started - Connected!")
    
    # 1. Initial Move (MoveJ)
    with Global.lock: start_target = Global.pose
    if start_target:
        p = start_target
        print(f"[UR Init] Moving to: {p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}") # DEBUG PRINT
        safe_cmd = f"movej(p[{p[0]:.4f},{p[1]:.4f},{p[2]:.4f},{p[3]:.4f},{p[4]:.4f},{p[5]:.4f}], a=0.5, v=0.25)\n"
        try:
            sock.sendall(safe_cmd.encode('ascii'))
            time.sleep(3.0) 
        except: pass
    
    # 2. Real-time Loop (ServoJ)
    frame_count = 0
    while not stop_evt.is_set():
        loop_start = time.time()
        target_pose = None
        with Global.lock: target_pose = Global.pose

        if target_pose:
            pose_str = f"p[{target_pose[0]:.4f},{target_pose[1]:.4f},{target_pose[2]:.4f},{target_pose[3]:.4f},{target_pose[4]:.4f},{target_pose[5]:.4f}]"
            cmd = f"servoj(get_inverse_kin({pose_str}, get_actual_joint_positions()), t={INTERNAL_T}, lookahead_time={INTERNAL_LH}, gain={INTERNAL_GAIN})\n"
            try: 
                sock.sendall(cmd.encode('ascii'))
                
                # Print current target coordinates every 30 frames (Debug)
                frame_count += 1
                if frame_count % 30 == 0:
                    print(f"[UR Sending] {pose_str}")
                    
            except Exception as e: 
                print(f"[UR Loop Error] {e}")
                break
        
        time.sleep(max(0, (1.0/50.0) - (time.time() - loop_start)))

    try:
        sock.sendall(b"stopj(2.0)\n")
        sock.close()
    except: pass
    print("[UR] Execution Thread Stopped")

# ==============================================================================
# OPERATORS & UI
# ==============================================================================
class UR_OT_Reset_Defaults(bpy.types.Operator):
    bl_idname, bl_label = "ur.reset_defaults", "Reset Settings"
    bl_description = "Reset all parameters"

    def execute(self, context):
        wm = context.window_manager
        wm.ur_ip = DEF_IP
        wm.ur_max_speed = DEF_MAX_SPEED
        wm.ur_rot_smooth = DEF_ROT_SMOOTH
        wm.ur_bbox_enable = DEF_LOGIC_ENABLE
        self.report({'INFO'}, "Settings Reset")
        return {'FINISHED'}

class UR_OT_F2_Live(bpy.types.Operator):
    bl_idname, bl_label = "ur.f2_live", "F2: Live Sync"
    bl_description = "Start Real-time synchronization"
    _timer = None

    def modal(self, context, event):
        if not Global.is_live: return self.cancel(context)
        if event.type == 'TIMER':
            wm = context.window_manager
            dist_to_target = 0.0
            
            # 1. Execute logic (Move MCP_Empty)
            if Global.stop_requested:
                dist_to_target = update_robot_logic(context.scene, force_return=True)
                context.workspace.status_text_set(f"Returning Home... Dist: {dist_to_target:.4f}")
                if dist_to_target < 0.001: 
                    self.report({'INFO'}, "Returned Home.")
                    return self.cancel(context)
            else:
                if wm.ur_bbox_enable == 'YES': 
                    update_robot_logic(context.scene, force_return=False)
            
            # 2. Force scene update (Let Blender calculate GeoNodes/Parent)
            context.view_layer.update()

            # 3. Read TCP_Empty
            obj = bpy.data.objects.get(OBJ_NAME)
            if obj:
                _, _, pose = get_ur_pose(obj)
                with Global.lock: Global.pose = pose
            
            context.area.tag_redraw()
        return {'PASS_THROUGH'}

    def execute(self, context):
        if Global.is_live: 
            if not Global.stop_requested:
                Global.stop_requested = True
                self.report({'WARNING'}, "Stopping...")
            return {'RUNNING_MODAL'} 

        wm = context.window_manager
        
        # Return to home on start
        mcp = bpy.data.objects.get(ROBOT_CTRL_NAME)
        home_obj = bpy.data.objects.get(HOME_NAME)
        
        if mcp and home_obj:
            mcp.matrix_world.translation = home_obj.matrix_world.translation
            mcp.rotation_euler = home_obj.matrix_world.to_euler()
            context.view_layer.update()

        send_cmd(wm.ur_ip, PORT_DASH, "stop")
        Global.is_live = True
        Global.stop_requested = False
        Global.stop_evt = threading.Event()
        Global.thread_exec = threading.Thread(target=execution_thread_func, args=(wm.ur_ip, Global.stop_evt), daemon=True)
        Global.thread_exec.start()
        self._timer = wm.event_timer_add(0.033, window=context.window)
        wm.modal_handler_add(self)
        return {'RUNNING_MODAL'}

    def cancel(self, context):
        Global.is_live = False
        Global.stop_requested = False
        context.workspace.status_text_set(None)
        if self._timer: context.window_manager.event_timer_remove(self._timer)
        if Global.stop_evt: Global.stop_evt.set()
        if Global.thread_exec: Global.thread_exec.join(1.0)
        return {'FINISHED'}

class UR_PT_Panel(bpy.types.Panel):
    bl_label = "UR Live Control"
    bl_idname = "UR_PT_Panel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "UR Control"

    def draw(self, context):
        wm = context.window_manager
        layout = self.layout
        
        b = layout.box()
        b.label(text="Connection", icon='PREFERENCES')
        b.prop(wm, "ur_ip", text="Robot IP")
        
        layout.separator()
        b = layout.box()
        b.label(text="Real Time Control", icon='GRAPH')
        b.prop(wm, "ur_bbox_enable", text="Follow Logic")
        if wm.ur_bbox_enable == 'YES':
            col = b.column(align=True)
            col.prop(wm, "ur_max_speed", text="Max Speed")
            col.prop(wm, "ur_rot_smooth", text="Rot Smooth")

        layout.separator()
        layout.operator("ur.reset_defaults", text="Reset Defaults", icon='LOOP_BACK')
        layout.separator()
        
        col = layout.column(align=True)
        col.scale_y = 1.5
        if not Global.is_live:
            col.operator("ur.f2_live", text="START LIVE SYNC", icon='PLAY')
        else:
            if Global.stop_requested:
                col.alert = False
                col.label(text="Returning Home...", icon='TIME')
            else:
                col.alert = True
                col.operator("ur.f2_live", text="STOP & RETURN HOME", icon='PAUSE')

classes = (UR_OT_Reset_Defaults, UR_OT_F2_Live, UR_PT_Panel)
addon_keymaps = []

def register():
    for cls in classes: bpy.utils.register_class(cls)
    wm = bpy.types.WindowManager
    wm.ur_ip = bpy.props.StringProperty(name="IP", default=DEF_IP)
    wm.ur_bbox_enable = bpy.props.EnumProperty(name="Logic", items=[('NO',"Disabled", ""),('YES',"Enabled", "")], default=DEF_LOGIC_ENABLE)
    wm.ur_max_speed = bpy.props.FloatProperty(name="Max Speed", default=DEF_MAX_SPEED, min=0.01, max=1.0)
    wm.ur_rot_smooth = bpy.props.FloatProperty(name="Rot Smooth", default=DEF_ROT_SMOOTH, min=0.01, max=1.0)
    
    kc = bpy.context.window_manager.keyconfigs.addon
    if kc:
        km = kc.keymaps.new(name='3D View', space_type='VIEW_3D')
        addon_keymaps.append((km, km.keymap_items.new("ur.f2_live", 'F2', 'PRESS')))

def unregister():
    if update_robot_logic in bpy.app.handlers.frame_change_post: bpy.app.handlers.frame_change_post.remove(update_robot_logic)
    for km, kmi in addon_keymaps: km.keymap_items.remove(kmi)
    addon_keymaps.clear()
    for cls in classes: bpy.utils.unregister_class(cls)

if __name__ == "__main__": register()
