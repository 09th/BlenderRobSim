import bpy, mathutils
from math import degrees, radians

selected_robot = None
selected_exkin = None
                                        
class SimpleRobotPanel(bpy.types.Panel):
    bl_idname = "OBJECT_PT_simple_robot_panel"
    bl_label = "Robot"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Item'
    bl_order = 10

    bpy.types.Scene.A1_val = bpy.props.FloatProperty(name = 'A1', default = 0, 
                                        get = lambda s: selected_robot.get_axis_angle('A1'),
                                        set = lambda s, v: selected_robot.set_axis_angle('A1', v, propagate=True))
    bpy.types.Scene.A2_val = bpy.props.FloatProperty(name = 'A2', default = 0, 
                                        get = lambda s: selected_robot.get_axis_angle('A2'),
                                        set = lambda s, v: selected_robot.set_axis_angle('A2', v, propagate=True))
    bpy.types.Scene.A3_val = bpy.props.FloatProperty(name = 'A3', default = 0, 
                                        get = lambda s: selected_robot.get_axis_angle('A3'),
                                        set = lambda s, v: selected_robot.set_axis_angle('A3', v, propagate=True))
    bpy.types.Scene.A4_val = bpy.props.FloatProperty(name = 'A4', default = 0, 
                                        get = lambda s: selected_robot.get_axis_angle('A4'),
                                        set = lambda s, v: selected_robot.set_axis_angle('A4', v, propagate=True))
    bpy.types.Scene.A5_val = bpy.props.FloatProperty(name = 'A5', default = 0, 
                                        get = lambda s: selected_robot.get_axis_angle('A5'),
                                        set = lambda s, v: selected_robot.set_axis_angle('A5', v, propagate=True))
    bpy.types.Scene.A6_val = bpy.props.FloatProperty(name = 'A6', default = 0, 
                                        get = lambda s: selected_robot.get_axis_angle('A6'),
                                        set = lambda s, v: selected_robot.set_axis_angle('A6', v, propagate=True))

    bpy.types.Scene.axis_config = bpy.props.BoolVectorProperty(name = 'Config', size = 3,
                                        get = lambda s: selected_robot.get_axis_config_bool(),
                                        set = lambda s, v: selected_robot.set_axis_config_bool(v))
                                        
    bpy.types.Scene.E1_val = bpy.props.FloatProperty(name = 'E1', default = 0, 
                                        get = lambda s: selected_exkin.get_axis_angle('E1'),
                                        set = lambda s, v: selected_exkin.set_axis_angle('E1', v))
    bpy.types.Scene.E1_tcp_follow = bpy.props.BoolProperty(name = 'TCP', default = False,
                                        get = lambda s: selected_exkin.get_flange_state('E1'),
                                        set = lambda s, v: selected_exkin.set_flange_state('E1', v))
    bpy.types.Scene.E2_val = bpy.props.FloatProperty(name = 'E2', default = 0, 
                                        get = lambda s: selected_exkin.get_axis_angle('E2'),
                                        set = lambda s, v: selected_exkin.set_axis_angle('E2', v))
    bpy.types.Scene.E2_tcp_follow = bpy.props.BoolProperty(name = 'TCP', default = False,
                                        get = lambda s: selected_exkin.get_flange_state('E2'),
                                        set = lambda s, v: selected_exkin.set_flange_state('E2', v))
                                        
    def draw(self, context):
        
        if not selected_robot:
            return

        axis_names = selected_robot.parameters['axis_names']
        #self.robot._update_ptrs()

        row = self.layout.row()
        row.prop(context.scene, 'axis_config')
        
        for ax_n in axis_names:
            row = self.layout.row()
            row.prop(context.scene, ax_n+'_val')
        
        if selected_exkin:
            exax_names = selected_exkin.parameters['exax_names']
            flanges = selected_exkin.parameters['ek_flanges']
            for eax_n in exax_names:
                row = self.layout.row()
                row.prop(context.scene, eax_n+'_val')
                if eax_n in flanges.keys():
                    row.prop(context.scene, eax_n+'_tcp_follow')
        
        #debug_axis_info = False
        #if debug_axis_info:
        #    for i, ax_n in enumerate(axis_names):
        #        pa = context.scene.objects['p{0}'.format(ax_n).lower()]
        #        mat = pa.matrix_world
        #        if i > 0:
        #            pa_prev = context.scene.objects['p{0}'.format(axis_names[i-1]).lower()]
        #            mat = pa_prev.matrix_world.inverted_safe() @ pa.matrix_world
        #        mat = mat.to_euler()
        #        row = self.layout.row()
        #        row.label(text=ax_n)
        #        row.label(text='X: {:.3f}'.format(degrees(mat.x)))
        #        row.label(text='Y: {:.3f}'.format(degrees(mat.y)))
        #        row.label(text='Z: {:.3f}'.format(degrees(mat.z)))
                
        row = self.layout.row()
        op = row.operator("object.wp_add", text='PTP')
        op.wp_name = 'PTP'
        op.tcp_name = selected_robot.parameters['robot_objects']['TCP']
        op = row.operator("object.wp_add", text='LIN')
        op.wp_name = 'LIN'
        op.tcp_name = selected_robot.parameters['robot_objects']['TCP']


class AddWPOperator(bpy.types.Operator):
    
    bl_idname = 'object.wp_add'
    bl_label = 'Add waypoint'
    
    wp_name: bpy.props.StringProperty(
        name = 'wp_name',
        default = 'PTP'
        )
        
    tcp_name: bpy.props.StringProperty(
        name = 'tcp_name',
        default = 'TCP'
        )
        
    #tcp: bpy.props.PointerProperty(name='tcp', type=bpy.types.Object)
        
    def execute(self, context):
        tcp = context.scene.objects[self.tcp_name]
        new_obj = bpy.data.objects.new(self.wp_name, None) 
        new_obj.matrix_world = tcp.matrix_world
        new_obj.empty_display_type = 'ARROWS'
        new_obj.empty_display_size = 0.1
        new_obj.show_name = True
        bpy.data.collections['waypoints'].objects.link(new_obj)
        #print(self.wp_name, tcp, context.scene.link)
        return {'FINISHED'}            

bpy.utils.register_class(AddWPOperator)
bpy.utils.register_class(SimpleRobotPanel)
