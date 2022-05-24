import bpy, mathutils, sys
from math import degrees, radians



class RobotSimulation():

    def __init__(self, robot_kinemtics, external_kinematics):
        self.robot = robot_kinemtics
        self.external = external_kinematics
        self.sequence = []
        self.sequence_times = [0]
        self._tcp_mat = mathutils.Matrix()
        self._root_mat = mathutils.Matrix()

    def get_current_axis_angles(self):
        # получение словаря текущих значений осей робота
        axis = {}
        for an in self.robot.parameters['axis_names']:
            axis[an] = self.robot.get_axis_angle(an)
        return axis

    def get_delta_angles(self, from_angles, to_angles, delta_time):
        # отдаём два словаря со значениями углов робота в начальном и конечном положении + время [0.0, 1.0], 
        # получаем словарь с приращением углов от начального положения в заданный момент времени
        # если время == 1, то просто получаем разницу углов
        ad = {}
        for an in self.robot.parameters['axis_names']:
            ad[an] = (to_angles[an] - from_angles[an])*delta_time
        for an in self.robot.parameters['exax_names']:
            ad[an] = (to_angles[an] - from_angles[an])*delta_time
        return ad

    def calc_ptp_time(self, from_angles, to_angles, spd):
        # считаем время РТР движения с учётом заданных в конфигурации скоростей осей и указанной скорости РТР
        da = self.get_delta_angles(from_angles, to_angles, 1)
        ptp_time = 0
        for an in self.robot.parameters['axis_names']:
            ptp_time = max(ptp_time, abs(da[an])/self.robot.parameters['axis_spd'][an]/spd)
        for an in self.robot.parameters['exax_names']:
            ptp_time = max(ptp_time, abs(da[an])/self.robot.parameters['exax_spd'][an]/spd)
        return ptp_time
        
    def _calc_sequence_times(self):
        # считаем абсолютные метки времени для каждой позиции 
        # т.к. в self.sequence сохраняются только относительные
        seq = self.sequence
        self.sequence_times = [0]
        for p in seq[1:]:
            self.sequence_times.append(self.sequence_times[-1]+p[4])
        

    def get_segment_at_time(self, tm):
        # получаем сегмент из двух точек между которыми находится робот в заданный момент
        # + время в этих точках
        seq = self.sequence
        times = self.sequence_times
        for i, t in enumerate(times[1:]):
            if times[i] <= tm <= t:
                return [seq[i], seq[i+1], times[i], times[i+1]]

    def update_simulation(self, scene):
        # обновление симуляции при "проигрывании" или перетаскивании курсора в шкале времени
        frame = scene.frame_current
        cur_time = frame / scene.render.fps
        segment = self.get_segment_at_time(cur_time)
        if segment:
            # имя точки, фрейм, скорость, значения осей, время движения от предыдущей точки
            pname1, p1, spd1, ax1, delta_time1 = segment[0]
            pname2, p2, spd2, ax2, delta_time2 = segment[1]
            start_time = segment[2]
            end_time = segment[3]
            time_factor = 1.0
            if (end_time - start_time) > 0:
                time_factor = (cur_time - start_time) / (end_time - start_time)
            da = self.get_delta_angles(ax1, ax2, time_factor)
            for an in self.robot.parameters['exax_names']:
                self.external.set_axis_angle(an, ax1[an] + da[an])
            if pname2.startswith('PTP'):
                self.robot.ik_active = False
                for an in self.robot.parameters['axis_names']:
                    self.robot.set_axis_angle(an, ax1[an] + da[an])
                #self.robot.update_tool_pos()
                self.robot.ik_active = True

            elif pname2.startswith('LIN'):
                self.robot.tcp.matrix_world = p1.lerp(p2, time_factor)
                self.robot.update_ik()
                
    def interactive_update(self, scene):
        # интерактивное обновление инверсной кинематики при перемещении TCP или базы
        self.robot._update_ptrs()
        # Check external kinematics influence
        if self.external:
            self.external._update_ptrs()
            changes = self.external.get_flange_changes()
            for an, matx in changes.items():
                if self.external.flange_state[an] == 1: # state 1 - TCP follows the flange
                    mat_from, mat_to = matx
                    tcp_loc = mat_from.inverted() @ self.robot.tcp.matrix_world
                    self.robot.tcp.matrix_world = mat_to @ tcp_loc
        # Update robot kinematics            
        if self._tcp_mat != self.robot.tcp.matrix_world or self._root_mat != self.robot.rob_root.matrix_world:
            self._tcp_mat = self.robot.tcp.matrix_world.copy()
            self._root_mat = self.robot.rob_root.matrix_world.copy()
            self.robot.update_ik()
            #bpy.context.view_layer.update()
                
    def update_program(self, program):
        # Расчёт движения на основе списка переданных точек и генерация self.sequence
        tmp_mat = self.robot.tcp.matrix_world.copy()
        tmp_conf = self.robot.axis_config
        for record in program:
            pname, spd, config, exkin, tool = record
            p = bpy.context.scene.objects[pname]
            self.robot.tcp.matrix_world = p.matrix_world
            if config and pname.startswith('PTP'):
                self.robot.axis_config = config
            bpy.context.view_layer.update()
            self.robot.update_ik()
            aa = self.get_current_axis_angles()
            aa.update(exkin)
            delta_time = 0
            # если есть предыдущие точки, то считаем время перемещения от предыдущей в текущую
            if len(self.sequence) > 0:
                if pname.startswith('PTP'):
                    aa0 = self.sequence[-1][3] # Предыдущее значение углов
                    delta_time = self.calc_ptp_time(aa0, aa, spd) # время перемещения
                elif pname.startswith('LIN'):
                    pp0 = self.sequence[-1][1] # Предыдущая точка
                    delta_time = (p.matrix_world.translation - pp0.translation).length/spd
            self.sequence.append([pname, p.matrix_world, spd, aa, delta_time])
        self._calc_sequence_times()
        bpy.context.scene.frame_end = int(self.sequence_times[-1] * bpy.context.scene.render.fps) + 1
        self.robot.tcp.matrix_world = tmp_mat
        self.robot.axis_config = tmp_conf
        self.robot.update_ik()
        
    
def register_handlers(sim):
    # Добавление процедуры обновления для панели анимации
    bpy.app.handlers.frame_change_pre.clear()
    bpy.app.handlers.frame_change_pre.append(lambda s: sim.update_simulation(s))
    bpy.app.handlers.frame_change_post.clear()
    #bpy.app.handlers.frame_change_post.append(my_handler)
    
    # Добавление обновления для интерактивного взаимодействия
    bpy.app.handlers.depsgraph_update_pre.clear()
    #bpy.app.handlers.depsgraph_update_pre.append(test_update)
    bpy.app.handlers.depsgraph_update_post.clear()
    bpy.app.handlers.depsgraph_update_post.append(lambda s: sim.interactive_update(s))


if __name__ == '__main__':
    # Test
    rp = bpy.data.texts['rob_parameters'].as_module()
    kin = bpy.data.texts['rob_kinematics'].as_module()

    rk = kin.RobotKinematics(rp.parameters)
    sim = RobotSimulation(rk)
    
    program = [
        ['PTP', 0.25, [0,1,0]], 
        ['PTP.001', 0.25, None], 
        ['PTP.002', 0.25, None], 
        ['LIN.002', 0.5, None],
        ['LIN.003', 0.5, None],
        ['LIN.004', 0.5, None]
    ]
    
    sim.update_program(program)
    register_handlers(sim)

