import bpy, mathutils
from math import degrees, radians, atan2, acos, pi, atan2

class RobotFK():
    # Прямая кинематика
    
    def __init__(self, parameters):
        self.parameters = parameters # параметры робота
        self.axis = {}               # ссылки на объекты представляющие оси робота
        self.axis_local_mat = {}     # хранилище локальных матриц осей
        self.tcp = None              # ссылка на объект предсталяющий TCP
        self.rob_root = None         # ссылка на объект - основание робота
        self.axis_config = [0,1,0]   # конфигурация осей (state)
        self._update_ptrs()
        
    def _update_ptrs(self):
        # обновление ссылок на объекты. При операциях типа отмены 
        # контекст сцены теряется и ссылки надо получать заново
        # TODO возможно если использовать PointerProperty обновление не потребуется
        for an in self.parameters['axis_names']:
            self.axis[an] = bpy.context.scene.objects[self.parameters['robot_objects'][an]]
            #self.axis[an] = bpy.data.objects[self.parameters['robot_objects'][an]]            
        self.tcp = bpy.context.scene.objects[self.parameters['robot_objects']['TCP']]
        self.rob_root = bpy.context.scene.objects[self.parameters['robot_objects']['rob_root']]
            
    def _save_axis_local_mat(self):
        # сохранение матриц осей в системе координат родителя
        # требуется для упрощения обновления позиций без использования
        # встроенной системы наследования
        for i,an in enumerate(self.parameters['axis_names']):
            mat = self.axis[an].matrix_world.copy()
            if i > 0:
                mat = self.axis[self.parameters['axis_names'][i-1]].matrix_world.inverted() @ mat
            else:
                mat = self.rob_root.matrix_world.inverted() @ mat
            self.axis_local_mat[an] = mat
            
            
    def get_axis_parent(self, axis_name):
        # получение родительской оси
        axis_number = self.parameters['axis_names'].index(axis_name)
        if axis_number > 0:
            axis_parent = self.axis[self.parameters['axis_names'][axis_number-1]]
            return axis_parent

    def _update_axis(self, axis_name):
        # обновление положения оси когда поменялось положение "родительской"
        # TODO попробовать использовать родную блендеровскую иерархию, 
        # но при первых проверках она мешала работе IK
        axis_parent = self.get_axis_parent(axis_name)
        if axis_parent:
            self.axis[axis_name].matrix_world = axis_parent.matrix_world @ self.axis_local_mat[axis_name]
        
            
    def get_axis_angle(self, axis_name, form='degrees'):
        # Получение угла оси робота из представляющего её объекта
        mat = self.axis[axis_name].matrix_world
        axis_parent = self.get_axis_parent(axis_name)
        if axis_parent:
            mat = axis_parent.matrix_world.inverted_safe() @ mat #self.axis[axis_name].matrix_world
        else:
            mat = self.rob_root.matrix_world.inverted() @ mat
        idx = self.parameters['axis_dir_idx'][axis_name]
        if form == 'degrees':
            return degrees(mat.to_euler()[idx]) * self.parameters['axis_rot_direction'][axis_name]
        return mat.to_euler()[idx] * self.parameters['axis_rot_direction'][axis_name]
    
    def set_axis_angle(self, axis_name, value, form='degrees', propagate=False):
        # Преобразование угла оси робота в матрицу соответствующего этой оси объекта
        self._save_axis_local_mat()
        eul = mathutils.Euler((0.0, 0.0, 0.0), 'XYZ')
        idx = self.parameters['axis_dir_idx'][axis_name]
        if form == 'degrees':
            eul[idx] = radians(value) * self.parameters['axis_rot_direction'][axis_name]
        else:
            eul[idx] = value * self.parameters['axis_rot_direction'][axis_name]
        mat = mathutils.Matrix.LocRotScale(self.parameters['axis_disp'][axis_name], eul, (1,1,1))
        axis_parent = self.get_axis_parent(axis_name)
        if axis_parent:
            mat = axis_parent.matrix_world @ mat
        else:
            mat = self.rob_root.matrix_world @ mat
        self.axis[axis_name].matrix_world = mat

        axis_number = self.parameters['axis_names'].index(axis_name)        
        if propagate and axis_number < (len(self.parameters['axis_names']) - 1):
            for an in self.parameters['axis_names'][axis_number+1:]:
                self._update_axis(an)
                
        self.update_tool_pos()
        self.update_axis_config()
                
        
    def update_tool_pos(self):
        # установка объекта представляющего TCP относительно фланца
        bpy.context.view_layer.update()
        tool_mat = self.parameters['tool_mat']
        self.tcp.matrix_world = self.axis[self.parameters['axis_names'][-1]].matrix_world @ tool_mat
        
    
    def update_axis_config(self):
        # Обновление конфигурации (то что у KUKA называется status) в зависимости от
        # положения и поворота 5 оси
        # угол 5 оси больше-меньше 0
        self.axis_config[0] = int(self.get_axis_angle('A5') < 0) 
        # положение центра а5 относительно плоскости XY второй оси
        mat = self.axis['A2'].matrix_world.inverted() @ self.axis['A5'].matrix_world
        self.axis_config[1] = int(mat.translation.z < 0)
        # положение центра а5 относительно плоскости YZ первой оси
        mat = self.axis['A1'].matrix_world.inverted() @ self.axis['A5'].matrix_world
        self.axis_config[2] = int(mat.translation.x < 0)
        
        



class RobotKinematics6Ax(RobotFK):
    # Инверсная кинематика
    
    def __init__(self, *args):
        RobotFK.__init__(self, *args)
        self._store_axis_length()
        self.ik_active = True        
    
    def _store_axis_length(self):
        # Сохранение длин сегментов между центрами осей
        a_disp = self.parameters['axis_disp']
        self._pa2_3_len = a_disp['A3'].length
        self._pa3_5_len = (a_disp['A4'] + a_disp['A5']).length
        self._pa4_5_len = a_disp['A5'].length
        self._pa3_4_len = a_disp['A4'].length
        # угол, образованный смещением продольной оси А4 относительно А3 в треугольнике А3-А4-А5
        # он неизменен, поэтому считаем сразу
        self._a34 = atan2(self._pa3_4_len, self._pa4_5_len)
        
    def get_axis_config_bool(self):
        # для UI получение конфигурации осей в виде массива bool
        return [bool(v) for v in self.axis_config]
    
    def set_axis_config_bool(self, vals):
        # для UI сохранение конфигурации осей из массива bool, обновление IK
        self.axis_config = [int(v) for v in vals]
        self.update_ik()
        
    def update_ik(self):
        if not self.ik_active:
            return

        pa6 = self.axis['A6']
        pa5 = self.axis['A5']
        pa4 = self.axis['A4']
        pa3 = self.axis['A3']
        pa2 = self.axis['A2']
        pa1 = self.axis['A1']
        a_disp = self.parameters['axis_disp']
        #a_rot = self.parameters['axis_rot'] # TODO пока не используется
        err = False
        
        # положение А6 жёстко привязано к инструменту, можем получить сразу из TCP
        pa6.matrix_world = self.tcp.matrix_world @ self.parameters['tool_mat'].inverted()
        # так же можем получить положение центра А5, но не поворот. Поворот А5 считается в самом конце
        pa5.matrix_world = pa6.matrix_world @ mathutils.Matrix.Translation(a_disp['A6']).inverted()
        # поворот А1. Проецируем положение А5 на плоскость в основании робота, считаем поворот на эту точку
        # TODO не учтён вариант со смещением А1 от rob_root
        pa5_local_mat = self.rob_root.matrix_world.inverted() @ pa5.matrix_world
        if self.axis_config[2] == 1:
            pa1.matrix_world = mathutils.Matrix.Rotation(atan2(pa5_local_mat.translation.y, pa5_local_mat.translation.x)-pi,4,'Z')
        else:
            pa1.matrix_world = mathutils.Matrix.Rotation(atan2(pa5_local_mat.translation.y, pa5_local_mat.translation.x),4,'Z')
        pa1.matrix_world = self.rob_root.matrix_world @ pa1.matrix_world
        # устанавливаем центр А2, пока без поворота
        pa2.matrix_world = pa1.matrix_world @ mathutils.Matrix.Translation(a_disp['A2'])
        # расстояние между центром А2 и А5
        pa2_5_len = (pa5.matrix_world.translation - pa2.matrix_world.translation).length
        # нам известны длины сторон треугольника, образованного центрами А2, А3, А5
        # из этого можно получить угол между вектором, направленным вдоль А2 и от центра А2 на А5
        try:
            a = acos((self._pa2_3_len**2 + pa2_5_len**2 - self._pa3_5_len**2) / (2 * self._pa2_3_len * pa2_5_len))
        except:
            print('ERR:', (self._pa2_3_len**2 + pa2_5_len**2 - self._pa3_5_len**2) / (2 * self._pa2_3_len * pa2_5_len))
            a = 0
            err = True
        # и второй угол - между вектором А2-А5 и вертикальной осью в локальной системе координат А1
        lock_m_pa5 = pa1.matrix_world.inverted() @ pa5.matrix_world
        lock_m_pa2 = pa1.matrix_world.inverted() @ pa2.matrix_world
        dx = lock_m_pa5.translation.x - lock_m_pa2.translation.x
        dz = lock_m_pa5.translation.z - lock_m_pa2.translation.z
        a2 = atan2(dx, dz)
        # из двух полученных ранее углов считаем и применяем поворот А2 
        # в зависимости от заданной конфигурации осей
        if self.axis_config[1] == 1:
            pa2.matrix_world = pa2.matrix_world @ mathutils.Matrix.Rotation((a2 - a) - pi*0.5, 4, 'Y')
        else:
            pa2.matrix_world = pa2.matrix_world @ mathutils.Matrix.Rotation((a2 + a) - pi*0.5, 4, 'Y')
        # А3 получаем схожим образом, но учитываем треугольник образованный осями А3-А4-А5
        pa3.matrix_world = pa2.matrix_world @ mathutils.Matrix.Translation(a_disp['A3'])
        try:
            a = acos((self._pa2_3_len**2 + self._pa3_5_len**2 - pa2_5_len**2) / (2 * self._pa2_3_len * self._pa3_5_len))
        except:
            print('ERR:', (self._pa2_3_len**2 + self._pa3_5_len**2 - pa2_5_len**2) / (2 * self._pa2_3_len * self._pa3_5_len))
            a = pi 
            err = True
        
        if self.axis_config[1] == 1:
            pa3.matrix_world = pa3.matrix_world @ mathutils.Matrix.Rotation(pi - (a - self._a34), 4, 'Y')
        else:
            pa3.matrix_world = pa3.matrix_world @ mathutils.Matrix.Rotation(pi + (a + self._a34), 4, 'Y')
        # положение А4
        pa4.matrix_world = pa3.matrix_world @ mathutils.Matrix.Translation(a_disp['A4'])
        # Если при расчёте углов ранее обнаружили ошибку, значит TCP недостижим
        # необходимо сдвинуть TCP, А5 и А6 вдоль А4 чтобы дистанция стала соответствовать заданной длине оси
        if err:
            v5to4 = pa4.matrix_world.translation - pa5.matrix_world.translation
            brake_len = v5to4.length - self._pa4_5_len
            v5to4.normalize()
            v5to4 = v5to4 * brake_len
            pa5.matrix_world.translation = pa5.matrix_world.translation + v5to4
            pa6.matrix_world.translation = pa6.matrix_world.translation + v5to4
            self.tcp.matrix_world.translation = self.tcp.matrix_world.translation + v5to4
            
        # ось сгибания между А4 и А5 можно посчитать как перпендикуляр к плоскости, образованной 
        # двумя векторами один направлен вдоль А4, второй от А5 к А6
        v = pa6.matrix_world.translation - pa5.matrix_world.translation
        v.normalize()
        v1 = pa5.matrix_world.translation - pa4.matrix_world.translation
        v1.normalize()
        if self.axis_config[0] == 1:
            v2 = mathutils.Vector.cross(v,v1)
        else:
            v2 = mathutils.Vector.cross(v1,v)
        v2.normalize() # Ось сгибания получена
        # теперь у нас есть ось сгибания и ось направления для А4 и А5 (вектор оси сгибания общий),
        # получаем для каждой пары векторов перпендикуляр и из полученных трёх взаимно перпендикулярных
        # векторов строим матрицы поворота для A4 A5
        v3 = mathutils.Vector.cross(v1,v2)
        v3.normalize()
        v4 = mathutils.Vector.cross(v,v2)
        v4.normalize()
        m4 = mathutils.Matrix(([v1.x, v2.x, v3.x], [v1.y, v2.y, v3.y], [v1.z, v2.z, v3.z]))
        m5 = mathutils.Matrix(([v.x, v2.x, v4.x], [v.y, v2.y, v4.y], [v.z, v2.z, v4.z]))
        m4.normalize()
        m5.normalize()
        pa4.matrix_world = mathutils.Matrix.Translation(pa4.matrix_world.translation) @ m4.to_4x4()
        pa5.matrix_world = mathutils.Matrix.Translation(pa5.matrix_world.translation) @ m5.to_4x4()


class ExternalFK():
    
    def __init__(self, parameters):
        self.parameters = parameters # параметры
        self.objects = {}               # ссылки на объекты кинематики
        self._update_ptrs()
        self.flange_state = {fn:0 for fn in self.parameters['ek_flanges'].keys()}
        self._last_flange_mat = {}
        self._update_last_flange_mat()
    
    
    def _update_ptrs(self):
        for o, on in self.parameters['exkin_objects'].items():
            self.objects[o] = bpy.context.scene.objects[on]
    
    def _update_last_flange_mat(self):
        for an, fn in self.parameters['ek_flanges'].items():
            self._last_flange_mat[an] = self.objects[fn].matrix_world.copy()
    
    def get_flange_state(self, ax_name):
        return self.flange_state[ax_name]

    
    def set_flange_state(self, ax_name, val):
        for fn, fv in self.flange_state.items():
            if fv == val:
                self.flange_state[fn] = 0
        self.flange_state[ax_name] = val
        self._update_last_flange_mat()
    
    def get_axis_angle(self, axis_name, form='degrees'):
        # 
        mat = self.objects[axis_name].matrix_world
        #axis_parent = self.get_axis_parent(axis_name)
        #if axis_parent:
        #    mat = axis_parent.matrix_world.inverted_safe() @ mat #self.axis[axis_name].matrix_world
        #else:
        #    mat = self.rob_root.matrix_world.inverted() @ mat
        idx = self.parameters['exax_dir_idx'][axis_name]
        if form == 'degrees':
            return degrees(mat.to_euler()[idx]) * self.parameters['exax_rot_direction'][axis_name]
        return mat.to_euler()[idx] * self.parameters['exax_rot_direction'][axis_name]
    
    def set_axis_angle(self, axis_name, value, form='degrees'):
        # 
        #self._save_axis_local_mat()
        eul = mathutils.Euler((0.0, 0.0, 0.0), 'XYZ')
        idx = self.parameters['exax_dir_idx'][axis_name]
        if form == 'degrees':
            eul[idx] = radians(value) * self.parameters['exax_rot_direction'][axis_name]
        else:
            eul[idx] = value * self.parameters['exax_rot_direction'][axis_name]
        mat = mathutils.Matrix.LocRotScale(self.objects[axis_name].matrix_world.translation, eul, (1,1,1))
        #axis_parent = self.get_axis_parent(axis_name)
        #if axis_parent:
        #    mat = axis_parent.matrix_world @ mat
        #else:
        #    mat = self.rob_root.matrix_world @ mat
        self.objects[axis_name].matrix_world = mat
        
    def get_flange_changes(self, flush = True):
        changes = {}
        for an, fn in self.parameters['ek_flanges'].items():
            fl = self.objects[fn]
            if fl.matrix_world != self._last_flange_mat[an]:
                changes[an] = [self._last_flange_mat[an], fl.matrix_world]
        if flush:
            self._update_last_flange_mat()
        return changes


if __name__ == '__main__':
    #print('!!!')
    rp = bpy.data.texts['rob_parameters'].as_module()
    #fk = RobotFK(rp.parameters)
    #fk.update_tool_pos()
    rk = RobotKinematics(rp.parameters)
    rk.axis_config=[0,1,0]
    rk.update_ik()
    #print(fk.get_axis_angle('A3'))
    #fk.set_axis_angle('A1', 0, propagate=True)
    #fk.set_axis_angle('A3', -90, propagate=True)
