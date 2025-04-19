import usb.core
import usb.util
#from trajectory import TrajPlan
import math
import os
import time
import serial


class TrajPlan:
    def __init__(self):
        self.old_nub = 0 #
        self.interpo_n = 0 #
        self.time_step = [0.0] * 502 #
        self.vi_vf = [[0.0] * 6 for _ in range(6)] #
        self.ai_af = [[0.0] * 6 for _ in range(6)] #
        self.ti_tf = [[0.0] * 6 for _ in range(6)] #
        self.thetai_thetaf = [[0.0] * 6 for _ in range(6)] #
        self.out_traj_theta = [[0.0] * 502 for _ in range(6)] #
        self.out_traj_theta_V = [[0.0] * 502 for _ in range(6)] #
        self.out_traj_theta_A = [[0.0] * 502 for _ in range(6)] #

    def step_clc(self, step_size, t_i, t_f):
        tmp = int((t_f - t_i) / step_size) + 2
        for i in range(self.old_nub, self.old_nub + tmp):
            self.time_step[i] = i * step_size
        self.old_nub += tmp - 1
        return self.old_nub

    def traj_plan(self, point_n, index_angle, t_step_size):
        a0, a1, a2, a3, a4, a5 = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        i = 0
        T = 0
        tmp = 0
        for a in range(point_n - 1):
            self.interpo_n = self.step_clc(t_step_size, self.ti_tf[index_angle][a], self.ti_tf[index_angle][a + 1])
            T = self.ti_tf[index_angle][a + 1] - self.ti_tf[index_angle][a]
            a0 = self.thetai_thetaf[index_angle][a]
            a1 = self.vi_vf[index_angle][a]
            a2 = self.ai_af[index_angle][a] / 2
            a3 = (20 * self.thetai_thetaf[index_angle][a + 1] - 20 * self.thetai_thetaf[index_angle][a] -
                  (8 * self.vi_vf[index_angle][a + 1] + 12 * self.vi_vf[index_angle][a]) * T -
                  (3 * self.ai_af[index_angle][a] - self.ai_af[index_angle][a + 1]) * math.pow(T, 2)) / (2 * math.pow(T, 3))
            a4 = (30 * self.thetai_thetaf[index_angle][a] - 30 * self.thetai_thetaf[index_angle][a + 1] +
                  (14 * self.vi_vf[index_angle][a + 1] + 16 * self.vi_vf[index_angle][a]) * T +
                  (3 * self.ai_af[index_angle][a] - 2 * self.ai_af[index_angle][a + 1]) * math.pow(T, 2)) / (2 * math.pow(T, 4))
            a5 = (12 * self.thetai_thetaf[index_angle][a + 1] - 12 * self.thetai_thetaf[index_angle][a] -
                  (6 * self.vi_vf[index_angle][a + 1] + 6 * self.vi_vf[index_angle][a]) * T -
                  (self.ai_af[index_angle][a] - self.ai_af[index_angle][a + 1]) * math.pow(T, 2)) / (2 * math.pow(T, 5))
            
            for i in range(tmp, self.interpo_n + 1):
                self.out_traj_theta[index_angle][i] = (a0 + a1 * (self.time_step[i] - self.ti_tf[index_angle][a]) +
                                                       a2 * math.pow(self.time_step[i] - self.ti_tf[index_angle][a], 2) +
                                                       a3 * math.pow(self.time_step[i] - self.ti_tf[index_angle][a], 3) +
                                                       a4 * math.pow(self.time_step[i] - self.ti_tf[index_angle][a], 4) +
                                                       a5 * math.pow(self.time_step[i] - self.ti_tf[index_angle][a], 5))
                self.out_traj_theta_V[index_angle][i] = (a1 + 2 * a2 * (self.time_step[i] - self.ti_tf[index_angle][a]) +
                                                         3 * a3 * math.pow(self.time_step[i] - self.ti_tf[index_angle][a], 2) +
                                                         4 * a4 * math.pow(self.time_step[i] - self.ti_tf[index_angle][a], 3) +
                                                         5 * a5 * math.pow(self.time_step[i] - self.ti_tf[index_angle][a], 4))
                self.out_traj_theta_A[index_angle][i] = (2 * a2 + 6 * a3 * (self.time_step[i] - self.ti_tf[index_angle][a]) +
                                                         12 * a4 * math.pow(self.time_step[i] - self.ti_tf[index_angle][a], 2) +
                                                         20 * a5 * math.pow(self.time_step[i] - self.ti_tf[index_angle][a], 3))
            tmp = i

        self.old_nub = 0

class ClcAngle:
    def __init__(self):
        self.out_result_index = 0
        self.c_angle = None
        self.alpha = None
        self.beta = None
        self.gama = None
        self.thtaValue = [[0.0] * 8 for _ in range(6)]
        self.out_thtaValue = [[0.0] * 8 for _ in range(6)]
        self.out_edit_thtaValue = [0.0 for _ in range(8)]
        self.out_edit_thtaValue_flag_1 = self.out_edit_thtaValue_flag_2 = False
        self.out_edit_thtaValue_flag_3 = self.out_edit_thtaValue_flag_4 = False
        self.out_edit_thtaValue_flag_5 = self.out_edit_thtaValue_flag_6 = False
        self.out_traj_flag_1 = self.out_traj_flag_2 = self.out_traj_flag_3 = False
        self.out_traj_flag_4 = self.out_traj_flag_5 = self.out_traj_flag_6 = False
        self.out_edit_xyz_abg_flag_1 = False
        self.Function_FK_state = False

        self.angle_axis_flag = [0 for _ in range(6)]
        self.r11 = self.r21 = self.r31 = 0.0
        self.r12 = self.r22 = self.r32 = 0.0
        self.r13 = self.r23 = self.r33 = 0.0
        self.s1_1 = self.c1_1 = self.s1_2 = self.c1_2 = 0.0
        self.s2 = self.c2 = self.c5 = self.s5 = 0.0
        self.s23 = self.c23 = 0.0
        self.x = self.y = self.z = 0.0
        self.g1 = self.g2 = self.g3 = 0.0
        self.a2 = self.a3 = self.a4 = 0.0
        self.f1 = self.f2 = self.f3 = 0.0
        self.k1 = self.k2 = self.k3 = 0.0
        self.d2 = self.d3 = self.d4 = self.d6 = 0.0
        self.a = self.b = self.g = 0.0
        self.px = self.py = self.pz = 0.0
        self.alpha_out = self.beta_out = self.gama_out = 0.0
        self.px_out = self.py_out = self.pz_out = 0.0
        self.out_fk_4_last = 0.0
        self.out_fk_5_last = 0.0
        self.out_fk_6_last = 0.0
        self.c4 = [0.0 for _ in range(8)]
        self.s4 = [0.0 for _ in range(8)]
        self.out_edit_xyz_abg = [0.0 for _ in range(7)]
        self.get_edit_xyz_abg = [0.0 for _ in range(7)]
        self.angle_flag = [0.0 for _ in range(6)]
        self.theta23_1 = [0.0 for _ in range(8)]
        self.tmp_thtaValue = [[0.0] * 8 for _ in range(6)]
        self.start()

    def start(self):
        self.a2 = 37.5; self.a3 = 160.1; self.a4 = 15  # mm
        self.d2 = 0; self.d3 = 0; self.d4 = 142.15; self.d6 = 33  # mm
        self.x = 0; self.y = 0; self.z = 0
        self.bata = 0; self.alpha = 0; self.gama = 0  # rad

    @staticmethod
    def clamp_angle(angle, min_val, max_val):
        i_ = 0
        if angle < min_val:
            i_ += 1
            angle += 2 * math.pi
        if angle > max_val:
            i_ += 1
            angle = 2 * math.pi - angle
        if angle > max_val:
            i_ += 1
        if i_ >= 3:
            return 3000
        return angle
    
    @staticmethod
    def sign(x):
        return math.copysign(1, x) if x != 0 else 0

    def FK_CLC(self, result_index, tmp_thtaValue):
        s1, s2, s3, s4, s5, s6, s23, c23 = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        c1, c2, c3, c4, c5, c6 = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        pai_2 = math.pi / 2.0
        s1 = math.sin(tmp_thtaValue[0][result_index])
        s2 = math.sin(tmp_thtaValue[1][result_index] - pai_2)
        s3 = math.sin(tmp_thtaValue[2][result_index])
        s4 = math.sin(tmp_thtaValue[3][result_index])
        s5 = math.sin(tmp_thtaValue[4][result_index])
        s6 = math.sin(tmp_thtaValue[5][result_index])
        s23 = math.sin(tmp_thtaValue[1][result_index] + tmp_thtaValue[2][result_index] - pai_2)
        c1 = math.cos(tmp_thtaValue[0][result_index])
        c2 = math.cos(tmp_thtaValue[1][result_index] - pai_2)
        c3 = math.cos(tmp_thtaValue[2][result_index])
        c4 = math.cos(tmp_thtaValue[3][result_index])
        c5 = math.cos(tmp_thtaValue[4][result_index])
        c6 = math.cos(tmp_thtaValue[5][result_index])
        c23 = math.cos(tmp_thtaValue[1][result_index] + tmp_thtaValue[2][result_index] - pai_2)

        r11_ = s6 * (c4 * s1 - s4 * (c1 * c2 * c3 - c1 * s2 * s3)) - c6 * (s5 * (c1 * c2 * s3 + c1 * c3 * s2) - c5 * (s1 * s4 + c4 * (c1 * c2 * c3 - c1 * s2 * s3)))
        r21_ = -c6 * (s5 * (c2 * s1 * s3 + c3 * s1 * s2) + c5 * (c1 * s4 - c4 * (c2 * c3 * s1 - s1 * s2 * s3))) - s6 * (c1 * c4 + s4 * (c2 * c3 * s1 - s1 * s2 * s3))
        r31_ = s4 * s6 * (c2 * s3 + c3 * s2) - c6 * (s5 * (c2 * c3 - s2 * s3) + c4 * c5 * (c2 * s3 + c3 * s2))
        r12_ = s6 * (s5 * (c1 * c2 * s3 + c1 * c3 * s2) - c5 * (s1 * s4 + c4 * (c1 * c2 * c3 - c1 * s2 * s3))) + c6 * (c4 * s1 - s4 * (c1 * c2 * c3 - c1 * s2 * s3))
        r22_ = s6 * (s5 * (c2 * s1 * s3 + c3 * s1 * s2) + c5 * (c1 * s4 - c4 * (c2 * c3 * s1 - s1 * s2 * s3))) - c6 * (c1 * c4 + s4 * (c2 * c3 * s1 - s1 * s2 * s3))
        r32_ = s6 * (s5 * (c2 * c3 - s2 * s3) + c4 * c5 * (c2 * s3 + c3 * s2)) + c6 * s4 * (c2 * s3 + c3 * s2)
        r13_ = -c5 * (c1 * c2 * s3 + c1 * c3 * s2) - s5 * (s1 * s4 + c4 * (c1 * c2 * c3 - c1 * s2 * s3))
        r23_ = s5 * (c1 * s4 - c4 * (c2 * c3 * s1 - s1 * s2 * s3)) - c5 * (c2 * s1 * s3 + c3 * s1 * s2)
        r33_ = c4 * s5 * (c2 * s3 + c3 * s2) - c5 * (c2 * c3 - s2 * s3)

        self.px = self.a2 * c1 - self.d4 * (c1 * c2 * s3 + c1 * c3 * s2) - self.a4 * (c1 * s2 * s3 - c1 * c2 * c3) - self.d2 * s1 - self.d3 * s1 + self.a3 * c1 * c2
        self.py = self.d2 * c1 - self.d4 * (c2 * s1 * s3 + c3 * s1 * s2) - self.a4 * (s1 * s2 * s3 - c2 * c3 * s1) + self.d3 * c1 + self.a2 * s1 + self.a3 * c2 * s1
        self.pz = -self.d4 * c23 - self.a4 * s23 - self.d3 * c2 - self.a3 * s2

        th_1, th_2, th_3, th_4, th_5, th_6 = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 #姿态角计算xyz
        pitch = math.atan2(r13_, math.sqrt(r23_ * r23_ + r33_ * r33_))
        yaw = 0.0
        roll = 0.0
        if pitch == math.pi / 2:
            yaw = 0
            roll = math.atan2(r21_, -r31_)
        elif pitch == -math.pi / 2:
            yaw = 0
            roll = math.atan2(-r21_, r31_)
        else:
            roll = math.atan2(-r23_ / math.cos(pitch), r33_ / math.cos(pitch))
            yaw = math.atan2(-r12_ / math.cos(pitch), r11_ / math.cos(pitch))
        self.g = roll
        self.b = pitch
        self.a = yaw

        thtaValue_test = [0.0] * 6
        for i in range(8):
            thtaValue_test[0] = tmp_thtaValue[0][0] * 180 / math.pi
            thtaValue_test[1] = tmp_thtaValue[1][0] * 180 / math.pi
            thtaValue_test[2] = tmp_thtaValue[2][0] * 180 / math.pi
            thtaValue_test[3] = tmp_thtaValue[3][0] * 180 / math.pi
            thtaValue_test[4] = tmp_thtaValue[4][0] * 180 / math.pi
            thtaValue_test[5] = tmp_thtaValue[5][0] * 180 / math.pi

        if (abs(self.px - self.x) <= 1) and (abs(self.py - self.y) <= 1) and (abs(self.pz - self.z) <= 1) and (self.Function_FK_state == True):
            
            if tmp_thtaValue[0][result_index] <= math.pi / 2.0: # 小于90度情况
                if tmp_thtaValue[0][result_index] >= 0: # 为正90度内
                    th_1 = tmp_thtaValue[0][result_index]
                else: # 为负
                    if abs(tmp_thtaValue[0][result_index]) <= (math.pi / 2): # 为负90度内
                        th_1 = tmp_thtaValue[0][result_index]
                    else:
                        if (math.pi * 2) - abs(tmp_thtaValue[0][result_index]) <= (math.pi / 2): #反转在270度后
                            th_1 = ((math.pi * 2) + (tmp_thtaValue[0][result_index]))
                        else:
                            self.out_result_index -= 1
                            return
            else: #正转超过90度在270度后
                if (math.pi * 2) - abs(tmp_thtaValue[0][result_index]) <= (math.pi / 2): #转在270度后
                    th_1 = -(math.pi * 2 - tmp_thtaValue[0][result_index])
                else:
                    self.out_result_index -= 1
                    return
                
            if tmp_thtaValue[1][result_index] <= math.pi / 2.0: # 小于90度状态
                if tmp_thtaValue[1][result_index] >= 0: #大于0状态
                    th_2 = tmp_thtaValue[1][result_index]
                else: #小于0状态
                    if abs(tmp_thtaValue[1][result_index]) <= (35.0 * math.pi / 180): #在负30度范围内
                        th_2 = tmp_thtaValue[1][result_index]
                    else:
                        if (math.pi * 2) - abs(tmp_thtaValue[1][result_index]) <= (35.0 * math.pi / 180): #反转在270度后
                            th_2 = math.pi * 2 + tmp_thtaValue[1][result_index]
                        else:
                            self.out_result_index -= 1
                            return
            else: #正转超90度在330度情况
                if (math.pi * 2 - abs(tmp_thtaValue[1][result_index])) <= (math.pi / 2):
                    th_2 = -(math.pi * 2.0 - tmp_thtaValue[1][result_index])
                else:
                    self.out_result_index -= 1
                    return

            if tmp_thtaValue[2][result_index] <= math.pi / 2.0: # 小于90度情况
                if tmp_thtaValue[2][result_index] >= 0: #为正90度内
                    th_3 = tmp_thtaValue[2][result_index]
                else: #为负
                    if (tmp_thtaValue[2][result_index]) >= -(math.pi / 2): #为负90度内
                        th_3 = tmp_thtaValue[2][result_index]
                    else:
                        if (math.pi * 2) - abs(tmp_thtaValue[2][result_index]) <= (math.pi / 2):
                            th_3 = ((math.pi * 2) + (tmp_thtaValue[2][result_index]))
                        else:
                            self.out_result_index -= 1
                            return
            else: #正转超过90度在270度后
                if (math.pi * 2) - abs(tmp_thtaValue[2][result_index]) <= (math.pi / 2):
                    th_3 = -(math.pi * 2.0 - tmp_thtaValue[2][result_index])
                else:
                    self.out_result_index -= 1
                    return
                
            if self.out_result_index < 0:
                self.out_result_index = 0

            self.out_thtaValue[0][self.out_result_index] = th_1
            self.out_thtaValue[1][self.out_result_index] = th_2
            self.out_thtaValue[2][self.out_result_index] = th_3

            th_4 = self.clamp_angle(tmp_thtaValue[3][result_index], -math.pi * 2, math.pi * 2) #判断是否超过360度
            th_5 = self.clamp_angle(tmp_thtaValue[4][result_index], -math.pi * 2, math.pi * 2) #判断是否超过360度
            th_6 = self.clamp_angle(tmp_thtaValue[5][result_index], -math.pi * 2, math.pi * 2) #判断是否超过360度

            if abs(th_4) > math.pi: #保证正反转180度内
                th_4 = -self.sign(th_4) * (math.pi * 2 - abs(th_4))
            if abs(th_5) > math.pi: #保证正反转180度内
                th_5 = -self.sign(th_5) * (math.pi * 2 - abs(th_5))
            if abs(th_6) > math.pi: #保证正反转180度内
                th_6 = -self.sign(th_6) * (math.pi * 2 - abs(th_6))

            out_fk_4 = th_4 + math.pi #第二组解将4，5，6轴翻转与当前角度对比
            out_fk_5 = -th_5
            out_fk_6 = th_6 + math.pi
            if abs(out_fk_4) > math.pi: #保证正反转180度内
                out_fk_4 = -self.sign(out_fk_4) * (math.pi * 2 - abs(out_fk_4))
            if abs(out_fk_5) > math.pi: #保证正反转180度内
                out_fk_5 = -self.sign(out_fk_5) * (math.pi * 2 - abs(out_fk_5))
            if abs(out_fk_6) > math.pi: #保证正反转180度内
                out_fk_6 = -self.sign(out_fk_6) * (math.pi * 2 - abs(out_fk_6))

            if (abs(out_fk_4) - abs(self.out_fk_4_last)) < (abs(th_4) - abs(self.out_fk_4_last)):
                self.out_thtaValue[3][self.out_result_index] = out_fk_4
                self.out_thtaValue[4][self.out_result_index] = out_fk_5
                self.out_thtaValue[5][self.out_result_index] = out_fk_6
            else:
                self.out_thtaValue[3][self.out_result_index] = th_4
                self.out_thtaValue[4][self.out_result_index] = th_5
                self.out_thtaValue[5][self.out_result_index] = th_6
            self.out_fk_4_last = self.out_thtaValue[3][self.out_result_index]
            self.out_fk_5_last = self.out_thtaValue[4][self.out_result_index]
            self.out_fk_6_last = self.out_thtaValue[5][self.out_result_index]
            self.alpha_out = self.a
            self.beta_out = self.b
            self.gama_out = self.g
            self.px_out = -r13_ * (-33) + self.px
            self.py_out = -r23_ * (-33) + self.py
            self.pz_out = -r33_ * (-33) + self.pz
            self.out_result_index += 1
            return
        else:
            self.alpha_out = self.a
            self.beta_out = self.b
            self.gama_out = self.g
            self.px_out = -r13_ * (-33) + self.px
            self.py_out = -r23_ * (-33) + self.py
            self.pz_out = -r33_ * (-33) + self.pz
            return
        
    def IK_CLC(self):
        tmp_d = 0.0
        px, py = 0.0, 0.0
        c3 = [0.0] * 8
        s3 = [0.0] * 8
        pai_2 = math.pi / 2.0
        self.end_effector_clc()

        #thta1 
        self.thtaValue[0][0] = (math.atan2(self.y, self.x) - math.atan2(self.d2, math.sqrt(self.x * self.x + self.y * self.y - self.d2 * self.d2)))
        self.thtaValue[0][1] = (math.atan2(self.y, self.x) - math.atan2(self.d2, -math.sqrt(self.x * self.x + self.y * self.y - self.d2 * self.d2)))
        self.s1_1 = math.sin(self.thtaValue[0][0])
        self.c1_1 = math.cos(self.thtaValue[0][0])
        self.s1_2 = math.sin(self.thtaValue[0][1])
        self.c1_2 = math.cos(self.thtaValue[0][1])

        #thta3
        px = 2 * self.d3 * self.a4 + 2 * self.d4 * self.a3
        py = 2 * self.a4 * self.a3 - 2 * self.d4 * self.d3
        tmp_sq = self.d2 * self.d2 + self.a4 * self.a4 + self.a3 * self.a3 + self.d4 * self.d4 + self.d3 * self.d3
        tmp_test = self.x * self.x + self.y * self.y + self.z * self.z + self.a2 * self.a2
        tmp_d = tmp_test - tmp_sq - 2 * self.x * self.c1_1 * self.a2 - 2 * self.y * self.s1_1 * self.a2
        
        self.thtaValue[2][0] = math.atan2(py, px) - math.atan2(tmp_d, math.sqrt(px * px + py * py - tmp_d * tmp_d))
        self.thtaValue[2][1] = math.atan2(py, px) - math.atan2(tmp_d, -math.sqrt(px * px + py * py - tmp_d * tmp_d))
        tmp_d = tmp_test - tmp_sq - 2 * self.x * self.c1_2 * self.a2 - 2 * self.y * self.s1_2 * self.a2
        denominator = px * px + py * py - tmp_d * tmp_d
        if denominator < 0:
            denominator = 0 
        self.thtaValue[2][2] = math.atan2(py, px) - math.atan2(tmp_d, math.sqrt(denominator)) #重大错误！！！
        self.thtaValue[2][3] = math.atan2(py, px) - math.atan2(tmp_d, -math.sqrt(denominator)) #重大错误！！！

        #thta2
        c3[0] = math.cos(self.thtaValue[2][0])
        c3[1] = math.cos(self.thtaValue[2][1])
        c3[2] = math.cos(self.thtaValue[2][2])
        c3[3] = math.cos(self.thtaValue[2][3])
        s3[0] = math.sin(self.thtaValue[2][0])
        s3[1] = math.sin(self.thtaValue[2][1])
        s3[2] = math.sin(self.thtaValue[2][2])
        s3[3] = math.sin(self.thtaValue[2][3])

        px = self.a4 * c3[0] + self.a3 - self.d4 * s3[0]
        py = -self.d4 * c3[0] - self.a4 * s3[0] - self.d3
        tmp_d = self.z

        self.thtaValue[1][0] = math.atan2(py, px) - math.atan2(tmp_d, math.sqrt(px * px + py * py - tmp_d * tmp_d)) + pai_2
        self.thtaValue[1][1] = math.atan2(py, px) - math.atan2(tmp_d, -math.sqrt(px * px + py * py - tmp_d * tmp_d)) + pai_2

        px = self.a4 * c3[1] + self.a3 - self.d4 * s3[1]
        py = -self.d4 * c3[1] - self.a4 * s3[1] - self.d3
        self.thtaValue[1][2] = math.atan2(py, px) - math.atan2(tmp_d, math.sqrt(px * px + py * py - tmp_d * tmp_d)) + pai_2
        self.thtaValue[1][3] = math.atan2(py, px) - math.atan2(tmp_d, -math.sqrt(px * px + py * py - tmp_d * tmp_d)) + pai_2

        px = self.a4 * c3[2] + self.a3 - self.d4 * s3[2]
        py = -self.d4 * c3[2] - self.a4 * s3[2] - self.d3
        self.thtaValue[1][4] = math.atan2(py, px) - math.atan2(tmp_d, math.sqrt(px * px + py * py - tmp_d * tmp_d)) + pai_2
        self.thtaValue[1][5] = math.atan2(py, px) - math.atan2(tmp_d, -math.sqrt(px * px + py * py - tmp_d * tmp_d)) + pai_2

        px = self.a4 * c3[3] + self.a3 - self.d4 * s3[3]
        py = -self.d4 * c3[3] - self.a4 * s3[3] - self.d3
        self.thtaValue[1][6] = math.atan2(py, px) - math.atan2(tmp_d, math.sqrt(px * px + py * py - tmp_d * tmp_d)) + pai_2
        self.thtaValue[1][7] = math.atan2(py, px) - math.atan2(tmp_d, -math.sqrt(px * px + py * py - tmp_d * tmp_d)) + pai_2
        self.theta23_1[0] = self.thtaValue[1][0] + self.thtaValue[2][0] - pai_2
        self.theta23_1[1] = self.thtaValue[1][1] + self.thtaValue[2][0] - pai_2
        self.theta23_1[2] = self.thtaValue[1][2] + self.thtaValue[2][1] - pai_2
        self.theta23_1[3] = self.thtaValue[1][3] + self.thtaValue[2][1] - pai_2
        self.theta23_1[4] = self.thtaValue[1][4] + self.thtaValue[2][2] - pai_2
        self.theta23_1[5] = self.thtaValue[1][5] + self.thtaValue[2][2] - pai_2
        self.theta23_1[6] = self.thtaValue[1][6] + self.thtaValue[2][3] - pai_2
        self.theta23_1[7] = self.thtaValue[1][7] + self.thtaValue[2][3] - pai_2

        self.caclu_one_result()

    def caclu_one_result(self):
        s12 = [0.0] * 8
        c12 = [0.0] * 8
        c23 = [0.0] * 8
        s23 = [0.0] * 8
        px, py = 0.0, 0.0

        #theta4, theta5, theta6
        s23[0] = math.sin(self.theta23_1[0]); c23[0] = math.cos(self.theta23_1[0])
        s23[1] = math.sin(self.theta23_1[1]); c23[1] = math.cos(self.theta23_1[1])
        s23[2] = math.sin(self.theta23_1[2]); c23[2] = math.cos(self.theta23_1[2])
        s23[3] = math.sin(self.theta23_1[3]); c23[3] = math.cos(self.theta23_1[3])
        s23[4] = math.sin(self.theta23_1[4]); c23[4] = math.cos(self.theta23_1[4])
        s23[5] = math.sin(self.theta23_1[5]); c23[5] = math.cos(self.theta23_1[5])
        s23[6] = math.sin(self.theta23_1[6]); c23[6] = math.cos(self.theta23_1[6])
        s23[7] = math.sin(self.theta23_1[7]); c23[7] = math.cos(self.theta23_1[7])

        #theta4
        tmp_index = 0
        px = self.r33 * s23[tmp_index] - self.r13 * self.c1_1 * c23[tmp_index] - self.r23 * c23[tmp_index] * self.s1_1
        py = self.r23 * self.c1_1 - self.r13 * self.s1_1
        self.thtaValue[3][tmp_index] = math.atan2(py, px)
        tmp_index = 1
        px = self.r33 * s23[tmp_index] - self.r13 * self.c1_2 * c23[tmp_index] - self.r23 * c23[tmp_index] * self.s1_2
        py = self.r23 * self.c1_2 - self.r13 * self.s1_2
        self.thtaValue[3][tmp_index] = math.atan2(py, px)
        tmp_index = 2
        px = self.r33 * s23[tmp_index] - self.r13 * self.c1_1 * c23[tmp_index] - self.r23 * c23[tmp_index] * self.s1_1
        py = self.r23 * self.c1_1 - self.r13 * self.s1_1
        self.thtaValue[3][tmp_index] = math.atan2(py, px)
        tmp_index = 3
        px = self.r33 * s23[tmp_index] - self.r13 * self.c1_2 * c23[tmp_index] - self.r23 * c23[tmp_index] * self.s1_2
        py = self.r23 * self.c1_2 - self.r13 * self.s1_2
        self.thtaValue[3][tmp_index] = math.atan2(py, px)

        tmp_index = 4
        px = self.r33 * s23[tmp_index] - self.r13 * self.c1_2 * c23[tmp_index] - self.r23 * c23[tmp_index] * self.s1_1
        py = self.r23 * self.c1_1 - self.r13 * self.s1_1
        self.thtaValue[3][tmp_index] = math.atan2(py, px)
        tmp_index = 5
        px = self.r33 * s23[tmp_index] - self.r13 * self.c1_2 * c23[tmp_index] - self.r23 * c23[tmp_index] * self.s1_2
        py = self.r23 * self.c1_2 - self.r13 * self.s1_2
        self.thtaValue[3][tmp_index] = math.atan2(py, px)
        tmp_index = 6
        px = self.r33 * s23[tmp_index] - self.r13 * self.c1_1 * c23[tmp_index] - self.r23 * c23[tmp_index] * self.s1_1
        py = self.r23 * self.c1_1 - self.r13 * self.s1_1
        self.thtaValue[3][tmp_index] = math.atan2(py, px)
        tmp_index = 7
        px = self.r33 * s23[tmp_index] - self.r13 * self.c1_2 * c23[tmp_index] - self.r23 * c23[tmp_index] * self.s1_2
        py = self.r23 * self.c1_2 - self.r13 * self.s1_2
        self.thtaValue[3][tmp_index] = math.atan2(py, px)

        self.s4[0] = math.sin(self.thtaValue[3][0]); self.c4[0] = math.cos(self.thtaValue[3][0])
        self.s4[1] = math.sin(self.thtaValue[3][1]); self.c4[1] = math.cos(self.thtaValue[3][1])
        self.s4[2] = math.sin(self.thtaValue[3][2]); self.c4[2] = math.cos(self.thtaValue[3][2])
        self.s4[3] = math.sin(self.thtaValue[3][3]); self.c4[3] = math.cos(self.thtaValue[3][3])
        self.s4[4] = math.sin(self.thtaValue[3][4]); self.c4[4] = math.cos(self.thtaValue[3][4])
        self.s4[5] = math.sin(self.thtaValue[3][5]); self.c4[5] = math.cos(self.thtaValue[3][5])
        self.s4[6] = math.sin(self.thtaValue[3][6]); self.c4[6] = math.cos(self.thtaValue[3][6])
        self.s4[7] = math.sin(self.thtaValue[3][7]); self.c4[7] = math.cos(self.thtaValue[3][7])

        #theta5
        tmp_index = 0
        py = -self.r13 * (self.c1_1 * c23[tmp_index] * self.c4[tmp_index] + self.s1_1 * self.s4[tmp_index]) + self.r23 * (-self.s1_1 * c23[tmp_index] * self.c4[tmp_index] + self.c1_1 * self.s4[tmp_index]) + self.r33 * (s23[tmp_index] * self.c4[tmp_index])
        px = self.r13 * (-self.c1_1 * s23[tmp_index]) + self.r23 * (-self.s1_1 * s23[tmp_index]) + self.r33 * (-c23[tmp_index])
        self.thtaValue[4][tmp_index] = math.atan2(py, px)

        tmp_index = 1
        py = -self.r13 * (self.c1_2 * c23[tmp_index] * self.c4[tmp_index] + self.s1_2 * self.s4[tmp_index]) + self.r23 * (-self.s1_2 * c23[tmp_index] * self.c4[tmp_index] + self.c1_2 * self.s4[tmp_index]) + self.r33 * (s23[tmp_index] * self.c4[tmp_index])
        px = self.r13 * (-self.c1_2 * s23[tmp_index]) + self.r23 * (-self.s1_2 * s23[tmp_index]) + self.r33 * (-c23[tmp_index])
        self.thtaValue[4][tmp_index] = math.atan2(py, px)

        tmp_index = 2
        py = -self.r13 * (self.c1_1 * c23[tmp_index] * self.c4[tmp_index] + self.s1_1 * self.s4[tmp_index]) + self.r23 * (-self.s1_1 * c23[tmp_index] * self.c4[tmp_index] + self.c1_1 * self.s4[tmp_index]) + self.r33 * (s23[tmp_index] * self.c4[tmp_index])
        px = self.r13 * (-self.c1_1 * s23[tmp_index]) + self.r23 * (-self.s1_1 * s23[tmp_index]) + self.r33 * (-c23[tmp_index])
        self.thtaValue[4][tmp_index] = math.atan2(py, px)

        tmp_index = 3
        py = -self.r13 * (self.c1_2 * c23[tmp_index] * self.c4[tmp_index] + self.s1_2 * self.s4[tmp_index]) + self.r23 * (-self.s1_2 * c23[tmp_index] * self.c4[tmp_index] + self.c1_2 * self.s4[tmp_index]) + self.r33 * (s23[tmp_index] * self.c4[tmp_index])
        px = self.r13 * (-self.c1_2 * s23[tmp_index]) + self.r23 * (-self.s1_2 * s23[tmp_index]) + self.r33 * (-c23[tmp_index])
        self.thtaValue[4][tmp_index] = math.atan2(py, px)

        tmp_index = 4
        py = -self.r13 * (self.c1_1 * c23[tmp_index] * self.c4[tmp_index] + self.s1_1 * self.s4[tmp_index]) + self.r23 * (-self.s1_1 * c23[tmp_index] * self.c4[tmp_index] + self.c1_1 * self.s4[tmp_index]) + self.r33 * (s23[tmp_index] * self.c4[tmp_index])
        px = self.r13 * (-self.c1_1 * s23[tmp_index]) + self.r23 * (-self.s1_1 * s23[tmp_index]) + self.r33 * (-c23[tmp_index])
        self.thtaValue[4][tmp_index] = math.atan2(py, px)

        tmp_index = 5
        py = -self.r13 * (self.c1_2 * c23[tmp_index] * self.c4[tmp_index] + self.s1_2 * self.s4[tmp_index]) + self.r23 * (-self.s1_2 * c23[tmp_index] * self.c4[tmp_index] + self.c1_2 * self.s4[tmp_index]) + self.r33 * (s23[tmp_index] * self.c4[tmp_index])
        px = self.r13 * (-self.c1_2 * s23[tmp_index]) + self.r23 * (-self.s1_2 * s23[tmp_index]) + self.r33 * (-c23[tmp_index])
        self.thtaValue[4][tmp_index] = math.atan2(py, px)

        tmp_index = 6
        py = -self.r13 * (self.c1_1 * c23[tmp_index] * self.c4[tmp_index] + self.s1_1 * self.s4[tmp_index]) + self.r23 * (-self.s1_1 * c23[tmp_index] * self.c4[tmp_index] + self.c1_1 * self.s4[tmp_index]) + self.r33 * (s23[tmp_index] * self.c4[tmp_index])
        px = self.r13 * (-self.c1_1 * s23[tmp_index]) + self.r23 * (-self.s1_1 * s23[tmp_index]) + self.r33 * (-c23[tmp_index])
        self.thtaValue[4][tmp_index] = math.atan2(py, px)

        tmp_index = 7
        py = -self.r13 * (self.c1_2 * c23[tmp_index] * self.c4[tmp_index] + self.s1_2 * self.s4[tmp_index]) + self.r23 * (-self.s1_2 * c23[tmp_index] * self.c4[tmp_index] + self.c1_2 * self.s4[tmp_index]) + self.r33 * (s23[tmp_index] * self.c4[tmp_index])
        px = self.r13 * (-self.c1_2 * s23[tmp_index]) + self.r23 * (-self.s1_2 * s23[tmp_index]) + self.r33 * (-c23[tmp_index])
        self.thtaValue[4][tmp_index] = math.atan2(py, px)
        
        self.theta_6()

    def theta_6(self):
        px, py = 0.0, 0.0
        s23 = [0.0] * 8
        c23 = [0.0] * 8
        s5 = [0.0] * 8
        c5 = [0.0] * 8

        s23[0] = math.sin(self.theta23_1[0]); c23[0] = math.cos(self.theta23_1[0])
        s23[1] = math.sin(self.theta23_1[1]); c23[1] = math.cos(self.theta23_1[1])
        s23[2] = math.sin(self.theta23_1[2]); c23[2] = math.cos(self.theta23_1[2])
        s23[3] = math.sin(self.theta23_1[3]); c23[3] = math.cos(self.theta23_1[3])
        s23[4] = math.sin(self.theta23_1[4]); c23[4] = math.cos(self.theta23_1[4])
        s23[5] = math.sin(self.theta23_1[5]); c23[5] = math.cos(self.theta23_1[5])
        s23[6] = math.sin(self.theta23_1[6]); c23[6] = math.cos(self.theta23_1[6])
        s23[7] = math.sin(self.theta23_1[7]); c23[7] = math.cos(self.theta23_1[7])
        s5[0] = math.sin(self.thtaValue[4][0]); c5[0] = math.cos(self.thtaValue[4][0])
        s5[1] = math.sin(self.thtaValue[4][1]); c5[1] = math.cos(self.thtaValue[4][1])
        s5[2] = math.sin(self.thtaValue[4][2]); c5[2] = math.cos(self.thtaValue[4][2])
        s5[3] = math.sin(self.thtaValue[4][3]); c5[3] = math.cos(self.thtaValue[4][3])
        s5[4] = math.sin(self.thtaValue[4][4]); c5[4] = math.cos(self.thtaValue[4][4])
        s5[5] = math.sin(self.thtaValue[4][5]); c5[5] = math.cos(self.thtaValue[4][5])
        s5[6] = math.sin(self.thtaValue[4][6]); c5[6] = math.cos(self.thtaValue[4][6])
        s5[7] = math.sin(self.thtaValue[4][7]); c5[7] = math.cos(self.thtaValue[4][7])

        #theta6
        tmp_index = 0
        px = (-self.r31 * c23[tmp_index] - self.r11 * self.c1_1 * s23[tmp_index] - self.r21 * self.s1_1 * s23[tmp_index]) / s5[tmp_index]
        py = (self.r32 * c23[tmp_index] + self.r12 * self.c1_1 * s23[tmp_index] + self.r22 * self.s1_1 * s23[tmp_index]) / s5[tmp_index]
        self.thtaValue[5][tmp_index] = math.atan2(py, px)

        tmp_index = 1
        px = (-self.r31 * c23[tmp_index] - self.r11 * self.c1_2 * s23[tmp_index] - self.r21 * self.s1_2 * s23[tmp_index]) / s5[tmp_index]
        py = (self.r32 * c23[tmp_index] + self.r12 * self.c1_2 * s23[tmp_index] + self.r22 * self.s1_2 * s23[tmp_index]) / s5[tmp_index]
        self.thtaValue[5][tmp_index] = math.atan2(py, px)

        tmp_index = 2
        px = (-self.r31 * c23[tmp_index] - self.r11 * self.c1_1 * s23[tmp_index] - self.r21 * self.s1_1 * s23[tmp_index]) / s5[tmp_index]
        py = (self.r32 * c23[tmp_index] + self.r12 * self.c1_1 * s23[tmp_index] + self.r22 * self.s1_1 * s23[tmp_index]) / s5[tmp_index]
        self.thtaValue[5][tmp_index] = math.atan2(py, px)

        tmp_index = 3
        px = (-self.r31 * c23[tmp_index] - self.r11 * self.c1_2 * s23[tmp_index] - self.r21 * self.s1_2 * s23[tmp_index]) / s5[tmp_index]
        py = (self.r32 * c23[tmp_index] + self.r12 * self.c1_2 * s23[tmp_index] + self.r22 * self.s1_2 * s23[tmp_index]) / s5[tmp_index]
        self.thtaValue[5][tmp_index] = math.atan2(py, px)

        tmp_index = 4
        px = (-self.r31 * c23[tmp_index] - self.r11 * self.c1_1 * s23[tmp_index] - self.r21 * self.s1_1 * s23[tmp_index]) / s5[tmp_index]
        py = (self.r32 * c23[tmp_index] + self.r12 * self.c1_1 * s23[tmp_index] + self.r22 * self.s1_1 * s23[tmp_index]) / s5[tmp_index]
        self.thtaValue[5][tmp_index] = math.atan2(py, px)

        tmp_index = 5
        px = (-self.r31 * c23[tmp_index] - self.r11 * self.c1_2 * s23[tmp_index] - self.r21 * self.s1_2 * s23[tmp_index]) / s5[tmp_index]
        py = (self.r32 * c23[tmp_index] + self.r12 * self.c1_2 * s23[tmp_index] + self.r22 * self.s1_2 * s23[tmp_index]) / s5[tmp_index]
        self.thtaValue[5][tmp_index] = math.atan2(py, px)

        tmp_index = 6
        px = (-self.r31 * c23[tmp_index] - self.r11 * self.c1_1 * s23[tmp_index] - self.r21 * self.s1_1 * s23[tmp_index]) / s5[tmp_index]
        py = (self.r32 * c23[tmp_index] + self.r12 * self.c1_1 * s23[tmp_index] + self.r22 * self.s1_1 * s23[tmp_index]) / s5[tmp_index]
        self.thtaValue[5][tmp_index] = math.atan2(py, px)

        tmp_index = 7
        px = (-self.r31 * c23[tmp_index] - self.r11 * self.c1_2 * s23[tmp_index] - self.r21 * self.s1_2 * s23[tmp_index]) / s5[tmp_index]
        py = (self.r32 * c23[tmp_index] + self.r12 * self.c1_2 * s23[tmp_index] + self.r22 * self.s1_2 * s23[tmp_index]) / s5[tmp_index]
        self.thtaValue[5][tmp_index] = math.atan2(py, px)

    def IK_PUBLIC_F(self):
        self.IK_CLC()
        tmp_index = 0
        i = 0
        self.Function_FK_state = True
        for tmp_index in range(4):
            self.tmp_thtaValue[0][tmp_index] = self.thtaValue[0][0]
            self.tmp_thtaValue[1][tmp_index] = self.thtaValue[1][tmp_index * 2]
            self.tmp_thtaValue[2][tmp_index] = self.thtaValue[2][tmp_index]
            self.tmp_thtaValue[3][tmp_index] = self.thtaValue[3][tmp_index * 2]
            self.tmp_thtaValue[4][tmp_index] = self.thtaValue[4][tmp_index * 2]
            self.tmp_thtaValue[5][tmp_index] = self.thtaValue[5][tmp_index * 2]

        #for tmp_index in range(8):
        tmp_index = 0
        while tmp_index < 8:
            self.tmp_thtaValue[0][i + 4] = self.thtaValue[0][1]
            self.tmp_thtaValue[1][i + 4] = self.thtaValue[1][tmp_index + 1]
            self.tmp_thtaValue[2][i + 4] = self.thtaValue[2][i]
            self.tmp_thtaValue[3][i + 4] = self.thtaValue[3][tmp_index + 1]
            self.tmp_thtaValue[4][i + 4] = self.thtaValue[4][tmp_index + 1]
            self.tmp_thtaValue[5][i + 4] = self.thtaValue[5][tmp_index + 1]
            i += 1
            tmp_index += 2
        for tmp_index in range(8):
            self.FK_CLC(tmp_index, self.tmp_thtaValue)
        self.Function_FK_state = False

    def end_effector_clc(self):
        xyz_end_effector = [0.0] * 4
        self.x = self.out_edit_xyz_abg[0]; self.y = self.out_edit_xyz_abg[1]; self.z = self.out_edit_xyz_abg[2]
        self.bata = self.out_edit_xyz_abg[4]; self.alpha = self.out_edit_xyz_abg[3]; self.gama = self.out_edit_xyz_abg[5] #rad

        self.d6 = -33 #旋转轴中心点到末端的长度（逆矩阵）
				      # PS：当你将两个相同的点输入坐标进入动画，
				      #但是动画的末端点却没有在同一位置或者就是离坐标点有一定距离这是因为末端长度是我根据实体测量得出
				      #要想动画得出接近相同点自己测量模型末端坐标填入这个长度

        self.r11 = math.cos(self.bata) * math.cos(self.alpha)
        self.r21 = math.sin(self.gama) * math.sin(self.bata) * math.cos(self.alpha) + math.cos(self.gama) * math.sin(self.alpha) #旋转矩阵
        self.r31 = -math.cos(self.alpha) * math.sin(self.bata) * math.cos(self.gama) + math.sin(self.alpha) * math.sin(self.gama) #旋转矩阵

        self.r12 = -math.cos(self.bata) * math.sin(self.alpha)
        self.r22 = -math.sin(self.alpha) * math.sin(self.bata) * math.sin(self.gama) + math.cos(self.alpha) * math.cos(self.gama) #旋转矩阵

        self.r32 = math.cos(self.gama) * math.sin(self.alpha) * math.sin(self.bata) + math.sin(self.gama) * math.cos(self.alpha) #旋转矩阵

        self.r13 = math.sin(self.bata) #旋转矩阵
        self.r23 = -math.sin(self.gama) * math.cos(self.bata) #旋转矩阵
        self.r33 = math.cos(self.gama) * math.cos(self.bata) #旋转矩阵

        self.x = self.r13 * (self.d6) + self.x
        self.y = self.r23 * (self.d6) + self.y
        self.z = self.r33 * (self.d6) + self.z

class Can_transfer:
    def __init__(self, idVendor=0x28E9, idProduct=0x018A):
        self.traj_out = TrajPlan()
        self.c_angle = ClcAngle()
        self.open_deviceflag = False
        self.read_angle_flag = False
        self.write_angle_flag = False
        self.write_traj_flag = False
        self.write_res_angle_flag = False
        self.write_zero_angle_flag = False
        self.motor_enable_state = False
        self.motor_disable_state = False
        self.read_Buffer = bytearray(58)
        self.reader = None
        self.writer = None
        self._1_link_angle, self._2_link_angle, self._3_link_angle,  self._4_link_angle, self._5_link_angle, self._6_link_angle= 0,0,0,0,0,0
        self._1_edit_angle, self._2_edit_angle, self._3_edit_angle, self._4_edit_angle, self._5_edit_angle, self._6_edit_angle = 0,0,0,0,0,0
        self.send_inte_point = 0
        self.zero_1, self.zero_2, self.zero_3, self.zero_4, self.zero_5, self.zero_6 = 0,0,0,0,0,0
        self.res_1, self.res_2, self.res_3, self.res_4, self.res_5, self.res_6 = 0,0,0,0,0,0
        self.angle_radian = [[0.0] * 8 for _ in range(6)]
        self.OPEN_DEVICE(idVendor, idProduct)

    def Start(self, res, zero):
        self.res_1, self.res_2, self.res_3, self.res_4, self.res_5, self.res_6 = res
        self.zero_1, self.zero_2, self.zero_3, self.zero_4, self.zero_5, self.zero_6 = zero
        
        # self.USBWrite_motor_disable()  #初始化alpha beta gama
        # self.USBWrite_motor_enable()  #初始化alpha beta gama

    def OPEN_DEVICE(self, idVendor, idProduct):
        # 根据设备的vendor_id和product_id查找设备
        self.MyUsbDevice = usb.core.find(idVendor=idVendor, idProduct=idProduct)

        if self.MyUsbDevice is None:
            self.open_deviceflag = False
            return self.open_deviceflag
        
        # 设置配置
        self.MyUsbDevice.set_configuration()
        usb.util.claim_interface(self.MyUsbDevice, 0)

        # 打开读取数据的端点
        cfg = self.MyUsbDevice.get_active_configuration()
        intf = cfg[(0,0)]

        self.reader = usb.util.find_descriptor(
            intf,
            # 找到IN端点
            custom_match = lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN
        )

        self.writer = usb.util.find_descriptor(
            intf,
            # 找到OUT端点
            custom_match = lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_OUT
        )

        self.open_deviceflag = True
        return self.open_deviceflag

    def USBRead_data(self):
        try:
            data = self.reader.read(self.reader.wMaxPacketSize, timeout=1000)  # 读取数据
            self.read_Buffer = bytearray(data)
            return self.read_Buffer
        except usb.core.USBError as e:
            if e.errno == 110:  # 超时错误
                pass  # 可以添加处理超时的代码
            else:
                raise e

    def USBWrite_Request_to_positiondata(self):
        witeBuffer = bytearray(57)
        witeBuffer[0] = 0x01  # 说明请求电机的用途(01读取电机位置信息,外部功能码相对USB)
        witeBuffer[1] = 0x00  # 给轨迹规划功能码使用（轨迹插值点数）
        witeBuffer[2] = 0x00  # 给轨迹规划功能码使用（轨迹插值当前点数用来校对数据丢失）
        tmp_adress = 0
        for i in range(6):
            tmp_adress += 1
            witeBuffer[i * 9 + 3] = tmp_adress
            witeBuffer[i * 9 + 4] = 0x01  # 01功能码(对应电机设备功能码读)
            witeBuffer[i * 9 + 5] = 0x11  # (对应电机设备寄存器0x11为速度)
            witeBuffer[i * 9 + 6] = 0x00
            witeBuffer[i * 9 + 7] = 0x00
            witeBuffer[i * 9 + 8] = 0x00
            witeBuffer[i * 9 + 9] = 0x00
            witeBuffer[i * 9 + 10] = 0x00
            witeBuffer[i * 9 + 11] = 0x00
        
        if self.writer is None:
            self.open_deviceflag = False
            return
        ec = self.writer.write(witeBuffer, timeout=1000)
        return ec

    def USBWrite_send_angle_pos(self):
        witeBuffer = bytearray(57)
        witeBuffer[0] = 0x04  # 说明请求电机的用途(02写入电机位置速度信息,外部功能码相对USB)
        witeBuffer[1] = 0  # 给轨迹规划功能码使用（轨迹插值点数）
        witeBuffer[2] = 0  # 给轨迹规划功能码使用（轨迹插值当前点数用来校对数据丢失）
        tmp_angle = 0
        tmp_speed = 0
        tmp_adress = 0
        self.read_angle_flag = False

        for i in range(6):
            if i == 0:
                tmp_angle = self._1_edit_angle
                tmp_speed = 500 if tmp_angle > 0 else -500
            if i == 1:
                tmp_angle = self._2_edit_angle
                tmp_speed = 50 if tmp_angle > 0 else 50
            if i == 2:
                tmp_angle = self._3_edit_angle
                tmp_speed = 50 if tmp_angle > 0 else 50
            if i == 3:
                tmp_angle = self._4_edit_angle
                tmp_speed = 50 if tmp_angle > 0 else 50
            if i == 4:
                tmp_angle = self._5_edit_angle
                tmp_speed = 50 if tmp_angle > 0 else 50
            if i == 5:
                tmp_angle = self._6_edit_angle
                # tmp_speed = 50 if tmp_angle > 0 else 50
                tmp_speed = 0

            witeBuffer[i * 9 + 3] = tmp_adress
            witeBuffer[i * 9 + 4] = 0x02  # 01功能码
            witeBuffer[i * 9 + 5] = 0x0f
            witeBuffer[i * 9 + 6] = (tmp_speed >> 8) & 0xFF
            witeBuffer[i * 9 + 7] = tmp_speed & 0xFF
            if tmp_angle < 0:
                tmp_angle = (1 << 32) + tmp_angle
            witeBuffer[i * 9 + 8] = (tmp_angle >> 24) & 0xFF
            witeBuffer[i * 9 + 9] = (tmp_angle >> 16) & 0xFF
            witeBuffer[i * 9 + 10] = (tmp_angle >> 8) & 0xFF
            witeBuffer[i * 9 + 11] = tmp_angle & 0xFF

        self.write_angle_flag = False
        self.write_res_angle_flag = False
        self.write_zero_angle_flag = False
        self.read_angle_flag = True

        if self.writer is None:
            self.open_deviceflag = False
            return

        ec = self.writer.write(witeBuffer, timeout=1000)

    def USBWrite_send_traj_pos(self):
        witeBuffer = bytearray(57)
        witeBuffer[0] = 0x02  # 说明请求电机的用途(02写入电机位置速度信息,外部功能码相对USB)
        witeBuffer[1] = self.traj_out.interpo_n #.to_bytes(1, byteorder='big')
        witeBuffer[2] = self.send_inte_point #.to_bytes(1, byteorder='big')
        tmp_angle = 0
        tmp_speed = 0
        tmp_adress = 0
        # self.read_angle_flag = False

        for i in range(6):
            tmp_adress += 1
            if i == 0:
                tmp_angle = int(self.traj_out.out_traj_theta[i][self.send_inte_point] * (51200.0 / 360.0) * 27 * 5 * (180.0 / math.pi)) + self.zero_1
                tmp_speed = int(self.traj_out.out_traj_theta_V[i][self.send_inte_point] * 9.549 * 27 * 5)
            elif i == 1:
                tmp_angle = -int(self.traj_out.out_traj_theta[i][self.send_inte_point] * (51200.0 / 360.0) * 27 * 5 * (180.0 / math.pi)) + self.zero_2
                tmp_speed = int(self.traj_out.out_traj_theta_V[i][self.send_inte_point] * 9.549 * 5 * 27)
            elif i == 2:
                tmp_angle = int(self.traj_out.out_traj_theta[i][self.send_inte_point] * (51200.0 / 360.0) * 5 * 14.0 * (180.0 / math.pi)) + self.zero_3
                tmp_speed = int(self.traj_out.out_traj_theta_V[i][self.send_inte_point] * 9.549 * 5 * 14.0)
            elif i == 3:
                tmp_angle = int(self.traj_out.out_traj_theta[i][self.send_inte_point] * (51200.0 / 360.0) * 3.0 * (180.0 / math.pi) * 5.18) + self.zero_4
                tmp_speed = int(self.traj_out.out_traj_theta_V[i][self.send_inte_point] * 9.549 * 3.0 * 5.18)
            elif i == 4:
                tmp_angle = int(self.traj_out.out_traj_theta[i][self.send_inte_point] * (51200.0 / 360.0) * 1.6 * (180.0 / math.pi) ) + self.zero_5
                tmp_speed = int(self.traj_out.out_traj_theta_V[i][self.send_inte_point] * 9.549 * 1.6 )
            elif i == 5:
                tmp_angle = int(self.traj_out.out_traj_theta[i][self.send_inte_point] * (51200.0 / 360.0) * (180.0 / math.pi)) + self.zero_6
                tmp_speed = int(self.traj_out.out_traj_theta_V[i][self.send_inte_point] * 9.549)
            # print(i,'==',tmp_angle,tmp_speed)
            witeBuffer[i * 9 + 3] = tmp_adress
            witeBuffer[i * 9 + 4] = 0x02  # 01功能码
            witeBuffer[i * 9 + 5] = 0x0f
            witeBuffer[i * 9 + 6] = (tmp_speed >> 8) & 0xFF
            witeBuffer[i * 9 + 7] = tmp_speed & 0xFF
            if tmp_angle < 0:
               tmp_angle = (1 << 32) + tmp_angle
            witeBuffer[i * 9 + 8] = (tmp_angle >> 24) & 0xFF
            witeBuffer[i * 9 + 9] = (tmp_angle >> 16) & 0xFF
            witeBuffer[i * 9 + 10] = (tmp_angle >> 8) & 0xFF
            witeBuffer[i * 9 + 11] = tmp_angle & 0xFF

        self.send_inte_point += 1
        
        if self.send_inte_point >= self.traj_out.interpo_n:
            self.traj_out.interpo_n = 0
            self.send_inte_point = 0
            self.write_traj_flag = False
            self.read_angle_flag = True

        # self.write_traj_flag = False
        # self.read_angle_flag = True
        if self.writer is None:
            self.open_deviceflag = False
            return

        ec = self.writer.write(witeBuffer, timeout=1000)
        # print(ec)

    def USBWrite_motor_enable(self): # 电机使能
        witeBuffer = bytearray(57)
        witeBuffer[0] = 0x04  # 说明请求电机的用途(02写入电机位置速度信息,外部功能码相对USB)
        witeBuffer[1] = 0  # 给轨迹规划功能码使用（轨迹插值点数）
        witeBuffer[2] = 0  # 给轨迹规划功能码使用（轨迹插值当前点数用来校对数据丢失）
        tmp_angle = 0
        tmp_speed = 0
        tmp_adress = 0
        self.read_angle_flag = False

        for i in range(6):
            if i == 0:
                tmp_angle = 0
                tmp_speed = 1
            if i == 1:
                tmp_angle = 0
                tmp_speed = 1
            if i == 2:
                tmp_angle = 0
                tmp_speed = 1
            if i == 3:
                tmp_angle = 0
                tmp_speed = 1
            if i == 4:
                tmp_angle = 0
                tmp_speed = 1
            if i == 5:
                tmp_angle = 0
                tmp_speed = 1

            witeBuffer[i * 9 + 3] = tmp_adress
            witeBuffer[i * 9 + 4] = 0x03  # 01功能码
            witeBuffer[i * 9 + 5] = 0x00
            witeBuffer[i * 9 + 6] = (tmp_speed >> 8) & 0xFF
            witeBuffer[i * 9 + 7] = tmp_speed & 0xFF
            if tmp_angle < 0:
                tmp_angle = (1 << 32) + tmp_angle
            witeBuffer[i * 9 + 8] = (tmp_angle >> 24) & 0xFF
            witeBuffer[i * 9 + 9] = (tmp_angle >> 16) & 0xFF
            witeBuffer[i * 9 + 10] = (tmp_angle >> 8) & 0xFF
            witeBuffer[i * 9 + 11] = tmp_angle & 0xFF
        self.motor_enable_state = False
        
        if self.writer is None:
            self.open_deviceflag = False
            return

        ec = self.writer.write(witeBuffer, timeout=1000)
        self.read_angle_flag = True

    def USBWrite_motor_disable(self): # 电机失能
        witeBuffer = bytearray(57)
        witeBuffer[0] = 0x04  # 说明请求电机的用途(02写入电机位置速度信息,外部功能码相对USB)
        witeBuffer[1] = 0  # 给轨迹规划功能码使用（轨迹插值点数）
        witeBuffer[2] = 0  # 给轨迹规划功能码使用（轨迹插值当前点数用来校对数据丢失）
        tmp_angle = 0
        tmp_speed = 0
        tmp_adress = 0
        self.read_angle_flag = False

        for i in range(6):
            if i == 0:
                tmp_angle = 0
                tmp_speed = 0
            if i == 1:
                tmp_angle = 0
                tmp_speed = 0
            if i == 2:
                tmp_angle = 0
                tmp_speed = 0
            if i == 3:
                tmp_angle = 0
                tmp_speed = 0
            if i == 4:
                tmp_angle = 0
                tmp_speed = 0
            if i == 5:
                tmp_angle = 0
                tmp_speed = 0

            witeBuffer[i * 9 + 3] = tmp_adress
            witeBuffer[i * 9 + 4] = 0x03  # 01功能码
            witeBuffer[i * 9 + 5] = 0x00
            witeBuffer[i * 9 + 6] = (tmp_speed >> 8) & 0xFF
            witeBuffer[i * 9 + 7] = tmp_speed & 0xFF
            if tmp_angle < 0:
                tmp_angle = (1 << 32) + tmp_angle
            witeBuffer[i * 9 + 8] = (tmp_angle >> 24) & 0xFF
            witeBuffer[i * 9 + 9] = (tmp_angle >> 16) & 0xFF
            witeBuffer[i * 9 + 10] = (tmp_angle >> 8) & 0xFF
            witeBuffer[i * 9 + 11] = tmp_angle & 0xFF
        self.motor_disable_state = False
        
        if self.writer is None:
            self.open_deviceflag = False
            return

        ec = self.writer.write(witeBuffer, timeout=1000)
        self.read_angle_flag = True

    def out_traj_button(self, xyz_abg):
        self.c_angle.out_edit_xyz_abg[0] = self.c_angle.px_out
        self.c_angle.out_edit_xyz_abg[1] = self.c_angle.py_out
        self.c_angle.out_edit_xyz_abg[2] = self.c_angle.pz_out
        self.c_angle.out_edit_xyz_abg[3] = self.c_angle.alpha_out
        self.c_angle.out_edit_xyz_abg[4] = self.c_angle.beta_out
        self.c_angle.out_edit_xyz_abg[5] = self.c_angle.gama_out

        self.c_angle.IK_PUBLIC_F()

        self.traj_out.ti_tf[0][0] = 0.0
        self.traj_out.ti_tf[0][1] = 1.0
        self.traj_out.ti_tf[1][0] = 0.0
        self.traj_out.ti_tf[1][1] = 1.0
        self.traj_out.ti_tf[2][0] = 0.0
        self.traj_out.ti_tf[2][1] = 1.0
        self.traj_out.ti_tf[3][0] = 0.0
        self.traj_out.ti_tf[3][1] = 1.0
        self.traj_out.ti_tf[4][0] = 0.0
        self.traj_out.ti_tf[4][1] = 1.0
        self.traj_out.ti_tf[5][0] = 0.0
        self.traj_out.ti_tf[5][1] = 1.0
        current_0 = self.c_angle.out_thtaValue[0][0] #传入弧度每秒为单位；
        self.traj_out.thetai_thetaf[0][0] = current_0
        self.traj_out.thetai_thetaf[0][1] = self.traj_out.thetai_thetaf[0][0] + current_0
        old_0 = self.traj_out.thetai_thetaf[0][1]
        self.traj_out.thetai_thetaf[0][2] = 0.0
        self.traj_out.thetai_thetaf[0][3] = 0.0
        self.traj_out.thetai_thetaf[0][4] = 0.0
        current_1 = self.c_angle.out_thtaValue[1][0]
        self.traj_out.thetai_thetaf[1][0] = current_1
        self.traj_out.thetai_thetaf[1][1] = self.traj_out.thetai_thetaf[1][0] + current_1
        old_1 = self.traj_out.thetai_thetaf[1][1]
        self.traj_out.thetai_thetaf[1][2] = 0.0
        self.traj_out.thetai_thetaf[1][3] = 0.0
        self.traj_out.thetai_thetaf[1][4] = 0.0
        current_2 = self.c_angle.out_thtaValue[2][0]
        self.traj_out.thetai_thetaf[2][0] = current_2
        self.traj_out.thetai_thetaf[2][1] = self.traj_out.thetai_thetaf[2][0] + current_2
        old_2 = self.traj_out.thetai_thetaf[2][1]
        self.traj_out.thetai_thetaf[2][2] = 0.0
        self.traj_out.thetai_thetaf[2][3] = 0.0
        self.traj_out.thetai_thetaf[2][4] = 0.0
        current_3 = self.c_angle.out_thtaValue[3][0]
        self.traj_out.thetai_thetaf[3][0] = current_3
        self.traj_out.thetai_thetaf[3][1] = self.traj_out.thetai_thetaf[3][0] + current_3
        old_3 = self.traj_out.thetai_thetaf[3][1]
        self.traj_out.thetai_thetaf[3][2] = 0.0
        self.traj_out.thetai_thetaf[3][3] = 0.0
        self.traj_out.thetai_thetaf[3][4] = 0.0
        current_4 = self.c_angle.out_thtaValue[4][0]
        self.traj_out.thetai_thetaf[4][0] = current_4
        self.traj_out.thetai_thetaf[4][1] = self.traj_out.thetai_thetaf[4][0] + current_4
        old_4 = self.traj_out.thetai_thetaf[4][1]
        self.traj_out.thetai_thetaf[4][2] = 0.0
        self.traj_out.thetai_thetaf[4][3] = 0.0
        self.traj_out.thetai_thetaf[4][4] = 0.0
        current_5 = self.c_angle.out_thtaValue[5][0]
        self.traj_out.thetai_thetaf[5][0] = current_5
        self.traj_out.thetai_thetaf[5][1] = self.traj_out.thetai_thetaf[5][0] + current_5
        old_5 = self.traj_out.thetai_thetaf[5][1]
        self.traj_out.thetai_thetaf[5][2] = 0.0
        self.traj_out.thetai_thetaf[5][3] = 0.0
        self.traj_out.thetai_thetaf[5][4] = 0.0
        
        self.c_angle.out_edit_xyz_abg[0] = xyz_abg[0]
        self.c_angle.out_edit_xyz_abg[1] = xyz_abg[1]
        self.c_angle.out_edit_xyz_abg[2] = xyz_abg[2]
        self.c_angle.out_edit_xyz_abg[3] = xyz_abg[3]
        self.c_angle.out_edit_xyz_abg[4] = xyz_abg[4]
        self.c_angle.out_edit_xyz_abg[5] = xyz_abg[5]

        self.c_angle.IK_PUBLIC_F()
        self.traj_out.thetai_thetaf[0][1] = self.c_angle.out_thtaValue[0][0]
        self.traj_out.thetai_thetaf[1][1] = self.c_angle.out_thtaValue[1][0]
        self.traj_out.thetai_thetaf[2][1] = self.c_angle.out_thtaValue[2][0]
        self.traj_out.thetai_thetaf[3][1] = self.c_angle.out_thtaValue[3][0]
        self.traj_out.thetai_thetaf[4][1] = self.c_angle.out_thtaValue[4][0]
        self.traj_out.thetai_thetaf[5][1] = self.c_angle.out_thtaValue[5][0]

        self.traj_out.vi_vf[0][0] = 0.0
        self.traj_out.vi_vf[0][1] = 0.0
        self.traj_out.vi_vf[1][0] = 0.0
        self.traj_out.vi_vf[1][1] = 0.0
        self.traj_out.vi_vf[2][0] = 0.0
        self.traj_out.vi_vf[2][1] = 0.0
        self.traj_out.vi_vf[3][0] = 0.0
        self.traj_out.vi_vf[3][1] = 0.0 / 5.18
        self.traj_out.vi_vf[4][0] = 0.0
        self.traj_out.vi_vf[4][1] = 0.0 / 5.18
        self.traj_out.vi_vf[5][0] = 0.0
        self.traj_out.vi_vf[5][1] = 0.0 / 5.18
        for point_index in range(2):
            for p in range(6):
                self.traj_out.traj_plan(2, p, 0.004)

        self.c_angle.out_traj_flag_1 = True
        self.c_angle.out_traj_flag_2 = True
        self.c_angle.out_traj_flag_3 = True
        self.c_angle.out_traj_flag_4 = True
        self.c_angle.out_traj_flag_5 = True
        self.c_angle.out_traj_flag_6 = True


    def sign_extend(self, value, bits):
        sign_bit = 1 << (bits - 1)
        return (value & (sign_bit - 1)) - (value & sign_bit)

    def Update(self):
        if self.open_deviceflag and self.read_angle_flag:
            self.USBWrite_Request_to_positiondata()
            self.USBRead_data()
        else:
            print("Failed to open USB device.")

        if self.open_deviceflag and self.write_angle_flag:
            self.USBWrite_send_angle_pos()

        if self.open_deviceflag and self.write_res_angle_flag:
            self.USBWrite_send_angle_pos()

        if self.open_deviceflag and self.write_zero_angle_flag:
            self.USBWrite_send_angle_pos()

        if self.open_deviceflag and self.write_traj_flag:
            self.USBWrite_send_traj_pos()

        if self.open_deviceflag and self.motor_enable_state:
            self.USBWrite_motor_enable()

        if self.open_deviceflag and self.motor_disable_state:
            self.USBWrite_motor_disable()
        
        # 处理读取的数据
        angle_tmp = self.sign_extend((self.read_Buffer[8] << 24) + (self.read_Buffer[9] << 16) + (self.read_Buffer[10] << 8) + self.read_Buffer[11], 32)
        self._1_link_angle = angle_tmp
        
        angle_tmp = self.sign_extend((self.read_Buffer[17] << 24) + (self.read_Buffer[18] << 16) + (self.read_Buffer[19] << 8) + self.read_Buffer[20], 32)
        self._2_link_angle = angle_tmp
        
        angle_tmp = self.sign_extend((self.read_Buffer[26] << 24) + (self.read_Buffer[27] << 16) + (self.read_Buffer[28] << 8) + self.read_Buffer[29], 32)
        self._3_link_angle = angle_tmp
        
        angle_tmp = self.sign_extend((self.read_Buffer[35] << 24) + (self.read_Buffer[36] << 16) + (self.read_Buffer[37] << 8) + self.read_Buffer[38], 32)
        self._4_link_angle = angle_tmp
        
        angle_tmp = self.sign_extend((self.read_Buffer[44] << 24) + (self.read_Buffer[45] << 16) + (self.read_Buffer[46] << 8) + self.read_Buffer[47], 32)
        self._5_link_angle = angle_tmp
        
        angle_tmp = self.sign_extend((self.read_Buffer[53] << 24) + (self.read_Buffer[54] << 16) + (self.read_Buffer[55] << 8) + self.read_Buffer[56], 32)
        self._6_link_angle = angle_tmp

        if abs(self.c_angle.px_out) < 0.0001: self.c_angle.px_out = 0 
        if abs(self.c_angle.py_out) < 0.0001: self.c_angle.py_out = 0
        if abs(self.c_angle.pz_out) < 0.0001: self.c_angle.pz_out = 0
        if abs(self.c_angle.alpha_out) < 0.0001: self.c_angle.alpha_out = 0
        if abs(self.c_angle.beta_out) < 0.0001: self.c_angle.beta_out = 0
        if abs(self.c_angle.gama_out) < 0.0001: self.c_angle.gama_out = 0

        angle_tmp = self._1_link_angle - self.zero_1
        if abs(angle_tmp) < 5:
            angle_tmp = 0
        self.angle_radian[0][0] = angle_tmp / (51200.0 / 360.0) / 27.0 / 5.0 * math.pi / 180
        angle_tmp = - (self._2_link_angle - self.zero_2)
        if abs(angle_tmp) < 5:
            angle_tmp = 0
        self.angle_radian[1][0] = angle_tmp / (51200.0 / 360.0) / 27.0 / 5.0 * math.pi / 180
        angle_tmp = self._3_link_angle - self.zero_3
        if abs(angle_tmp) < 5:
            angle_tmp = 0
        self.angle_radian[2][0] = angle_tmp / (51200.0 / 360.0) / 14.0 / 5.0 * math.pi / 180
        angle_tmp = self._4_link_angle - self.zero_4
        if abs(angle_tmp) < 5:
            angle_tmp = 0
        self.angle_radian[3][0] = angle_tmp / (51200.0 / 360.0) / 5.18 / 3.0 * math.pi / 180
        angle_tmp = self._5_link_angle - self.zero_5
        if abs(angle_tmp) < 5:
            angle_tmp = 0
        self.angle_radian[4][0] = angle_tmp / (51200.0 / 360.0)  / 1.6 * math.pi / 180
        angle_tmp = self._6_link_angle - self.zero_6
        if abs(angle_tmp) < 5:
            angle_tmp = 0
        self.angle_radian[5][0] = angle_tmp / (51200.0 / 360.0) * math.pi / 180

        self.c_angle.FK_CLC(0, self.angle_radian)

        return

    def send_command(self, command_data):
        """
        处理并发送命令数据
        :param command_data: 包含命令的列表
        """
        #print(f"发送命令: {command_data}")
        # 根据 command_data 设置对应电机的目标角度
        motor_id = command_data[0]  # 电机编号
        target_angle = command_data[2]  # 目标角度

        # 根据电机编号设置目标角度
        if motor_id == 1:
            self._1_edit_angle = target_angle
        elif motor_id == 2:
            self._2_edit_angle = target_angle
        elif motor_id == 3:
            self._3_edit_angle = target_angle
        elif motor_id == 4:
            self._4_edit_angle = target_angle
        elif motor_id == 5:
            self._5_edit_angle = target_angle
        elif motor_id == 6:
            self._6_edit_angle = target_angle
        else:
            print(f"未知的电机编号: {motor_id}")
            return

        self.write_angle_flag = True  # 设置标志位，触发 Update 方法中的写入逻辑
        self.Update()  # 调用 Update 方法以发送命令

# 控制舵机
def claw_control(claw_state, COM):
    # 设置机械爪串口连接参数
    ser = serial.Serial(COM, 9600)  # 'COM6'应替换为实际的ESP端口号
    angle = 80  # 机械爪的开合角度
    
    while True:
        if claw_state.value==1.0 and 0 <= angle <= 90:
            ser.write(f"{angle}\n".encode())  # 发送角度数据到ESP
            time.sleep(0.1)
        elif claw_state.value==-1.0:
            ser.close()
            break
        else:
            time.sleep(0.1)
