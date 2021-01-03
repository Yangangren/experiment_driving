#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =====================================
# @Time    : 2020/12/28
# @Author  : Yangang Ren (Tsinghua Univ.)
# @FileName: traffic.py
# =====================================

import multiprocessing as mp
import time
from collections import OrderedDict
from math import cos, sin, pi

import matplotlib.pyplot as plt
import numpy as np

from utils.coordi_convert import rotate_coordination

CROSSROAD_SIZE = 22
LANE_WIDTH = 3.5
START_OFFSET = 3
LANE_NUMBER = 1
EGO_LENGTH = 4.8
EGO_WIDTH = 2.0

TRAFFICSETTINGS = dict(left=[dict(ego=dict(v_x=2., v_y=0., r=0., x=3.5/2, y=-29, phi=90.,),
                                  others=OrderedDict(dl=dict(x=3.5/2, y=-11, phi=90, l=4.8, w=2.0, v=0., route=('1o', '4i')),
                                                     ud=dict(x=-3.5/2, y=12, phi=-90, l=4.8, w=2.0, v=3., route=('3o', '1i')),
                                                     ul=dict(x=-3.5/2, y=45, phi=-90, l=4.8, w=2.0, v=3., route=('3o', '4i')),),
                                  v_light=0,
                                  ),
                             dict(ego=dict(v_x=2., v_y=0., r=0., x=3.5/2, y=-29, phi=90.,),
                                  others=OrderedDict(dl=dict(x=3.5/2, y=-14, phi=90, l=4.8, w=2.0, v=0., route=('1o', '4i')),
                                                     ud=dict(x=-3.5/2, y=11, phi=-90, l=4.8, w=2.0, v=3., route=('3o', '1i')),
                                                     ul=dict(x=-3.5/2, y=40, phi=-90, l=4.8, w=2.0, v=5., route=('3o', '4i')),),
                                  v_light=0,
                                  ),
                             ],
                       straight=[dict(ego=dict(v_x=2., v_y=0., r=0., x=3.5/2, y=-29, phi=90.),
                                      others=OrderedDict(du=dict(x=3.5/2, y=-11, phi=90, l=4.8, w=2.0, v=3., route=('1o', '3i')),
                                                         ur=dict(x=-3.5/2, y=11, phi=-90, l=4.8, w=2.0, v=0., route=('1o', '3i')),
                                                         ru=dict(x=45, y=3.5/2, phi=180, l=4.8, w=2.0, v=3., route=('1o', '3i')),
                                                         ),
                                      v_light=0
                                      ),
                                 dict()],

                       right=[dict(ego=dict(v_x=2., v_y=0., r=0., x=3.5/2, y=-29, phi=90.),
                                      others=OrderedDict(dr=dict(x=3.5/2, y=-11, phi=90, l=4.8, w=2.0, v=3., route=('1o', '2i')),
                                                         ur=dict(x=-3.5/2, y=11, phi=-90, l=4.8, w=2.0, v=3., route=('1o', '2i')),
                                                         ),
                                      v_light=0
                                      ),
                              dict()
                              ]
                       )


class Traffic(object):
    def __init__(self, shared_list, lock, task='left', case=0, surr_flag=True):
        self.shared_list = shared_list
        self.abso_time_start = None
        self.abso_time_in_this_step = None
        self.is_triggered = False
        self.lock = lock
        self.task = task
        self.case = case
        self.case_dict = TRAFFICSETTINGS[task][case].copy()
        self.others = self.case_dict['others'].copy()
        self.base_frequency = 10.
        self.time_since_triggered = 0.
        self.surr_flag = surr_flag

    def prediction(self, mode, veh, acc, delta_time, max_v=5.):
        x, y, v, phi = veh['x'], veh['y'], veh['v'], veh['phi']
        phi_rad = phi * np.pi / 180.
        x_delta = v * delta_time * cos(phi_rad)
        y_delta = v * delta_time * sin(phi_rad)
        v_delta = acc * delta_time
        middle_cond = (-CROSSROAD_SIZE / 2 < x < CROSSROAD_SIZE / 2) and (-CROSSROAD_SIZE / 2 < y < CROSSROAD_SIZE / 2)
        phi_rad_delta = 0.
        if middle_cond:
            if mode in ['dl', 'rd', 'ur', 'lu']:
                phi_rad_delta = (v / (CROSSROAD_SIZE / 2 + 0.5 * LANE_WIDTH)) * delta_time
                # phi_rad_delta += 0.001
            elif mode in ['dr', 'ru', 'ul', 'ld']:
                phi_rad_delta = -(v / (CROSSROAD_SIZE / 2 - 0.5 * LANE_WIDTH)) * delta_time

        next_x, next_y, next_v, next_phi_rad = \
            x + x_delta, y + y_delta, v + v_delta, phi_rad + phi_rad_delta

        if mode.endswith('l'):
            after_cond = (x < -CROSSROAD_SIZE/2)
        elif mode.endswith('u'):
            after_cond = (y > CROSSROAD_SIZE/2)
        elif mode.endswith('r'):
            after_cond = (x > CROSSROAD_SIZE/2)
        else:
            after_cond = (y < -CROSSROAD_SIZE/2)
        if after_cond:
            for tar_phi in [0., np.pi/2., np.pi, -np.pi/2., -np.pi]:
                if np.isclose(next_phi_rad, tar_phi, rtol=0.1, atol=0.1):
                    next_phi_rad = tar_phi

        next_phi_rad = next_phi_rad - 2*np.pi if next_phi_rad > np.pi else next_phi_rad
        next_phi_rad = next_phi_rad + 2*np.pi if next_phi_rad < -np.pi else next_phi_rad
        next_phi = next_phi_rad * 180 / np.pi
        next_v = np.clip(next_v, 0., max_v)
        return next_x, next_y, next_v, next_phi

    def is_triggered_func(self, ego_y):
        if ego_y > -14 - 15:
            self.is_triggered = True
            self.abso_time_start = time.time()
            self.abso_time_in_this_step = time.time()

    def step(self, delta_time):
        _, ego_y = self.shared_list[0]['GaussX'], self.shared_list[0]['GaussY']
        if not self.is_triggered:
            self.is_triggered_func(ego_y)
        self.time_since_triggered = time.time() - self.abso_time_start if self.is_triggered else 0.
        state_other = {'x_other': [], 'y_other': [], 'v_other': [], 'phi_other': []}
        state_other['v_light'] = self.case_dict['v_light']
        if self.surr_flag:
            if self.task == 'left':
                veh_dl, veh_ud, veh_ul = self.others['dl'], self.others['ud'], self.others['ul']
                if self.is_triggered:
                    if self.case == 0:
                        # veh_dl
                        if self.time_since_triggered < 8.:
                            dl_next_x, dl_next_y, dl_next_v, dl_next_phi = self.prediction('dl', veh_dl, 0., delta_time)
                        else:
                            dl_next_x, dl_next_y, dl_next_v, dl_next_phi = self.prediction('dl', veh_dl, 2.0, delta_time)
                        # print(dl_next_y)
                        # veh_ud
                        ud_next_x, ud_next_y, ud_next_v, ud_next_phi = self.prediction('ud', veh_ud, 2., delta_time)
                        # veh_ul
                        ul_next_x, ul_next_y, ul_next_v, ul_next_phi = self.prediction('ul', veh_ul, 0., delta_time)
                    elif self.case == 1:
                        # veh_dl
                        dl_next_x, dl_next_y, dl_next_v, dl_next_phi = self.prediction('dl', veh_dl, 2.5, delta_time)
                        # veh_ud
                        ud_next_x, ud_next_y, ud_next_v, ud_next_phi = self.prediction('ud', veh_ud, -0.5, delta_time)
                        # veh_ul
                        ul_next_x, ul_next_y, ul_next_v, ul_next_phi = self.prediction('ul', veh_ul, 0., delta_time)
                    elif self.case == 2:
                        pass

                else:
                    veh_dl, veh_ud, veh_ul = self.others['dl'], self.others['ud'], self.others['ul']
                    dl_next_x, dl_next_y, dl_next_v, dl_next_phi = veh_dl['x'], veh_dl['y'], veh_dl['v'], veh_dl['phi']
                    ud_next_x, ud_next_y, ud_next_v, ud_next_phi = veh_ud['x'], veh_ud['y'], veh_ud['v'], veh_ud['phi']
                    ul_next_x, ul_next_y, ul_next_v, ul_next_phi = veh_ul['x'], veh_ul['y'], veh_ul['v'], veh_ul['phi']

                state_other['x_other'].extend([dl_next_x, ud_next_x, ul_next_x])
                state_other['y_other'].extend([dl_next_y, ud_next_y, ul_next_y])
                state_other['v_other'].extend([dl_next_v, ud_next_v, ul_next_v])
                state_other['phi_other'].extend([dl_next_phi, ud_next_phi, ul_next_phi])
                self.others['dl'].update(x=dl_next_x, y=dl_next_y, v=dl_next_v, phi=dl_next_phi)
                self.others['ud'].update(x=ud_next_x, y=ud_next_y, v=ud_next_v, phi=ud_next_phi)
                self.others['ul'].update(x=ul_next_x, y=ul_next_y, v=ul_next_v, phi=ul_next_phi)

            elif self.task == 'straight':
                veh_du, veh_ur, veh_ru = self.others['du'], self.others['ur'], self.others['ru']
                if self.is_triggered:
                    if self.case == 0:
                        # veh_ur
                        if self.time_since_triggered < 6.5:
                            ur_next_x, ur_next_y, ur_next_v, ur_next_phi = self.prediction('ur', veh_ur, 0., delta_time)
                        else:
                            ur_next_x, ur_next_y, ur_next_v, ur_next_phi = self.prediction('ur', veh_ur, 2.0, delta_time)
                        # veh_du
                        du_next_x, du_next_y, du_next_v, du_next_phi = self.prediction('ud', veh_du, 2., delta_time)
                        # veh_ru
                        ru_next_x, ru_next_y, ru_next_v, ru_next_phi = self.prediction('ul', veh_ru, 0., delta_time)
                    elif self.case == 1:
                        # veh_ur
                        ur_next_x, ur_next_y, ur_next_v, ur_next_phi = self.prediction('ur', veh_ur, 2.5, delta_time)
                        # veh_du
                        du_next_x, du_next_y, du_next_v, du_next_phi = self.prediction('du', veh_du, -0.5, delta_time)
                        # veh_ru
                        ru_next_x, ru_next_y, ru_next_v, ru_next_phi = self.prediction('ru', veh_ru, 0., delta_time)
                    else:
                        assert self.case == 2
                        pass

                else:
                    veh_du, veh_ur, veh_ru = self.others['du'], self.others['ur'], self.others['ru']
                    ur_next_x, ur_next_y, ur_next_v, ur_next_phi = veh_ur['x'], veh_ur['y'], veh_ur['v'], veh_ur['phi']
                    du_next_x, du_next_y, du_next_v, du_next_phi = veh_du['x'], veh_du['y'], veh_du['v'], veh_du['phi']
                    ru_next_x, ru_next_y, ru_next_v, ru_next_phi = veh_ru['x'], veh_ru['y'], veh_ru['v'], veh_ru['phi']

                state_other['x_other'].extend([ur_next_x, du_next_x, ru_next_x])
                state_other['y_other'].extend([ur_next_y, du_next_y, ru_next_y])
                state_other['v_other'].extend([ur_next_v, du_next_v, ru_next_v])
                state_other['phi_other'].extend([ur_next_phi, du_next_phi, ru_next_phi])
                self.others['ur'].update(x=ur_next_x, y=ur_next_y, v=ur_next_v, phi=ur_next_phi)
                self.others['du'].update(x=du_next_x, y=du_next_y, v=du_next_v, phi=du_next_phi)
                self.others['ru'].update(x=ru_next_x, y=ru_next_y, v=ru_next_v, phi=ru_next_phi)
            elif self.task == 'right':
                veh_dr, veh_ur = self.others['dr'], self.others['ur']
                if self.is_triggered:
                    if self.case == 0:
                        # veh_ur
                        ur_next_x, ur_next_y, ur_next_v, ur_next_phi = self.prediction('ur', veh_ur, 0, delta_time)
                        # veh_dr
                        dr_next_x, dr_next_y, dr_next_v, dr_next_phi = self.prediction('dr', veh_dr, 0.2, delta_time)

                    elif self.case == 1:
                        # veh_ur
                        ur_next_x, ur_next_y, ur_next_v, ur_next_phi = self.prediction('ur', veh_ur, 2.5, delta_time)
                        # veh_dr
                        dr_next_x, dr_next_y, dr_next_v, dr_next_phi = self.prediction('dr', veh_dr, -0.5, delta_time)
                    else:
                        assert self.case == 2
                        pass

                else:
                    veh_dr, veh_ur = self.others['dr'], self.others['ur']
                    ur_next_x, ur_next_y, ur_next_v, ur_next_phi = veh_ur['x'], veh_ur['y'], veh_ur['v'], veh_ur['phi']
                    dr_next_x, dr_next_y, dr_next_v, dr_next_phi = veh_dr['x'], veh_dr['y'], veh_dr['v'], veh_dr['phi']

                state_other['x_other'].extend([ur_next_x, dr_next_x])
                state_other['y_other'].extend([ur_next_y, dr_next_y])
                state_other['v_other'].extend([ur_next_v, dr_next_v])
                state_other['phi_other'].extend([ur_next_phi, dr_next_phi])
                self.others['ur'].update(x=ur_next_x, y=ur_next_y, v=ur_next_v, phi=ur_next_phi)
                self.others['dr'].update(x=dr_next_x, y=dr_next_y, v=dr_next_v, phi=dr_next_phi)
        return state_other

    def run(self):
        while True:
            time.sleep(0.05)
            delta_time_in_this_step = time.time() - self.abso_time_in_this_step if self.is_triggered else 0.
            # print(delta_time_in_this_step)
            state_other = self.step(delta_time_in_this_step)
            self.render()
            self.abso_time_in_this_step = time.time()
            time_receive_radar = self.abso_time_in_this_step

            with self.lock:
                self.shared_list[4] = state_other.copy()
                self.shared_list[5] = time_receive_radar

            # if time_receive_radar > 0.1:
            #     print("Subscriber of radar is more than 0.1s!", time_receive_radar)

    def render(self):
        square_length = CROSSROAD_SIZE
        start_offset = START_OFFSET
        extension = 40
        lane_width = LANE_WIDTH
        light_line_width = 3
        dotted_line_style = '--'
        solid_line_style = '-'

        plt.cla()
        plt.title("Crossroad")
        ax = plt.axes(xlim=(-square_length / 2 - extension, square_length / 2 + extension),
                      ylim=(-square_length / 2 - extension, square_length / 2 + extension))
        plt.axis("equal")
        plt.axis('off')

        # ax.add_patch(plt.Rectangle((-square_length / 2, -square_length / 2),
        #                            square_length, square_length, edgecolor='black', facecolor='none'))
        ax.add_patch(plt.Rectangle((-square_length / 2 - extension, -square_length / 2 - extension - start_offset),
                                   square_length + 2 * extension, square_length + 2 * extension + start_offset,
                                   edgecolor='black',
                                   facecolor='none'))

        # ----------arrow--------------
        plt.arrow(lane_width / 2, -square_length / 2 - 10, 0, 5, color='b')
        plt.arrow(lane_width / 2, -square_length / 2 - 10 + 5, -0.5, 0, color='b', head_width=1)
        plt.arrow(lane_width / 2, -square_length / 2 - 10, 0, 5, color='b', head_width=1)
        plt.arrow(lane_width / 2, -square_length / 2 - 10, 0, 5, color='b')
        plt.arrow(lane_width / 2, -square_length / 2 - 10 + 5, 0.5, 0, color='b', head_width=1)

        # ----------horizon--------------
        plt.plot([-square_length / 2 - extension, -square_length / 2], [0, 0], color='black')
        plt.plot([square_length / 2 + extension, square_length / 2], [0, 0], color='black')

        #
        for i in range(1, LANE_NUMBER + 1):
            linestyle = dotted_line_style if i < LANE_NUMBER else solid_line_style
            plt.plot([-square_length / 2 - extension, -square_length / 2], [i * lane_width, i * lane_width],
                     linestyle=linestyle, color='black')
            plt.plot([square_length / 2 + extension, square_length / 2], [i * lane_width, i * lane_width],
                     linestyle=linestyle, color='black')
            plt.plot([-square_length / 2 - extension, -square_length / 2], [-i * lane_width, -i * lane_width],
                     linestyle=linestyle, color='black')
            plt.plot([square_length / 2 + extension, square_length / 2], [-i * lane_width, -i * lane_width],
                     linestyle=linestyle, color='black')

        # ----------vertical----------------
        plt.plot([0, 0], [-square_length / 2 - extension - start_offset, -square_length / 2 - start_offset],
                 color='black')
        plt.plot([0, 0], [square_length / 2 + extension, square_length / 2], color='black')

        #
        for i in range(1, LANE_NUMBER + 1):
            linestyle = dotted_line_style if i < LANE_NUMBER else solid_line_style
            plt.plot([i * lane_width, i * lane_width],
                     [-square_length / 2 - extension - start_offset, -square_length / 2 - start_offset],
                     linestyle=linestyle, color='black')
            plt.plot([i * lane_width, i * lane_width], [square_length / 2 + extension, square_length / 2],
                     linestyle=linestyle, color='black')
            plt.plot([-i * lane_width, -i * lane_width],
                     [-square_length / 2 - extension - start_offset, -square_length / 2 - start_offset],
                     linestyle=linestyle, color='black')
            plt.plot([-i * lane_width, -i * lane_width], [square_length / 2 + extension, square_length / 2],
                     linestyle=linestyle, color='black')

        # ----------Oblique--------------
        plt.plot([LANE_NUMBER * lane_width, square_length / 2],
                 [-square_length / 2 - start_offset, -LANE_NUMBER * lane_width],
                 color='black')
        plt.plot([LANE_NUMBER * lane_width, square_length / 2], [square_length / 2, LANE_NUMBER * lane_width],
                 color='black')
        plt.plot([-LANE_NUMBER * lane_width, -square_length / 2],
                 [-square_length / 2 - start_offset, -LANE_NUMBER * lane_width],
                 color='black')
        plt.plot([-LANE_NUMBER * lane_width, -square_length / 2], [square_length / 2, LANE_NUMBER * lane_width],
                 color='black')

        def is_in_plot_area(x, y, tolerance=5):
            if -square_length / 2 - extension + tolerance < x < square_length / 2 + extension - tolerance and \
                    -square_length / 2 - extension + tolerance < y < square_length / 2 + extension - tolerance:
                return True
            else:
                return False

        def draw_rotate_rec(x, y, a, l, w, color, linestyle='-'):
            RU_x, RU_y, _ = rotate_coordination(l / 2, w / 2, 0, -a)
            RD_x, RD_y, _ = rotate_coordination(l / 2, -w / 2, 0, -a)
            LU_x, LU_y, _ = rotate_coordination(-l / 2, w / 2, 0, -a)
            LD_x, LD_y, _ = rotate_coordination(-l / 2, -w / 2, 0, -a)
            ax.plot([RU_x + x, RD_x + x], [RU_y + y, RD_y + y], color=color, linestyle=linestyle)
            ax.plot([RU_x + x, LU_x + x], [RU_y + y, LU_y + y], color=color, linestyle=linestyle)
            ax.plot([LD_x + x, RD_x + x], [LD_y + y, RD_y + y], color=color, linestyle=linestyle)
            ax.plot([LD_x + x, LU_x + x], [LD_y + y, LU_y + y], color=color, linestyle=linestyle)

        def plot_phi_line(x, y, phi, color):
            line_length = 5
            x_forw, y_forw = x + line_length * cos(phi * pi / 180.), \
                             y + line_length * sin(phi * pi / 180.)
            plt.plot([x, x_forw], [y, y_forw], color=color, linewidth=0.5)

        # plot surrounding vehicles
        state_others = self.others.copy()
        for key, val in state_others.items():
            veh_x = val['x']
            veh_y = val['y']
            # if key == 'dl':
            #     print('traffic',veh_y)
            veh_phi = val['phi']
            veh_l = val['l']
            veh_w = val['w']

            if is_in_plot_area(veh_x, veh_y):
                plot_phi_line(veh_x, veh_y, veh_phi, 'black')
                draw_rotate_rec(veh_x, veh_y, veh_phi, veh_l, veh_w, 'black')

        # plot ego vehicles
        ego_veh = self.case_dict['ego']
        veh_x = ego_veh['x']
        veh_y = ego_veh['y']
        veh_phi = ego_veh['phi']
        plot_phi_line(veh_x, veh_y, veh_phi, 'red')
        draw_rotate_rec(veh_x, veh_y, veh_phi, EGO_LENGTH, EGO_WIDTH, 'red')

        # plt.show()
        plt.pause(0.01)


if __name__ == '__main__':
    ego_list = [dict(GaussX=3.5 / 2, GaussY=-19.5), 0, 0, 0, 0, 0]
    traffic = Traffic(shared_list=ego_list, lock=mp.Lock(), task='left', case=0)
    traffic.run()
