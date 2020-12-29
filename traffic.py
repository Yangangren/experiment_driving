#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# =====================================
# @Time    : 2020/12/28
# @Author  : Yangang Ren (Tsinghua Univ.)
# @FileName: traffic.py
# =====================================
from collections import OrderedDict
import time
import matplotlib.pyplot as plt
import numpy as np
import math
from math import cos, sin, pi

CROSSROAD_SIZE = 22
LANE_WIDTH = 3.5
START_OFFSET = 3
LANE_NUMBER = 1


def rotate_coordination(orig_x, orig_y, orig_d, coordi_rotate_d):
    """
    :param orig_x: original x
    :param orig_y: original y
    :param orig_d: original degree
    :param coordi_rotate_d: coordination rotation d, positive if anti-clockwise, unit: deg
    :return:
    transformed_x, transformed_y, transformed_d(range:(-180 deg, 180 deg])
    """
    coordi_rotate_d_in_rad = coordi_rotate_d * math.pi / 180
    transformed_x = orig_x * math.cos(coordi_rotate_d_in_rad) + orig_y * math.sin(coordi_rotate_d_in_rad)
    transformed_y = -orig_x * math.sin(coordi_rotate_d_in_rad) + orig_y * math.cos(coordi_rotate_d_in_rad)
    transformed_d = orig_d - coordi_rotate_d
    if transformed_d > 180:
        while transformed_d > 180:
            transformed_d = transformed_d - 360
    elif transformed_d <= -180:
        while transformed_d <= -180:
            transformed_d = transformed_d + 360
    else:
        transformed_d = transformed_d
    return transformed_x, transformed_y, transformed_d


TRAFFICSETTINGS = dict(left=[OrderedDict(ego=dict(v_x=2., v_y=0., r=0., x=3.5/2, y=-5, phi=90.,),
                                         ud=dict(x=-3.5/2, y=11, phi=-90, l=5., w=2., v=3., acc_type='uniform', route=('3o', '1i')),
                                         ul=dict(x=-3.5/2, y=18, phi=-90, l=5., w=2., v=3., acc_type='stop', route=('3o', '4i')),
                                         v_light=11111,
                                         ),
                             OrderedDict(ego=dict(v_x=2., v_y=0., r=0., x=3.5/2, y=-5, phi=90.,),
                                         dl=dict(x=3.5/2, y=0, phi=90, l=5., w=2., v=3., route=('1o', '4i')),
                                         v_light=11111,
                                         ),
                             ],
                       straight=[OrderedDict(), ],
                       right=[OrderedDict(), ]
                       )


class Traffic(object):
    def __init__(self, shared_list, State_Other_List, lock):
        self.shared_list = shared_list
        self.state_other_List = State_Other_List
        self.time_start = 0
        self.lock = lock
        self.task = 'left'
        self.mode_index = 0                                        # 'straight' or 'right' or 'left'
        self.mode = TRAFFICSETTINGS[self.task][self.mode_index]
        self.base_frequency = 10

    def prediction(self, mode):
        state_other = {'x_other': [], 'y_other': [], 'v_other': [], 'phi_other': [], 'v_light': []}
        for vehicle, info in mode.items():
            if vehicle == 'v_light':
                state_other['v_light'].append(info)
            else:
                if vehicle != 'ego':
                    veh_xs, veh_ys, veh_phis_rad, veh_vs = info['x'], info['y'], info['phi'] * np.pi / 180, info['v']
                    veh_type = info['acc_type']

                    middle_cond = np.logical_and(np.logical_and(veh_xs > -CROSSROAD_SIZE / 2, veh_xs < CROSSROAD_SIZE / 2),
                                              np.logical_and(veh_ys > -CROSSROAD_SIZE / 2, veh_ys < CROSSROAD_SIZE / 2))

                    zeros = np.zeros_like(veh_xs)

                    if vehicle in ['dl', 'rd', 'ur', 'lu']:
                        pass
                    elif vehicle in ['dr', 'ru', 'ul', 'ld']:
                        if veh_type == 'stop':
                            acc = -0.3
                            delta_dist = veh_vs / self.base_frequency + 0.5 * acc / (self.base_frequency ** 2)
                            delta_phi_rad = np.where(middle_cond, -(delta_dist / (CROSSROAD_SIZE / 2 - 0.5 * LANE_WIDTH)), zeros)

                    elif vehicle in ['ud', 'du', 'rl', 'lr']:
                        if veh_type == 'uniform':
                            delta_phi_rad = 0.
                            acc = 0.

                    next_veh_vs = veh_vs + acc / self.base_frequency
                    next_veh_xs = veh_xs + (veh_vs / self.base_frequency + 0.5 * acc / (self.base_frequency ** 2)) * np.cos(veh_phis_rad)
                    next_veh_ys = veh_ys + (veh_vs / self.base_frequency + 0.5 * acc / (self.base_frequency ** 2)) * np.sin(veh_phis_rad)
                    next_veh_phis_rad = veh_phis_rad + delta_phi_rad

                    next_veh_phis_rad = np.where(next_veh_phis_rad > np.pi, next_veh_phis_rad - 2 * np.pi,
                                                 next_veh_phis_rad)
                    next_veh_phis_rad = np.where(next_veh_phis_rad <= -np.pi, next_veh_phis_rad + 2 * np.pi,
                                                 next_veh_phis_rad)
                    next_veh_phis = next_veh_phis_rad * 180 / np.pi

                    state_other['x_other'].append(next_veh_xs)
                    state_other['y_other'].append(next_veh_ys)
                    state_other['v_other'].append(next_veh_vs)
                    state_other['phi_other'].append(next_veh_phis)
                    mode[vehicle].update(x=next_veh_xs, y=next_veh_ys, phi=next_veh_phis, v=next_veh_vs)

        return state_other, mode

    def run(self):
        time_receive_radar = 0
        while True:
            time.sleep(0.07)
            state_other, self.mode = self.prediction(self.mode)
            self.render()
            time_receive_radar = time.time() - self.time_start
            self.time_start = time.time()

            with self.lock:
                self.shared_list[4] = time_receive_radar
                self.state_other_list[0] = state_other.copy()

            if time_receive_radar > 0.1:
                print("Subscriber of radar is more than 0.1s!", time_receive_radar)

            print(state_other['x_other'], state_other['y_other'])

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

        state_others = self.mode.copy()
        # plot cars
        for index, veh in state_others.items():
            if index != 'ego' and index != 'v_light':
                veh_x = veh['x']
                veh_y = veh['y']
                veh_phi = veh['phi']
                veh_l = veh['l']
                veh_w = veh['w']

                if is_in_plot_area(veh_x, veh_y):
                    plot_phi_line(veh_x, veh_y, veh_phi, 'black')
                    draw_rotate_rec(veh_x, veh_y, veh_phi, veh_l, veh_w, 'black')
        # plt.show()
        plt.pause(0.1)


if __name__ == '__main__':
    traffic = Traffic(shared_list=[0, 0, 0, 0, 0], State_Other_List=[2, 3], lock=1)
    traffic.run()
