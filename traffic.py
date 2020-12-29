#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# =====================================
# @Time    : 2020/12/28
# @Author  : Yangang Ren (Tsinghua Univ.)
# @FileName: traffic.py
# =====================================
from collections import OrderedDict
import numpy as np
import time

CROSSROAD_SIZE = 22
LANE_WIDTH = 3.5
TRAFFICSETTINGS = dict(left=[OrderedDict(ego=dict(v_x=2., v_y=0., r=0., x=3.5/2, y=-5, phi=90.,),
                                         ud=dict(x=-3.5/2, y=11, phi=-90, l=5., w=2., v=3., acc_type='uniform', route=('3o', '1i')),
                                         ul=dict(x=-3.5/2, y=13, phi=-90, l=5., w=2., v=3., acc_type='stop', route=('3o', '4i'))
                                         ),
                             OrderedDict(ego=dict(v_x=2., v_y=0., r=0., x=3.5/2, y=-5, phi=90.,),
                                         dl=dict(x=3.5/2, y=0, phi=90, l=5., w=2., v=3., route=('1o', '4i'))
                                         ),
                             ],
                       straight=[OrderedDict(), ],
                       right=[OrderedDict(), ]
                       )


class Traffic(object):
    def __init__(self, shared_list, State_Other_List, lock):
        self.shared_list = shared_list
        self.State_Other_List = State_Other_List
        self.time_start = 0
        self.lock = lock
        self.task = 'left'
        self.mode_index = 0                      # 'straight' or 'right' or 'left'
        self.mode = TRAFFICSETTINGS[self.task][self.mode_index]
        self.base_frequency = 10

    def prediction(self, mode):
        State_other = {'x_other': [], 'y_other': [], 'v_other': [], 'heading_next': []}
        for vehicle, info in mode.items():
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
                        delta_phi_rad = np.where(middle_cond, (veh_vs / (CROSSROAD_SIZE / 2 - 0.5 * LANE_WIDTH)) / self.base_frequency, zeros)
                        acc = -0.5
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

                State_other['x_other'].append(next_veh_xs)
                State_other['y_other'].append(next_veh_ys)
                State_other['v_other'].append(next_veh_vs)
                State_other['heading_next'].append(next_veh_phis)
                mode[vehicle].update(x=next_veh_xs, y=next_veh_ys, phi=next_veh_phis, v=next_veh_vs)
        return State_other

    def run(self):
        time_receive_radar = 0
        State_other = {'x_other': [], 'y_other': [], 'v_other': [], 'heading_next': []}
        mode = self.mode

        while True:
            State_other, mode = self.prediction(mode)
            time_receive_radar = time.time() - self.time_start
            self.time_start = time.time()

            # with self.lock:
            #     self.shared_list[4] = time_receive_radar
            #     self.State_Other_List[0] = State_other.copy()

            if time_receive_radar > 0.1:
                print("Subscriber of radar is more than 0.1s!", time_receive_radar)
            print(State_other['x_other'], State_other['y_other'])

        # State_other = {}
        # time_receive_radar = 0
        # while True:
        #     try:
        #         State_other["x_other"] = list(RadarJson["Radar"]["Radar"]["Data"]["DistLong"].values())
        #         State_other["y_other"] = list(RadarJson["Radar"]["Radar"]["Data"]["DistLat"].values())
        #         v_Lon = np.array(list(RadarJson["Radar"]["Radar"]["Data"]["VelocityLon"].values()))
        #         v_Lat = np.array(list(RadarJson["Radar"]["Radar"]["Data"]["VelocityLat"].values()))
        #         State_other["v_other"] = np.sqrt(v_Lon**2+v_Lat**2).tolist()
        #         State_other["heading_next"] = list(RadarJson["Radar"]["Radar"]["Data"]["orientation"].values())
        #         # print("-----------heading_next------",State_other["heading_next"])
        #         time_receive_radar = time.time() - self.time_start
        #         self.time_start = time.time()
        #
        #     except:
        #         pass
        #
        #     with self.lock:
        #         self.shared_list[4] = time_receive_radar
        #         self.State_Other_List[0]=State_other.copy()
        #
        #     if time_receive_radar > 0.1:
        #         print("Subscriber of radar is more than 0.1s!", time_receive_radar)


if __name__ == '__main__':
    traffic = Traffic(shared_list=[0, 0, 0, 0, 0], State_Other_List=[2, 3], lock=1)
    traffic.run()
