#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# =====================================
# @Time    : 2020/12/28
# @Author  : Yang Guan (Tsinghua Univ.)
# @FileName: convert_map.py
# =====================================

import numpy as np
from utils.coordi_convert import vec_convert_gps_coordi_to_intersection_coordi
import matplotlib.pyplot as plt


def convert_map():
    left_txt = np.loadtxt('../map/roadMap_ws.txt')
    stra_txt = np.loadtxt('../map/roadMap_ww.txt')
    right_txt = np.loadtxt('../map/roadMap_wn.txt')

    left_phi_in_gps, left_x_in_gps, left_y_in_gps = left_txt[:, 3], left_txt[:, 4], left_txt[:, 5]
    stra_phi_in_gps, stra_x_in_gps, stra_y_in_gps = stra_txt[:, 3], stra_txt[:, 4], stra_txt[:, 5]
    right_phi_in_gps, right_x_in_gps, right_y_in_gps = right_txt[:, 3], right_txt[:, 4], right_txt[:, 5]

    left_x, left_y, left_phi = vec_convert_gps_coordi_to_intersection_coordi(left_x_in_gps, left_y_in_gps, left_phi_in_gps)
    stra_x, stra_y, stra_phi = vec_convert_gps_coordi_to_intersection_coordi(stra_x_in_gps, stra_y_in_gps, stra_phi_in_gps)
    right_x, right_y, right_phi = vec_convert_gps_coordi_to_intersection_coordi(right_x_in_gps, right_y_in_gps, right_phi_in_gps)

    np.save('../map/left_ref.npy', np.array([left_x, left_y, left_phi]), allow_pickle=True)
    np.save('../map/straight_ref.npy', np.array([stra_x, stra_y, stra_phi]), allow_pickle=True)
    np.save('../map/right_ref.npy', np.array([right_x, right_y, right_phi]), allow_pickle=True)

def chect_converted_map():
    left_ref = np.load('../map/left_ref.npy')
    left_x, left_y, left_phi = left_ref[0], left_ref[1], left_ref[2]
    straight_ref = np.load('../map/straight_ref.npy')
    straight_x, straight_y, straight_phi = straight_ref[0], straight_ref[1], straight_ref[2]
    right_ref = np.load('../map/right_ref.npy')
    right_x, right_y, right_phi = right_ref[0], right_ref[1], right_ref[2]
    plt.plot(left_x, left_y, 'r')
    plt.plot(straight_x, straight_y, 'g')
    plt.plot(right_x, right_y, 'b')

    print(left_phi, straight_phi, right_phi)
    plt.show()


if __name__ == '__main__':
    chect_converted_map()



