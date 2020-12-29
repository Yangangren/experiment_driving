#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# =====================================
# @Time    : 2020/12/28
# @Author  : Yang Guan (Tsinghua Univ.)
# @FileName: coordi_convert.py
# =====================================

import math
import numpy as np

ROTATE_ANGLE = 90.


def shift_coordination(orig_x, orig_y, coordi_shift_x, coordi_shift_y):
    shifted_x = orig_x - coordi_shift_x
    shifted_y = orig_y - coordi_shift_y
    return shifted_x, shifted_y


def rotate_coordination(orig_x, orig_y, orig_d, coordi_rotate_d):
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


def shift_and_rotate_coordination(orig_x, orig_y, orig_d, coordi_shift_x, coordi_shift_y, coordi_rotate_d):
    shift_x, shift_y = shift_coordination(orig_x, orig_y, coordi_shift_x, coordi_shift_y)
    transformed_x, transformed_y, transformed_d \
        = rotate_coordination(shift_x, shift_y, orig_d, coordi_rotate_d)
    return transformed_x, transformed_y, transformed_d


def convert_gps_coordi_to_intersection_coordi(x, y, phi):
    phi_in_anticlockwise = -(phi-90.)
    intersection_x_in_gps_coordi, intersection_y_in_gps_coordi = 21277043.5350594-14., 3447703.03112017-3.5/2
    trans_x, trans_y, trans_phi = shift_and_rotate_coordination(x, y, phi_in_anticlockwise,
                                  intersection_x_in_gps_coordi, intersection_y_in_gps_coordi, ROTATE_ANGLE)
    return trans_x, trans_y, trans_phi


def vec_shift_coordination(orig_x, orig_y, coordi_shift_x, coordi_shift_y):
    shifted_x = orig_x - coordi_shift_x
    shifted_y = orig_y - coordi_shift_y
    return shifted_x, shifted_y


def vec_rotate_coordination(orig_x, orig_y, orig_d, coordi_rotate_d):
    coordi_rotate_d_in_rad = coordi_rotate_d * np.pi / 180
    transformed_x = orig_x * np.cos(coordi_rotate_d_in_rad) + orig_y * np.sin(coordi_rotate_d_in_rad)
    transformed_y = -orig_x * np.sin(coordi_rotate_d_in_rad) + orig_y * np.cos(coordi_rotate_d_in_rad)
    transformed_d = orig_d - coordi_rotate_d
    transformed_d = np.where(transformed_d > 180., transformed_d - 360., transformed_d)
    transformed_d = np.where(transformed_d <= -180., transformed_d + 360., transformed_d)
    return transformed_x, transformed_y, transformed_d


def vec_shift_and_rotate_coordination(orig_x, orig_y, orig_d, coordi_shift_x, coordi_shift_y, coordi_rotate_d):
    shift_x, shift_y = vec_shift_coordination(orig_x, orig_y, coordi_shift_x, coordi_shift_y)
    transformed_x, transformed_y, transformed_d \
        = vec_rotate_coordination(shift_x, shift_y, orig_d, coordi_rotate_d)
    return transformed_x, transformed_y, transformed_d


def vec_convert_gps_coordi_to_intersection_coordi(x, y, phi):
    phi_in_anticlockwise = -(phi-90.)
    intersection_x_in_gps_coordi, intersection_y_in_gps_coordi = 21277043.5350594-14., 3447703.03112017-3.5/2
    trans_x, trans_y, trans_phi = vec_shift_and_rotate_coordination(x, y, phi_in_anticlockwise,
                                  intersection_x_in_gps_coordi, intersection_y_in_gps_coordi, ROTATE_ANGLE)
    return trans_x, trans_y, trans_phi


