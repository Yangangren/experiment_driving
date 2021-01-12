import json
import os
import struct
import time
from collections import OrderedDict
from datetime import datetime
from math import pi

import bezier
import numpy as np
import tensorflow as tf
import zmq
from utils.load_policy import LoadPolicy
from utils.coordi_convert import convert_center_to_rear

VEHICLE_MODE_DICT = dict(left=OrderedDict(dl=1, du=1, dr=1, ud=2, ul=1),           # dl=2, du=2, ud=2, ul=2
                         straight=OrderedDict(dl=1, du=1, dr=1, ud=1, ru=2, ur=2), # dl=1, du=2, ud=2, ru=2, ur=2
                         right=OrderedDict(dl=1, du=1, dr=1, ur=2, lr=2))

ROUTE2MODE = {('1o', '2i'): 'dr', ('1o', '3i'): 'du', ('1o', '4i'): 'dl',
              ('2o', '1i'): 'rd', ('2o', '3i'): 'ru', ('2o', '4i'): 'rl',
              ('3o', '1i'): 'ud', ('3o', '2i'): 'ur', ('3o', '4i'): 'ul',
              ('4o', '1i'): 'ld', ('4o', '2i'): 'lr', ('4o', '3i'): 'lu'}
LANE_WIDTH = 3.5
CROSSROAD_SIZE = 22
LANE_NUMBER = 1
MODE2ROUTE = {'dr': ('1o', '2i'), 'du': ('1o', '3i'), 'dl': ('1o', '4i'),
              'rd': ('2o', '1i'), 'ru': ('2o', '3i'), 'rl': ('2o', '4i'),
              'ud': ('3o', '1i'), 'ur': ('3o', '2i'), 'ul': ('3o', '4i'),
              'ld': ('4o', '1i'), 'lr': ('4o', '2i'), 'lu': ('4o', '3i')}

MODE2TASK = {'dr': 'right', 'du': 'straight', 'dl': 'left',
             'rd': 'left', 'ru': 'right', 'rl': ' straight',
             'ud': 'straight', 'ur': 'left', 'ul': 'right',
             'ld': 'right', 'lr': 'straight', 'lu': 'left'}

EXPECTED_V = 3.
START_OFFSET = 3.0
TASKCASE2MODELIST = dict(left=[['dl', 'ud', 'ul'], ['dl', 'ud', 'ul'], ['dl', 'ud', 'ul'], ['ud']],
                         straight=[['du', 'ru', 'ur'], ['du', 'ru', 'ur'], ['du', 'ru', 'ur']],
                         right=[['dr', 'ur'], ['dr', 'ur'], ['dr', 'lr']])


def deal_with_phi_diff(phi_diff):
    phi_diff = tf.where(phi_diff > 180., phi_diff - 360., phi_diff)
    phi_diff = tf.where(phi_diff < -180., phi_diff + 360., phi_diff)
    return phi_diff


class VehicleDynamics(object):
    def __init__(self, ):
        self.vehicle_params = dict(C_f=-155495,  # front wheel cornering stiffness [N/rad]
                                   C_r=-155495,  # rear wheel cornering stiffness [N/rad]
                                   a=1.19,  # distance from CG to front axle [m]
                                   b=1.46,  # distance from CG to rear axle [m]
                                   mass=1520.,  # mass [kg]
                                   I_z=2642,  # Polar moment of inertia at CG [kg*m^2]
                                   miu=0.8,  # tire-road friction coefficient
                                   g=9.81,  # acceleration of gravity [m/s^2]
                                   )
        # self.vehicle_params = dict(C_f=-128915.5,  # front wheel cornering stiffness [N/rad]
        #                            C_r=-85943.6,  # rear wheel cornering stiffness [N/rad]
        #                            a=1.06,  # distance from CG to front axle [m]
        #                            b=1.85,  # distance from CG to rear axle [m]
        #                            mass=1412.,  # mass [kg]
        #                            I_z=1536.7,  # Polar moment of inertia at CG [kg*m^2]
        #                            miu=1.0,  # tire-road friction coefficient
        #                            g=9.81,  # acceleration of gravity [m/s^2]
        #                            )
        a, b, mass, g = self.vehicle_params['a'], self.vehicle_params['b'], \
                        self.vehicle_params['mass'], self.vehicle_params['g']
        F_zf, F_zr = b * mass * g / (a + b), a * mass * g / (a + b)
        self.vehicle_params.update(dict(F_zf=F_zf,
                                        F_zr=F_zr))
        self.states = np.array([[0., 0., 0., 1.75, -40., 90.]], dtype=np.float32)
        self.states = tf.convert_to_tensor(self.states, dtype=tf.float32)

    def f_xu(self, actions, tau):  # states and actions are tensors, [[], [], ...]
        v_x, v_y, r, x, y, phi = self.states[:, 0], self.states[:, 1], self.states[:, 2],\
                                 self.states[:, 3], self.states[:, 4], self.states[:, 5]
        phi = phi * np.pi / 180.
        steer, a_x = actions[:, 0], actions[:, 1]
        C_f = tf.convert_to_tensor(self.vehicle_params['C_f'], dtype=tf.float32)
        C_r = tf.convert_to_tensor(self.vehicle_params['C_r'], dtype=tf.float32)
        a = tf.convert_to_tensor(self.vehicle_params['a'], dtype=tf.float32)
        b = tf.convert_to_tensor(self.vehicle_params['b'], dtype=tf.float32)
        mass = tf.convert_to_tensor(self.vehicle_params['mass'], dtype=tf.float32)
        I_z = tf.convert_to_tensor(self.vehicle_params['I_z'], dtype=tf.float32)
        v_x_next = v_x + tau * (a_x + v_y * r)
        v_x_next = tf.clip_by_value(v_x_next, 0.,10.)

        next_state = [v_x_next,
                      (mass * v_y * v_x + tau * (
                                  a * C_f - b * C_r) * r - tau * C_f * steer * v_x - tau * mass * tf.square(
                          v_x) * r) / (mass * v_x - tau * (C_f + C_r)),
                      (-I_z * r * v_x - tau * (a * C_f - b * C_r) * v_y + tau * a * C_f * steer * v_x) / (
                                  tau * (tf.square(a) * C_f + tf.square(b) * C_r) - I_z * v_x),
                      x + tau * (v_x * tf.cos(phi) - v_y * tf.sin(phi)),
                      y + tau * (v_x * tf.sin(phi) + v_y * tf.cos(phi)),
                      (phi + tau * r) * 180 / np.pi]

        return tf.stack(next_state, 1)

    def prediction(self, u_1, tau):
        self.states = self.f_xu(u_1, tau)
        return self.states.numpy()

    def set_states(self, states):
        self.states = tf.convert_to_tensor(states, dtype=tf.float32)

    def get_states(self):
        return self.states.numpy()

    def model_step(self, state_gps, vehiclemode, action4model, delta_t, prefix):
        if vehiclemode == 0:
            v_x, r, x, y, phi = state_gps['GpsSpeed'], state_gps['YawRate'], state_gps['GaussX'], \
                                state_gps['GaussY'], state_gps['Heading']
            self.set_states(np.array([[v_x, 0., r, x, y, phi]]))
            out = [('model_vx_in_{}_action'.format(prefix), v_x),
                   ('model_vy_in_{}_action'.format(prefix), 0.),
                   ('model_r_in_{}_action'.format(prefix), r),
                   ('model_x_in_{}_action'.format(prefix), x),
                   ('model_y_in_{}_action'.format(prefix), y),
                   ('model_phi_in_{}_action'.format(prefix), phi),
                   ('model_front_wheel_rad_in_{}_action'.format(prefix), 0.),
                   ('model_acc_in_{}_action'.format(prefix), 0.),
                   ]
            return OrderedDict(out)
        else:
            front_wheel_rad, acc = action4model[0][0], action4model[0][1]
            model_state = self.prediction(action4model, delta_t)
            v_x, v_y, r, x, y, phi = model_state[0][0], model_state[0][1], model_state[0][2], \
                                     model_state[0][3], model_state[0][4], model_state[0][5]
            out = [('model_vx_in_{}_action'.format(prefix), v_x),
                   ('model_vy_in_{}_action'.format(prefix), v_y),
                   ('model_r_in_{}_action'.format(prefix), r),
                   ('model_x_in_{}_action'.format(prefix), x),
                   ('model_y_in_{}_action'.format(prefix), y),
                   ('model_phi_in_{}_action'.format(prefix), phi),
                   ('model_front_wheel_rad_in_{}_action'.format(prefix), front_wheel_rad),
                   ('model_acc_in_{}_action'.format(prefix), acc),
                   ]
            return OrderedDict(out)


class ReferencePath(object):
    def __init__(self, task, mode=None, ref_index=None):
        self.mode = mode
        self.traj_mode = None
        self.exp_v = EXPECTED_V
        self.task = task
        self.path_list = []
        self.path_len_list = []
        self._construct_ref_path(self.task)
        self.ref_index = np.random.choice(len(self.path_list)) if ref_index is None else ref_index
        self.path = self.path_list[self.ref_index]

    def set_path(self, traj_mode, path_index=None, path=None):
        self.traj_mode = traj_mode
        if traj_mode == 'dyna_traj':
            self.path = path
        elif traj_mode == 'static_traj':
            self.ref_index = path_index
            self.path = self.path_list[self.ref_index]

    def _construct_ref_path(self, task):
        sl = 40  # straight length
        meter_pointnum_ratio = 30
        control_ext = CROSSROAD_SIZE/3.
        if task == 'left':
            end_offsets = [LANE_WIDTH*(i+0.5) for i in range(LANE_NUMBER)]
            start_offsets = [LANE_WIDTH*0.5]
            for start_offset in start_offsets:
                for end_offset in end_offsets:
                    control_point1 = start_offset, -CROSSROAD_SIZE/2
                    control_point2 = start_offset, -CROSSROAD_SIZE/2 + control_ext
                    control_point3 = -CROSSROAD_SIZE/2 + control_ext, end_offset
                    control_point4 = -CROSSROAD_SIZE/2, end_offset

                    node = np.asfortranarray([[control_point1[0], control_point2[0], control_point3[0], control_point4[0]],
                                              [control_point1[1], control_point2[1], control_point3[1], control_point4[1]]],
                                             dtype=np.float32)
                    curve = bezier.Curve(node, degree=3)
                    s_vals = np.linspace(0, 1.0, int(pi/2*(CROSSROAD_SIZE/2+LANE_WIDTH/2)) * meter_pointnum_ratio)
                    trj_data = curve.evaluate_multi(s_vals)
                    trj_data = trj_data.astype(np.float32)
                    # rules: -0.7 for left case 0 and case 2, -0.4 for left case 1;
                    # start_straight_line_x = (LANE_WIDTH/2-0.7) * np.ones(shape=(sl * meter_pointnum_ratio,),
                    # dtype=np.float32)[:-1]
                    start_straight_line_x = start_offsets * np.ones(shape=(sl * meter_pointnum_ratio,), dtype=np.float32)[:-1]
                    start_straight_line_y = np.linspace(-CROSSROAD_SIZE/2 - sl, -CROSSROAD_SIZE/2, sl * meter_pointnum_ratio, dtype=np.float32)[:-1]
                    end_straight_line_x = np.linspace(-CROSSROAD_SIZE/2, -CROSSROAD_SIZE/2 - sl, sl * meter_pointnum_ratio, dtype=np.float32)[1:]
                    end_straight_line_y = end_offset * np.ones(shape=(sl * meter_pointnum_ratio,), dtype=np.float32)[1:]
                    planed_trj = np.append(np.append(start_straight_line_x, trj_data[0]), end_straight_line_x), \
                                 np.append(np.append(start_straight_line_y, trj_data[1]), end_straight_line_y)

                    xs_1, ys_1 = planed_trj[0][:-1], planed_trj[1][:-1]
                    xs_2, ys_2 = planed_trj[0][1:], planed_trj[1][1:]
                    phis_1 = np.arctan2(ys_2 - ys_1,
                                        xs_2 - xs_1) * 180 / pi
                    planed_trj = xs_1, ys_1, phis_1
                    self.path_list.append(planed_trj)
                    self.path_len_list.append((sl * meter_pointnum_ratio, len(trj_data[0]), len(xs_1)))

        elif task == 'straight':
            end_offsets = [LANE_WIDTH*(i+0.5) for i in range(LANE_NUMBER)]
            start_offsets = [LANE_WIDTH*0.5]
            for start_offset in start_offsets:
                for end_offset in end_offsets:
                    control_point1 = start_offset, -CROSSROAD_SIZE/2
                    control_point2 = start_offset, -CROSSROAD_SIZE/2 + control_ext
                    control_point3 = end_offset, CROSSROAD_SIZE/2 - control_ext
                    control_point4 = end_offset, CROSSROAD_SIZE/2

                    node = np.asfortranarray([[control_point1[0], control_point2[0], control_point3[0], control_point4[0]],
                                              [control_point1[1], control_point2[1], control_point3[1], control_point4[1]]]
                                             , dtype=np.float32)
                    curve = bezier.Curve(node, degree=3)
                    s_vals = np.linspace(0, 1.0, CROSSROAD_SIZE * meter_pointnum_ratio)
                    trj_data = curve.evaluate_multi(s_vals)
                    trj_data = trj_data.astype(np.float32)
                    start_straight_line_x = start_offset * np.ones(shape=(sl * meter_pointnum_ratio,), dtype=np.float32)[:-1]
                    start_straight_line_y = np.linspace(-CROSSROAD_SIZE/2 - sl, -CROSSROAD_SIZE/2, sl * meter_pointnum_ratio, dtype=np.float32)[:-1]
                    end_straight_line_x = end_offset * np.ones(shape=(sl * meter_pointnum_ratio,), dtype=np.float32)[1:]
                    end_straight_line_y = np.linspace(CROSSROAD_SIZE/2, CROSSROAD_SIZE/2 + sl, sl * meter_pointnum_ratio, dtype=np.float32)[1:]
                    planed_trj = np.append(np.append(start_straight_line_x, trj_data[0]), end_straight_line_x), \
                                 np.append(np.append(start_straight_line_y, trj_data[1]), end_straight_line_y)
                    xs_1, ys_1 = planed_trj[0][:-1], planed_trj[1][:-1]
                    xs_2, ys_2 = planed_trj[0][1:], planed_trj[1][1:]
                    phis_1 = np.arctan2(ys_2 - ys_1,
                                        xs_2 - xs_1) * 180 / pi
                    planed_trj = xs_1, ys_1, phis_1
                    self.path_list.append(planed_trj)
                    self.path_len_list.append((sl * meter_pointnum_ratio, len(trj_data[0]), len(xs_1)))

        else:
            assert task == 'right'
            control_ext = CROSSROAD_SIZE/5. + 3.
            end_offsets = [-LANE_WIDTH * 0.5]
            start_offsets = [LANE_WIDTH*(LANE_NUMBER-0.5)]

            for start_offset in start_offsets:
                for end_offset in end_offsets:
                    control_point1 = start_offset, -CROSSROAD_SIZE/2
                    control_point2 = start_offset, -CROSSROAD_SIZE/2 + control_ext
                    control_point3 = CROSSROAD_SIZE/2 - control_ext, end_offset
                    control_point4 = CROSSROAD_SIZE/2, end_offset

                    node = np.asfortranarray([[control_point1[0], control_point2[0], control_point3[0], control_point4[0]],
                                              [control_point1[1], control_point2[1], control_point3[1], control_point4[1]]],
                                             dtype=np.float32)
                    curve = bezier.Curve(node, degree=3)
                    s_vals = np.linspace(0, 1.0, int(pi/2*(CROSSROAD_SIZE/2-LANE_WIDTH*(LANE_NUMBER-0.5))) * meter_pointnum_ratio)
                    trj_data = curve.evaluate_multi(s_vals)
                    trj_data = trj_data.astype(np.float32)

                    def fn(i):
                        if i < 600:
                            return 1.2
                        else:
                            return 1.2-1.2/600*(i-600)
                    # rules: used for right case 0 and case 2
                    # start_straight_line_x = np.array([1.75+fn(x) for x in range(1199)], dtype=np.float32)
                    start_straight_line_x = start_offset * np.ones(shape=(sl * meter_pointnum_ratio,), dtype=np.float32)[:-1]
                    start_straight_line_y = np.linspace(-CROSSROAD_SIZE/2 - sl, -CROSSROAD_SIZE/2, sl * meter_pointnum_ratio, dtype=np.float32)[:-1]
                    end_straight_line_x = np.linspace(CROSSROAD_SIZE/2, CROSSROAD_SIZE/2 + sl, sl * meter_pointnum_ratio, dtype=np.float32)[1:]
                    end_straight_line_y = end_offset * np.ones(shape=(sl * meter_pointnum_ratio,), dtype=np.float32)[1:]
                    planed_trj = np.append(np.append(start_straight_line_x, trj_data[0]), end_straight_line_x), \
                                 np.append(np.append(start_straight_line_y, trj_data[1]), end_straight_line_y)
                    xs_1, ys_1 = planed_trj[0][:-1], planed_trj[1][:-1]
                    xs_2, ys_2 = planed_trj[0][1:], planed_trj[1][1:]
                    phis_1 = np.arctan2(ys_2 - ys_1,
                                        xs_2 - xs_1) * 180 / pi
                    planed_trj = xs_1, ys_1, phis_1
                    self.path_list.append(planed_trj)
                    self.path_len_list.append((sl * meter_pointnum_ratio, len(trj_data[0]), len(xs_1)))

    def find_closest_point(self, xs, ys, ratio=6):
        path_len = len(self.path[0])
        reduced_idx = np.arange(0, path_len, ratio)
        reduced_len = len(reduced_idx)
        reduced_path_x, reduced_path_y = self.path[0][reduced_idx], self.path[1][reduced_idx]
        xs_tile = tf.tile(tf.reshape(xs, (-1, 1)), tf.constant([1, reduced_len]))
        ys_tile = tf.tile(tf.reshape(ys, (-1, 1)), tf.constant([1, reduced_len]))
        pathx_tile = tf.tile(tf.reshape(reduced_path_x, (1, -1)), tf.constant([len(xs), 1]))
        pathy_tile = tf.tile(tf.reshape(reduced_path_y, (1, -1)), tf.constant([len(xs), 1]))

        dist_array = tf.square(xs_tile - pathx_tile) + tf.square(ys_tile - pathy_tile)

        indexs = tf.argmin(dist_array, 1) * ratio
        return indexs, self.indexs2points(indexs)

    def future_n_data(self, current_indexs, n):
        future_data_list = []
        current_indexs = tf.cast(current_indexs, tf.int32)
        for _ in range(n):
            current_indexs += 80
            current_indexs = tf.where(current_indexs >= len(self.path[0]) - 2, len(self.path[0]) - 2, current_indexs)
            future_data_list.append(self.indexs2points(current_indexs))
        return future_data_list

    def indexs2points(self, indexs):
        indexs = tf.where(indexs >= 0, indexs, 0)
        indexs = tf.where(indexs < len(self.path[0]), indexs, len(self.path[0])-1)
        points = tf.gather(self.path[0], indexs), \
                 tf.gather(self.path[1], indexs), \
                 tf.gather(self.path[2], indexs)

        return points[0], points[1], points[2]

    def tracking_error_vector(self, ego_xs, ego_ys, ego_phis, ego_vs, n, func=None):
        def two2one(ref_xs, ref_ys):
            if self.task == 'left':
                delta_ = tf.sqrt(tf.square(ego_xs - (-CROSSROAD_SIZE/2)) + tf.square(ego_ys - (-CROSSROAD_SIZE/2))) - \
                         tf.sqrt(tf.square(ref_xs - (-CROSSROAD_SIZE/2)) + tf.square(ref_ys - (-CROSSROAD_SIZE/2)))
                delta_ = tf.where(ego_ys < -CROSSROAD_SIZE/2, ego_xs - ref_xs, delta_)
                delta_ = tf.where(ego_xs < -CROSSROAD_SIZE/2, ego_ys - ref_ys, delta_)
                return -delta_
            elif self.task == 'straight':
                delta_ = ego_xs - ref_xs
                return -delta_
            else:
                assert self.task == 'right'
                delta_ = -(tf.sqrt(tf.square(ego_xs - CROSSROAD_SIZE/2) + tf.square(ego_ys - (-CROSSROAD_SIZE/2))) -
                           tf.sqrt(tf.square(ref_xs - CROSSROAD_SIZE/2) + tf.square(ref_ys - (-CROSSROAD_SIZE/2))))
                delta_ = tf.where(ego_ys < -CROSSROAD_SIZE/2, ego_xs - ref_xs, delta_)
                delta_ = tf.where(ego_xs > CROSSROAD_SIZE/2, -(ego_ys - ref_ys), delta_)
                return -delta_

        indexs, current_points = self.find_closest_point(ego_xs, ego_ys)
        # print('Index:', indexs.numpy(), 'points:', current_points[:])
        n_future_data = self.future_n_data(indexs, n)

        tracking_error = tf.stack([two2one(current_points[0], current_points[1]),
                                           deal_with_phi_diff(ego_phis - current_points[2]),
                                           ego_vs - self.exp_v], 1)

        final = tracking_error
        if n > 0:
            future_points = tf.concat([tf.stack([ref_point[0] - ego_xs,
                                                 ref_point[1] - ego_ys,
                                                 deal_with_phi_diff(ego_phis - ref_point[2])], 1)
                                       for ref_point in n_future_data], 1)
            final = tf.concat([final, future_points], 1)

        return final


class Controller(object):
    def __init__(self, shared_list, receive_index, if_save, if_radar, lock, task, case,
                 noise_factor, load_dir, load_ite, result_dir, model_only_test, clipped_v):
        self.time_out = 0
        self.task = task
        self.case = case
        self.ref_path = ReferencePath(self.task)
        self.num_future_data = 0
        self.noise_factor = noise_factor
        self.model = LoadPolicy(load_dir, load_ite)
        self.steer_factor = 15
        self.shared_list = shared_list
        self.read_index_old = 0
        self.receive_index_shared = receive_index
        self.model_only_test = model_only_test
        self.clipped_v = clipped_v
        # self.read_index_old = Info_List[0]

        self.lock = lock
        context = zmq.Context()
        self.socket_pub = context.socket(zmq.PUB)
        self.socket_pub.bind("tcp://*:6970")

        context = zmq.Context()
        self.socket_pub_radar = context.socket(zmq.PUB)
        self.socket_pub_radar.bind("tcp://*:5555")
        self.time_initial = time.time()
        self.step = 0
        self.if_save = if_save
        self.if_radar = if_radar
        self.save_path = result_dir
        self.t_interval = 0
        self.time_decision = 0
        self.time_in = time.time()

        self.last_steer_output = 0
        self.model_driven_by_model_action = VehicleDynamics()
        self.model_driven_by_real_action = VehicleDynamics()

        self.run_time = 0.

    def _construct_ego_vector(self, state_gps, model_flag):
        if model_flag:
            v_x, v_y, r, x, y, phi = state_gps['v_x'], state_gps['v_y'], state_gps['r'],\
                                     state_gps['x'],  state_gps['y'], state_gps['phi']
            self.ego_info_dim = 6
            ego_feature = [v_x, v_y, r, x, y, phi]
            return np.array(ego_feature, dtype=np.float32)
        else:
            ego_phi = state_gps['Heading']
            ego_phi_rad = ego_phi * np.pi / 180.
            ego_x, ego_y = state_gps['GaussX'], state_gps['GaussY']
            v_in_y_coord, v_in_x_coord = -state_gps['EastVelocity'], state_gps['NorthVelocity']
            ego_v_x = v_in_y_coord * np.sin(ego_phi_rad) + v_in_x_coord * np.cos(ego_phi_rad)
            ego_v_y = v_in_y_coord * np.cos(ego_phi_rad) - v_in_x_coord * np.sin(ego_phi_rad)  # todo: check the sign
            ego_v_y = - ego_v_y
            ego_r = state_gps['YawRate']                      # rad/s
            self.ego_info_dim = 6
            ego_feature = [ego_v_x, ego_v_y, ego_r, ego_x, ego_y, ego_phi]
            return np.array(ego_feature, dtype=np.float32)

    def _construct_veh_vector(self, ego_x, ego_y, state_others):
        mode_list = TASKCASE2MODELIST[self.task][self.case]
        all_vehicles = []
        if self.case == 3:
            v_light = 2 if self.run_time < 40. else 0
        else:
            v_light = state_others['v_light']
        xs, ys, vs, phis = state_others['x_other'], state_others['y_other'],\
                           state_others['v_other'], state_others['phi_other']
        if not xs:
            mode_list = []
        for i, veh_mode in enumerate(mode_list):
            all_vehicles.append(dict(x=xs[i], y=ys[i], v=vs[i], phi=phis[i], l=5., w=2., route=MODE2ROUTE[veh_mode]))
        vehs_vector = []
        name_setting = dict(do='1o', di='1i', ro='2o', ri='2i', uo='3o', ui='3i', lo='4o', li='4i')

        def filter_interested_vehicles(vs, task):
            dl, du, dr, rd, rl, ru, ur, ud, ul, lu, lr, ld = [], [], [], [], [], [], [], [], [], [], [], []
            for v in vs:
                route_list = v['route']
                start = route_list[0]
                end = route_list[1]
                if start == name_setting['do'] and end == name_setting['li']:
                    dl.append(v)
                elif start == name_setting['do'] and end == name_setting['ui']:
                    du.append(v)
                elif start == name_setting['do'] and end == name_setting['ri']:
                    dr.append(v)

                elif start == name_setting['ro'] and end == name_setting['di']:
                    rd.append(v)
                elif start == name_setting['ro'] and end == name_setting['li']:
                    rl.append(v)
                elif start == name_setting['ro'] and end == name_setting['ui']:
                    ru.append(v)

                elif start == name_setting['uo'] and end == name_setting['ri']:
                    ur.append(v)
                elif start == name_setting['uo'] and end == name_setting['di']:
                    ud.append(v)
                elif start == name_setting['uo'] and end == name_setting['li']:
                    ul.append(v)

                elif start == name_setting['lo'] and end == name_setting['ui']:
                    lu.append(v)
                elif start == name_setting['lo'] and end == name_setting['ri']:
                    lr.append(v)
                elif start == name_setting['lo'] and end == name_setting['di']:
                    ld.append(v)
            if v_light != 0 and ego_y < -CROSSROAD_SIZE/2 - START_OFFSET:
                dl.append(dict(x=LANE_WIDTH/2, y=-CROSSROAD_SIZE/2, v=0., phi=90, l=5, w=2.5, route=None))
                dl.append(dict(x=LANE_WIDTH/2, y=-CROSSROAD_SIZE/2+2.5, v=0., phi=90, l=5, w=2.5, route=None))
                du.append(dict(x=LANE_WIDTH*1.5, y=-CROSSROAD_SIZE/2, v=0., phi=90, l=5, w=2.5, route=None))
                du.append(dict(x=LANE_WIDTH*1.5, y=-CROSSROAD_SIZE/2+2.5, v=0., phi=90, l=5, w=2.5, route=None))

            # fetch veh in range
            if task == 'left':
                dl = list(filter(lambda v: v['x'] > -CROSSROAD_SIZE/2-10 and v['y'] > ego_y-2, dl))
                du = list(filter(lambda v: ego_y-2 < v['y'] < CROSSROAD_SIZE/2+10 and v['x'] < ego_x+2, du))
                dr = list(filter(lambda v: v['x'] < ego_x+2 and v['y'] > ego_y-2, dr))
            elif task == 'straight':
                dl = list(filter(lambda v: v['x'] > ego_x-2 and v['y'] > ego_y - 2, dl))
                du = list(filter(lambda v: ego_y - 2 < v['y'] < CROSSROAD_SIZE / 2 + 10, du))
                dr = list(filter(lambda v: v['x'] < ego_x+2 and v['y'] > ego_y-2, dr))
            else:
                assert task == 'right'
                dl = list(filter(lambda v: v['x'] > ego_x - 2 and v['y'] > ego_y - 2, dl))
                du = list(filter(lambda v: v['x'] > ego_x - 2 and v['y'] > ego_y - 2, du))
                dr = list(filter(lambda v: v['x'] < CROSSROAD_SIZE/2+10 and v['y'] > ego_y-2, dr))

            rd = rd  # not interest in case of traffic light
            rl = rl  # not interest in case of traffic light
            ru = list(filter(lambda v: v['x'] < CROSSROAD_SIZE/2+10 and v['y'] < CROSSROAD_SIZE/2+10, ru))

            ur_straight = list(filter(lambda v: v['x'] < ego_x + 2 and ego_y < v['y'] < CROSSROAD_SIZE/2+10, ur))
            ur_right = list(filter(lambda v: v['x'] < CROSSROAD_SIZE/2+10 and v['y'] < CROSSROAD_SIZE/2, ur))
            ud = list(filter(lambda v: max(ego_y-2, -CROSSROAD_SIZE/2) < v['y'] < CROSSROAD_SIZE/2
                                       and ego_x > v['x'] and ego_y > -CROSSROAD_SIZE/2-START_OFFSET, ud))
            ul = list(filter(lambda v: -CROSSROAD_SIZE/2-10 < v['x'] < ego_x and v['y'] < CROSSROAD_SIZE/2, ul))

            lu = lu  # not interest in case of traffic light
            lr = list(filter(lambda v: -CROSSROAD_SIZE/2-10 < v['x'] < CROSSROAD_SIZE/2+10, lr))  # interest of right
            ld = ld  # not interest in case of traffic light

            # sort
            dl = sorted(dl, key=lambda v: (v['y'], -v['x']))
            du = sorted(du, key=lambda v: v['y'])
            dr = sorted(dr, key=lambda v: (v['y'], v['x']))

            ru = sorted(ru, key=lambda v: (-v['x'], v['y']), reverse=True)

            ur_straight = sorted(ur_straight, key=lambda v: v['y'])
            ur_right = sorted(ur_right, key=lambda v: (-v['y'], v['x']), reverse=True)

            ud = sorted(ud, key=lambda v: v['y'])
            ul = sorted(ul, key=lambda v: (-v['y'], -v['x']), reverse=True)

            lr = sorted(lr, key=lambda v: -v['x'])

            # slice or fill to some number
            def slice_or_fill(sorted_list, fill_value, num):
                if len(sorted_list) >= num:
                    return sorted_list[:num]
                else:
                    while len(sorted_list) < num:
                        sorted_list.append(fill_value)
                    return sorted_list

            fill_value_for_dl = dict(x=LANE_WIDTH/2, y=-(CROSSROAD_SIZE/2+30), v=0, phi=90, w=2.5, l=5, route=('1o', '4i'))
            fill_value_for_du = dict(x=LANE_WIDTH*0.5, y=-(CROSSROAD_SIZE/2+30), v=0, phi=90, w=2.5, l=5, route=('1o', '3i'))
            fill_value_for_dr = dict(x=LANE_WIDTH*(LANE_NUMBER-0.5), y=-(CROSSROAD_SIZE/2+30), v=0, phi=90, w=2.5, l=5, route=('1o', '2i'))

            fill_value_for_ru = dict(x=(CROSSROAD_SIZE/2+15), y=LANE_WIDTH*(LANE_NUMBER-0.5), v=0, phi=180, w=2.5, l=5, route=('2o', '3i'))

            fill_value_for_ur_straight = dict(x=-LANE_WIDTH/2, y=(CROSSROAD_SIZE/2+20), v=0, phi=-90, w=2.5, l=5, route=('3o', '2i'))
            fill_value_for_ur_right = dict(x=-LANE_WIDTH/2, y=(CROSSROAD_SIZE/2+20), v=0, phi=-90, w=2.5, l=5, route=('3o', '2i'))

            fill_value_for_ud = dict(x=-LANE_WIDTH*0.5, y=(CROSSROAD_SIZE/2+20), v=0, phi=-90, w=2.5, l=5, route=('3o', '1i'))
            fill_value_for_ul = dict(x=-LANE_WIDTH*(LANE_NUMBER-0.5), y=(CROSSROAD_SIZE/2+20), v=0, phi=-90, w=2.5, l=5, route=('3o', '4i'))

            fill_value_for_lr = dict(x=-(CROSSROAD_SIZE/2+20), y=-LANE_WIDTH*0.5, v=0, phi=0, w=2.5, l=5, route=('4o', '2i'))

            tmp = OrderedDict()
            if task == 'left':
                tmp['dl'] = slice_or_fill(dl, fill_value_for_dl, VEHICLE_MODE_DICT['left']['dl'])
                tmp['du'] = slice_or_fill(du, fill_value_for_du, VEHICLE_MODE_DICT['left']['du'])
                tmp['dr'] = slice_or_fill(dr, fill_value_for_dr, VEHICLE_MODE_DICT['left']['dr'])
                tmp['ud'] = slice_or_fill(ud, fill_value_for_ud, VEHICLE_MODE_DICT['left']['ud'])
                tmp['ul'] = slice_or_fill(ul, fill_value_for_ul, VEHICLE_MODE_DICT['left']['ul'])
            elif task == 'straight':
                tmp['dl'] = slice_or_fill(dl, fill_value_for_dl, VEHICLE_MODE_DICT['straight']['dl'])
                tmp['du'] = slice_or_fill(du, fill_value_for_du, VEHICLE_MODE_DICT['straight']['du'])
                tmp['dr'] = slice_or_fill(dr, fill_value_for_dr, VEHICLE_MODE_DICT['straight']['dr'])
                tmp['ud'] = slice_or_fill(ud, fill_value_for_ud, VEHICLE_MODE_DICT['straight']['ud'])
                tmp['ru'] = slice_or_fill(ru, fill_value_for_ru, VEHICLE_MODE_DICT['straight']['ru'])
                tmp['ur'] = slice_or_fill(ur_straight, fill_value_for_ur_straight, VEHICLE_MODE_DICT['straight']['ur'])
            elif task == 'right':
                tmp['dl'] = slice_or_fill(dl, fill_value_for_dl, VEHICLE_MODE_DICT['right']['dl'])
                tmp['du'] = slice_or_fill(du, fill_value_for_du, VEHICLE_MODE_DICT['right']['du'])
                tmp['dr'] = slice_or_fill(dr, fill_value_for_dr, VEHICLE_MODE_DICT['right']['dr'])
                tmp['ur'] = slice_or_fill(ur_right, fill_value_for_ur_right, VEHICLE_MODE_DICT['right']['ur'])
                tmp['lr'] = slice_or_fill(lr, fill_value_for_lr, VEHICLE_MODE_DICT['right']['lr'])

            return tmp

        list_of_interested_veh_dict = []
        self.interested_vehs = filter_interested_vehicles(all_vehicles, self.task)
        for part in list(self.interested_vehs.values()):
            list_of_interested_veh_dict.extend(part)
        self.per_veh_info_dim = 4
        for veh in list_of_interested_veh_dict:
            veh_x, veh_y, veh_v, veh_phi = veh['x'], veh['y'], veh['v'], veh['phi']
            vehs_vector.extend([veh_x, veh_y, veh_v, veh_phi])
        return np.array(vehs_vector, dtype=np.float32)

    def _get_obs(self, state_gps, state_others, model_flag):
        if model_flag:
            ego_v_x, _, _, ego_x, ego_y, ego_phi = state_gps['v_x'], state_gps['v_y'], state_gps['r'],\
                                                   state_gps['x'],  state_gps['y'], state_gps['phi']
        else:
            ego_x, ego_y = state_gps['GaussX'], state_gps['GaussY']
            ego_phi = state_gps['Heading']
            ego_v_x, ego_v_y = state_gps['GpsSpeed'], 0.
        vehs_vector = self._construct_veh_vector(ego_x, ego_y, state_others)
        ego_vector = self._construct_ego_vector(state_gps, model_flag)
        tracking_error = self.ref_path.tracking_error_vector(np.array([ego_x], dtype=np.float32),
                                                             np.array([ego_y], dtype=np.float32),
                                                             np.array([ego_phi], dtype=np.float32),
                                                             np.array([ego_v_x], dtype=np.float32),
                                                             self.num_future_data).numpy()[0]
        self.per_tracking_info_dim = 3
        vector = np.concatenate((ego_vector, tracking_error, vehs_vector), axis=0)
        veh_idx_start = self.ego_info_dim + self.per_tracking_info_dim * (self.num_future_data + 1)

        noise = np.zeros_like(vector)
        nf = self.noise_factor
        noise[self.ego_info_dim] = nf * np.clip(np.random.normal(0, 0.017), -0.051, 0.051)
        noise[self.ego_info_dim+1] = nf * np.clip(np.random.normal(0, 0.17), -0.51, 0.51)
        for veh_idx in range(int(len(vehs_vector)/self.per_veh_info_dim)):
            noise[veh_idx_start+self.per_veh_info_dim*veh_idx] = nf * np.clip(np.random.normal(0, 0.05), -0.15, 0.15)
            noise[veh_idx_start+self.per_veh_info_dim*veh_idx+1] = nf * np.clip(np.random.normal(0, 0.05), -0.15, 0.15)
            noise[veh_idx_start+self.per_veh_info_dim*veh_idx+2] = nf * np.clip(np.random.normal(0, 0.05), -0.15, 0.15)
            noise[veh_idx_start+self.per_veh_info_dim*veh_idx+3] = nf * np.clip(np.random.normal(0, 1.4), -5.2, 5.2)

        vector_with_noise = vector + noise
        obs_dict = OrderedDict(ego_vx=ego_vector[0], ego_vy=ego_vector[1], ego_r=ego_vector[2], ego_x=ego_vector[3],
                               ego_y=ego_vector[4], ego_phi=ego_vector[5],
                               tracking_delta_y=tracking_error[0], tracking_delta_phi=tracking_error[1],
                               tracking_delta_v=tracking_error[2],
                               )
        for i in range(int(len(vehs_vector)/self.per_veh_info_dim)):
            obs_dict.update({'other{}_x'.format(i): vehs_vector[self.per_veh_info_dim*i],
                             'other{}_y'.format(i): vehs_vector[self.per_veh_info_dim*i+1],
                             'other{}_v'.format(i): vehs_vector[self.per_veh_info_dim*i+2],
                             'other{}_phi'.format(i): vehs_vector[self.per_veh_info_dim*i+3]})
        return vector_with_noise, obs_dict, vehs_vector  # todo: if output vector without noise

    def _action_transformation_for_end2end(self, action, state_gps, model_flag):  # [-1, 1]
        ego_v_x = state_gps['GpsSpeed'] if not model_flag else state_gps['v_x']
        torque_clip = 100. if ego_v_x > self.clipped_v else 250.         # todo: clipped v
        action = np.clip(action, -1.0, 1.0)
        front_wheel_norm_rad, a_x_norm = action[0], action[1]
        front_wheel_deg = 0.4 / pi * 180 * front_wheel_norm_rad
        steering_wheel = front_wheel_deg * self.steer_factor

        # rule: used in right case 1
        # ego_y = state_gps['GaussY'] if not model_flag else state_gps['y']
        # if ego_y < -10:
        #     steering_wheel = np.clip(steering_wheel, -5., 5)
        # else:
        #     steering_wheel = np.clip(steering_wheel, -360., 360)

        steering_wheel = np.clip(steering_wheel, -360., 360)
        a_x = 2.25*a_x_norm - 0.75
        if a_x > -0.1:
            # torque = np.clip(a_x * 300., 0., 350.)
            torque = np.clip((a_x+0.1-0.4)/0.4*80+150., 0., torque_clip)
            decel = 0.
            tor_flag = 1
            dec_flag = 0
        else:
            torque = 0.
            # decel = np.clip(-a_x, 0., 4.)
            decel = a_x-0.1
            tor_flag = 0
            dec_flag = 1

        # out: steer_wheel_deg, torque, deceleration, tor_flag, dec_flag:
        # [-360,360]deg, [0., 350,]N (1), [0, 5]m/s^2 (0.05)
        return steering_wheel, torque, decel, tor_flag, dec_flag, front_wheel_deg, a_x

    def run(self):
        start_time = time.time()
        with open(self.save_path + '/record.txt', 'a') as file_handle:
            file_handle.write(str("保存时间：" + datetime.now().strftime("%Y%m%d_%H%M%S")))
            file_handle.write('\n')
            while True:
                time.sleep(0.07)
                if self.model_only_test:   # test using model
                    model_state = self.model_driven_by_model_action.get_states()[0]
                    v_x, v_y, r, x, y, phi = model_state[0], model_state[1], model_state[2],\
                                             model_state[3], model_state[4], model_state[5]
                    with self.lock:
                        state_gps = self.shared_list[0].copy()
                        time_receive_gps = self.shared_list[1]
                        state_can = self.shared_list[2].copy()
                        time_receive_can = self.shared_list[3]
                        state_other = self.shared_list[4].copy()
                        time_receive_radar = self.shared_list[5] if self.if_radar else 0.

                    state_ego = OrderedDict()
                    state_gps.update(dict(GaussX=x, GaussY=y, Heading=phi, GpsSpeed=v_x))  # only for plot online
                    state_ego.update(state_gps)
                    state_ego.update(state_can)

                    state_gps_modified_by_model = dict(v_x=v_x, v_y=v_y, r=r, x=x, y=y, phi=phi)
                    self.time_in = time.time()
                    obs, obs_dict, veh_vec = self._get_obs(state_gps_modified_by_model, state_other, model_flag=True)
                    action = self.model.run(obs)
                    steer_wheel_deg, torque, decel, tor_flag, dec_flag, front_wheel_deg, a_x = \
                        self._action_transformation_for_end2end(action, state_gps_modified_by_model, model_flag=True)
                    action = np.array([[front_wheel_deg * np.pi / 180, a_x]], dtype=np.float32)
                    state_model_in_model_action = self.model_driven_by_model_action.model_step(state_gps, 1,
                                                                                               action,
                                                                                               time.time() - start_time,
                                                                                               'model')
                    state_ego.update(state_model_in_model_action)
                    start_time = time.time()
                    control = {'Decision': {
                        'Control': {#'VehicleSpeedAim': 20/3.6,
                                    'Deceleration': decel,
                                    'Torque': torque,
                                    'Dec_flag': dec_flag,
                                    'Tor_flag': tor_flag,
                                    'SteerAngleAim': np.float64(steer_wheel_deg+1.7),
                                    'VehicleGearAim': 1,
                                    'IsValid': True}}}
                    json_cotrol = json.dumps(control)
                    self.socket_pub.send(json_cotrol.encode('utf-8'))

                    x_center_in_radar_coordi, y_center_in_radar_coordi, phi_intersection = state_ego['GaussX']+21277000., state_ego['GaussY']+3447700.-0.3, \
                                state_ego['Heading']

                    x_rear_in_radar_coordi, y_rear_in_radar_coordi, phi_intersection = convert_center_to_rear(x_center_in_radar_coordi, y_center_in_radar_coordi, phi_intersection)
                    phi_in_radar_coordi = -phi_intersection + 90

                    msg4radar = struct.pack('6d', 0., 0., 0., x_rear_in_radar_coordi, y_rear_in_radar_coordi, phi_in_radar_coordi)
                    self.socket_pub_radar.send(msg4radar)

                    time_decision = time.time() - self.time_in
                    self.run_time = time.time() - self.time_initial

                    decision = OrderedDict({'Deceleration': decel,  # [m/s^2]
                                            'Torque': torque,  # [N*m]
                                            'Dec_flag': dec_flag,
                                            'Tor_flag': tor_flag,
                                            'SteerAngleAim': steer_wheel_deg,  # [deg]
                                            'front_wheel_deg': front_wheel_deg,
                                            'a_x': a_x})  # [m/s^2]

                    with self.lock:
                        self.shared_list[6] = self.step
                        self.shared_list[7] = self.run_time
                        self.shared_list[8] = decision.copy()
                        self.shared_list[9] = state_ego.copy()
                        self.shared_list[10] = list(veh_vec)

                    self.step += 1
                else:  # real test
                    shared_index = self.receive_index_shared.value
                    if shared_index > self.read_index_old:
                        self.read_index_old = shared_index
                        with self.lock:
                            state_gps = self.shared_list[0].copy()
                            time_receive_gps = self.shared_list[1]
                            state_can = self.shared_list[2].copy()
                            time_receive_can = self.shared_list[3]
                            state_other = self.shared_list[4].copy()
                            time_receive_radar = self.shared_list[5] if self.if_radar else 0.

                        state_ego = OrderedDict()
                        state_ego.update(state_gps)
                        state_ego.update(state_can)

                        self.time_in = time.time()
                        obs, obs_dict, veh_vec = self._get_obs(state_gps, state_other, model_flag=False)
                        action = self.model.run(obs)
                        steer_wheel_deg, torque, decel, tor_flag, dec_flag, front_wheel_deg, a_x = \
                            self._action_transformation_for_end2end(action, state_gps, model_flag=False)
                        # ==============================================================================================
                        # ------------------drive model in real action---------------------------------
                        realaction4model = np.array([[front_wheel_deg*np.pi/180, a_x]], dtype=np.float32)
                        state_model_in_real_action = self.model_driven_by_real_action.model_step(state_gps, state_can['VehicleMode'],
                                                                                  realaction4model, time.time()-start_time, 'real')
                        state_ego.update(state_model_in_real_action)
                        # ------------------drive model in real action---------------------------------

                        # ------------------drive model in model action---------------------------------
                        if self.step % 5 == 0:
                            v_in_y_coord, v_in_x_coord = -state_gps['EastVelocity'], state_gps['NorthVelocity']
                            ego_phi = state_gps['Heading']
                            ego_phi_rad = ego_phi * np.pi / 180.
                            ego_vx = v_in_y_coord * np.sin(ego_phi_rad) + v_in_x_coord * np.cos(ego_phi_rad)
                            ego_vy = v_in_y_coord * np.cos(ego_phi_rad) - v_in_x_coord * np.sin(ego_phi_rad)
                            ego_vy = -ego_vy
                            ego_r, ego_x, ego_y, ego_phi = state_gps['YawRate'], state_gps['GaussX'], state_gps[
                                'GaussY'], state_gps['Heading']
                            self.model_driven_by_model_action.set_states(np.array([[ego_vx, ego_vy, ego_r, ego_x, ego_y, ego_phi]], dtype=np.float32))
                        state_driven_by_model_action = self.model_driven_by_model_action.get_states()[0]
                        v_x, v_y, r, x, y, phi = state_driven_by_model_action[0], state_driven_by_model_action[1], state_driven_by_model_action[2], \
                                                 state_driven_by_model_action[3], state_driven_by_model_action[4], state_driven_by_model_action[5]
                        state_gps_modified_by_model = dict(v_x=v_x, v_y=v_y, r=r, x=x, y=y, phi=phi)
                        obs_model, obs_dict_model, veh_vec_model = self._get_obs(state_gps_modified_by_model,
                                                                                 state_other, model_flag=True)
                        action_model = self.model.run(obs_model)
                        steer_wheel_deg_model, torque_model, decel_model, tor_flag_model, dec_flag_model, front_wheel_deg_model, a_x_model = \
                            self._action_transformation_for_end2end(action_model, state_gps_modified_by_model, model_flag=True)
                        modelaction4model = np.array([[front_wheel_deg_model*np.pi/180, a_x_model]], dtype=np.float32)
                        state_model_in_model_action = self.model_driven_by_model_action.model_step(state_gps, state_can['VehicleMode'],
                                                                                                   modelaction4model,
                                                                                                   time.time() - start_time,
                                                                                                   'model')
                        state_ego.update(state_model_in_model_action)
                        # ------------------drive model in model action---------------------------------
                        # ==============================================================================================

                        start_time = time.time()
                        # control = {'Decision': {
                        #     'Control': {#'VehicleSpeedAim': 20/3.6,
                        #                 'Deceleration': decel,
                        #                 'Torque': torque,
                        #                 'Dec_flag': dec_flag,
                        #                 'Tor_flag': tor_flag,
                        #                 'SteerAngleAim': np.float64(steer_wheel_deg+1.7),
                        #                 'VehicleGearAim': 1,
                        #                 'IsValid': True}}}
                        control = {'Decision': {
                            'Control': {  # 'VehicleSpeedAim': 20/3.6,
                                'Deceleration': decel_model,
                                'Torque': torque_model,
                                'Dec_flag': dec_flag_model,
                                'Tor_flag': tor_flag_model,
                                'SteerAngleAim': np.float64(steer_wheel_deg_model + 1.7),
                                'VehicleGearAim': 1,
                                'IsValid': True}}}
                        # control = {'Decision': {
                        #     'Control': {  # 'VehicleSpeedAim': 20/3.6,
                        #         'Deceleration': -2.0,
                        #         'Torque': 0,
                        #         'Dec_flag': 1,
                        #         'Tor_flag': 0,
                        #         'SteerAngleAim': np.float64(0. + 1.7),
                        #         'VehicleGearAim': 1,
                        #         'IsValid': True}}}
                        json_cotrol = json.dumps(control)
                        self.socket_pub.send(json_cotrol.encode('utf-8'))

                        x_center_in_radar_coordi, y_center_in_radar_coordi, phi_intersection = state_ego[
                                                                                                   'GaussX'] + 21277000., \
                                                                                               state_ego[
                                                                                                   'GaussY'] + 3447700. - 0.3, \
                                                                                               state_ego['Heading']

                        x_rear_in_radar_coordi, y_rear_in_radar_coordi, phi_intersection = convert_center_to_rear(
                            x_center_in_radar_coordi, y_center_in_radar_coordi, phi_intersection)
                        phi_in_radar_coordi = -phi_intersection + 90

                        msg4radar = struct.pack('6d', 0., 0., 0., x_rear_in_radar_coordi, y_rear_in_radar_coordi,
                                                phi_in_radar_coordi)
                        self.socket_pub_radar.send(msg4radar)

                        time_decision = time.time() - self.time_in
                        self.run_time = time.time() - self.time_initial

                        decision = OrderedDict({'Deceleration': decel_model,  # [m/s^2]
                                                'Torque': torque_model,  # [N*m]
                                                'Dec_flag': dec_flag_model,
                                                'Tor_flag': tor_flag_model,
                                                'SteerAngleAim': steer_wheel_deg_model,  # [deg]
                                                'front_wheel_deg': front_wheel_deg_model,
                                                'a_x': a_x_model})  # [m/s^2]

                        with self.lock:
                            self.shared_list[6] = self.step
                            self.shared_list[7] = self.run_time
                            self.shared_list[8] = decision.copy()
                            self.shared_list[9] = state_ego.copy()
                            self.shared_list[10] = list(veh_vec)

                        self.step += 1

                if self.if_save:
                    if decision != {} and state_ego != {} and state_other != {}:
                        file_handle.write("Decision ")
                        for k1, v1 in decision.items():
                            file_handle.write(k1 + ":" + str(v1) + ", ")
                        file_handle.write('\n')

                        file_handle.write("State_ego ")
                        for k2, v2 in state_ego.items():
                            file_handle.write(k2 + ":" + str(v2) + ", ")
                        file_handle.write('\n')

                        file_handle.write("State_other ")
                        for k3, v3 in state_other.items():
                            file_handle.write(k3 + ":" + str(v3) + "| ")
                        file_handle.write('\n')

                        file_handle.write("Obs_dict ")
                        for k4, v4 in obs_dict.items():
                            file_handle.write(k4 + ":" + str(v4) + ", ")
                        file_handle.write('\n')

                        file_handle.write("Time Time:" + str(self.run_time) + ", " +
                                          "time_decision:"+str(time_decision) + ", " +
                                          "time_receive_gps:"+str(time_receive_gps) + ", " +
                                          "time_receive_can:"+str(time_receive_can) + ", " +
                                          "time_receive_radar:"+str(time_receive_radar)+ ", " + '\n')


if __name__ == "__main__":
    pass