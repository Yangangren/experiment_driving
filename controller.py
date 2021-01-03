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

from traffic import TRAFFICSETTINGS
from utils.load_policy import LoadPolicy

VEHICLE_MODE_DICT = dict(left=OrderedDict(dl=1, du=1, dr=1, ud=2, ul=1), # dl=2, du=2, ud=2, ul=2
                         straight=OrderedDict(dl=1, du=1, dr=1, ud=1, ru=2, ur=2), #vdl=1, du=2, ud=2, ru=2, ur=2
                         right=OrderedDict(dl=1, du=1, dr=1, ur=2, lr=2)) #TODO: temp relevant to filter interested vehicle


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

EXPECTED_V = 4.
START_OFFSET = 3.0


def deal_with_phi_diff(phi_diff):
    phi_diff = tf.where(phi_diff > 180., phi_diff - 360., phi_diff)
    phi_diff = tf.where(phi_diff < -180., phi_diff + 360., phi_diff)
    return phi_diff


class VehicleDynamics(object):
    def __init__(self, ):
        self.vehicle_params = dict(C_f=-128915.5,  # front wheel cornering stiffness [N/rad]
                                   C_r=-85943.6,  # rear wheel cornering stiffness [N/rad]
                                   a=1.06,  # distance from CG to front axle [m]
                                   b=1.85,  # distance from CG to rear axle [m]
                                   mass=1412.,  # mass [kg]
                                   I_z=1536.7,  # Polar moment of inertia at CG [kg*m^2]
                                   miu=1.0,  # tire-road friction coefficient
                                   g=9.81,  # acceleration of gravity [m/s^2]
                                   )
        a, b, mass, g = self.vehicle_params['a'], self.vehicle_params['b'], \
                        self.vehicle_params['mass'], self.vehicle_params['g']
        F_zf, F_zr = b * mass * g / (a + b), a * mass * g / (a + b)
        self.vehicle_params.update(dict(F_zf=F_zf,
                                        F_zr=F_zr))

    def f_xu(self, states, actions, tau):  # states and actions are tensors, [[], [], ...]
        v_x, v_y, r, x, y, phi = states[:, 0], states[:, 1], states[:, 2], states[:, 3], states[:, 4], states[:, 5]
        phi = phi * np.pi / 180.
        steer, a_x = actions[:, 0], actions[:, 1]
        C_f = tf.convert_to_tensor(self.vehicle_params['C_f'], dtype=tf.float32)
        C_r = tf.convert_to_tensor(self.vehicle_params['C_r'], dtype=tf.float32)
        a = tf.convert_to_tensor(self.vehicle_params['a'], dtype=tf.float32)
        b = tf.convert_to_tensor(self.vehicle_params['b'], dtype=tf.float32)
        mass = tf.convert_to_tensor(self.vehicle_params['mass'], dtype=tf.float32)
        I_z = tf.convert_to_tensor(self.vehicle_params['I_z'], dtype=tf.float32)
        miu = tf.convert_to_tensor(self.vehicle_params['miu'], dtype=tf.float32)
        g = tf.convert_to_tensor(self.vehicle_params['g'], dtype=tf.float32)

        F_zf, F_zr = b * mass * g / (a + b), a * mass * g / (a + b)
        F_xf = tf.where(a_x < 0, mass * a_x / 2, tf.zeros_like(a_x))
        F_xr = tf.where(a_x < 0, mass * a_x / 2, mass * a_x)

        next_state = [v_x + tau * (a_x + v_y * r),
                      (mass * v_y * v_x + tau * (
                                  a * C_f - b * C_r) * r - tau * C_f * steer * v_x - tau * mass * tf.square(
                          v_x) * r) / (mass * v_x - tau * (C_f + C_r)),
                      (-I_z * r * v_x - tau * (a * C_f - b * C_r) * v_y + tau * a * C_f * steer * v_x) / (
                                  tau * (tf.square(a) * C_f + tf.square(b) * C_r) - I_z * v_x),
                      x + tau * (v_x * tf.cos(phi) - v_y * tf.sin(phi)),
                      y + tau * (v_x * tf.sin(phi) + v_y * tf.cos(phi)),
                      (phi + tau * r) * 180 / np.pi]

        return tf.stack(next_state, 1)

    def prediction(self, x_1, u_1, tau):
        x_next = self.f_xu(x_1, u_1, tau)
        return x_next.numpy()


class VehicleDynamics1(object):
    def __init__(self, ):
        self.vehicle_params = dict(C_f=-128915.5,  # front wheel cornering stiffness [N/rad]
                                   C_r=-85943.6,  # rear wheel cornering stiffness [N/rad]
                                   a=1.06,  # distance from CG to front axle [m]
                                   b=1.85,  # distance from CG to rear axle [m]
                                   mass=1412.,  # mass [kg]
                                   I_z=1536.7,  # Polar moment of inertia at CG [kg*m^2]
                                   miu=1.0,  # tire-road friction coefficient
                                   g=9.81,  # acceleration of gravity [m/s^2]
                                   )
        a, b, mass, g = self.vehicle_params['a'], self.vehicle_params['b'], \
                        self.vehicle_params['mass'], self.vehicle_params['g']
        F_zf, F_zr = b * mass * g / (a + b), a * mass * g / (a + b)
        self.vehicle_params.update(dict(F_zf=F_zf,
                                        F_zr=F_zr))
        self.states = np.array([[3., 0., 0., 1.75, -40., 90.]], dtype=np.float32)
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

        next_state = [v_x + tau * (a_x + v_y * r),
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


class ReferencePath(object):
    def __init__(self, task, mode=None, ref_index=None):
        self.mode = mode
        self.traj_mode = None
        self.exp_v = EXPECTED_V #TODO: temp
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
        control_ext = CROSSROAD_SIZE/3. #TODO: temp
        if task == 'left':
            end_offsets = [LANE_WIDTH*(i+0.5) for i in range(LANE_NUMBER)] #TODO: temp
            start_offsets = [LANE_WIDTH*0.5] #TODO: temp
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
                    start_straight_line_x = LANE_WIDTH/2 * np.ones(shape=(sl * meter_pointnum_ratio,), dtype=np.float32)[:-1]
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

    def find_closest_point(self, xs, ys, ratio=6): #TODO: temp ratio yasuobili
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
                 noise_factor, load_dir, load_ite, result_dir):
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
        self.model_driven_by_can = VehicleDynamics()
        self.model_driven_by_own = VehicleDynamics1()
        # self.model_state = np.array([[3., 0., 0., 1.75, -30., 90.]], dtype=np.float32)

    def model_step(self, state_gps, state_can, delta_t):
        speed = state_gps['GpsSpeed']
        steering_wheel = state_can['SteerAngleAct']
        front_wheel = steering_wheel/self.steer_factor *np.pi/180.
        acc = 0.
        u = np.array([[front_wheel, acc]], dtype=np.float32)
        self.model_state = self.model_driven_by_can.prediction(self.model_state, u, delta_t)
        self.model_state[0][0] = speed
        v_x, v_y, r, x, y, phi = self.model_state[0][0], self.model_state[0][1], self.model_state[0][2], \
                                 self.model_state[0][3], self.model_state[0][4], self.model_state[0][5]
        return OrderedDict(model_vx=v_x, model_vy=v_y, model_r=r, model_x=x, model_y=y, model_phi=phi)

    def _construct_ego_vector(self, state_gps):
        ego_phi = state_gps['Heading']
        ego_x, ego_y = state_gps['GaussX'], state_gps['GaussY']
        ego_v_x, ego_v_y = state_gps['GpsSpeed'], 0.
        ego_r = state_gps['YawRate']                      # rad/s
        self.ego_info_dim = 6
        ego_feature = [ego_v_x, ego_v_y, ego_r, ego_x, ego_y, ego_phi]
        return np.array(ego_feature, dtype=np.float32)

    def _construct_veh_vector(self, ego_x, ego_y, state_others): #TODO: temp mht
        mode_list = list(TRAFFICSETTINGS[self.task][self.case]['others'].keys())
        all_vehicles = []
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

    def convert_vehs_to_rela(self, obs_abso):
        ego_infos, tracking_infos, veh_infos = obs_abso[:self.ego_info_dim], \
                                               obs_abso[self.ego_info_dim:self.ego_info_dim + self.per_tracking_info_dim * (
                                                         self.num_future_data + 1)], \
                                               obs_abso[self.ego_info_dim + self.per_tracking_info_dim * (
                                                           self.num_future_data + 1):]
        _, _, _, ego_x, ego_y, _ = ego_infos
        ego = np.array([ego_x, ego_y, 0, 0]*int(len(veh_infos)/self.per_veh_info_dim), dtype=np.float32)
        vehs_rela = veh_infos - ego
        out = np.concatenate((ego_infos, tracking_infos, vehs_rela), axis=0)
        return out

    def _get_obs(self, state_gps, state_others):
        ego_x, ego_y = state_gps['GaussX'], state_gps['GaussY']
        ego_phi = state_gps['Heading']
        ego_v_x, ego_v_y = state_gps['GpsSpeed'], 0.
        vehs_vector = self._construct_veh_vector(ego_x, ego_y, state_others)
        ego_vector = self._construct_ego_vector(state_gps)
        tracking_error = self.ref_path.tracking_error_vector(np.array([ego_x], dtype=np.float32),
                                                             np.array([ego_y], dtype=np.float32),
                                                             np.array([ego_phi], dtype=np.float32),
                                                             np.array([ego_v_x], dtype=np.float32),
                                                             self.num_future_data).numpy()[0]
        self.per_tracking_info_dim = 3
        vector = np.concatenate((ego_vector, tracking_error, vehs_vector), axis=0)
        veh_idx_start = self.ego_info_dim + self.per_tracking_info_dim * (self.num_future_data + 1)
        vector = self.convert_vehs_to_rela(vector)
        vehs_vector_rela = vector[veh_idx_start:]

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
            obs_dict.update({'other{}_delta_x'.format(i): vehs_vector_rela[self.per_veh_info_dim*i],
                             'other{}_delta_y'.format(i): vehs_vector_rela[self.per_veh_info_dim*i+1],
                             'other{}_delta_v'.format(i): vehs_vector_rela[self.per_veh_info_dim*i+2],
                             'other{}_delta_phi'.format(i): vehs_vector_rela[self.per_veh_info_dim*i+3]})
        return vector_with_noise, obs_dict, vehs_vector  # todo: if output vector without noise

    def _set_inertia(self, steer_from_policy, inertia_time=0.2, sampletime=0.1, k_G=1.): # todo: adjust the inertia time
        steer_output = (1. - sampletime / inertia_time) * self.last_steer_output + \
                       k_G * sampletime / inertia_time * steer_from_policy

        self.last_steer_output = steer_output
        return steer_output

    def _action_transformation_for_end2end(self, action):  # [-1, 1] # TODO: wait real car
        action = np.clip(action, -1.0, 1.0)
        front_wheel_norm_rad, a_x_norm = action[0], action[1]
        front_wheel_deg = 0.4 / pi * 180 * front_wheel_norm_rad
        steering_wheel = front_wheel_deg * self.steer_factor
        # steering_wheel = self._set_inertia(steering_wheel)

        steering_wheel = np.clip(steering_wheel, -360., 360)
        a_x = 2.25*a_x_norm - 0.75
        if a_x > 0:
            # torque = np.clip(a_x * 300., 0., 350.)
            torque = np.clip((a_x-0.4)/0.4*50+150., 0., 250.)  #todo
            decel = 0.
            tor_flag = 1
            dec_flag = 0
        else:
            torque = 0.
            # decel = np.clip(-a_x, 0., 4.)
            decel = -np.clip(-a_x, 0., 3.)
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
                if True:   # test using model
                    state_model = self.model_driven_by_own.states.numpy()[0]
                    v_x, v_y, r, x, y, phi = state_model[0], state_model[1], state_model[2],\
                                             state_model[3], state_model[4], state_model[5]
                    with self.lock:
                        state_gps = self.shared_list[0].copy()
                        time_receive_gps = self.shared_list[1]
                        state_can = self.shared_list[2].copy()
                        time_receive_can = self.shared_list[3]
                        state_other = self.shared_list[4].copy()
                        time_receive_radar = self.shared_list[5] if self.if_radar else 0.

                    state_gps.update(GaussX=x, GaussY=y, Heading=phi, GpsSpeed=v_x, YawRate=r)
                    state_ego = OrderedDict()
                    state_ego.update(state_gps)
                    state_ego.update(state_can)

                    self.time_in = time.time()
                    obs, obs_dict, veh_vec = self._get_obs(state_gps, state_other)
                    action = self.model.run(obs)
                    steer_wheel_deg, torque, decel, tor_flag, dec_flag, front_wheel_deg, a_x = \
                        self._action_transformation_for_end2end(action)
                    self.model_driven_by_own.prediction(np.array([[front_wheel_deg*np.pi/180, a_x]]), time.time()-start_time)
                    start_time = time.time()
                    # state_model = self.model_step(state_gps, state_can, delta_t)
                    # state_ego.update(state_model)
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

                    x, y, phi = state_ego['GaussX']+21277000., state_ego['GaussY']+3447700., \
                                -state_ego['Heading'] + 90
                    msg4radar = struct.pack('6d', 0., 0., 0., x, y, phi)
                    self.socket_pub_radar.send(msg4radar)

                    time_decision = time.time() - self.time_in
                    run_time = time.time() - self.time_initial

                    decision = OrderedDict({'Deceleration': decel,  # [m/s^2]
                                            'Torque': torque,  # [N*m]
                                            'Dec_flag': dec_flag,
                                            'Tor_flag': tor_flag,
                                            'SteerAngleAim': steer_wheel_deg,  # [deg]
                                            'front_wheel_deg': front_wheel_deg,
                                            'a_x': a_x})  # [m/s^2]

                    with self.lock:
                        self.shared_list[6] = self.step
                        self.shared_list[7] = run_time
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
                        obs, obs_dict, veh_vec = self._get_obs(state_gps, state_other)
                        action = self.model.run(obs)
                        steer_wheel_deg, torque, decel, tor_flag, dec_flag, front_wheel_deg, a_x = \
                            self._action_transformation_for_end2end(action)
                        # state_model = self.model_step(state_gps, state_can, delta_t)
                        # state_ego.update(state_model)
                        control = {'Decision': {
                            'Control': {#'VehicleSpeedAim': 20/3.6,
                                        'Deceleration': decel,
                                        'Torque': torque,
                                        'Dec_flag': dec_flag,
                                        'Tor_flag': tor_flag,
                                        'SteerAngleAim': np.float64(steer_wheel_deg+1.7),
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

                        x, y, phi = state_ego['GaussX']+21277000., state_ego['GaussY']+3447700., \
                                    -state_ego['Heading'] + 90
                        msg4radar = struct.pack('6d', 0., 0., 0., x, y, phi)
                        self.socket_pub_radar.send(msg4radar)

                        time_decision = time.time() - self.time_in
                        run_time = time.time() - self.time_initial

                        decision = OrderedDict({'Deceleration': decel,  # [m/s^2]
                                                'Torque': torque,  # [N*m]
                                                'Dec_flag': dec_flag,
                                                'Tor_flag': tor_flag,
                                                'SteerAngleAim': steer_wheel_deg,  # [deg]
                                                'front_wheel_deg': front_wheel_deg,
                                                'a_x': a_x})  # [m/s^2]

                        with self.lock:
                            self.shared_list[6] = self.step
                            self.shared_list[7] = run_time
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
                            file_handle.write(k3 + ":" + str(v3) + ", ")
                        file_handle.write('\n')

                        file_handle.write("Obs_dict ")
                        for k4, v4 in obs_dict.items():
                            file_handle.write(k4 + ":" + str(v4) + ", ")
                        file_handle.write('\n')

                        file_handle.write("Time Time:" + str(run_time)  + ", " +
                                          "time_decision:"+str(time_decision) + ", " +
                                          "time_receive_gps:"+str(time_receive_gps) + ", " +
                                          "time_receive_can:"+str(time_receive_can) + ", " +
                                          "time_receive_radar:"+str(time_receive_radar)+ ", " + '\n')


def test_control():
    task, case = 'left', 0
    controller = Controller([], [0], [], [], [], [], [], task, case)
    state_gps = dict(GaussX=3.5/2,
                     GaussY=-2,
                     GpsSpeed=3,
                     Heading=90.,
                     YawRate=0.,
                     )
    x_other, y_other, v_other, phi_other = [], [], [], []
    for key, val in TRAFFICSETTINGS[task][case]['others'].items():
        x_other.append(val['x'])
        y_other.append(val['y'])
        v_other.append(val['v'])
        phi_other.append(val['phi'])
    state_other = dict(x_other=x_other,
                       y_other=y_other,
                       v_other=v_other,
                       phi_other=phi_other,
                       v_light=0)
    obs = controller._get_obs(state_gps, state_other)
    # print(obs)


if __name__ == "__main__":
    test_control()
