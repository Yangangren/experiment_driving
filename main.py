#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# =====================================
# @Time    : 2020/12/27
# @Author  : Yangang Ren (Tsinghua Univ.)
# @FileName: Online application of trained network.py
# =====================================

from __future__ import print_function

import argparse
import json
import multiprocessing as mp
import os
import time
from datetime import datetime
from multiprocessing import Process

from controller import Controller
from plot_online import Plot
from subscriber_can import SubscriberCan
from subscriber_gps import SubscriberGps
from subscriber_radar import SubscriberRadar
from traffic import Traffic

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'


def controller_agent(shared_list, receive_index, if_save, if_radar,
                     lock, task, case, noise_factor, load_dir, load_ite,
                     result_dir, model_only_test):
    publisher_ = Controller(shared_list, receive_index, if_save, if_radar,
                            lock, task, case, noise_factor, load_dir, load_ite,
                            result_dir, model_only_test)
    time.sleep(0.5)
    publisher_.run()


def subscriber_can_agent(shared_list, receive_index, lock):
    subscriber_ = SubscriberCan(shared_list, receive_index, lock)
    subscriber_.run()


def subscriber_gps_agent(shared_list, receive_index, lock):
    subscriber_ = SubscriberGps(shared_list, receive_index, lock)
    subscriber_.run()


def traffic(shared_list, lock, task, case, surr_flag):
    subscriber_ = Traffic(shared_list, lock, task, case, surr_flag)
    subscriber_.run()


def subscriber_radar_agent(shared_list, lock):
    subscriber_radar = SubscriberRadar(shared_list, lock)
    subscriber_radar.run()


def plot_agent(shared_list, lock, task, model_only_test):
    plot_ = Plot(shared_list, lock, task, model_only_test)
    time.sleep(3)
    plot_.run()


def built_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('--task', type=str, default='left')
    parser.add_argument('--case', type=int, default=0)
    parser.add_argument('--if_save', type=bool, default=True)
    parser.add_argument('--if_radar', type=bool, default=False)
    task = parser.parse_args().task
    case = parser.parse_args().case
    parser.add_argument('--load_dir', type=str, default='./utils/models/{}/experiment-2021-01-05-01-12-20'.format(task))
    parser.add_argument('--load_ite', type=str, default=100000)
    parser.add_argument('--noise_factor', type=float, default=1.)
    parser.add_argument('--surr_flag', type=bool, default=False)
    parser.add_argument('--model_only_test', type=bool, default=False)
    parser.add_argument('--backup', type=str, default='abso_POLICY: add_random init:0103_model_parameters CLIP TORQUE TO 250: CANCEL inertia: '
    'debug vehicle dynamics, modify done position: add noise in all states: change phi init whole traj 100000')
    model_only_test = parser.parse_args().model_only_test
    flag = 'model' if model_only_test else 'real'
    noise = int(parser.parse_args().noise_factor)
    result_dir = './record/{task}/case{case}/noise{noise}/{time}_{flag}'.format(task=task,
                                                                                case=case,
                                                                                noise=noise,
                                                                                time=datetime.now().strftime("%Y%m%d_%H%M%S"),
                                                                                flag=flag)
    parser.add_argument('--result_dir', type=str, default=result_dir)
    return parser.parse_args()


def main():
    args = built_parser()
    os.makedirs(args.result_dir)
    with open(args.result_dir + '/config.json', 'w', encoding='utf-8') as f:
        json.dump(vars(args), f, ensure_ascii=False, indent=4)
    shared_list = mp.Manager().list([0.]*11)
    # [state_gps, time_gps, state_can, time_can, state_other, time_radar,
    #  step, runtime, decision, state_ego, obs_vec]

    # state_gps['GaussX'] = 0   # intersection coordinate [m]
    # state_gps['GaussY'] = 0   # intersection coordinate [m]
    # state_gps['Heading'] = 0  # intersection coordinate [deg]
    # state_gps['GpsSpeed'] = 0          # [m/s]
    # state_gps['NorthVelocity'] = 0     # [m/s]
    # state_gps['EastVelocity'] = 0      # [m/s]
    # state_gps['YawRate'] = 0           # [rad/s]
    # state_gps['LongitudinalAcc'] = 0   # [m/s^2]
    # state_gps['LateralAcc'] = 0        # [m/s^2]
    # state_gps['Longitude'] = 0
    # state_gps['Latitude'] = 0

    # state_can['VehicleSPeedAct'] = 0  # [m/s]
    # state_can['SteerAngleAct'] = 0    # [m/s]
    # state_can['AutoGear'] = 0
    # state_can['VehicleMode'] = 0      # 0: manual driving; 1: autonomous driving
    # state_can['Throttle'] = 0
    # state_can['BrkOn'] = 0

    # state_other: dict(x_other=[],  # intersection coordination
    #                   y_other=[],  # intersection coordination
    #                   v_other=[],
    #                   phi_other=[],  # intersection coordination
    #                   v_light=0)

    # decision: {'Deceleration': decel,  # [m/s^2]
    #             'Torque': torque,  # [N*m]
    #             'Dec_flag': dec_flag,
    #             'Tor_flag': tor_flag,
    #             'SteerAngleAim': steer_wheel_deg,  # [deg]
    #             'front_wheel_deg': front_wheel_deg,
    #             'a_x': a_x})  # [m/s^2]

    receive_index = mp.Value('d', 0.0)
    lock = mp.Lock()
    procs = [Process(target=subscriber_gps_agent, args=(shared_list, receive_index, lock)),
             Process(target=subscriber_can_agent, args=(shared_list, receive_index, lock))]

    if args.if_radar:
        procs.append(Process(target=subscriber_radar_agent, args=(shared_list, lock)))
    else:
        procs.append(Process(target=traffic, args=(shared_list, lock, args.task, args.case, args.surr_flag)))
    procs.append(Process(target=controller_agent, args=(shared_list, receive_index, args.if_save, args.if_radar, lock,
                                                        args.task, args.case, args.noise_factor, args.load_dir,
                                                        args.load_ite, args.result_dir, args.model_only_test)))
    procs.append(Process(target=plot_agent, args=(shared_list, lock, args.task, args.model_only_test)))

    for p in procs:
        p.start()
    for p in procs:
        p.join()


if __name__ == '__main__':
    os.environ["OMP_NUM_THREADS"] = "1"
    main()





