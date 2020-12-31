#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# =====================================
# @Time    : 2020/12/27
# @Author  : Yangang Ren (Tsinghua Univ.)
# @FileName: Online application of trained network.py
# =====================================

from __future__ import print_function
import multiprocessing as mp
from multiprocessing import Process, Queue
import os
import time
from controller import Controller
from subscriber_can import SubscriberCan
from subscriber_gps import SubscriberGps
from subscriber_radar import SubscriberRadar
from traffic import Traffic
from plot_online import Plot
import argparse

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'


def controller_agent(shared_list, Info_List, State_Other_List,receive_index,
                     if_save,if_radar,lock, task, case):
    publisher_ = Controller(shared_list,Info_List, State_Other_List,receive_index,
                            if_save,if_radar,lock,  task, case)
    time.sleep(0.5)
    publisher_.run()


def subscriber_can_agent(shared_list, Info_List, receive_index,lock):
    subscriber_ = SubscriberCan(shared_list,Info_List,receive_index,lock)
    subscriber_.run()


def subscriber_gps_agent(shared_list, Info_List, receive_index, lock):
    subscriber_ = SubscriberGps(shared_list, Info_List, receive_index, lock)
    subscriber_.run()


def traffic(shared_list, Info_List, lock, task, case):
    subscriber_ = Traffic(shared_list, Info_List, lock, task, case)
    subscriber_.run()


def subscriber_radar_agent(shared_list,State_Other_List,lock):
    subscriber_radar = SubscriberRadar(shared_list,State_Other_List,lock)
    subscriber_radar.run()


def plot_agent(Info_List,lock, task):
    plot_ = Plot(Info_List,lock, task)
    time.sleep(2)
    plot_.run()


def built_parser():
    parser = argparse.ArgumentParser()

    parser.add_argument('--task', type=str, default='left')
    parser.add_argument('--case', type=int, default=0)
    return parser.parse_args()


def main():
    args = built_parser()
    if_save = True
    if_radar = True # True: with digital twin system

    shared_list = mp.Manager().list([0]*5)
    # [state_gps, state_can, time_gps, time_can, time_radar]
    # state_gps: State_gps['GaussX'] = 0            # intersection coordinate [m]
    #            State_gps['GaussY'] = 0            # intersection coordinate [m]
    #            State_gps['Heading'] = 0           # intersection coordinate [deg]
    #            State_gps['GpsSpeed'] = 0          # [m/s]
    #            State_gps['NorthVelocity'] = 0     # [m/s]
    #            State_gps['EastVelocity'] = 0      # [m/s]
    #            State_gps['YawRate'] = 0           # [rad/s]
    #            State_gps['LongitudinalAcc'] = 0   # [m/s^2]
    #            State_gps['LateralAcc'] = 0        # [m/s^2]
    #            State_gps['Longitude'] = 0
    #            State_gps['Latitude'] = 0

    # state_can: State_can['VehicleSPeedAct'] = 0   # [m/s]
    #            State_can['SteerAngleAct'] = 0     # [m/s]
    #            State_can['AutoGear'] = 0
    #            State_can['VehicleMode'] = 0
    #            State_can['Throttle'] = 0
    #            State_can['BrkOn'] = 0

    Info_List = mp.Manager().list([0.0]*5)  # [step, time, decision, state_ego(state_can+state_gps), state_other]
    # decision: {'Deceleration': decel,  # [m/s^2]
    #            'Torque': torque,  # [N*m]
    #            'Dec_flag': dec_flag,
    #            'Tor_flag': tor_flag,
    #            'SteerAngleAim': steer_wheel_deg,  # [deg]
    #            'front_wheel_rad': front_wheel_rad,  # [rad]
    #            'a_x': a_x}

    # state_other: dict(x_other=[],  # intersection coordination
    #                   y_other=[],  # intersection coordination
    #                   v_other=[],
    #                   phi_other=[],  # intersection coordination
    #                   v_light=0)

    State_Other_List =mp.Manager().list([0]*1)  # [state_other]
    receive_index = mp.Value('d', 0.0)
    lock = mp.Lock()

    procs = []
    procs.append(Process(target=subscriber_gps_agent, args=(shared_list, Info_List, receive_index, lock)))
    procs.append(Process(target=subscriber_can_agent, args=(shared_list, Info_List, receive_index, lock)))

    if if_radar:
        procs.append(Process(target=subscriber_radar_agent, args=(shared_list, State_Other_List, lock)))
    else:
        procs.append(Process(target=traffic, args=(shared_list, State_Other_List, lock, args.task, args.case)))
    procs.append(Process(target=controller_agent, args=(shared_list,Info_List,State_Other_List,receive_index,
                                                        if_save,if_radar,lock, args.task, args.case)))
    procs.append(Process(target=plot_agent, args=(Info_List,lock, args.task)))

    for p in procs:
        p.start()
    for p in procs:
        p.join()


if __name__ == '__main__':
    os.environ["OMP_NUM_THREADS"] = "1"
    main()





