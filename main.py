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


def traffic(shared_list, Info_List, receive_index, lock, task, case):
    subscriber_ = Traffic(shared_list, Info_List, receive_index, lock, task, case)
    subscriber_.run()


def subscriber_radar_agent(shared_list,State_Other_List,lock):
    subscriber_radar = SubscriberRadar(shared_list,State_Other_List,lock)
    subscriber_radar.run()


def plot_agent(Info_List,lock, task):
    plot_ = Plot(Info_List,lock, task)
    time.sleep(1)
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

    shared_list = mp.Manager().list([0]*4)
    Info_List = mp.Manager().list([0.0]*8)
    State_Other_List =mp.Manager().list([0]*1)
    receive_index = mp.Value('d',0.0)
    lock = mp.Lock()
    procs=[]
    procs.append(Process(target=subscriber_can_agent, args=(shared_list, Info_List, receive_index, lock)))
    procs.append(Process(target=subscriber_gps_agent, args=(shared_list, Info_List, receive_index, lock)))

    if if_radar:
        procs.append(Process(target=subscriber_radar_agent, args=(shared_list, State_Other_List, lock)))
    else:
        procs.append(Process(target=traffic, args=(State_Other_List, lock, args.task, args.case)))
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





