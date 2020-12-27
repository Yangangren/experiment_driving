from __future__ import print_function
import numpy as np
import torch
import torch.multiprocessing as mp
from torch.multiprocessing import Process, Queue
import os
import time
from publisher import Publisher
from subscriber import Subscriber
from subscriber_radar import Subscriber_Radar
from plot_online import Plot


def publisher_agent(shared_list,Info_List,State_Other_List,receive_index,if_save,if_radar,lock):
    publisher_=Publisher(shared_list,Info_List,State_Other_List,receive_index,if_save,if_radar,lock)
    time.sleep(0.5)
    publisher_.run()


def subscriber_agent(shared_list,Info_List,receive_index,lock):
    subscriber_ = Subscriber(shared_list,Info_List,receive_index,lock)
    subscriber_.run()


def subscriber_radar_agent(shared_list,State_Other_List,lock):
    subscriber_radar = Subscriber_Radar(shared_list,State_Other_List,lock)
    subscriber_radar.run()


def plot_agent(Info_List,lock):
    plot_ = Plot(Info_List,lock)
    time.sleep(1)
    plot_.run()


def main():
    # print("save log or not")
    # Input = input()
    # if Input == 'yes' or 'YES':
    #     if_save = True
    # else:
    #     if_save = False
    #
    # print("if radar")
    # Radar = input()
    # if Radar == 'yes' or 'YES':
    #     if_radar = True
    # else:
    #     if_radar = False
    if_save = True
    if_radar = True

    shared_list = mp.Manager().list([0]*4)
    Info_List = mp.Manager().list([0.0]*8)
    State_Other_List =mp.Manager().list([0]*1)
    receive_index = mp.Value('d',0.0)
    lock = mp.Lock()
    procs=[]
    procs.append(Process(target=subscriber_agent, args=(shared_list, Info_List, receive_index, lock)))
    if if_radar:
        procs.append(Process(target=subscriber_radar_agent, args=(shared_list, State_Other_List, lock)))
    procs.append(Process(target=publisher_agent, args=(shared_list,Info_List,State_Other_List,receive_index,if_save,if_radar,lock)))

    procs.append(Process(target=plot_agent, args=(Info_List,lock)))
    for p in procs:
        p.start()
    for p in procs:
        p.join()


if __name__ == '__main__':
    os.environ["OMP_NUM_THREADS"] = "1"
    main()





