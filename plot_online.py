import matplotlib.pyplot as plt
import pandas as pd
from Application_final import load_map
import re
import numpy as np
import seaborn as sns
import os
import matplotlib.patches as mpathes
import math

class Plot():
    def __init__(self,Info_List,lock):
        self.Info_List = Info_List
        self.lock = lock
        self.step_old = -1
        step = []
    def run(self):
        plt.figure(0)
        plt.ion()
        lane_list, lane_center_list, road_angle = load_map()
        plt.figsize = (20, 5)
        plt.title("X-Y")
        ax = plt.gca()
        ax.set_aspect('equal')
        ax.invert_xaxis()
        ax.invert_yaxis()
        for i in range(len(lane_list)):
            plt.plot(lane_list[i][:, 0], lane_list[i][:, 1], color='green', linewidth='2')
        for i in range(len(lane_center_list)):
            plt.plot(lane_center_list[i][:, 0], lane_center_list[i][:, 1], color='red', linewidth='2')

        while True:
            Info_List = self.Info_List
            self.step = Info_List[0]
            if self.step > self.step_old:
                self.step_old= self.step
                with self.lock:
                    Time  = Info_List[1]
                    Decision = Info_List[2].copy()
                    State_ego = Info_List[3].copy()
                    State_Other = Info_List[4].copy()

                # 轨迹显示
                plt.scatter(State_ego['GaussX'], State_ego['GaussY'], color='blue')
                ego_show=mpathes.Rectangle([State_ego['GaussX']-4.5/2, State_ego['GaussY']-1.855/2], 4.5, 1.855,270-State_ego['Heading'], fc='magenta', ec='magenta',alpha = 0.1)
                ax.add_patch(ego_show)

                # plt.plot([State_ego['GaussX']-100,State_ego['GaussX']-100],[State_ego['GaussY']-5,State_ego['GaussY']+5])
                # plt.plot([State_ego['GaussX']+80,State_ego['GaussX']+80],[State_ego['GaussY']-5,State_ego['GaussY']+5])
                # plt.plot([State_ego['GaussX'] - 100, State_ego['GaussX'] + 80],
                #          [State_ego['GaussY'] +5, State_ego['GaussY'] + 5])
                # plt.plot([State_ego['GaussX'] -100, State_ego['GaussX'] + 80],
                #          [State_ego['GaussY'] - 5, State_ego['GaussY'] -5])

                for i in range(len(State_Other['x_other'])):
                    x =  State_Other['x_other'][i]
                    y = State_Other['y_other'][i]
                    heading_next=State_Other['heading_next'][i]
                    other_show=mpathes.Rectangle([x-5.169/2, y-2.392/2], 5.169, 2.392, 270-heading_next, fc='cadetblue', ec='cadetblue', alpha=0.01)
                    ax.add_patch(other_show)
                    plt.scatter(x, y, color='blue')


                plt.pause(0.01)
                # plt.clf()



