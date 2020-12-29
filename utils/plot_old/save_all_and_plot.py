import matplotlib.pyplot as plt
from Application_final import load_map
import re
import numpy as np
import seaborn as sns
import matplotlib.patches as mpathes
from matplotlib import cm
import matplotlib as mpl
import csv
import pandas as pd
import itertools
import matplotlib.font_manager as fm
import math
zhfont1 = fm.FontProperties(fname='D:/installation-directory/Anaconda/Library/SimSun.ttf')

# 获取文件的内容
class save_all():
    def __init__(self):
        self.root = 'D:/Troy.Z/THU/iDlab/1_小车测试/4_苏州实验/实验数据/experiment20200819/'
        self.path_s1_mode1 = self.root + 'scenario1/mode1/record.txt'
        self.path_s1_mode2 = self.root + 'scenario1/mode2/record.txt'
        self.path_s1_mode3 = self.root + 'scenario1/mode3/record.txt'
        self.path_s1_mode4 = self.root + 'scenario1/mode4/record.txt'
        self.path_s1_mode5 = self.root + 'scenario1/mode5/record.txt'
        self.path_s1_mode6 = self.root + 'scenario1/mode6/record.txt'
        self.path_s1_mode7 = self.root + 'scenario1/mode7/record.txt'
        self.path_s1_mode8 = self.root + 'scenario1/mode8/record.txt'
        self.path_s1_mode9 = self.root + 'scenario1/mode9/record.txt'
        self.path_s1_mode10 = self.root + 'scenario1/mode10/record.txt'
        self.path_s1_mode11 = self.root + 'scenario1/mode11/record.txt'
        self.path_s2_mode1 = self.root + 'scenario2/mode1/record.txt'
        self.path_s2_mode2 = self.root + 'scenario2/mode2/record.txt'
        self.path_s2_mode3 = self.root + 'scenario2/mode3/record.txt'
        self.path_s2_mode4 = self.root + 'scenario2/mode4/record.txt'
        self.path_s2_mode5 = self.root + 'scenario2/mode5/record.txt'
        self.path_s2_mode6 = self.root + 'scenario2/mode6/record.txt'
        self.path_s2_mode7 = self.root + 'scenario2/mode7/record.txt'
        self.path_s2_mode8 = self.root + 'scenario2/mode8/record.txt'
        self.path_s2_mode9 = self.root + 'scenario2/mode9/record.txt'
        self.mode_all=[]
        self.time_all=[]
        self.steerangle_all=[]
        self.speed_all=[]
        self.dist2center_all=[]
        self.dist2road_bound_1_all=[]
        self.dist2road_bound_2_all=[]
        self.relate_x_all=[]
        self.relate_y_all=[]
        self.yawrate_all=[]
        self.Heading_all=[]
    def get_contends(self):
        with open(self.path_s1_mode1,'r', encoding='UTF-8') as file_object:
            self.contends_1 = file_object.readlines()
        with open(self.path_s1_mode2,'r', encoding='UTF-8') as file_object:
            self.contends_2 = file_object.readlines()
        with open(self.path_s1_mode3,'r', encoding='UTF-8') as file_object:
            self.contends_3 = file_object.readlines()
        with open(self.path_s1_mode4,'r', encoding='UTF-8') as file_object:
            self.contends_4 = file_object.readlines()
        with open(self.path_s1_mode5,'r', encoding='UTF-8') as file_object:
            self.contends_5 = file_object.readlines()
        with open(self.path_s1_mode6,'r', encoding='UTF-8') as file_object:
            self.contends_6 = file_object.readlines()
        with open(self.path_s1_mode7,'r', encoding='UTF-8') as file_object:
            self.contends_7 = file_object.readlines()
        with open(self.path_s1_mode8,'r', encoding='UTF-8') as file_object:
            self.contends_8 = file_object.readlines()
        with open(self.path_s1_mode9,'r', encoding='UTF-8') as file_object:
            self.contends_9 = file_object.readlines()
        with open(self.path_s1_mode10,'r', encoding='UTF-8') as file_object:
            self.contends_10 = file_object.readlines()
        with open(self.path_s1_mode11,'r', encoding='UTF-8') as file_object:
            self.contends_11 = file_object.readlines()
        with open(self.path_s2_mode1,'r', encoding='UTF-8') as file_object:
            self.contends_12 = file_object.readlines()
        with open(self.path_s2_mode2,'r', encoding='UTF-8') as file_object:
            self.contends_13 = file_object.readlines()
        with open(self.path_s2_mode3,'r', encoding='UTF-8') as file_object:
            self.contends_14 = file_object.readlines()
        with open(self.path_s2_mode4,'r', encoding='UTF-8') as file_object:
            self.contends_15 = file_object.readlines()
        with open(self.path_s2_mode5,'r', encoding='UTF-8') as file_object:
            self.contends_16 = file_object.readlines()
        with open(self.path_s2_mode6,'r', encoding='UTF-8') as file_object:
            self.contends_17 = file_object.readlines()
        with open(self.path_s2_mode7,'r', encoding='UTF-8') as file_object:
            self.contends_18 = file_object.readlines()
        with open(self.path_s2_mode8,'r', encoding='UTF-8') as file_object:
            self.contends_19 = file_object.readlines()
        with open(self.path_s2_mode9,'r', encoding='UTF-8') as file_object:
            self.contends_20 = file_object.readlines()

    def get_all(self):
        with open(self.path_s1_mode1,'r', encoding='UTF-8') as file_object:
            self.all_1 = file_object.read()
        with open(self.path_s1_mode2,'r', encoding='UTF-8') as file_object:
            self.all_2 = file_object.read()
        with open(self.path_s1_mode3,'r', encoding='UTF-8') as file_object:
            self.all_3 = file_object.read()
        with open(self.path_s1_mode4,'r', encoding='UTF-8') as file_object:
            self.all_4 = file_object.read()
        with open(self.path_s1_mode5,'r', encoding='UTF-8') as file_object:
            self.all_5 = file_object.read()
        with open(self.path_s1_mode6,'r', encoding='UTF-8') as file_object:
            self.all_6 = file_object.read()
        with open(self.path_s1_mode7,'r', encoding='UTF-8') as file_object:
            self.all_7 = file_object.read()
        with open(self.path_s1_mode8,'r', encoding='UTF-8') as file_object:
            self.all_8 = file_object.read()
        with open(self.path_s1_mode9,'r', encoding='UTF-8') as file_object:
            self.all_9 = file_object.read()
        with open(self.path_s1_mode10,'r', encoding='UTF-8') as file_object:
            self.all_10 = file_object.read()
        with open(self.path_s1_mode11,'r', encoding='UTF-8') as file_object:
            self.all_11 = file_object.read()
        with open(self.path_s2_mode1,'r', encoding='UTF-8') as file_object:
            self.all_12 = file_object.read()
        with open(self.path_s2_mode2,'r', encoding='UTF-8') as file_object:
            self.all_13 = file_object.read()
        with open(self.path_s2_mode3,'r', encoding='UTF-8') as file_object:
            self.all_14 = file_object.read()
        with open(self.path_s2_mode4,'r', encoding='UTF-8') as file_object:
            self.all_15 = file_object.read()
        with open(self.path_s2_mode5,'r', encoding='UTF-8') as file_object:
            self.all_16 = file_object.read()
        with open(self.path_s2_mode6,'r', encoding='UTF-8') as file_object:
            self.all_17 = file_object.read()
        with open(self.path_s2_mode7,'r', encoding='UTF-8') as file_object:
            self.all_18 = file_object.read()
        with open(self.path_s2_mode8,'r', encoding='UTF-8') as file_object:
            self.all_19 = file_object.read()
        with open(self.path_s2_mode9,'r', encoding='UTF-8') as file_object:
            self.all_20 = file_object.read()

    def read_element_ori_real(self,all):
        relate_x = []
        relate_y = []
        Other_X = []
        Other_Y = []
        Other_Heading = []
        keyStart = 'element_ori_real:'
        keyEnd = 'element_ori:'
        pat = re.compile(keyStart + '(.*?)' + keyEnd, re.S)
        result = pat.findall(all)
        for i in result:
            result_num = re.findall(r'-?\d+\.?\d*e?-?\d*?',i)
            result_num_array = np.asarray(result_num).reshape(-1,6)
            relate_x.append(result_num_array[:,0])
            # print(relate_x)
            relate_y.append(result_num_array[:,1])

        key_State_other = "State_other"
        key_Time = 'Time'
        pat_State_other = re.compile(key_State_other + '(.*?)' + key_Time, re.S)
        result_State_other = pat_State_other.findall(all)
        for i in result_State_other:
            result_num = re.findall(r'-?\d+\.?\d*e?-?\d*?', i)
            # print(result_num)
            result_num_array = np.asarray(result_num).reshape(4, -1)
            Other_X.append(result_num_array[0])
            Other_Y.append(result_num_array[1])
            Other_Heading.append(result_num_array[2])
        return relate_x[self.auto_s:self.auto_e],relate_y[self.auto_s:self.auto_e]

    def load_data(self,contends):
        SteerAngleAct = []
        YawRate=[]
        GPSSPEED=[]
        VehicleMode=[]
        dist2center =[]
        dist2road_bound_1=[]
        dist2road_bound_2=[]
        state_ego_dict_real_heading = []
        X = []
        Heading=[]
        run_time = []

        for row in contends:
            key_Time = "Time"
            if key_Time in row:
                strtemp = re.findall(r"\d+\.?\d*", row)
                run_time.append(float(strtemp[0]))

            key_State_ego = "State_ego"
            if key_State_ego in row:
                SteerAngleA = row.split(',')[1]
                SteerAngleAct.append(float(SteerAngleA.split(':')[1]))
                GaussX = row.split(',')[3]
                X.append(float(GaussX.split(':')[1]))
                HEADING = row.split(',')[5]
                Heading.append(float(HEADING.split(':')[1]))
            key_State_ego = "State_ego"
            if key_State_ego in row:
                SteerAngleA= row.split(',')[1]
                SteerAngleAct.append(float(SteerAngleA.split(':')[1]))
                GpsSpeed = row.split(',' )[6]
                GPSSPEED.append(float(GpsSpeed.split(':')[1]))
                Yawrate = row.split(',')[9]
                YawRate.append(float(Yawrate.split(':')[1]))
                VehicleMod = row.split(',')[14]
                VehicleMode.append(float(VehicleMod.split(':')[1]))
            key = "state_ego_dict_real"
            if key in row:
                state_ego_real_heading = row.split(',')[3]
                state_ego_dict_real_heading.append(float(state_ego_real_heading.split(':')[1]))
                dist2cente = row.split(',')[6]
                dist2center.append(float(dist2cente.split(':')[1]))
                dist2road_bound_ = row.split(',')[7]
                dist2road_bound_1.append(float(dist2road_bound_.split(':')[1]))
                dist2road_bound_22=row.split(',')[8]
                dist2road_bound_2.append(float(dist2road_bound_22.split(':')[1]))
        self.auto_state = []
        for on in range(len(VehicleMode)):
            if VehicleMode[on] == 1 and X[on] > 21276950 and X[on] < 21277097.717294:
                self.auto_state.append(on)
        self.auto_s = self.auto_state[0]
        self.auto_e = self.auto_state[-1]
        return run_time[self.auto_s:self.auto_e],SteerAngleAct[self.auto_s:self.auto_e],GPSSPEED[self.auto_s:self.auto_e],YawRate[self.auto_s:self.auto_e],VehicleMode[self.auto_s:self.auto_e],dist2center[self.auto_s:self.auto_e],dist2road_bound_1[self.auto_s:self.auto_e],dist2road_bound_2[self.auto_s:self.auto_e],state_ego_dict_real_heading[self.auto_s:self.auto_e],Heading[self.auto_s:self.auto_e]

    def Save_all(self):
        run_time_1,SteerAngleAct_1, GPSSPEED_1, YawRate_1, VehicleMode_1, dist2center_1, dist2road_bound_1_1, dist2road_bound_2_1, state_ego_dict_real_heading_1,Heading_1 =self.load_data(contends=self.contends_1)
        relate_x_1, relate_y_1=self.read_element_ori_real(self.all_1)

        run_time_2,SteerAngleAct_2, GPSSPEED_2, YawRate_2, VehicleMode_2, dist2center_2, dist2road_bound_1_2, dist2road_bound_2_2, state_ego_dict_real_heading_2 ,Heading_2= self.load_data(
            self.contends_2)
        relate_x_2, relate_y_2 = self.read_element_ori_real(self.all_2)

        run_time_3,SteerAngleAct_3, GPSSPEED_3, YawRate_3, VehicleMode_3, dist2center_3, dist2road_bound_1_3, dist2road_bound_2_3, state_ego_dict_real_heading_3,Heading_3 = self.load_data(
            self.contends_3)
        relate_x_3, relate_y_3 = self.read_element_ori_real(self.all_3)
        run_time_4,SteerAngleAct_4, GPSSPEED_4, YawRate_4, VehicleMode_4, dist2center_4, dist2road_bound_1_4, dist2road_bound_2_4,state_ego_dict_real_heading_4,Heading_4 = self.load_data(
            self.contends_4)
        relate_x_4, relate_y_4 = self.read_element_ori_real(self.all_4)
        run_time_5,SteerAngleAct_5, GPSSPEED_5, YawRate_5, VehicleMode_5, dist2center_5, dist2road_bound_1_5,dist2road_bound_2_5, state_ego_dict_real_heading_5,Heading_5 = self.load_data(
            self.contends_5)
        relate_x_5, relate_y_5 = self.read_element_ori_real(self.all_5)
        run_time_6,SteerAngleAct_6, GPSSPEED_6, YawRate_6, VehicleMode_6, dist2center_6, dist2road_bound_1_6,dist2road_bound_2_6, state_ego_dict_real_heading_6,Heading_6 = self.load_data(
            self.contends_6)
        relate_x_6, relate_y_6 = self.read_element_ori_real(self.all_6)
        run_time_7,SteerAngleAct_7, GPSSPEED_7, YawRate_7, VehicleMode_7, dist2center_7, dist2road_bound_1_7,dist2road_bound_2_7, state_ego_dict_real_heading_7,Heading_7 = self.load_data(
            self.contends_7)
        relate_x_7, relate_y_7 = self.read_element_ori_real(self.all_7)
        run_time_8,SteerAngleAct_8, GPSSPEED_8, YawRate_8, VehicleMode_8, dist2center_8, dist2road_bound_1_8,dist2road_bound_2_8, state_ego_dict_real_heading_8,Heading_8 = self.load_data(
            self.contends_8)
        relate_x_8, relate_y_8 = self.read_element_ori_real(self.all_8)
        run_time_9,SteerAngleAct_9, GPSSPEED_9, YawRate_9, VehicleMode_9, dist2center_9, dist2road_bound_1_9,dist2road_bound_2_9, state_ego_dict_real_heading_9,Heading_9 = self.load_data(
            self.contends_9)
        relate_x_9, relate_y_9 = self.read_element_ori_real(self.all_10)
        run_time_10,SteerAngleAct_10, GPSSPEED_10, YawRate_10, VehicleMode_10, dist2center_10, dist2road_bound_1_10, dist2road_bound_2_10,state_ego_dict_real_heading_10,Heading_10 = self.load_data(
            self.contends_10)
        relate_x_10, relate_y_10 = self.read_element_ori_real(self.all_10)
        run_time_11,SteerAngleAct_11, GPSSPEED_11, YawRate_11, VehicleMode_11, dist2center_11, dist2road_bound_1_11,dist2road_bound_2_11, state_ego_dict_real_heading_11,Heading_11 = self.load_data(
            self.contends_11)
        relate_x_11, relate_y_11 = self.read_element_ori_real(self.all_11)
        run_time_12,SteerAngleAct_12, GPSSPEED_12, YawRate_12, VehicleMode_12, dist2center_12, dist2road_bound_1_12,dist2road_bound_2_12, state_ego_dict_real_heading_12,Heading_12 = self.load_data(
            self.contends_12)
        relate_x_12, relate_y_12 = self.read_element_ori_real(self.all_12)
        run_time_13,SteerAngleAct_13, GPSSPEED_13, YawRate_13, VehicleMode_13, dist2center_13, dist2road_bound_1_13,dist2road_bound_2_13, state_ego_dict_real_heading_13 ,Heading_13= self.load_data(
            self.contends_13)
        relate_x_13, relate_y_13 = self.read_element_ori_real(self.all_13)
        run_time_14,SteerAngleAct_14, GPSSPEED_14, YawRate_14, VehicleMode_14, dist2center_14, dist2road_bound_1_14, dist2road_bound_2_14,state_ego_dict_real_heading_14,Heading_14 = self.load_data(
            self.contends_14)
        relate_x_14, relate_y_14 = self.read_element_ori_real(self.all_14)
        run_time_15,SteerAngleAct_15, GPSSPEED_15, YawRate_15, VehicleMode_15, dist2center_15, dist2road_bound_1_15, dist2road_bound_2_15,state_ego_dict_real_heading_15,Heading_15 = self.load_data(
            self.contends_15)
        relate_x_15, relate_y_15 = self.read_element_ori_real(self.all_15)
        run_time_16,SteerAngleAct_16, GPSSPEED_16, YawRate_16, VehicleMode_16, dist2center_16, dist2road_bound_1_16,dist2road_bound_2_16, state_ego_dict_real_heading_16,Heading_16 = self.load_data(
            self.contends_16)
        relate_x_16, relate_y_16 = self.read_element_ori_real(self.all_16)
        run_time_17,SteerAngleAct_17, GPSSPEED_17, YawRate_17, VehicleMode_17, dist2center_17, dist2road_bound_1_17,dist2road_bound_2_17, state_ego_dict_real_heading_17,Heading_17 = self.load_data(
            self.contends_17)
        relate_x_17, relate_y_17 = self.read_element_ori_real(self.all_17)
        run_time_18,SteerAngleAct_18, GPSSPEED_18, YawRate_18, VehicleMode_18, dist2center_18, dist2road_bound_1_18,dist2road_bound_2_18, state_ego_dict_real_heading_18,Heading_18 = self.load_data(
            self.contends_18)
        relate_x_18, relate_y_18 = self.read_element_ori_real(self.all_18)
        run_time_19,SteerAngleAct_19, GPSSPEED_19, YawRate_19, VehicleMode_19, dist2center_19, dist2road_bound_1_19,dist2road_bound_2_19, state_ego_dict_real_heading_19,Heading_19 = self.load_data(
            self.contends_19)
        relate_x_19, relate_y_19 = self.read_element_ori_real(self.all_19)
        run_time_20,SteerAngleAct_20, GPSSPEED_20, YawRate_20, VehicleMode_20, dist2center_20, dist2road_bound_1_20, dist2road_bound_2_20,state_ego_dict_real_heading_20,Heading_20 = self.load_data(
            self.contends_20)
        relate_x_20, relate_y_20 = self.read_element_ori_real(self.all_20)

        self.mode_all.append(np.repeat([1],len(SteerAngleAct_1)).tolist())
        self.time_all.append(run_time_1)
        self.steerangle_all.append(SteerAngleAct_1)
        self.speed_all.append(GPSSPEED_1)
        self.dist2center_all.append(dist2center_1)
        self.dist2road_bound_1_all.append(dist2road_bound_1_1)
        self.dist2road_bound_2_all.append(dist2road_bound_2_1)
        self.relate_x_all.append(relate_x_1)
        self.relate_y_all.append(relate_y_1)
        self.yawrate_all.append(YawRate_1)
        self.Heading_all.append(Heading_1)
        self.mode_all.append(np.repeat([2], len(SteerAngleAct_2)).tolist())
        self.time_all.append(run_time_2)
        self.steerangle_all.append(SteerAngleAct_2)
        self.speed_all.append(GPSSPEED_2)
        self.dist2center_all.append(dist2center_2)
        self.dist2road_bound_1_all.append(dist2road_bound_1_2)
        self.relate_x_all.append(relate_x_2)
        self.relate_y_all.append(relate_y_2)
        self.yawrate_all.append(YawRate_2)
        self.Heading_all.append(Heading_2)
        self.dist2road_bound_2_all.append(dist2road_bound_2_2)

        self.mode_all.append(np.repeat([3], len(SteerAngleAct_3)).tolist())
        self.time_all.append(run_time_3)
        self.steerangle_all.append(SteerAngleAct_3)
        self.speed_all.append(GPSSPEED_3)
        self.dist2center_all.append(dist2center_3)
        self.dist2road_bound_1_all.append(dist2road_bound_1_3)
        self.relate_x_all.append(relate_x_3)
        self.relate_y_all.append(relate_y_3)
        self.yawrate_all.append(YawRate_3)
        self.Heading_all.append(Heading_3)
        self.dist2road_bound_2_all.append(dist2road_bound_2_3)

        self.mode_all.append(np.repeat([4], len(SteerAngleAct_4)).tolist())
        self.time_all.append(run_time_4)
        self.steerangle_all.append(SteerAngleAct_4)
        self.speed_all.append(GPSSPEED_4)
        self.dist2center_all.append(dist2center_4)
        self.dist2road_bound_1_all.append(dist2road_bound_1_4)
        self.relate_x_all.append(relate_x_4)
        self.relate_y_all.append(relate_y_4)
        self.yawrate_all.append(YawRate_4)
        self.Heading_all.append(Heading_4)
        self.dist2road_bound_2_all.append(dist2road_bound_2_4)

        self.mode_all.append(np.repeat([5], len(SteerAngleAct_5)).tolist())
        self.time_all.append(run_time_5)
        self.steerangle_all.append(SteerAngleAct_5)
        self.speed_all.append(GPSSPEED_5)
        self.dist2center_all.append(dist2center_5)
        self.dist2road_bound_1_all.append(dist2road_bound_1_5)
        self.relate_x_all.append(relate_x_5)
        self.relate_y_all.append(relate_y_5)
        self.yawrate_all.append(YawRate_5)
        self.Heading_all.append(Heading_5)
        self.dist2road_bound_2_all.append(dist2road_bound_2_5)

        self.mode_all.append(np.repeat([6], len(SteerAngleAct_6)).tolist())
        self.time_all.append(run_time_6)
        self.steerangle_all.append(SteerAngleAct_6)
        self.speed_all.append(GPSSPEED_6)
        self.dist2center_all.append(dist2center_6)
        self.dist2road_bound_1_all.append(dist2road_bound_1_6)
        self.relate_x_all.append(relate_x_6)
        self.relate_y_all.append(relate_y_6)
        self.yawrate_all.append(YawRate_6)
        self.Heading_all.append(Heading_6)

        self.mode_all.append(np.repeat([7], len(SteerAngleAct_7)).tolist())
        self.time_all.append(run_time_7)
        self.dist2road_bound_2_all.append(dist2road_bound_2_6)
        self.steerangle_all.append(SteerAngleAct_7)
        self.speed_all.append(GPSSPEED_7)
        self.dist2center_all.append(dist2center_7)
        self.dist2road_bound_1_all.append(dist2road_bound_1_7)
        self.relate_x_all.append(relate_x_7)
        self.relate_y_all.append(relate_y_7)
        self.yawrate_all.append(YawRate_7)
        self.Heading_all.append(Heading_7)
        self.dist2road_bound_2_all.append(dist2road_bound_2_7)

        self.mode_all.append(np.repeat([8], len(SteerAngleAct_8)).tolist())
        self.time_all.append(run_time_8)
        self.steerangle_all.append(SteerAngleAct_8)
        self.speed_all.append(GPSSPEED_8)
        self.dist2center_all.append(dist2center_8)
        self.dist2road_bound_1_all.append(dist2road_bound_1_8)
        self.relate_x_all.append(relate_x_8)
        self.relate_y_all.append(relate_y_8)
        self.yawrate_all.append(YawRate_8)
        self.Heading_all.append(Heading_8)
        self.dist2road_bound_2_all.append(dist2road_bound_2_8)

        self.mode_all.append(np.repeat([9], len(SteerAngleAct_9)).tolist())
        self.time_all.append(run_time_9)
        self.steerangle_all.append(SteerAngleAct_9)
        self.speed_all.append(GPSSPEED_9)
        self.dist2center_all.append(dist2center_9)
        self.dist2road_bound_1_all.append(dist2road_bound_1_9)
        self.relate_x_all.append(relate_x_9)
        self.relate_y_all.append(relate_y_9)
        self.yawrate_all.append(YawRate_9)
        self.Heading_all.append(Heading_9)
        self.dist2road_bound_2_all.append(dist2road_bound_2_9)

        self.mode_all.append(np.repeat([10], len(SteerAngleAct_10)).tolist())
        self.time_all.append(run_time_10)
        self.steerangle_all.append(SteerAngleAct_10)
        self.speed_all.append(GPSSPEED_10)
        self.dist2center_all.append(dist2center_10)
        self.dist2road_bound_1_all.append(dist2road_bound_1_10)
        self.relate_x_all.append(relate_x_10)
        self.relate_y_all.append(relate_y_10)
        self.yawrate_all.append(YawRate_10)
        self.Heading_all.append(Heading_10)
        self.dist2road_bound_2_all.append(dist2road_bound_2_10)

        self.mode_all.append(np.repeat([11], len(SteerAngleAct_11)).tolist())
        self.time_all.append(run_time_11)
        self.steerangle_all.append(SteerAngleAct_11)
        self.speed_all.append(GPSSPEED_11)
        self.dist2center_all.append(dist2center_11)
        self.dist2road_bound_1_all.append(dist2road_bound_1_11)
        self.relate_x_all.append(relate_x_11)
        self.relate_y_all.append(relate_y_11)
        self.yawrate_all.append(YawRate_11)
        self.Heading_all.append(Heading_11)
        self.dist2road_bound_2_all.append(dist2road_bound_2_11)

        self.mode_all.append(np.repeat([12], len(SteerAngleAct_12)).tolist())
        self.time_all.append(run_time_12)
        self.steerangle_all.append(SteerAngleAct_12)
        self.speed_all.append(GPSSPEED_12)
        self.dist2center_all.append(dist2center_12)
        self.dist2road_bound_1_all.append(dist2road_bound_1_12)
        self.relate_x_all.append(relate_x_12)
        self.relate_y_all.append(relate_y_12)
        self.yawrate_all.append(YawRate_12)
        self.Heading_all.append(Heading_12)
        self.dist2road_bound_2_all.append(dist2road_bound_2_12)

        self.mode_all.append(np.repeat([13], len(SteerAngleAct_13)).tolist())
        self.time_all.append(run_time_13)
        self.steerangle_all.append(SteerAngleAct_13)
        self.speed_all.append(GPSSPEED_13)
        self.dist2center_all.append(dist2center_13)
        self.dist2road_bound_1_all.append(dist2road_bound_1_13)
        self.relate_x_all.append(relate_x_13)
        self.relate_y_all.append(relate_y_13)
        self.yawrate_all.append(YawRate_13)
        self.Heading_all.append(Heading_13)
        self.dist2road_bound_2_all.append(dist2road_bound_2_13)

        self.mode_all.append(np.repeat([14], len(SteerAngleAct_14)).tolist())
        self.time_all.append(run_time_14)
        self.steerangle_all.append(SteerAngleAct_14)
        self.speed_all.append(GPSSPEED_14)
        self.dist2center_all.append(dist2center_14)
        self.dist2road_bound_1_all.append(dist2road_bound_1_14)
        self.relate_x_all.append(relate_x_14)
        self.relate_y_all.append(relate_y_14)
        self.yawrate_all.append(YawRate_14)
        self.Heading_all.append(Heading_14)
        self.dist2road_bound_2_all.append(dist2road_bound_2_14)

        self.mode_all.append(np.repeat([15], len(SteerAngleAct_15)).tolist())
        self.time_all.append(run_time_15)
        self.steerangle_all.append(SteerAngleAct_15)
        self.speed_all.append(GPSSPEED_15)
        self.dist2center_all.append(dist2center_15)
        self.dist2road_bound_1_all.append(dist2road_bound_1_15)
        self.relate_x_all.append(relate_x_15)
        self.relate_y_all.append(relate_y_15)
        self.yawrate_all.append(YawRate_15)
        self.Heading_all.append(Heading_15)
        self.dist2road_bound_2_all.append(dist2road_bound_2_15)

        self.mode_all.append(np.repeat([16], len(SteerAngleAct_16)).tolist())
        self.time_all.append(run_time_16)
        self.steerangle_all.append(SteerAngleAct_16)
        self.speed_all.append(GPSSPEED_16)
        self.dist2center_all.append(dist2center_16)
        self.dist2road_bound_1_all.append(dist2road_bound_1_16)
        self.relate_x_all.append(relate_x_16)
        self.relate_y_all.append(relate_y_16)
        self.yawrate_all.append(YawRate_16)
        self.Heading_all.append(Heading_16)
        self.dist2road_bound_2_all.append(dist2road_bound_2_16)

        self.mode_all.append(np.repeat([17], len(SteerAngleAct_17)).tolist())
        self.time_all.append(run_time_17)
        self.steerangle_all.append(SteerAngleAct_17)
        self.speed_all.append(GPSSPEED_17)
        self.dist2center_all.append(dist2center_17)
        self.dist2road_bound_1_all.append(dist2road_bound_1_17)
        self.relate_x_all.append(relate_x_17)
        self.relate_y_all.append(relate_y_17)
        self.yawrate_all.append(YawRate_17)
        self.Heading_all.append(Heading_17)
        self.dist2road_bound_2_all.append(dist2road_bound_2_17)
        self.mode_all.append(np.repeat([18], len(SteerAngleAct_18)).tolist())
        self.time_all.append(run_time_18)
        self.steerangle_all.append(SteerAngleAct_18)
        self.speed_all.append(GPSSPEED_18)
        self.dist2center_all.append(dist2center_18)
        self.dist2road_bound_1_all.append(dist2road_bound_1_18)
        self.relate_x_all.append(relate_x_18)
        self.relate_y_all.append(relate_y_18)
        self.yawrate_all.append(YawRate_18)
        self.Heading_all.append(Heading_18)
        self.dist2road_bound_2_all.append(dist2road_bound_2_18)

        self.mode_all.append(np.repeat([19], len(SteerAngleAct_19)).tolist())
        self.time_all.append(run_time_19)
        self.steerangle_all.append(SteerAngleAct_19)
        self.speed_all.append(GPSSPEED_19)
        self.dist2center_all.append(dist2center_19)
        self.dist2road_bound_1_all.append(dist2road_bound_1_19)
        self.relate_x_all.append(relate_x_19)
        self.relate_y_all.append(relate_y_19)
        self.yawrate_all.append(YawRate_19)
        self.Heading_all.append(Heading_19)
        self.dist2road_bound_2_all.append(dist2road_bound_2_19)

        self.mode_all.append(np.repeat([20], len(SteerAngleAct_20)).tolist())
        self.time_all.append(run_time_20)
        self.steerangle_all.append(SteerAngleAct_20)
        self.speed_all.append(GPSSPEED_20)
        self.dist2center_all.append(dist2center_20)
        self.dist2road_bound_1_all.append(dist2road_bound_1_20)
        self.relate_x_all.append(relate_x_20)
        self.relate_y_all.append(relate_y_20)
        self.yawrate_all.append(YawRate_20)
        self.Heading_all.append(Heading_20)
        self.dist2road_bound_2_all.append(dist2road_bound_2_20)
        with open(self.root + 'save_all/all.csv', 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            mode=[]
            Time=[]
            steerangle=[]
            speed=[]
            dist2center=[]
            dist2road_bound_1=[]
            dist2road_bound_2 = []
            relate_x=[]
            relate_y=[]
            yawrate=[]
            heading=[]

            for i in self.mode_all:
                for j in i:
                    mode.append(j)
            writer.writerow(mode)

            for i in self.time_all:
                for j in i:
                    Time.append(j)
            writer.writerow(Time)

            for i in self.steerangle_all:
                for j in i:
                    steerangle.append(j)
            writer.writerow(steerangle)
            for i in self.speed_all:
                for j in i:
                    speed.append(j)
            writer.writerow(speed)

            for i in self.dist2center_all:
                for j in i:
                    dist2center.append(j)
            writer.writerow(dist2center)

            for i in self.dist2road_bound_1_all:
                for j in i:
                    dist2road_bound_1.append(j)
            writer.writerow(dist2road_bound_1)

            for i in self.dist2road_bound_2_all:
                for j in i:
                    dist2road_bound_2.append(j)
            writer.writerow(dist2road_bound_2)

            for i in self.relate_x_all:
                for j in i:
                    relate_x.append(j)
            writer.writerow(relate_x)
            for i in self.relate_y_all:
                for j in i:
                    relate_y.append(j)
            writer.writerow(relate_y)
            for i in self.yawrate_all:
                for j in i:
                    yawrate.append(j)
            writer.writerow(yawrate)
            for i in self.Heading_all:
                for j in i:
                    heading.append(j)
            writer.writerow(heading)

            print("save all success")

    def Save_all_mode(self):
        run_time_1,SteerAngleAct_1, GPSSPEED_1, YawRate_1, VehicleMode_1, dist2center_1, dist2road_bound_1_1, dist2road_bound_2_1, state_ego_dict_real_heading_1,Heading_1 =self.load_data(contends=self.contends_1)
        relate_x_1, relate_y_1=self.read_element_ori_real(self.all_1)

        run_time_2,SteerAngleAct_2, GPSSPEED_2, YawRate_2, VehicleMode_2, dist2center_2, dist2road_bound_1_2, dist2road_bound_2_2, state_ego_dict_real_heading_2 ,Heading_2= self.load_data(
            self.contends_2)
        relate_x_2, relate_y_2 = self.read_element_ori_real(self.all_2)

        run_time_3,SteerAngleAct_3, GPSSPEED_3, YawRate_3, VehicleMode_3, dist2center_3, dist2road_bound_1_3, dist2road_bound_2_3, state_ego_dict_real_heading_3,Heading_3 = self.load_data(
            self.contends_3)
        relate_x_3, relate_y_3 = self.read_element_ori_real(self.all_3)
        run_time_4,SteerAngleAct_4, GPSSPEED_4, YawRate_4, VehicleMode_4, dist2center_4, dist2road_bound_1_4, dist2road_bound_2_4,state_ego_dict_real_heading_4,Heading_4 = self.load_data(
            self.contends_4)
        relate_x_4, relate_y_4 = self.read_element_ori_real(self.all_4)
        run_time_5,SteerAngleAct_5, GPSSPEED_5, YawRate_5, VehicleMode_5, dist2center_5, dist2road_bound_1_5,dist2road_bound_2_5, state_ego_dict_real_heading_5,Heading_5 = self.load_data(
            self.contends_5)
        relate_x_5, relate_y_5 = self.read_element_ori_real(self.all_5)
        run_time_6,SteerAngleAct_6, GPSSPEED_6, YawRate_6, VehicleMode_6, dist2center_6, dist2road_bound_1_6,dist2road_bound_2_6, state_ego_dict_real_heading_6,Heading_6 = self.load_data(
            self.contends_6)
        relate_x_6, relate_y_6 = self.read_element_ori_real(self.all_6)
        run_time_7,SteerAngleAct_7, GPSSPEED_7, YawRate_7, VehicleMode_7, dist2center_7, dist2road_bound_1_7,dist2road_bound_2_7, state_ego_dict_real_heading_7,Heading_7 = self.load_data(
            self.contends_7)
        relate_x_7, relate_y_7 = self.read_element_ori_real(self.all_7)
        run_time_8,SteerAngleAct_8, GPSSPEED_8, YawRate_8, VehicleMode_8, dist2center_8, dist2road_bound_1_8,dist2road_bound_2_8, state_ego_dict_real_heading_8,Heading_8 = self.load_data(
            self.contends_8)
        relate_x_8, relate_y_8 = self.read_element_ori_real(self.all_8)
        run_time_9,SteerAngleAct_9, GPSSPEED_9, YawRate_9, VehicleMode_9, dist2center_9, dist2road_bound_1_9,dist2road_bound_2_9, state_ego_dict_real_heading_9,Heading_9 = self.load_data(
            self.contends_9)
        relate_x_9, relate_y_9 = self.read_element_ori_real(self.all_10)
        run_time_10,SteerAngleAct_10, GPSSPEED_10, YawRate_10, VehicleMode_10, dist2center_10, dist2road_bound_1_10, dist2road_bound_2_10,state_ego_dict_real_heading_10,Heading_10 = self.load_data(
            self.contends_10)
        relate_x_10, relate_y_10 = self.read_element_ori_real(self.all_10)
        run_time_11,SteerAngleAct_11, GPSSPEED_11, YawRate_11, VehicleMode_11, dist2center_11, dist2road_bound_1_11,dist2road_bound_2_11, state_ego_dict_real_heading_11,Heading_11 = self.load_data(
            self.contends_11)
        relate_x_11, relate_y_11 = self.read_element_ori_real(self.all_11)
        run_time_12,SteerAngleAct_12, GPSSPEED_12, YawRate_12, VehicleMode_12, dist2center_12, dist2road_bound_1_12,dist2road_bound_2_12, state_ego_dict_real_heading_12,Heading_12 = self.load_data(
            self.contends_12)
        relate_x_12, relate_y_12 = self.read_element_ori_real(self.all_12)
        run_time_13,SteerAngleAct_13, GPSSPEED_13, YawRate_13, VehicleMode_13, dist2center_13, dist2road_bound_1_13,dist2road_bound_2_13, state_ego_dict_real_heading_13 ,Heading_13= self.load_data(
            self.contends_13)
        relate_x_13, relate_y_13 = self.read_element_ori_real(self.all_13)
        run_time_14,SteerAngleAct_14, GPSSPEED_14, YawRate_14, VehicleMode_14, dist2center_14, dist2road_bound_1_14, dist2road_bound_2_14,state_ego_dict_real_heading_14,Heading_14 = self.load_data(
            self.contends_14)
        relate_x_14, relate_y_14 = self.read_element_ori_real(self.all_14)
        run_time_15,SteerAngleAct_15, GPSSPEED_15, YawRate_15, VehicleMode_15, dist2center_15, dist2road_bound_1_15, dist2road_bound_2_15,state_ego_dict_real_heading_15,Heading_15 = self.load_data(
            self.contends_15)
        relate_x_15, relate_y_15 = self.read_element_ori_real(self.all_15)
        run_time_16,SteerAngleAct_16, GPSSPEED_16, YawRate_16, VehicleMode_16, dist2center_16, dist2road_bound_1_16,dist2road_bound_2_16, state_ego_dict_real_heading_16,Heading_16 = self.load_data(
            self.contends_16)
        relate_x_16, relate_y_16 = self.read_element_ori_real(self.all_16)
        run_time_17,SteerAngleAct_17, GPSSPEED_17, YawRate_17, VehicleMode_17, dist2center_17, dist2road_bound_1_17,dist2road_bound_2_17, state_ego_dict_real_heading_17,Heading_17 = self.load_data(
            self.contends_17)
        relate_x_17, relate_y_17 = self.read_element_ori_real(self.all_17)
        run_time_18,SteerAngleAct_18, GPSSPEED_18, YawRate_18, VehicleMode_18, dist2center_18, dist2road_bound_1_18,dist2road_bound_2_18, state_ego_dict_real_heading_18,Heading_18 = self.load_data(
            self.contends_18)
        relate_x_18, relate_y_18 = self.read_element_ori_real(self.all_18)
        run_time_19,SteerAngleAct_19, GPSSPEED_19, YawRate_19, VehicleMode_19, dist2center_19, dist2road_bound_1_19,dist2road_bound_2_19, state_ego_dict_real_heading_19,Heading_19 = self.load_data(
            self.contends_19)
        relate_x_19, relate_y_19 = self.read_element_ori_real(self.all_19)
        run_time_20,SteerAngleAct_20, GPSSPEED_20, YawRate_20, VehicleMode_20, dist2center_20, dist2road_bound_1_20, dist2road_bound_2_20,state_ego_dict_real_heading_20,Heading_20 = self.load_data(
            self.contends_20)
        relate_x_20, relate_y_20 = self.read_element_ori_real(self.all_20)

        mode_1=['mode1']
        mode_2 = ['mode2']
        mode_3 = ['mode3']
        mode_4 = ['mode4']
        mode_5 = ['mode5']
        mode_6 = ['mode6']
        mode_7 = ['mode7']
        mode_8 = ['mode8']
        mode_9 = ['mode9']
        mode_10 = ['mode10']
        mode_11 = ['mode11']
        mode_12 = ['mode13']
        mode_13 = ['mode13']
        mode_14 = ['mode14']
        mode_15 = ['mode15']
        mode_16 = ['mode16']
        mode_17 = ['mode17']
        mode_18 = ['mode18']
        mode_19 = ['mode19']
        mode_20 = ['mode20']

        runtime_m1=run_time_1
        steerangle_m1=SteerAngleAct_1
        speed_m1=GPSSPEED_1
        dist2center_m1=dist2center_1
        dist2road_bound_1_m1=dist2road_bound_1_1
        dist2road_bound_2_m1=dist2road_bound_2_1
        relate_x_m1=relate_x_1
        relate_y_m1=relate_y_1
        yawrate_m1=YawRate_1
        Heading_m1=Heading_1


        runtime_m2 = run_time_2
        steerangle_m2=SteerAngleAct_2
        speed_m2=GPSSPEED_2
        dist2center_m2=dist2center_2
        dist2road_bound_1_m2=dist2road_bound_1_2
        relate_x_m2=relate_x_2
        relate_y_m2=relate_y_2
        yawrate_m2=YawRate_2
        Heading_m2=Heading_2
        dist2road_bound_2_m2=dist2road_bound_2_2

        runtime_m3 = run_time_3
        steerangle_m3=(SteerAngleAct_3)
        speed_m3=(GPSSPEED_3)
        dist2center_m3=(dist2center_3)
        dist2road_bound_1_m3=(dist2road_bound_1_3)
        relate_x_m3=(relate_x_3)
        relate_y_m3=(relate_y_3)
        yawrate_m3=(YawRate_3)
        Heading_m3=(Heading_3)
        dist2road_bound_2_m3=(dist2road_bound_2_3)
        runtime_m4 = run_time_4
        steerangle_m4=(SteerAngleAct_4)
        speed_m4=(GPSSPEED_4)
        dist2center_m4=(dist2center_4)
        dist2road_bound_1_m4=(dist2road_bound_1_4)
        relate_x_m4=(relate_x_4)
        relate_y_m4=(relate_y_4)
        yawrate_m4=(YawRate_4)
        Heading_m4=(Heading_4)
        dist2road_bound_2_m4=(dist2road_bound_2_4)
        runtime_m5 = run_time_5
        steerangle_m5=(SteerAngleAct_5)
        speed_m5=(GPSSPEED_5)
        dist2center_m5=(dist2center_5)
        dist2road_bound_1_m5=(dist2road_bound_1_5)
        relate_x_m5=(relate_x_5)
        relate_y_m5=(relate_y_5)
        yawrate_m5=(YawRate_5)
        Heading_m5=(Heading_5)
        dist2road_bound_2_m5=(dist2road_bound_2_5)

        runtime_m6 = run_time_6
        steerangle_m6=(SteerAngleAct_6)
        speed_m6=(GPSSPEED_6)
        dist2center_m6=(dist2center_6)
        dist2road_bound_1_m6=(dist2road_bound_1_6)
        relate_x_m6=(relate_x_6)
        relate_y_m6=(relate_y_6)
        yawrate_m6=(YawRate_6)
        Heading_m6=(Heading_6)
        dist2road_bound_2_m6=(dist2road_bound_2_6)
        runtime_m7 = run_time_7
        steerangle_m7=(SteerAngleAct_7)
        speed_m7=(GPSSPEED_7)
        dist2center_m7=(dist2center_7)
        dist2road_bound_1_m7=(dist2road_bound_1_7)
        relate_x_m7=(relate_x_7)
        relate_y_m7=(relate_y_7)
        yawrate_m7=(YawRate_7)
        Heading_m7=(Heading_7)
        dist2road_bound_2_m7=(dist2road_bound_2_7)
        runtime_m8 = run_time_8
        steerangle_m8=(SteerAngleAct_8)
        speed_m8=(GPSSPEED_8)
        dist2center_m8=(dist2center_8)
        dist2road_bound_1_m8=(dist2road_bound_1_8)
        relate_x_m8=(relate_x_8)
        relate_y_m8=(relate_y_8)
        yawrate_m8=(YawRate_8)
        Heading_m8=(Heading_8)
        dist2road_bound_2_m8=(dist2road_bound_2_8)
        runtime_m9 = run_time_9
        steerangle_m9=(SteerAngleAct_9)
        speed_m9=(GPSSPEED_9)
        dist2center_m9=(dist2center_9)
        dist2road_bound_1_m9=(dist2road_bound_1_9)
        relate_x_m9=(relate_x_9)
        relate_y_m9=(relate_y_9)
        yawrate_m9=(YawRate_9)
        Heading_m9=(Heading_9)
        dist2road_bound_2_m9=(dist2road_bound_2_9)
        runtime_m10 = run_time_10
        steerangle_m10=(SteerAngleAct_10)
        speed_m10=(GPSSPEED_10)
        dist2center_m10=(dist2center_10)
        dist2road_bound_1_m10=(dist2road_bound_1_10)
        relate_x_m10=(relate_x_10)
        relate_y_m10=(relate_y_10)
        yawrate_m10=(YawRate_10)
        Heading_m10=(Heading_10)
        dist2road_bound_2_m10=(dist2road_bound_2_10)

        runtime_m11 = run_time_11
        steerangle_m11=(SteerAngleAct_11)
        speed_m11=(GPSSPEED_11)
        dist2center_m11=(dist2center_11)
        dist2road_bound_1_m11=(dist2road_bound_1_11)
        relate_x_m11=(relate_x_11)
        relate_y_m11=(relate_y_11)
        yawrate_m11=(YawRate_11)
        Heading_m11=(Heading_11)
        dist2road_bound_2_m11=(dist2road_bound_2_11)
        runtime_m12 = run_time_12
        steerangle_m12=(SteerAngleAct_12)
        speed_m12=(GPSSPEED_12)
        dist2center_m12=(dist2center_12)
        dist2road_bound_1_m12=(dist2road_bound_1_12)
        relate_x_m12=(relate_x_12)
        relate_y_m12=(relate_y_12)
        yawrate_m12=(YawRate_12)
        Heading_m12=(Heading_12)
        dist2road_bound_2_m12=(dist2road_bound_2_12)
        runtime_m13 = run_time_13
        steerangle_m13=(SteerAngleAct_13)
        speed_m13=(GPSSPEED_13)
        dist2center_m13=(dist2center_13)
        dist2road_bound_1_m13=(dist2road_bound_1_13)
        relate_x_m13=(relate_x_13)
        relate_y_m13=(relate_y_13)
        yawrate_m13=(YawRate_13)
        Heading_m13=(Heading_13)
        dist2road_bound_2_m13=(dist2road_bound_2_13)
        runtime_m14 = run_time_14
        steerangle_m14=(SteerAngleAct_14)
        speed_m14=(GPSSPEED_14)
        dist2center_m14=(dist2center_14)
        dist2road_bound_1_m14=(dist2road_bound_1_14)
        relate_x_m14=(relate_x_14)
        relate_y_m14=(relate_y_14)
        yawrate_m14=(YawRate_14)
        Heading_m14=(Heading_14)
        dist2road_bound_2_m14=(dist2road_bound_2_14)
        runtime_m15 = run_time_15
        steerangle_m15=(SteerAngleAct_15)
        speed_m15=(GPSSPEED_15)
        dist2center_m15=(dist2center_15)
        dist2road_bound_1_m15=(dist2road_bound_1_15)
        relate_x_m15=(relate_x_15)
        relate_y_m15=(relate_y_15)
        yawrate_m15=(YawRate_15)
        Heading_m15=(Heading_15)
        dist2road_bound_2_m15=(dist2road_bound_2_15)
        runtime_m16 = run_time_16

        steerangle_m16=(SteerAngleAct_16)
        speed_m16=(GPSSPEED_16)
        dist2center_m16=(dist2center_16)
        dist2road_bound_1_m16=(dist2road_bound_1_16)
        relate_x_m16=(relate_x_16)
        relate_y_m16=(relate_y_16)
        yawrate_m16=(YawRate_16)
        Heading_m16=(Heading_16)
        dist2road_bound_2_m16=(dist2road_bound_2_16)
        runtime_m17 = run_time_17
        steerangle_m17=(SteerAngleAct_17)
        speed_m17=(GPSSPEED_17)
        dist2center_m17=(dist2center_17)
        dist2road_bound_1_m17=(dist2road_bound_1_17)
        relate_x_m17=(relate_x_17)
        relate_y_m17=(relate_y_17)
        yawrate_m17=(YawRate_17)
        Heading_m17=(Heading_17)
        dist2road_bound_2_m17=(dist2road_bound_2_17)
        runtime_m18 = run_time_18
        steerangle_m18=(SteerAngleAct_18)
        speed_m18=(GPSSPEED_18)
        dist2center_m18=(dist2center_18)
        dist2road_bound_1_m18=(dist2road_bound_1_18)
        relate_x_m18=(relate_x_18)
        relate_y_m18=(relate_y_18)
        yawrate_m18=(YawRate_18)
        Heading_m18=(Heading_18)
        dist2road_bound_2_m18=(dist2road_bound_2_18)
        runtime_m19 = run_time_19
        steerangle_m19=(SteerAngleAct_19)
        speed_m19=(GPSSPEED_19)
        dist2center_m19=(dist2center_19)
        dist2road_bound_1_m19=(dist2road_bound_1_19)
        relate_x_m19=(relate_x_19)
        relate_y_m19=(relate_y_19)
        yawrate_m19=(YawRate_19)
        Heading_m19=(Heading_19)
        dist2road_bound_2_m19=(dist2road_bound_2_19)
        runtime_m20 = run_time_20
        steerangle_m20=(SteerAngleAct_20)
        speed_m20=(GPSSPEED_20)
        dist2center_m20=(dist2center_20)
        dist2road_bound_1_m20=(dist2road_bound_1_20)
        relate_x_m20=(relate_x_20)
        relate_y_m20=(relate_y_20)
        yawrate_m20=(YawRate_20)
        Heading_m20=(Heading_20)
        dist2road_bound_2_m20=(dist2road_bound_2_20)
        with open(self.root + 'save_all/all.csv', 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(mode_1)
            writer.writerow(runtime_m1)
            writer.writerow(steerangle_m1)
            writer.writerow(speed_m1)
            writer.writerow(dist2center_m1)
            writer.writerow(dist2road_bound_1_m1)
            writer.writerow(dist2road_bound_2_m1)
            writer.writerow(relate_x_m1)
            writer.writerow(relate_y_m1)
            writer.writerow(yawrate_m1)
            writer.writerow(Heading_m1)

            writer.writerow(mode_2)
            writer.writerow(runtime_m2)
            writer.writerow(steerangle_m2)
            writer.writerow(speed_m2)
            writer.writerow(dist2center_m2)
            writer.writerow(dist2road_bound_1_m2)
            writer.writerow(dist2road_bound_2_m2)
            writer.writerow(relate_x_m2)
            writer.writerow(relate_y_m2)
            writer.writerow(yawrate_m2)
            writer.writerow(Heading_m2)

            writer.writerow(mode_3)
            writer.writerow(runtime_m3)
            writer.writerow(steerangle_m3)
            writer.writerow(speed_m3)
            writer.writerow(dist2center_m3)
            writer.writerow(dist2road_bound_1_m3)
            writer.writerow(dist2road_bound_2_m3)
            writer.writerow(relate_x_m3)
            writer.writerow(relate_y_m3)
            writer.writerow(yawrate_m3)
            writer.writerow(Heading_m3)

            writer.writerow(mode_4)
            writer.writerow(runtime_m4)
            writer.writerow(steerangle_m4)
            writer.writerow(speed_m4)
            writer.writerow(dist2center_m4)
            writer.writerow(dist2road_bound_1_m4)
            writer.writerow(dist2road_bound_2_m4)
            writer.writerow(relate_x_m4)
            writer.writerow(relate_y_m4)
            writer.writerow(yawrate_m4)
            writer.writerow(Heading_m4)

            writer.writerow(mode_5)
            writer.writerow(runtime_m5)
            writer.writerow(steerangle_m5)
            writer.writerow(speed_m5)
            writer.writerow(dist2center_m5)
            writer.writerow(dist2road_bound_1_m5)
            writer.writerow(dist2road_bound_2_m5)
            writer.writerow(relate_x_m5)
            writer.writerow(relate_y_m5)
            writer.writerow(yawrate_m5)
            writer.writerow(Heading_m5)

            writer.writerow(mode_6)
            writer.writerow(runtime_m6)
            writer.writerow(steerangle_m6)
            writer.writerow(speed_m6)
            writer.writerow(dist2center_m6)
            writer.writerow(dist2road_bound_1_m6)
            writer.writerow(dist2road_bound_2_m6)
            writer.writerow(relate_x_m6)
            writer.writerow(relate_y_m6)
            writer.writerow(yawrate_m6)
            writer.writerow(Heading_m6)

            writer.writerow(mode_7)
            writer.writerow(runtime_m7)
            writer.writerow(steerangle_m7)
            writer.writerow(speed_m7)
            writer.writerow(dist2center_m7)
            writer.writerow(dist2road_bound_1_m7)
            writer.writerow(dist2road_bound_2_m7)
            writer.writerow(relate_x_m7)
            writer.writerow(relate_y_m7)
            writer.writerow(yawrate_m7)
            writer.writerow(Heading_m7)

            writer.writerow(mode_8)
            writer.writerow(runtime_m8)
            writer.writerow(steerangle_m8)
            writer.writerow(speed_m8)
            writer.writerow(dist2center_m8)
            writer.writerow(dist2road_bound_1_m8)
            writer.writerow(dist2road_bound_2_m8)
            writer.writerow(relate_x_m8)
            writer.writerow(relate_y_m8)
            writer.writerow(yawrate_m8)
            writer.writerow(Heading_m8)

            writer.writerow(mode_9)
            writer.writerow(runtime_m9)
            writer.writerow(steerangle_m9)
            writer.writerow(speed_m9)
            writer.writerow(dist2center_m9)
            writer.writerow(dist2road_bound_1_m9)
            writer.writerow(dist2road_bound_2_m9)
            writer.writerow(relate_x_m9)
            writer.writerow(relate_y_m9)
            writer.writerow(yawrate_m9)
            writer.writerow(Heading_m9)

            writer.writerow(mode_10)
            writer.writerow(runtime_m10)
            writer.writerow(steerangle_m10)
            writer.writerow(speed_m10)
            writer.writerow(dist2center_m10)
            writer.writerow(dist2road_bound_1_m10)
            writer.writerow(dist2road_bound_2_m10)
            writer.writerow(relate_x_m10)
            writer.writerow(relate_y_m10)
            writer.writerow(yawrate_m10)
            writer.writerow(Heading_m10)

            writer.writerow(mode_11)
            writer.writerow(runtime_m11)
            writer.writerow(steerangle_m11)
            writer.writerow(speed_m11)
            writer.writerow(dist2center_m11)
            writer.writerow(dist2road_bound_1_m11)
            writer.writerow(dist2road_bound_2_m11)
            writer.writerow(relate_x_m11)
            writer.writerow(relate_y_m11)
            writer.writerow(yawrate_m11)
            writer.writerow(Heading_m11)

            writer.writerow(mode_12)
            writer.writerow(runtime_m12)
            writer.writerow(steerangle_m12)
            writer.writerow(speed_m12)
            writer.writerow(dist2center_m12)
            writer.writerow(dist2road_bound_1_m12)
            writer.writerow(dist2road_bound_2_m12)
            writer.writerow(relate_x_m12)
            writer.writerow(relate_y_m12)
            writer.writerow(yawrate_m12)
            writer.writerow(Heading_m12)

            writer.writerow(mode_13)
            writer.writerow(runtime_m13)
            writer.writerow(steerangle_m13)
            writer.writerow(speed_m13)
            writer.writerow(dist2center_m13)
            writer.writerow(dist2road_bound_1_m13)
            writer.writerow(dist2road_bound_2_m13)
            writer.writerow(relate_x_m13)
            writer.writerow(relate_y_m13)
            writer.writerow(yawrate_m13)
            writer.writerow(Heading_m13)

            writer.writerow(mode_14)
            writer.writerow(runtime_m14)
            writer.writerow(steerangle_m14)
            writer.writerow(speed_m14)
            writer.writerow(dist2center_m14)
            writer.writerow(dist2road_bound_1_m14)
            writer.writerow(dist2road_bound_2_m14)
            writer.writerow(relate_x_m14)
            writer.writerow(relate_y_m14)
            writer.writerow(yawrate_m14)
            writer.writerow(Heading_m14)

            writer.writerow(mode_15)
            writer.writerow(runtime_m15)
            writer.writerow(steerangle_m15)
            writer.writerow(speed_m15)
            writer.writerow(dist2center_m15)
            writer.writerow(dist2road_bound_1_m15)
            writer.writerow(dist2road_bound_2_m15)
            writer.writerow(relate_x_m15)
            writer.writerow(relate_y_m15)
            writer.writerow(yawrate_m15)
            writer.writerow(Heading_m15)

            writer.writerow(mode_16)
            writer.writerow(runtime_m16)
            writer.writerow(steerangle_m16)
            writer.writerow(speed_m16)
            writer.writerow(dist2center_m16)
            writer.writerow(dist2road_bound_1_m16)
            writer.writerow(dist2road_bound_2_m16)
            writer.writerow(relate_x_m16)
            writer.writerow(relate_y_m16)
            writer.writerow(yawrate_m16)
            writer.writerow(Heading_m16)

            writer.writerow(mode_17)
            writer.writerow(runtime_m17)
            writer.writerow(steerangle_m17)
            writer.writerow(speed_m17)
            writer.writerow(dist2center_m17)
            writer.writerow(dist2road_bound_1_m17)
            writer.writerow(dist2road_bound_2_m17)
            writer.writerow(relate_x_m17)
            writer.writerow(relate_y_m17)
            writer.writerow(yawrate_m17)
            writer.writerow(Heading_m17)

            writer.writerow(mode_18)
            writer.writerow(runtime_m18)
            writer.writerow(steerangle_m18)
            writer.writerow(speed_m18)
            writer.writerow(dist2center_m18)
            writer.writerow(dist2road_bound_1_m18)
            writer.writerow(dist2road_bound_2_m18)
            writer.writerow(relate_x_m18)
            writer.writerow(relate_y_m18)
            writer.writerow(yawrate_m18)
            writer.writerow(Heading_m18)

            writer.writerow(mode_19)
            writer.writerow(runtime_m19)
            writer.writerow(steerangle_m19)
            writer.writerow(speed_m19)
            writer.writerow(dist2center_m19)
            writer.writerow(dist2road_bound_1_m19)
            writer.writerow(dist2road_bound_2_m19)
            writer.writerow(relate_x_m19)
            writer.writerow(relate_y_m19)
            writer.writerow(yawrate_m19)
            writer.writerow(Heading_m19)

            writer.writerow(mode_20)
            writer.writerow(runtime_m20)
            writer.writerow(steerangle_m20)
            writer.writerow(speed_m20)
            writer.writerow(dist2center_m20)
            writer.writerow(dist2road_bound_1_m20)
            writer.writerow(dist2road_bound_2_m20)
            writer.writerow(relate_x_m20)
            writer.writerow(relate_y_m20)
            writer.writerow(yawrate_m20)
            writer.writerow(Heading_m20)

            print("save all success")


if __name__ == '__main__':
    save_all=save_all()
    save_all.get_contends()
    save_all.get_all()
    save_all.Save_all()
