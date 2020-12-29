import matplotlib.pyplot as plt
import pandas as pd
from Application_final import load_map
import re
import numpy as np
import seaborn as sns
import os
import matplotlib.patches as mpathes
# 获取文件的内容
def get_contends(path):
    with open(path) as file_object:
        contends = file_object.readlines()
    return contends
# 将一行内容变成数组
def get_all(path):
    with open(path) as file_object:
        all = file_object.read()
    return all

def read_element_ori_real(all):
    relate_x = []
    relate_y = []
    keyStart = 'element_ori_real:'
    keyEnd = 'element_ori:'
    pat = re.compile(keyStart + '(.*?)' + keyEnd, re.S)
    result = pat.findall(all)
    for i in result:
        result_num = re.findall(r'-?\d+\.?\d*e?-?\d*?',i)
        result_num_array = np.asarray(result_num).reshape(-1,6)
        relate_x.append(result_num_array[:,0])
        relate_y.append(result_num_array[:,1])
    return relate_x,relate_y

def load_data(contends):
    D_Spd = []
    D_Steer = []
    SteerAngleAct = []
    X =[]
    Y = []
    GPSSPEED=[]
    YawRate=[]
    run_time =[]
    Heading = []
    LongACC = []
    LatACC = []
    Throttle = []
    VehicleMode=[]
    BrkOn = []
    time_receive_gps = []
    time_receive_can = []
    t_interval = []
    state_ego_dict_real_heading = []
    dist2center =[]
    dist2road_bound_1=[]

    for row in contends:
        key_Decision = "Decision"
        if key_Decision in row:
            SteerAngle = row.split(',')[0]
            D_Steer.append(float(SteerAngle.split(':')[1]))
            VehicleSpeed = row.split(',')[1]
            D_Spd.append(float(VehicleSpeed.split(':')[1]))

        key_State_ego = "State_ego"
        if key_State_ego in row:
            SteerAngleA= row.split(',')[1]
            SteerAngleAct.append(float(SteerAngleA.split(':')[1]))
            GaussX = row.split(',')[3]
            X.append(float(GaussX.split(':')[1]))
            GaussY = row.split(',' )[4]
            Y.append(float(GaussY.split(':')[1]))
            HEADING = row.split(',')[5]
            Heading.append(float(HEADING.split(':')[1]))
            GpsSpeed = row.split(',' )[6]
            GPSSPEED.append(float(GpsSpeed.split(':')[1]))
            Yawrate = row.split(',')[9]
            YawRate.append(float(Yawrate.split(':')[1]))
            LongiACC = row.split(',')[10]
            LongACC.append(float(LongiACC.split(':')[1]))
            LatiACC = row.split(',')[11]
            LatACC.append(float(LatiACC.split(':')[1]))
            VehicleMod = row.split(',')[14]
            VehicleMode.append(float(VehicleMod.split(':')[1]))
            throttle = row.split(',')[15]
            Throttle.append(float(throttle.split(':')[1]))
            brkon = row.split(',')[16]
            BrkOn.append(float(brkon.split(':')[1]))

        key_Time= "Time"
        if key_Time in row:
            strtemp=re.findall(r"\d+\.?\d*",row)
            run_time.append(float(strtemp[0]))
            t_interval.append(float(strtemp[1]))
            time_receive_gps.append(float(strtemp[2]))
            time_receive_can.append(float(strtemp[3]))

        key = "state_ego_dict_real"
        if key in row:
            state_ego_real_heading= row.split(',')[3]
            state_ego_dict_real_heading.append(float(state_ego_real_heading.split(':')[1]))
            dist2cente = row.split(',')[6]
            dist2center.append(float(dist2cente.split(':')[1]))
            dist2road_bound_ = row.split(',')[7]
            dist2road_bound_1.append(float(dist2road_bound_.split(':')[1]))


    return D_Spd,D_Steer,SteerAngleAct,X,Y,Heading,GPSSPEED,YawRate,LongACC,LatACC,VehicleMode,BrkOn,Throttle,run_time,t_interval,time_receive_gps,time_receive_can,dist2center,dist2road_bound_1,state_ego_dict_real_heading

def Plot(D_Spd,D_Steer,SteerAngleAct,X,Y,Heading,GPSSPEED,YawRate,LongACC,LatACC,VehicleMode,BrkOn,Throttle,relate_x,relate_y,run_time,t_interval,time_receive_gps,time_receive_can,dist2center,dist2road_bound_1,state_ego_dict_real_heading):
    delta_T = []
    Auto_state = []

    for on in range(len(VehicleMode)):
        if  VehicleMode[on] == 1 and X[on]> 21276925:
            Auto_state.append(on)
    auto_s = Auto_state[0]
    auto_e=Auto_state[-1]

    #轨迹显示
    plt.figure('Trajectory')
    lane_list, lane_center_list,road_angle = load_map()
    plt.figsize = (20, 5)
    ax = plt.gca()
    for i in range(len(lane_list)):
        plt.plot(lane_list[i][:,0], lane_list[i][:,1], color='green', linewidth='2')
    for i in range(len(lane_center_list)):
        plt.plot(lane_center_list[i][:,0], lane_center_list[i][:,1], color='red', linewidth='2')

    for i in range(len(X[auto_s:auto_e])):
        x = X[i]
        y = Y[i]
        heading = Heading[i]
        ego_show = mpathes.Rectangle([x - 4.5/2,y-1.855 / 2], 4.5, 1.855,270 - heading, fc='magenta', ec='magenta', alpha=0.15)
        ax.add_patch(ego_show)
    # X= [21277100-i  for i in X[auto_s:auto_e]]
    # Y=[2123447700-j  for j in Y[auto_s:auto_e]]
    # plt.plot(X, Y, color='black', linewidth=2)
    plt.plot(X[auto_s:auto_e],Y[auto_s:auto_e], color='black',linewidth=2)
    plt.title("X-Y")
    ax.set_aspect('equal')
    ax.invert_xaxis()
    ax.invert_yaxis()

    plt.figure("Heading SteerAngle Speed")
    #heading
    plt.subplot(3, 1, 2)
    plt.subplots_adjust(hspace=0.5)
    plt.plot(run_time[auto_s:auto_e],Heading[auto_s:auto_e])
    plt.title("T-Heading")
    plt.ylabel("Heading")
    # #SteerAngle
    plt.subplot(3, 1, 1)
    plt.subplots_adjust(hspace=0.5)
    plt.plot(run_time[auto_s:auto_e],D_Steer[auto_s:auto_e],color='g')
    plt.plot(run_time[auto_s:auto_e], SteerAngleAct[auto_s:auto_e], color='r')
    plt.legend(labels=['SteerAngleCommand', 'SteerAngleAct'], loc='best')
    plt.title("T-SteerAngle")
    plt.ylabel("SteerAngle")
    # VehicleSpeed
    plt.subplot(3, 1, 3)
    plt.subplots_adjust(hspace=0.5)
    plt.plot(run_time[auto_s:auto_e], GPSSPEED[auto_s:auto_e])
    plt.title("T-Speed")
    plt.xlabel("Time(s)")
    plt.ylabel("Speed(m/s)")

    plt.figure("Time Distribution")
    plt.subplot(4, 1, 1)
    plt.subplots_adjust(hspace=0.5)
    plt.title("Time")
    for i in range(2,len(run_time),1):
        delta_T.append(run_time[i] - run_time[i-1])
    sns.distplot(delta_T)
    plt.subplot(4, 1, 2)
    plt.subplots_adjust(hspace=0.5)
    plt.title("Time_Interval")
    sns.distplot(t_interval[auto_s:auto_e])
    plt.subplot(4, 1, 3)
    plt.subplots_adjust(hspace=0.5)
    plt.title("Time_receive_gps")
    sns.distplot(time_receive_gps[auto_s:auto_e])
    plt.subplot(4, 1, 4)
    plt.subplots_adjust(hspace=0.5)
    plt.title("Time_receive_can")
    sns.distplot(time_receive_can[auto_s:auto_e])

    plt.figure('YawRate LongACC LatACC')
    # #Yawrate
    plt.subplot(3, 1, 1)
    plt.subplots_adjust(hspace=0.5)
    plt.plot(run_time[auto_s:auto_e],YawRate[auto_s:auto_e])
    plt.title("T-YawRate")
    # plt.xlabel("Time(s)")
    plt.ylabel("YawRate")
    # LongACC
    plt.subplot(3, 1, 2)
    plt.subplots_adjust(hspace=0.5)
    plt.plot(run_time[auto_s:auto_e], LongACC[auto_s:auto_e])
    plt.title("T-LongACC")
    plt.ylabel("LongACC")
    # LatACC
    plt.subplot(3, 1, 3)
    plt.subplots_adjust(hspace=0.5)
    plt.plot(run_time[auto_s:auto_e], LatACC[auto_s:auto_e])
    plt.title("T-LatACC")
    plt.xlabel("Time(s)")
    plt.ylabel("LatACC")

    plt.figure('dist2center dist2roadbound1')
    # dist2center
    plt.subplot(3, 1,1)
    plt.subplots_adjust(hspace=0.5)
    plt.plot(run_time[auto_s:auto_e],dist2center[auto_s:auto_e])
    plt.title("T-dist2center")
    plt.ylabel("dist2center")
    # dist2roadbound1
    plt.subplot(3, 1, 2)
    plt.subplots_adjust(hspace=0.5)
    plt.plot(run_time[auto_s:auto_e], dist2road_bound_1[auto_s:auto_e])
    plt.title("T-dist2roadbound1")
    plt.ylabel("dist2roadbound1")
    # dist2roadbound1
    plt.subplot(3, 1, 3)
    plt.subplots_adjust(hspace=0.5)
    plt.plot(run_time[auto_s:auto_e], state_ego_dict_real_heading[auto_s:auto_e])
    plt.title("T-state_ego_dict_real_heading")
    plt.xlabel("Time(s)")
    plt.ylabel("state_ego_dict_real_heading")


    plt.figure('relate_x relate_y')
    plt.subplot(2,1,1)
    relate_x_list=[]
    relate_y_list = []
    for t in range(len(relate_x)):
        relate_x_min = 100
        relate_y_min = 7
        for num in range(len(relate_x[t])):
            if abs(float(relate_y[t][num])) < 3.5:
                if abs(float(relate_x[t][num])) < relate_x_min:
                    relate_x_min = abs(float(relate_x[t][num]))
            if abs(float(relate_x[t][num])) < 5:
                if abs(float(relate_y[t][num])) < relate_y_min:
                    relate_y_min = abs(float(relate_y[t][num]))
        relate_x_list.append(relate_x_min)
        relate_y_list.append(relate_y_min)
    plt.title("T-relate_x_min")
    plt.ylabel("relate_x_min(m)")
    plt.plot(run_time[auto_s:auto_e],relate_x_list[auto_s:auto_e])
    plt.subplot(2, 1, 2)
    plt.title("T-relate_y_min")
    plt.xlabel("time(s)")
    plt.ylabel("relate_y_min(m)")
    plt.plot(run_time[auto_s:auto_e], relate_y_list[auto_s:auto_e])

    plt.figure('VehicleMode BrkMode Throttle')
    # #Yawrate
    plt.subplot(3, 1, 1)
    plt.subplots_adjust(hspace=0.5)
    plt.plot(run_time[auto_s:auto_e], VehicleMode[auto_s:auto_e])
    plt.title("T-VehicleMode")
    plt.ylabel("VehicleMode(0/1)")
    # LongACC
    plt.subplot(3, 1, 2)
    plt.subplots_adjust(hspace=0.5)
    plt.plot(run_time[auto_s:auto_e], BrkOn[auto_s:auto_e])
    plt.title("T-BrkMode")
    plt.ylabel("BrkMode(0/1)")
    # LatACC
    plt.subplot(3, 1, 3)
    plt.subplots_adjust(hspace=0.5)
    plt.plot(run_time[auto_s:auto_e], Throttle[auto_s:auto_e])
    plt.title("T-Throttle")
    plt.xlabel("Time(s)")
    plt.ylabel("Throttle")
    plt.show()

if __name__ == '__main__':
    root = '/home/fawang_troy_zhang/Desktop/record/'
    path = root + '20200817_220543/record.txt'
    contends = get_contends(path)
    all = get_all(path)
    D_Spd,D_Steer,SteerAngleAct,X,Y,Heading,GPSSPEED,YawRate,LongACC,LatACC,VehicleMode,BrkOn,Throttle,run_time ,t_interval,time_receive_gps,time_receive_can,dist2center,dist2road_bound_1,state_ego_dict_real_heading= load_data(contends)
    relate_x, relate_y = read_element_ori_real(all)
    Plot(D_Spd,D_Steer,SteerAngleAct,X,Y,Heading,GPSSPEED,YawRate,LongACC,LatACC,VehicleMode,BrkOn,Throttle,relate_x,relate_y,run_time,t_interval,time_receive_gps,time_receive_can,dist2center,dist2road_bound_1,state_ego_dict_real_heading)

    