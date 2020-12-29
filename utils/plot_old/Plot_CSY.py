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
def get_contends(path):
    with open(path,'r', encoding='UTF-8') as file_object:
        contends = file_object.readlines()
    return contends

def get_all(path):
    with open(path,'r', encoding='UTF-8') as file_object:
        all = file_object.read()
    return all

def read_element_ori_real(all):
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

    return relate_x,relate_y,Other_X,Other_Y,Other_Heading

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
    time_receive_radar=[]
    t_interval = []
    dist2center =[]
    dist2road_bound_1=[]
    state_ego_dict_real_heading = []

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

        key_Time = "Time"
        if key_Time in row:
            strtemp = re.findall(r"\d+\.?\d*", row)
            run_time.append(float(strtemp[0]))
            t_interval.append(float(strtemp[1]))
            time_receive_gps.append(float(strtemp[2]))
            time_receive_can.append(float(strtemp[3]))
            time_receive_radar.append(float(strtemp[4]))


        key = "state_ego_dict_real"
        if key in row:
            state_ego_real_heading = row.split(',')[3]
            state_ego_dict_real_heading.append(float(state_ego_real_heading.split(':')[1]))
            dist2cente = row.split(',')[6]
            dist2center.append(float(dist2cente.split(':')[1]))
            dist2road_bound_ = row.split(',')[7]
            dist2road_bound_1.append(float(dist2road_bound_.split(':')[1]))

    return D_Spd,D_Steer,SteerAngleAct,X,Y,Heading,GPSSPEED,YawRate,LongACC,LatACC,VehicleMode,BrkOn,Throttle,run_time,t_interval,time_receive_gps,time_receive_can,time_receive_radar,dist2center,dist2road_bound_1,state_ego_dict_real_heading

'''--------C S Y分别代表中心线距离 方向盘转角 和yawrate---------'''

def Save_C_S_Y(root,SteerAngleAct,YawRate,VehicleMode,run_time,dist2center,state_ego_dict_real_heading):
    #保存mode直行部分数据，认为输入起始时间点或者起始时间点和终止时间点
    Auto_state = []
    for on in range(len(VehicleMode)):
        if  VehicleMode[on] == 1 and X[on]> 21276950and X[on]<21277097.717294:
            Auto_state.append(on)
    auto_s = Auto_state[0]
    auto_e=Auto_state[-1]
    plt.subplot(2,1,1)
    plt.plot(run_time[auto_s:auto_e], dist2center[auto_s:auto_e])
    plt.title("T-dist2center", fontsize=15)
    plt.ylabel("dist2center", fontsize=12)
    plt.subplot(2, 1, 2)
    plt.plot(run_time[auto_s:auto_e], state_ego_dict_real_heading[auto_s:auto_e])
    plt.title("T-state_ego_dict_real_heading", fontsize=15)
    plt.ylabel("state_ego_dict_real_heading", fontsize=12)
    plt.show()

    # #输入起始和终止时间点
    # print("please input time start")
    # time_start = int(input())
    # print("please input time end")
    # time_end = int(input())
    # save_index_start = int(np.argwhere(np.array(run_time) > time_start)[0])
    # save_index_end = int(np.argwhere(np.array(run_time) > time_end)[0])
    # print(dist2center[save_index_start:save_index_end])
    # print(YawRate[save_index_start:save_index_end])
    # print(SteerAngleAct[save_index_start:save_index_end])
    # print(state_ego_dict_real_heading[save_index_start:save_index_end])
    # with open(root + '数据处理/c_s_y/S1/mode1.csv', 'w', newline='') as csvfile:
    #     writer = csv.writer(csvfile)
    #     writer.writerow(dist2center[save_index_start:save_index_end])
    #     writer.writerow(YawRate[save_index_start:save_index_end])
    #     writer.writerow(SteerAngleAct[save_index_start:save_index_end])
    #     writer.writerow(state_ego_dict_real_heading[save_index_start:save_index_end])

def Plot_C_S_Y(root):
    #读取数据并绘图
    csv_data1 = pd.read_csv(root+'数据处理/c_s_y/S1/mode1.csv', header=None)
    csv_data2 = pd.read_csv(root + '数据处理/c_s_y/S1/mode3.csv', header=None)
    csv_data3 = pd.read_csv(root + '数据处理/c_s_y/S1/mode4.csv', header=None)
    # csv_data4 = pd.read_csv(root + '数据处理/c_s_y/S2/mode7.csv', header=None)
    csv_data5 = pd.read_csv(root + '数据处理/c_s_y/S1/mode9.csv', header=None)

    Dist2center=csv_data1[0:1].values.tolist()+csv_data2[0:1].values.tolist()+csv_data3[0:1].values.tolist()+csv_data5[0:1].values.tolist()
    YawRate=csv_data1[1:2].values.tolist()+csv_data2[1:2].values.tolist()+csv_data3[1:2].values.tolist()+csv_data5[1:2].values.tolist()
    SteerAngle=csv_data1[2:3].values.tolist()+csv_data2[2:3].values.tolist()+csv_data3[2:3].values.tolist()+csv_data5[2:3].values.tolist()
    real_heading = csv_data1[3:].values.tolist() + csv_data2[3:].values.tolist() + csv_data3[3:].values.tolist() +  csv_data5[3:].values.tolist()
    Dist2center = list(itertools.chain.from_iterable(Dist2center))
    Dist2center = np.reshape(Dist2center, (1, -1))
    YawRate = list(itertools.chain.from_iterable(YawRate))
    YawRate = np.reshape(YawRate, (1, -1))
    SteerAngle = list(itertools.chain.from_iterable(SteerAngle))
    SteerAngle = np.reshape(SteerAngle, (1, -1))
    real_heading = list(itertools.chain.from_iterable(real_heading))
    real_heading = np.reshape(real_heading, (1, -1))
    # print(real_heading)
    plt.figure('Dist2center SteerAngle YawRate real_heading Distribution')
    plt.subplot(4, 1, 1)
    plt.subplots_adjust(hspace=0.5)
    plt.title("Dist2center",fontsize=15)
    sns.distplot(Dist2center,bins=3)
    Dist2center_var = np.std(Dist2center)
    plt.text(0.05,8,'Dist2center_var='+str(Dist2center_var))
    plt.subplot(4, 1, 2)
    plt.subplots_adjust(hspace=0.5)
    plt.title("SteerAngle",fontsize=15)
    sns.distplot(SteerAngle)
    SteerAngle_var = np.std(SteerAngle)
    plt.text(8,0.25,'SteerAngle_var='+str(SteerAngle_var))
    plt.subplot(4, 1, 3)
    plt.subplots_adjust(hspace=0.5)
    plt.title("YawRate",fontsize=15)
    sns.distplot(YawRate)
    YawRate_var = np.std(YawRate)
    plt.text(0.5,1,'YawRate_var='+str(YawRate_var))
    plt.subplot(4, 1, 4)
    plt.subplots_adjust(hspace=0.5)
    plt.title("real_heading",fontsize=15)
    sns.distplot(real_heading)
    real_heading_var = np.std(real_heading)
    plt.text(0.01,100,'real_heading_var='+str(real_heading_var))
    plt.show()

def Plot_C_S_Y_Nosie_VS(root):
    csv_data1 = pd.read_csv(root + '数据处理/c_s_y/Noise VS without Noise/S1M9/s1m9-0.csv', header=None)
    csv_data2 = pd.read_csv(root + '数据处理/c_s_y/Noise VS without Noise/S1M9/s1m9-0.5.csv', header=None)
    csv_data3 = pd.read_csv(root + '数据处理/c_s_y/Noise VS without Noise/S1M9/s1m9-1.0.csv', header=None)
    csv_data4 = pd.read_csv(root + '数据处理/c_s_y/Noise VS without Noise/S1M9/s1m9-1.5.csv', header=None)
    csv_data5 = pd.read_csv(root + '数据处理/c_s_y/Noise VS without Noise/S1M9/s1m9-2.0.csv', header=None)
    csv_data6 = pd.read_csv(root + '数据处理/c_s_y/Noise VS without Noise/S1M9/s1m9-2.5.csv', header=None)
    csv_data7 = pd.read_csv(root + '数据处理/c_s_y/Noise VS without Noise/S1M9/s1m9-3.0.csv', header=None)
    Dist2center =[]
    Dist2center_0 = csv_data1[0:1].values.tolist()
    Dist2center_0_5 =csv_data2[0:1].values.tolist()
    Dist2center_1_0 = csv_data3[0:1].values.tolist()
    Dist2center_1_5 = csv_data4[0:1].values.tolist()
    Dist2center_2_0 = csv_data5[0:1].values.tolist()
    Dist2center_2_5 =csv_data6[0:1].values.tolist()
    Dist2center_3_0=csv_data7[0:1].values.tolist()
    Dist2center.append(Dist2center_0)
    Dist2center.append(Dist2center_0_5)
    Dist2center.append(Dist2center_1_0)
    Dist2center.append(Dist2center_1_5)
    Dist2center.append(Dist2center_2_0)
    Dist2center.append(Dist2center_2_5)
    Dist2center.append(Dist2center_3_0)

    # YawRate_0 = np.array(csv_data1[1:2].values.tolist())*180/math.pi
    # YawRate_0_5 =np.array(csv_data2[1:2].values.tolist())*180/math.pi
    # YawRate_1_0 = np.array(csv_data3[1:2].values.tolist())*180/math.pi
    # YawRate_1_5 =np.array(csv_data4[1:2].values.tolist())*180/math.pi
    # YawRate_2_0 = np.array(csv_data5[1:2].values.tolist())*180/math.pi
    # YawRate_2_5 =np.array(csv_data6[1:2].values.tolist())*180/math.pi
    # YawRate_3_0 =np.array(csv_data7[1:2].values.tolist())*180/math.pi
    YawRate_0 = np.array(csv_data1[1:2].values.tolist())
    YawRate_0_5 = np.array(csv_data2[1:2].values.tolist())
    YawRate_1_0 = np.array(csv_data3[1:2].values.tolist())
    YawRate_1_5 = np.array(csv_data4[1:2].values.tolist())
    YawRate_2_0 = np.array(csv_data5[1:2].values.tolist())
    YawRate_2_5 = np.array(csv_data6[1:2].values.tolist())
    YawRate_3_0 = np.array(csv_data7[1:2].values.tolist())
    YawRate = []
    YawRate.append(YawRate_0)
    YawRate.append(YawRate_0_5)
    YawRate.append(YawRate_1_0)
    YawRate.append(YawRate_1_5)
    YawRate.append(YawRate_2_0)
    YawRate.append(YawRate_2_5)
    YawRate.append(YawRate_3_0)
    SteerAngle_0 = csv_data1[2:3].values.tolist()
    SteerAngle_0_5 =csv_data2[2:3].values.tolist()
    SteerAngle_1_0 = csv_data3[2:3].values.tolist()
    SteerAngle_1_5 =csv_data4[2:3].values.tolist()
    SteerAngle_2_0 = csv_data5[2:3].values.tolist()
    SteerAngle_2_5 =csv_data6[2:3].values.tolist()
    SteerAngle_3_0 = csv_data7[2:3].values.tolist()
    SteerAngle = []
    SteerAngle.append(SteerAngle_0)
    SteerAngle.append(SteerAngle_0_5)
    SteerAngle.append(SteerAngle_1_0)
    SteerAngle.append(SteerAngle_1_5)
    SteerAngle.append(SteerAngle_2_0)
    SteerAngle.append(SteerAngle_2_5)
    SteerAngle.append(SteerAngle_3_0)
    real_heading_0 = np.array(csv_data1[3:].values.tolist())*180/math.pi
    real_heading_0_5 =np.array(csv_data2[3:].values.tolist())*180/math.pi
    real_heading_1_0 = np.array(csv_data3[3:].values.tolist())*180/math.pi
    real_heading_1_5 =np.array(csv_data4[3:].values.tolist())*180/math.pi
    real_heading_2_0 = np.array(csv_data5[3:].values.tolist())*180/math.pi
    real_heading_2_5 =np.array(csv_data6[3:].values.tolist())*180/math.pi
    real_heading_3_0 = np.array(csv_data7[3:].values.tolist())*180/math.pi
    real_heading = []
    real_heading.append(real_heading_0)
    real_heading.append(real_heading_0_5)
    real_heading.append(real_heading_1_0)
    real_heading.append(real_heading_1_5)
    real_heading.append(real_heading_2_0)
    real_heading.append(real_heading_2_5)
    real_heading.append(real_heading_3_0)


    Noise=[0,0.5,1.0,1.5,2.0,2.5,3.0]
    Dist2center_var=[]
    Dist2center_var.append(np.std(np.array(Dist2center_0)))
    Dist2center_var.append(np.std(np.array(Dist2center_0_5)))
    Dist2center_var.append(np.std(np.array(Dist2center_1_0)))
    Dist2center_var.append(np.std(np.array(Dist2center_1_5)))
    Dist2center_var.append(np.std(np.array(Dist2center_2_0)))
    Dist2center_var.append(np.std(np.array(Dist2center_2_5)))
    Dist2center_var.append(np.std(np.array(Dist2center_3_0)))
    YawRate_var = []
    YawRate_var.append(np.std(YawRate_0))
    YawRate_var.append(np.std(YawRate_0_5))
    YawRate_var.append(np.std(YawRate_1_0))
    YawRate_var.append(np.std(YawRate_1_5))
    YawRate_var.append(np.std(YawRate_2_0))
    YawRate_var.append(np.std(YawRate_2_5))
    YawRate_var.append(np.std(YawRate_3_0))
    SteerAngle_var = []
    SteerAngle_var.append(np.std(SteerAngle_0))
    SteerAngle_var.append(np.std(SteerAngle_0_5))
    SteerAngle_var.append(np.std(SteerAngle_1_0))
    SteerAngle_var.append(np.std(SteerAngle_1_5))
    SteerAngle_var.append(np.std(SteerAngle_2_0))
    SteerAngle_var.append(np.std(SteerAngle_2_5))
    SteerAngle_var.append(np.std(SteerAngle_3_0))
    real_heading_var = []
    real_heading_var.append(np.std(real_heading_0))
    real_heading_var.append(np.std(real_heading_0_5))
    real_heading_var.append(np.std(real_heading_1_0))
    real_heading_var.append(np.std(real_heading_1_5))
    real_heading_var.append(np.std(real_heading_2_0))
    real_heading_var.append(np.std(real_heading_2_5))
    real_heading_var.append(np.std(real_heading_3_0))

    Dist2center_df_list = []
    for i in range(0, 7, 1):
        Dist2center_PD = pd.DataFrame(dict(Dist2center=np.array(Dist2center[i]).squeeze(),Noise=Noise[i],))
        Dist2center_df_list.append(Dist2center_PD)
    Dist2center_dataframe = Dist2center_df_list[0].append(Dist2center_df_list[1:], ignore_index=True, )

    YawRate_df_list = []
    for i in range(0, 7, 1):
        YawRate_PD = pd.DataFrame(dict(YawRate=np.array(YawRate[i]).squeeze(),Noise=Noise[i],))
        YawRate_df_list.append(YawRate_PD)
    YawRate_dataframe = YawRate_df_list[0].append(YawRate_df_list[1:], ignore_index=True, )

    SteerAngle_df_list = []
    for i in range(0, 7, 1):
        SteerAngle_PD = pd.DataFrame(dict(SteerAngle=np.array(SteerAngle[i]).squeeze(),Noise=Noise[i],))
        SteerAngle_df_list.append(SteerAngle_PD)
    SteerAngle_dataframe = SteerAngle_df_list[0].append(SteerAngle_df_list[1:], ignore_index=True, )

    real_heading_df_list = []
    for i in range(0, 7, 1):
        real_heading_PD = pd.DataFrame(dict(real_heading=np.array(real_heading[i]).squeeze(),Noise=Noise[i],))
        real_heading_df_list.append(real_heading_PD)
    real_heading_dataframe = real_heading_df_list[0].append(real_heading_df_list[1:], ignore_index=True, )

    sns.set(style="darkgrid")
    f1 = plt.figure("距车道中心线距离")
    ax1 = f1.add_axes([0.095, 0.14, 0.9,0.85])
    sns.lineplot(ax=ax1,x=np.array(Noise)*2, y=Dist2center_var,linewidth=5)
    ax1.legend("标准差",prop=zhfont1,loc=0)
    sns.boxplot(ax=ax1,x="Noise", y="Dist2center", data=Dist2center_dataframe,palette="bright",order=np.arange(0,3.1,0.5))
    ax1.set_ylabel('距车道中心线距离(m)', fontsize=12, fontproperties=zhfont1)
    plt.yticks(fontsize=12)
    plt.xticks(fontsize=12)

    sns.set(style="darkgrid")
    f2 = plt.figure("偏航角速度")
    ax2 = f2.add_axes([0.095, 0.14, 0.9, 0.85])
    sns.lineplot(ax=ax2, x=np.array(Noise) * 2, y=YawRate_var, linewidth=5)
    ax2.legend("标准差", prop=zhfont1, loc=0)
    sns.boxplot(ax=ax2, x="Noise", y="YawRate", data=YawRate_dataframe, palette="bright",
                order=np.arange(0, 3.1, 0.5))
    ax2.set_ylabel('偏航角速度', fontsize=12, fontproperties=zhfont1)
    plt.yticks(fontsize=12)
    plt.xticks(fontsize=12)

    sns.set(style="darkgrid")
    f3 = plt.figure("方向盘转角")
    ax3 = f3.add_axes([0.095, 0.14, 0.9, 0.85])
    sns.lineplot(ax=ax3, x=np.array(Noise) * 2, y=SteerAngle_var, linewidth=5)
    ax3.legend("标准差", prop=zhfont1, loc=0)
    sns.boxplot(ax=ax3, x="Noise", y="SteerAngle", data=SteerAngle_dataframe, palette="bright",
                order=np.arange(0, 3.1, 0.5))
    ax3.set_ylabel('方向盘转角', fontsize=12, fontproperties=zhfont1)
    plt.yticks(fontsize=12)
    plt.xticks(fontsize=12)

    sns.set(style="darkgrid")
    f4 = plt.figure("夹角")
    ax4 = f4.add_axes([0.095, 0.14, 0.9, 0.85])
    sns.lineplot(ax=ax4, x=np.array(Noise) * 2, y=real_heading_var, linewidth=5)
    ax4.legend("标准差", prop=zhfont1, loc=0)
    sns.boxplot(ax=ax4, x="Noise", y="real_heading", data=real_heading_dataframe, palette="bright",
                order=np.arange(0, 3.1, 0.5))
    ax4.set_ylabel('夹角', fontsize=12, fontproperties=zhfont1)
    plt.yticks(fontsize=12)
    plt.xticks(fontsize=12)

    plt.show()

def Plot_T_S_S_C(X,Y,Other_X,Other_Y,run_time):
    Auto_state = []
    for on in range(len(VehicleMode)):
        if VehicleMode[on] == 1 and X[on] > 21276950 and X[on]<21277097.717294:
            Auto_state.append(on)
    auto_s = Auto_state[0]
    auto_e = Auto_state[-1]
    # 轨迹显示
    fig = plt.figure('轨迹 转角 速度 中心线距离 夹角')
    plt.subplot(5, 1, 1)
    lane_list, lane_center_list, road_angle = load_map()
    ax = plt.gca()
    for i in range(len(lane_list)):
        plt.plot(21277100 - lane_list[i][700:-2000, 0], 3447706.99961897 - lane_list[i][700:-2000, 1], color='black', linewidth='1')
    for i in range(len(lane_center_list)):
        plt.plot(21277100 - lane_center_list[i][700:-2000, 0], 3447706.99961897 - lane_center_list[i][700:-2000, 1], color='black',ls='--',
                 linewidth='1')
    # car_other
    other_X_list = []
    other_Y_list = []

    X_5=[]
    Y_5=[]
    Runtime_5 = []
    for i in range(auto_s,auto_e,5):
        X_5.append(X[i])
        Y_5.append(Y[i])
        Runtime_5.append(run_time[i]-run_time[auto_s])
    l1=plt.scatter(21277100 - np.array(X_5), 3447706.99961897 - np.array(Y_5), marker='o',
                c=Runtime_5, cmap='plasma_r')
    for num in range(len(Other_X[0])):
        other_X_list.append([])
        other_Y_list.append([])
        for i in range(auto_s, auto_e, 5):
            x_other = Other_X[i][num]
            y_other = Other_Y[i][num]
            other_X_list[num].append(float(x_other))
            other_Y_list[num].append(float(y_other))
        l2=plt.scatter(21277100 - np.array(other_X_list[num]), 3447706.99961897 - np.array(other_Y_list[num]), marker='3',
                 c=Runtime_5,cmap='plasma_r')

    plt.legend(handles=[l1,l2],labels=['自车', '他车'], loc='best',prop=zhfont1)
    # plt.title("轨迹", fontsize=20,fontproperties=zhfont1)
    plt.ylabel("横向位移", fontsize=15, fontproperties=zhfont1)
    plt.xlabel("纵向位移", fontsize=15, fontproperties=zhfont1)
    ax.set_aspect('equal')
    ax1 = fig.add_axes([0.15, 0.95, 0.7, 0.02])
    cmap = mpl.cm.plasma_r
    norm = mpl.colors.Normalize(vmin=float(np.array(run_time[auto_s])-np.array(run_time[auto_s])), vmax=float(np.array(run_time[auto_e])-np.array(run_time[auto_s])))
    bar = mpl.colorbar.ColorbarBase(ax=ax1, cmap=cmap, norm=norm, orientation='horizontal')
    bar.set_label('时间(s)', fontsize=15,fontproperties=zhfont1)

    plt.subplot(5, 1, 2)
    plt.subplots_adjust(hspace=0.5)
    plt.plot(np.array(run_time[auto_s:auto_e])-np.array(run_time[auto_s]), D_Steer[auto_s:auto_e], color='g')
    plt.plot(np.array(run_time[auto_s:auto_e])-np.array(run_time[auto_s]), SteerAngleAct[auto_s:auto_e], color='r')
    plt.legend(labels=['目标转角', '实际转角'], loc='best',prop=zhfont1)
    # plt.title("方向盘转角-时间", fontsize=20,fontproperties=zhfont1)
    plt.ylabel("方向盘转角", fontsize=15,fontproperties=zhfont1)

    plt.subplot(5, 1,3)
    plt.subplots_adjust(hspace=0.5)
    plt.plot(np.array(run_time[auto_s:auto_e])-np.array(run_time[auto_s]), np.array(GPSSPEED[auto_s:auto_e])*3.6)
    # plt.title("速度-时间", fontsize=20,fontproperties=zhfont1)
    plt.ylabel("速度(km/h)", fontsize=15,fontproperties=zhfont1)
    plt.subplot(5, 1, 4)
    plt.subplots_adjust(hspace=0.5)
    plt.plot(np.array(run_time[auto_s:auto_e])-np.array(run_time[auto_s]), dist2center[auto_s:auto_e])
    # plt.title("距车道中心线距离-时间", fontsize=20,fontproperties=zhfont1)
    plt.ylabel("距车道中心线距离", fontsize=15,fontproperties=zhfont1)
    plt.subplot(5, 1, 5)
    plt.plot(np.array(run_time[auto_s:auto_e])-np.array(run_time[auto_s]), state_ego_dict_real_heading[auto_s:auto_e])
    # plt.title("夹角-时间", fontsize=15)
    plt.ylabel("夹角", fontsize=12,fontproperties=zhfont1)
    plt.xlabel("时间(s)", fontsize=15, fontproperties=zhfont1)
    plt.show()

def Plot_T_S_B_FD(X,Y,Other_X,Other_Y,run_time):
    Auto_state = []
    for on in range(len(VehicleMode)):
        if VehicleMode[on] == 1 and X[on] > 21276950 and X[on]<21277097.717294:
            Auto_state.append(on)
    auto_s = Auto_state[0]
    auto_e = Auto_state[-1]
    # 轨迹显示
    fig = plt.figure('轨迹 转角 路侧 前车距离')
    plt.subplot(5, 1, 1)
    lane_list, lane_center_list, road_angle = load_map()
    ax = plt.gca()
    for i in range(len(lane_list)):
        plt.plot(21277100 - lane_list[i][700:-2000, 0], 3447706.99961897 - lane_list[i][700:-2000, 1], color='black', linewidth='1')
    for i in range(len(lane_center_list)):
        plt.plot(21277100 - lane_center_list[i][700:-2000, 0], 3447706.99961897 - lane_center_list[i][700:-2000, 1], color='black',ls='--',
                 linewidth='1')
    # car_other
    other_X_list = []
    other_Y_list = []

    X_5=[]
    Y_5=[]
    Runtime_5 = []
    for i in range(auto_s,auto_e,5):
        X_5.append(X[i])
        Y_5.append(Y[i])
        Runtime_5.append(run_time[i]-run_time[auto_s])
    l1=plt.scatter(21277100 - np.array(X_5), 3447706.99961897 - np.array(Y_5), marker='o',
                c=Runtime_5, cmap='plasma_r')
    for num in range(len(Other_X[0])):
        other_X_list.append([])
        other_Y_list.append([])
        for i in range(auto_s, auto_e, 5):
            x_other = Other_X[i][num]
            y_other = Other_Y[i][num]
            other_X_list[num].append(float(x_other))
            other_Y_list[num].append(float(y_other))
        l2=plt.scatter(21277100 - np.array(other_X_list[num]), 3447706.99961897 - np.array(other_Y_list[num]), marker='3',
                 c=Runtime_5,cmap='plasma_r')

    plt.legend(handles=[l1,l2],labels=['自车', '他车'], loc='best',prop=zhfont1)
    # plt.title("轨迹", fontsize=20,fontproperties=zhfont1)
    plt.ylabel("横向位移", fontsize=15, fontproperties=zhfont1)
    plt.xlabel("纵向位移", fontsize=15, fontproperties=zhfont1)
    ax.set_aspect('equal')
    ax1 = fig.add_axes([0.15, 0.95, 0.7, 0.02])
    cmap = mpl.cm.plasma_r
    norm = mpl.colors.Normalize(vmin=float(np.array(run_time[auto_s])-np.array(run_time[auto_s])), vmax=float(np.array(run_time[auto_e])-np.array(run_time[auto_s])))
    bar = mpl.colorbar.ColorbarBase(ax=ax1, cmap=cmap, norm=norm, orientation='horizontal')
    bar.set_label('时间(s)', fontsize=15,fontproperties=zhfont1)

    plt.subplot(5, 1, 2)
    plt.subplots_adjust(hspace=0.5)
    plt.plot(np.array(run_time[auto_s:auto_e])-np.array(run_time[auto_s]), D_Steer[auto_s:auto_e], color='g')
    plt.plot(np.array(run_time[auto_s:auto_e])-np.array(run_time[auto_s]), SteerAngleAct[auto_s:auto_e], color='r')
    plt.legend(labels=['目标转角', '实际转角'], loc='best',prop=zhfont1)
    # plt.title("方向盘转角-时间", fontsize=20,fontproperties=zhfont1)
    plt.ylabel("方向盘转角", fontsize=15,fontproperties=zhfont1)

    plt.subplot(5, 1,3)
    plt.subplots_adjust(hspace=0.5)
    plt.plot(np.array(run_time[auto_s:auto_e])-np.array(run_time[auto_s]), np.array(GPSSPEED[auto_s:auto_e])*3.6)
    # plt.title("速度-时间", fontsize=20,fontproperties=zhfont1)
    plt.ylabel("速度(km/h)", fontsize=15,fontproperties=zhfont1)
    plt.subplot(5, 1, 4)
    plt.subplots_adjust(hspace=0.5)
    plt.plot(np.array(run_time[auto_s:auto_e])-np.array(run_time[auto_s]), dist2center[auto_s:auto_e])
    # plt.title("距车道中心线距离-时间", fontsize=20,fontproperties=zhfont1)
    plt.ylabel("距车道中心线距离", fontsize=15,fontproperties=zhfont1)
    plt.subplot(5, 1, 5)
    plt.plot(np.array(run_time[auto_s:auto_e])-np.array(run_time[auto_s]), state_ego_dict_real_heading[auto_s:auto_e])
    # plt.title("夹角-时间", fontsize=15)
    plt.ylabel("夹角", fontsize=12,fontproperties=zhfont1)
    plt.xlabel("时间(s)", fontsize=15, fontproperties=zhfont1)
    plt.show()

def Plot_time_box(t_interval,time_receive_gps,time_receive_can,time_receive_radar):
    Time=[]
    Time.append(t_interval)
    Time.append(time_receive_gps)
    Time.append(time_receive_can)
    Time.append(time_receive_radar)
    time_sequence={0:'t_decision',1:'t_gps',2:'t_can',3:'t_radar'}
    time_df_list = []
    for i in range(0, 4, 1):
        time_PD = pd.DataFrame(dict(time=np.array(Time[i]), time_sequence=time_sequence[i], ))
        time_df_list.append(time_PD)
    time_dataframe = time_df_list[0].append(time_df_list[1:], ignore_index=True, )
    sns.set(style="darkgrid")
    f1 = plt.figure("时间分布")
    ax1 = f1.add_axes([0.095, 0.14, 0.9,0.85])
    sns.boxplot(ax=ax1,x="time_sequence", y="time", data=time_dataframe,palette="bright")
    ax1.set_ylabel('时间分布', fontsize=12, fontproperties=zhfont1)
    plt.yticks(fontsize=12)
    plt.xticks(fontsize=12)
    plt.show()

if __name__ == '__main__':
    root = 'D:/Troy.Z/THU/iDlab/1_小车测试/4_苏州实验/实验数据/experiment20200819/'
    path = root + 'noise/s1m9-3./record.txt'
    # root = 'D:/Troy.Z/THU/iDlab/1_小车测试/4_苏州实验/实验数据/experiment20200819/'
    # path = root + 'vedio/s2m8action1/record.txt'
    contends = get_contends(path)
    all = get_all(path)
    D_Spd,D_Steer,SteerAngleAct,X,Y,Heading,GPSSPEED,YawRate,LongACC,LatACC,VehicleMode,BrkOn,Throttle,run_time ,t_interval,time_receive_gps,time_receive_can,time_receive_radar,dist2center,dist2road_bound_1,state_ego_dict_real_heading= load_data(contends)
    relate_x, relate_y,Other_X,Other_Y,Other_Heading = read_element_ori_real(all)
    # Plot_T_S_S_C(X, Y, Other_X, Other_Y, run_time)
    Save_C_S_Y(root,SteerAngleAct,YawRate,VehicleMode,run_time,dist2center,state_ego_dict_real_heading)
    # Plot_C_S_Y(root)
    # Plot_C_S_Y_Nosie_VS(root)
    # Plot_time_box(t_interval, time_receive_gps, time_receive_can, time_receive_radar)
