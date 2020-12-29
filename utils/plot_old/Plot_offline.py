import matplotlib.pyplot as plt
from Application_final import load_map
import re
import numpy as np
import seaborn as sns
import matplotlib.patches as mpathes
from matplotlib import cm
import matplotlib as mpl
import matplotlib.font_manager as fm
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

def Plot_2D(D_Spd,D_Steer,SteerAngleAct,X,Y,Heading,GPSSPEED,YawRate,LongACC,LatACC,VehicleMode,BrkOn,Throttle,relate_x,relate_y,run_time,t_interval,time_receive_gps,time_receive_can,time_receive_radar,dist2center,dist2road_bound_1,state_ego_dict_real_heading,Other_X,Other_Y,Other_Heading):
    delta_T = []
    Auto_state = []
    for on in range(len(VehicleMode)):
        if  VehicleMode[on] == 1 and X[on]> 21276950 and X[on]<21277097.717294:
            Auto_state.append(on)
    auto_s = Auto_state[0]
    auto_e=Auto_state[-1]
    plt.figure('Trajectory')
    lane_list, lane_center_list, road_angle = load_map()
    plt.figsize = (20, 5)
    ax = plt.gca()


    for i in range(len(lane_list)):
        plt.plot(21277100-lane_list[i][700:-2000, 0], 3447706.99961897-lane_list[i][700:-2000, 1], color='green', linewidth='2')
    for i in range(len(lane_center_list)):
        plt.plot(21277100-lane_center_list[i][700:-2000, 0], 3447706.99961897-lane_center_list[i][700:-2000, 1], color='red', linewidth='2')

    for i in range(auto_s,auto_e,1):
        # print(i)
        # i =auto_e-1 #只显示末端
        x = 21277100-np.array(X[i])
        y = 3447706.99961897-np.array(Y[i])
        heading = Heading[i]
        ego_show = mpathes.Rectangle([x-4.5/2, y-1.855/2], 4.5, 1.855, 270 - heading, fc='magenta',
                                     ec='magenta', alpha=0.15)
        ax.add_patch(ego_show)

    plt.plot(21277100-np.array(X[auto_s:auto_e]), 3447706.99961897-np.array(Y[auto_s:auto_e]), color='black', linewidth=2)
    plt.title("X-Y")
    ax.set_aspect('equal')
    # ax.invert_xaxis()
    # ax.invert_yaxis()

    #轨迹显示
    fig=plt.figure('Trajectory-Color')
    plt.subplot(2,1,1)
    lane_list, lane_center_list,road_angle = load_map()
    ax = plt.gca()
    for i in range(len(lane_list)):
        plt.plot(21277100 - lane_list[i][:, 0], 3447706.99961897 - lane_list[i][:, 1], color='green', linewidth='2')
    for i in range(len(lane_center_list)):
        plt.plot(21277100 - lane_center_list[i][:, 0], 3447706.99961897 - lane_center_list[i][:, 1], color='red',
                 linewidth='2')

    #car_other
    other_X_list = []
    other_Y_list = []
    for num in range(len(Other_X[0])):
        other_X_list.append([])
        other_Y_list.append([])
        for i in range(auto_s,auto_e,1):
            x_other = Other_X[i][num]
            y_other = Other_Y[i][num]
            other_X_list[num].append(float(x_other))
            other_Y_list[num].append(float(y_other))
            # other_X_list[num].append(21277100-float(x_other))
            # other_Y_list[num].append(3447700-float(y_other))
        plt.scatter(21277100 -np.array(other_X_list[num]),  3447706.99961897 -np.array(other_Y_list[num]), marker='3', s=10,c=run_time[auto_s:auto_e], cmap='plasma_r',alpha = 0.5)

    plt.scatter(21277100 -np.array(X[auto_s:auto_e]), 3447706.99961897 -np.array(Y[auto_s:auto_e]),marker='o',c=run_time[auto_s:auto_e],cmap='plasma_r')
    plt.figsize = (20, 8)
    plt.title("X-Y",fontsize=20)
    ax.set_aspect('equal')
    # ax.invert_xaxis()
    # ax.invert_yaxis()
    ax1 = fig.add_axes([0.15,0.5,0.7,0.02])
    cmap=mpl.cm.plasma_r
    norm = mpl.colors.Normalize(vmin=float(run_time[0]),vmax=float(run_time[-1]))
    bar =mpl.colorbar.ColorbarBase(ax=ax1,cmap=cmap,norm=norm,orientation='horizontal')
    bar.set_label('Time(s)',fontsize=10)

    plt.figure('GaussY Heading SteerAngle YawRate')
    plt.subplot(4, 1, 1)
    plt.subplots_adjust(hspace=0.5)
    plt.plot(run_time[auto_s:auto_e], Y[auto_s:auto_e])
    plt.title("T-GaussY",fontsize=15)
    plt.ylabel("GaussY",fontsize=12)


    plt.subplot(4, 1, 3)
    plt.subplots_adjust(hspace=0.5)
    plt.plot(run_time[auto_s:auto_e],Heading[auto_s:auto_e])
    plt.title("T-Heading",fontsize=15)
    plt.ylabel("Heading",fontsize=12)
    # #SteerAngle
    plt.subplot(4, 1, 2)
    plt.subplots_adjust(hspace=0.5)
    plt.plot(run_time[auto_s:auto_e],D_Steer[auto_s:auto_e],color='g')
    plt.plot(run_time[auto_s:auto_e], SteerAngleAct[auto_s:auto_e], color='r')
    plt.legend(labels=['SteerAngleCommand', 'SteerAngleAct'], loc='best')
    plt.title("T-SteerAngle",fontsize=15)
    plt.ylabel("SteerAngle",fontsize=12)
    # VehicleSpeed
    plt.subplot(4, 1, 4)
    plt.subplots_adjust(hspace=0.5)
    plt.plot(run_time[auto_s:auto_e],YawRate[auto_s:auto_e])
    plt.title("T-YawRate",fontsize=15)
    plt.ylabel("YawRate",fontsize=12)
    plt.xlabel("Time(s)",fontsize=12)


    plt.figure('Time Distribution')
    plt.subplot(5, 1, 1)
    plt.subplots_adjust(hspace=0.5)
    plt.title("Time",fontsize=15)
    for i in range(2,len(run_time),1):
        delta_T.append(run_time[i] - run_time[i-1])
    sns.distplot(delta_T)
    plt.subplot(5, 1, 2)
    plt.subplots_adjust(hspace=0.5)
    plt.title("Time_Interval",fontsize=15)
    sns.distplot(t_interval[auto_s:auto_e])
    plt.subplot(5, 1, 3)
    plt.subplots_adjust(hspace=0.5)
    plt.title("Time_receive_gps",fontsize=15)
    sns.distplot(time_receive_gps[auto_s:auto_e])
    plt.subplot(5, 1, 4)
    plt.subplots_adjust(hspace=0.5)
    plt.title("Time_receive_can",fontsize=15)
    sns.distplot(time_receive_can[auto_s:auto_e])
    plt.subplot(5, 1, 5)
    plt.subplots_adjust(hspace=0.5)
    plt.title("Time_receive_radar",fontsize=15)
    sns.distplot(time_receive_radar[auto_s:auto_e])


    plt.figure('Speed LongACC LatACC')
    # #Yawrate
    plt.subplot(3, 1, 1)
    plt.subplots_adjust(hspace=0.5)
    plt.plot(run_time[auto_s:auto_e], GPSSPEED[auto_s:auto_e])
    plt.title("T-Speed",fontsize=15)
    plt.ylabel("Speed(m/s)",fontsize=12)
    # LongACC
    plt.subplot(3, 1, 2)
    plt.subplots_adjust(hspace=0.5)
    plt.plot(run_time[auto_s:auto_e], LongACC[auto_s:auto_e])
    plt.title("T-LongACC",fontsize=15)
    plt.ylabel("LongACC",fontsize=12)
    # LatACC
    plt.subplot(3, 1, 3)
    plt.subplots_adjust(hspace=0.5)
    plt.plot(run_time[auto_s:auto_e], LatACC[auto_s:auto_e])
    plt.title("T-LatACC",fontsize=15)
    plt.xlabel("Time(s)",fontsize=12)
    plt.ylabel("LatACC",fontsize=12)


    plt.figure('Dist2center Dist2roadbound1')
    # dist2center
    plt.subplot(3, 1,1)
    plt.subplots_adjust(hspace=0.5)
    plt.plot(run_time[auto_s:auto_e],dist2center[auto_s:auto_e])
    plt.title("T-dist2center",fontsize=15)
    plt.ylabel("dist2center",fontsize=12)
    # dist2roadbound1
    plt.subplot(3, 1, 2)
    plt.subplots_adjust(hspace=0.5)
    plt.plot(run_time[auto_s:auto_e], dist2road_bound_1[auto_s:auto_e])
    plt.title("T-dist2roadbound1",fontsize=15)
    plt.ylabel("dist2roadbound1",fontsize=12)
    # dist2roadbound1
    plt.subplot(3, 1, 3)
    plt.subplots_adjust(hspace=0.5)
    plt.plot(run_time[auto_s:auto_e], state_ego_dict_real_heading[auto_s:auto_e])
    plt.title("T-state_ego_dict_real_heading",fontsize=15)
    plt.xlabel("Time(s)",fontsize=12)
    plt.ylabel("state_ego_dict_real_heading",fontsize=12)

    plt.figure('Relate_x Relate_y')
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
    plt.title("T-relate_x_min",fontsize=15)
    plt.ylabel("relate_x_min(m)",fontsize=12)
    plt.plot(run_time[auto_s:auto_e],relate_x_list[auto_s:auto_e])
    plt.subplot(2, 1, 2)
    plt.title("T-relate_y_min",fontsize=15)
    plt.xlabel("Time(s)",fontsize=12)
    plt.ylabel("relate_y_min(m)",fontsize=12)
    plt.plot(run_time[auto_s:auto_e], relate_y_list[auto_s:auto_e])

    plt.figure('VehicleMode BrkMode Throttle')
    plt.subplot(3, 1, 1)
    plt.subplots_adjust(hspace=0.5)
    plt.plot(run_time[auto_s:auto_e], VehicleMode[auto_s:auto_e])
    plt.title("T-VehicleMode",fontsize=15)
    plt.ylabel("VehicleMode(0/1)",fontsize=12)
    plt.subplot(3, 1, 2)
    plt.subplots_adjust(hspace=0.5)
    plt.plot(run_time[auto_s:auto_e], BrkOn[auto_s:auto_e])
    plt.title("T-BrkMode",fontsize=15)
    plt.ylabel("BrkMode(0/1)",fontsize=12)
    plt.subplot(3, 1, 3)
    plt.subplots_adjust(hspace=0.5)
    plt.plot(run_time[auto_s:auto_e], Throttle[auto_s:auto_e])
    plt.title("T-Throttle",fontsize=15)
    plt.xlabel("Time(s)",fontsize=12)
    plt.ylabel("Throttle",fontsize=12)
    plt.show()

def Plot_3D(X,Y,Heading,Other_X,Other_Y,Other_Heading,run_time):
    Auto_state = []
    for on in range(len(VehicleMode)):
        if VehicleMode[on] == 1 and X[on] > 21276950 and X[on]<21277097.717294:
            Auto_state.append(on)
    auto_s = Auto_state[0]
    auto_e = Auto_state[-1]
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    lane_list, lane_center_list, road_angle = load_map()
    lane_list = np.array(lane_list)
    lane_center_list = np.array(lane_center_list)
    t0,t1, t2, t3 = run_time[auto_s],run_time[auto_e]/3, 2*run_time[auto_e]/3,run_time[auto_e]
    time_index_layer0 = int(np.argwhere(np.array(run_time) > t0)[0])
    time_index_layer1 = int(np.argwhere(np.array(run_time)>t1)[0])
    time_index_layer2 = int(np.argwhere(np.array(run_time) > t2)[0])
    time_index_layer3 = int(np.argwhere(np.array(run_time) > t3)[0])

    T_lane_layer0 = np.array(np.tile([t0], len(lane_list[0][700:-2000, 1])), dtype=np.float64)
    T_lane_layer1 = np.array(np.tile([t1], len(lane_list[0][700:-2000, 1])),dtype=np.float64)
    T_lane_layer2 = np.array(np.tile([t2], len(lane_list[0][700:-2000, 1])), dtype=np.float64)
    T_lane_layer3 = np.array(np.tile([t3], len(lane_list[0][700:-2000, 1])), dtype=np.float64)

    T_ego_layer0 = [run_time[auto_s]]
    T_ego_layer1 = np.tile([t1], len(X[auto_s:time_index_layer1]))
    T_ego_layer2 = np.tile([t2], len(X[auto_s:time_index_layer2]))
    T_ego_layer3 = np.tile([t3], len(X[auto_s:time_index_layer3]))

    #map of every layer
    for i in range(len(lane_list)):
        ax.plot(21277100-lane_list[i][700:-2000, 0], 3447706.99961897-lane_list[i][700:-2000, 1], T_lane_layer0 , color='black', linewidth='1')
    for i in range(len(lane_center_list)):
        ax.plot(21277100-lane_center_list[i][700:-2000, 0], 3447706.99961897-lane_center_list[i][700:-2000, 1], T_lane_layer0,ls='--', color='black', linewidth='1')

    for i in range(len(lane_list)):
        ax.plot(21277100-lane_list[i][700:-2000, 0], 3447706.99961897-lane_list[i][700:-2000, 1], T_lane_layer1 , color='black', linewidth='1')
    for i in range(len(lane_center_list)):
        ax.plot(21277100-lane_center_list[i][700:-2000, 0], 3447706.99961897-lane_center_list[i][700:-2000, 1], T_lane_layer1,ls='--', color='black', linewidth='1')

    for i in range(len(lane_list)):
        ax.plot(21277100-lane_list[i][700:-2000, 0], 3447706.99961897-lane_list[i][700:-2000, 1], T_lane_layer2 , color='black', linewidth='1')
    for i in range(len(lane_center_list)):
        ax.plot(21277100 - lane_center_list[i][700:-2000, 0], 3447706.99961897-lane_center_list[i][700:-2000, 1], T_lane_layer2,ls='--', color='black', linewidth='1')

    for i in range(len(lane_list)):
        ax.plot(21277100-lane_list[i][700:-2000, 0], 3447706.99961897-lane_list[i][700:-2000, 1], T_lane_layer3 , color='black', linewidth='1')
    for i in range(len(lane_center_list)):
        ax.plot(21277100-lane_center_list[i][700:-2000, 0], 3447706.99961897-lane_center_list[i][700:-2000, 1], T_lane_layer3,ls='--', color='black', linewidth='1')

    ax.grid(False)
    ax.w_xaxis.set_pane_color((1.0, 1.0, 1.0, 0.3))
    ax.w_yaxis.set_pane_color((1.0, 1.0, 1.0, 0.3))
    ax.w_zaxis.set_pane_color((1.0, 1.0, 1.0, 0.3))

    ax.plot((21277100-np.array(X[auto_s:time_index_layer0])), (3447706.99961897-np.array(Y[auto_s:time_index_layer0])), T_ego_layer0, c='blue', marker='o',markersize=2)
    ax.plot((21277100-np.array(X[auto_s:time_index_layer1])), (3447706.99961897-np.array(Y[auto_s:time_index_layer1])),T_ego_layer1,c='blue',marker='o',markersize=2)
    ax.plot((21277100-np.array(X[auto_s:time_index_layer2])), (3447706.99961897-np.array(Y[auto_s:time_index_layer2])), T_ego_layer2, c='blue', marker='o',markersize=2)
    ax.plot((21277100-np.array(X[auto_s:time_index_layer3])), (3447706.99961897-np.array( Y[auto_s:time_index_layer3])), T_ego_layer3, c='blue', marker='o',markersize=2)

    # 红点
    ax.scatter((21277100 - np.array(X[time_index_layer0 - 1])), (3447706.99961897 - np.array(Y[time_index_layer0 - 1])),
               [t0], c='red', marker='o', s=30)
    ax.scatter((21277100 - np.array(X[time_index_layer1 - 1])), (3447706.99961897 - np.array(Y[time_index_layer1 - 1])),
               [t1], c='red', marker='o', s=30)
    ax.scatter((21277100 - np.array(X[time_index_layer2 - 1])), (3447706.99961897 - np.array(Y[time_index_layer2 - 1])),
               [t2], c='red', marker='o', s=30)
    ax.scatter((21277100 - np.array(X[time_index_layer3 - 1])), (3447706.99961897 - np.array(Y[time_index_layer3 - 1])),
               [t3], c='red', marker='o', s=30)
    # 连线
    ax.plot([21277100 - X[time_index_layer0], 21277100 - X[time_index_layer1],
             21277100 - X[time_index_layer2], 21277100 - X[time_index_layer3]],
            [3447706.99961897 - Y[time_index_layer0], 3447706.99961897 - Y[time_index_layer1],
             3447706.99961897 - Y[time_index_layer2], 3447706.99961897 - Y[time_index_layer3]], [t0, t1, t2, t3],
            c='green',
            ls='--')

    plt.title("Trajectory-3D",fontsize=15)

    #ax.set_aspect('equal')
    #x.axis("equal")
    plt.xlim([0,175])
    plt.ylim([-10,30])
    ax.set_xlabel('X(m)',fontsize=12)
    ax.set_ylabel('Y(m)',fontsize=12)
    ax.set_zlabel('Time(s)',fontsize=12)
    plt.show()

def Plot_other_car(X,Y,Heading,VehicleMode,run_time,Other_X,Other_Y):
    Auto_state = []
    for on in range(len(VehicleMode)):
        if VehicleMode[on] == 1 and X[on] > 21276950 and X[on] < 21277097.717294:
            Auto_state.append(on)
    auto_s = Auto_state[0]
    auto_e = Auto_state[-1]
    # 轨迹显示
    fig = plt.figure('轨迹(周车)')
    lane_list, lane_center_list, road_angle = load_map()
    ax = plt.gca()
    for i in range(len(lane_list)):
        plt.plot(21277100 - lane_list[i][700:-2000, 0], 3447706.99961897 - lane_list[i][700:-2000, 1], color='black',
                 linewidth='1')
    for i in range(len(lane_center_list)):
        plt.plot(21277100 - lane_center_list[i][700:-2000, 0], 3447706.99961897 - lane_center_list[i][700:-2000, 1],
                 color='black', ls='--',
                 linewidth='1')
    # car_other
    other_X_list = []
    other_Y_list = []

    X_5 = []
    Y_5 = []
    Runtime_5 = []
    for i in range(auto_s, auto_e, 5):
        X_5.append(X[i])
        Y_5.append(Y[i])
        Runtime_5.append(run_time[i] - run_time[auto_s])
    l1 = plt.scatter(21277100 - np.array(X_5), 3447706.99961897 - np.array(Y_5), marker='o',
                     c=Runtime_5, cmap='plasma_r')
    for num in range(len(Other_X[0])):
    # for num in range(1):
        other_X_list.append([])
        other_Y_list.append([])
        for i in range(auto_s, auto_e, 5):
            x_other = Other_X[i][num]
            y_other = Other_Y[i][num]
            other_X_list[num].append(float(x_other))
            other_Y_list[num].append(float(y_other))
        # if other_X_list[num]<21277097.717294:
        l2 = plt.scatter(21277100 - np.array(other_X_list[num]), 3447706.99961897 - np.array(other_Y_list[num]),
                         marker='3',
                         c=Runtime_5, cmap='plasma_r')

    plt.legend(handles=[l1, l2], labels=['自车', '他车'], loc='best', prop=zhfont1)
    plt.ylabel("横向位移", fontsize=15, fontproperties=zhfont1)
    plt.xlabel("纵向位移", fontsize=15, fontproperties=zhfont1)
    ax.set_aspect('equal')
    ax1 = fig.add_axes([0.15, 0.95, 0.7, 0.02])
    cmap = mpl.cm.plasma_r
    norm = mpl.colors.Normalize(vmin=float(np.array(run_time[auto_s]) - np.array(run_time[auto_s])),
                                vmax=float(np.array(run_time[auto_e]) - np.array(run_time[auto_s])))
    bar = mpl.colorbar.ColorbarBase(ax=ax1, cmap=cmap, norm=norm, orientation='horizontal')
    bar.set_label('时间(s)', fontsize=15, fontproperties=zhfont1)
    plt.show()


if __name__ == '__main__':
    # root = 'D:/Troy.Z/THU/iDlab/1_小车测试/4_苏州实验/experiment20200819/scenario1/'
    # path = root + 'mode2/record.txt'
    root = 'D:/Troy.Z/THU/iDlab/1_小车测试/4_苏州实验/实验数据/experiment20200819/vedio/'
    path = root + 's1m2action1/record.txt'
    #s1m10action1 确认是否存在数据丢失情况
    #s2m1action1-d 改len(other_X[0])为1
    contends = get_contends(path)
    all = get_all(path)
    D_Spd,D_Steer,SteerAngleAct,X,Y,Heading,GPSSPEED,YawRate,LongACC,LatACC,VehicleMode,BrkOn,Throttle,run_time ,t_interval,time_receive_gps,time_receive_can,time_receive_radar,dist2center,dist2road_bound_1,state_ego_dict_real_heading= load_data(contends)
    relate_x, relate_y,Other_X,Other_Y,Other_Heading = read_element_ori_real(all)
    # Plot_2D(D_Spd,D_Steer,SteerAngleAct,X,Y,Heading,GPSSPEED,YawRate,LongACC,LatACC,VehicleMode,BrkOn,Throttle,relate_x,relate_y,run_time,t_interval,time_receive_gps,time_receive_can,time_receive_radar,dist2center,dist2road_bound_1,state_ego_dict_real_heading,Other_X,Other_Y,Other_Heading)
    # Plot_3D(X,Y,Heading,Other_X,Other_Y,Other_Heading,run_time)
    Plot_other_car(X,Y,Heading,VehicleMode,run_time,Other_X,Other_Y)