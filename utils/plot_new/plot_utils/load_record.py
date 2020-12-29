import re
import os


# 获取文件的内容
def get_contends(path):
    with open(path,'r', encoding='UTF-8') as file_object:
        contends = file_object.readlines()
    return contends

def load_data(record_dir):
    """
    :param record_dir:
    :return:
    """
    # get project root directory
    proj_root_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))
    path = proj_root_dir + '/record/' + record_dir + '/record.txt'
    contends = get_contends(path)
    data_all = {}
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

def test_load_txt():
    load_data('left_case0_20201229_222608')

if __name__ == '__main__':
    test_load_txt()
