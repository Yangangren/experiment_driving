import re
import os
import numpy as np


# 获取文件的内容
def get_contents(path):
    with open(path,'r', encoding='UTF-8') as file_object:
        contents = file_object.readlines()
    with open(path, 'r', encoding='UTF-8') as file_object:
        str_contents = file_object.read()
    return contents, str_contents

def load_data(record_dir):
    """
    :param record_dir:
    :return:
    """
    # get project root directory
    proj_root_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))
    path = proj_root_dir + '/record/' + record_dir + '/record.txt'
    contents, str_contents = get_contents(path)
    data_all = {}
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
    # TODO: if API is modified, change here
    keys_decision = ['Deceleration',
                     'Torque',
                     'Dec_Flag',
                     'Tor_Flag',

                     'SteerAngleAim',
                     'first_out',
                     'a_x']
    # Decision
    # Deceleration: 0.0, Torque: 100.0, Dec_flag: 0, Tor_flag: 1, SteerAngleAim: 15.136967852006773, first_out: 144.2210555076599, a_x: 1.497500717639923,
    keys_gps = ['X',
                'Y',
                'Heading',
                'Heading_ori',
                'GpsSpeed',
                'NorthV','EastV',
                'YawRate',
                'LongAcc',
                'LatAcc',
                'Longitude',
                'Latitude']

    keys_can = ['VehicleSPeedAct',
                'SteerAngleAct',
                'AutoGear',
                'VehicleMode',
                'Throttle',
                'BrkOn']
    # State_ego
    # GaussX: -0.6689948966567469, GaussY: 17.386370842934493, Heading: 76.67000000000002, Heading_ori: 283.33, GpsSpeed: 1.810288651016738, NorthVelocity: 0.38900000000000007, EastVelocity: -1.7680000000000002, YawRate: -6.45, LongitudinalAcc: 0.05180000000000001, LateralAcc: 0.040900000000000006, Longitude: 120.66230729999998, Latitude: 31.129647799999997, VehicleSPeedAct: 1.765625, SteerAngleAct: -152.4, AutoGear: 3, VehicleMode: 0, Throttle: 63.55489407514398, BrkOn: 1, model_vx: 1.8102887, model_vy: -0.20022604, model_r: -0.10935349, model_x: -1.2790437, model_y: 18.738672, model_phi: 79.645485,

    keys_model = ['model_vx', 'model_vy', 'model_r', 'model_x', 'model_y', 'model_phi']
    # keys_model = []
    keys_others = ['x_other',
                   'y_other',
                   'v_other',
                   'phi_other',
                   'v_light']

    keys_time = ['Time',
                 'time_decision',
                 'time_receive_gps',
                 'time_receive_can',
                 'time_receive_radar']
    keys_dict = dict(Decision = keys_decision, State_ego = keys_gps + keys_can + keys_model, State_other=keys_others, Time = keys_time)

    # combine all data together for best reuse of plot func:
    for key in keys_decision + keys_can + keys_gps + keys_others + keys_model + keys_time:
        data_all[key] = []

    for row in contents:
        for keys_data in [ 'Decision', 'State_ego', 'Time']: #
            if keys_data in row:
                for i, d in enumerate(row.split(',')[:-1]):
                    print(d.split(':'))
                    data_number = d.split(':')[1]
                    data_all[keys_dict[keys_data][i]].append(float(data_number))

    # key_State_other = "State_other"
    # key_Time = 'Time'
    # pat_State_other = re.compile(key_State_other + '(.*?)' + key_Time, re.S)
    # result_State_other = pat_State_other.findall(str_contents)
    # for i in result_State_other:
    #     result_num = re.findall(r'-?\d+\.?\d*e?-?\d*?', i)
    #     keys_others_len = len(keys_others)
    #     try:
    #         result_num_array = np.array(result_num[:-1],dtype='float32').reshape(keys_others_len - 1, -1)
    #     except:
    #         print(i)
    #         print(result_num)
    #     for j in range(keys_others_len - 1):
    #         data_all[keys_dict[key_State_other][j]].append(result_num_array[j])
    #     # v-light
    #     data_all[keys_dict[key_State_other][j+1]].append(int(result_num[-1]))
        # for keys_data in ['State_other']:
        #     if keys_data in row:
        #         for i, d in enumerate(row.split('|')):
        #             data_number = d.split(':')[1]
        #             data_all[keys_dict[keys_data][i]].append(data_number)
    return data_all



def atest_load_txt():
    data_dict = load_data('left_case0_20210101_151956')
    a = 1


def atest_re():
    def get_all(path):
        with open(path, 'r', encoding='UTF-8') as file_object:
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
            result_num = re.findall(r'-?\d+\.?\d*e?-?\d*?', i)
            result_num_array = np.asarray(result_num).reshape(-1, 6)
            relate_x.append(result_num_array[:, 0])
            relate_y.append(result_num_array[:, 1])

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

        return relate_x, relate_y, Other_X, Other_Y, Other_Heading

    root = '/home/mht/PycharmProjects/experiment_driving/'
    path = root + 'noise/s1m2-2.5/record.txt'

    all = get_all(path)

    relate_x, relate_y, Other_X, Other_Y, Other_Heading = read_element_ori_real(all)

if __name__ == '__main__':
    atest_load_txt()
