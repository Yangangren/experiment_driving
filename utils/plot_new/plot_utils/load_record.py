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
    # State_ego GaussX:0, GaussY:0, Heading:0, GpsSpeed:0, NorthVelocity:0, EastVelocity:0, YawRate:0, LongitudinalAcc:0, LateralAcc:0, Longitude:0, Latitude:0, VehicleSPeedAct:0, SteerAngleAct:0, AutoGear:0, VehicleMode:0, Throttle:0, BrkOn:0,
    # State_other x_other:[-1.75, -1.75], y_other:[7.699999999999994, 14.881499999999999], v_other:[3.0, 2.670000000000002], phi_other:[-90.0, -90.0],
    # v_light:[0], Time:[0.9574568271636963]time_decision:0.3796060085296631time_receive_gps：0time_receive_can：0time_receive_radar：0.0
    keys_decision = ['Deceleration',
                     'Torque',
                     'DecFlag',
                     'TorFlag',
                     'FrontWheelRad',
                     'SteerAngle',
                     'Acc']
    keys_gps = ['X',
                'Y',
                'Heading',
                'GpsSpeed',
                'NorthV','EastV',
                'YawRate',
                'LongAcc',
                'LatAcc',
                'NorthVelocity',
                'EastVelocity',
                'Longitude',
                'Latitude']

    keys_can = ['VehicleSPeedAct',
                'SteerAngleAct',
                'AutoGear',
                'VehicleMode',
                'Throttle',
                'BrkOn']

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
    keys_dict = dict(Decision = keys_decision, State_ego = keys_gps + keys_can, State_other=keys_others, Time = keys_time)

    # combine all data together for best reuse of plot func:
    for key in keys_decision + keys_can + keys_gps + keys_others + keys_time:
        data_all[key] = []

    for row in contents:
        for keys_data in ['Decision', 'State_ego']:
            if keys_data in row:
                for i, d in enumerate(row.split(',')[:-1]):
                    data_number = d.split(':')[1]
                    data_all[keys_dict[keys_data][i]].append(float(data_number))

    key_State_other = "State_other"
    key_Time = 'Time'
    pat_State_other = re.compile(key_State_other + '(.*?)' + key_Time, re.S)
    result_State_other = pat_State_other.findall(str_contents)
    for i in result_State_other:
        result_num = re.findall(r'-?\d+\.?\d*e?-?\d*?', i)
        keys_others_len = len(keys_others)
        try:
            result_num_array = np.array(result_num[:-1],dtype='float32').reshape(keys_others_len - 1, -1)
        except:
            print(i)
            print(result_num)
        for j in range(keys_others_len - 1):
            data_all[keys_dict[key_State_other][j]].append(result_num_array[j])
        # v-light
        data_all[keys_dict[key_State_other][j+1]].append(int(result_num[-1]))
        # for keys_data in ['State_other']:
        #     if keys_data in row:
        #         for i, d in enumerate(row.split('|')):
        #             data_number = d.split(':')[1]
        #             data_all[keys_dict[keys_data][i]].append(data_number)
    return data_all



def atest_load_txt():
    data_dict = load_data('left_case0_20201230_125357')
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
