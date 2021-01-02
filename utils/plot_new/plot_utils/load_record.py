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
    data_all_dict = {}
    keys_for_data = {'Decision':[], 'State_ego':[], 'Obs_dict':[], 'Time':[]} # 'Obs_dict':[],
    for row in contents[0:len(keys_for_data.keys()) + 2]:
        for keys_class in keys_for_data.keys(): #
            if keys_class in row:
                for i, d in enumerate(row.split(',')[:-1]): # -1 is \n
                    data_key = d.split(':')[0]
                    data_key=data_key.split(' ')[1]
                    keys_for_data[keys_class].append(data_key)
                    data_all_dict[data_key] = []

    for row in contents:
        for keys_class in keys_for_data.keys(): #
            if keys_class in row:
                for i, d in enumerate(row.split(',')[:-1]):
                    data = d.split(':')[1]
                    data_all_dict[keys_for_data[keys_class][i]].append(float(data))

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
    # return data_all

    return data_all_dict, keys_for_data



def atest_load_txt():
    data_dict, keys_for_data = load_data('left_case0_20210101_170308')
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
