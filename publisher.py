import zmq
import json
import time
import math
from Application_final import E2E
import os
from datetime import datetime
import numpy as np


def mkdir(path):
    folder = os.path.exists(path)
    if not folder:         # 判断是否存在文件夹如果不存在则创建为文件夹
        os.makedirs(path)  # makedirs 创建文件时如果路径不存在会创建这个路径


class Publisher():
    def __init__(self,shared_list, Info_List,State_Other_List,receive_index,if_save,if_radar,lock):
        self.Decision = {}
        self.State_ego = {}
        self.State_other = {}
        self.time_out = 0
        self.E2E = E2E()
        self.Info_List = Info_List
        self.State_Other_List = State_Other_List
        self.Info_List[0] = -1
        self.shared_list = shared_list
        self.read_index_old = 0
        self.receive_index_shared = receive_index
        # self.read_index_old = Info_List[0]

        self.lock = lock
        context = zmq.Context()
        self.socket_pub = context.socket(zmq.PUB)
        self.socket_pub.bind("tcp://*:6970")
        self.time_initial = time.time()
        self.step = 0
        self.if_save = if_save
        self.if_radar = if_radar
        self.save_path = './record/' + datetime.now().strftime("%Y%m%d_%H%M%S")
        mkdir(self.save_path)
        self.x_next = [21277020.000, 21277060.000]
        self.y_next = [3447700.252461, 3447702.714812]
        self.heading_next = [271, 271]
        self.v_next = [0, 0]

        self.t_interval = 0
        self.time_decision=0
        self.time_in = time.time()

    def run(self):
        time_start = time.time()
        with open(self.save_path + '/record.txt', 'a') as file_handle:
            file_handle.write(str("保存时间：" + datetime.now().strftime("%Y%m%d_%H%M%S")))
            file_handle.write('\n')
            while True:
                time.sleep(0.07)
                shared_index = self.receive_index_shared.value
                if shared_index > self.read_index_old:
                    self.read_index_old = shared_index
                    self.Time = []
                    if time.time()-time_start > 0.1:
                        print("time!!!!!", time.time()-time_start)
                    else:
                        print("time:", time.time()-time_start)
                    time_start = time.time()
                    with self.lock:
                        State_ego = self.shared_list[0].copy()
                        time_receive_gps = self.shared_list[1]
                        time_receive_can= self.shared_list[2]
                        if self.if_radar:
                            time_receive_radar = self.shared_list[3]
                        else:
                            time_receive_radar = 0

                    self.time_in = time.time()
                    self.E2E.input(State_ego)

                    if self.if_radar:   # todo；若存在数字孪生系统，交通流直接读取
                        with self.lock:
                            State_Other = self.State_Other_List[0].copy()
                            # print(State_Other)
                            self.x_next=State_Other["x_other"]
                            self.y_next=State_Other["y_other"]
                            self.v_next=State_Other["v_other"]
                            self.heading_next = State_Other["heading_next"]
                            # if self.step != 0:
                            # _, _, _, head_road = self.E2E.generate_flow(0)
                            # delta_heading_next_array=(np.array(head_road)-np.array(self.heading_next))*math.pi/180
                            # delta_heading_next = delta_heading_next_array.tolist()
                            # else:
                            delta_heading_next = np.zeros(len(self.heading_next), dtype=np.float32).tolist()

                    else:
                        if self.step != 0:
                            self.t_interval = time.time()-time_last
                            # todo:手动产生交通流
                            self.x_next, self.y_next, self.v_next,self.heading_next = self.E2E.generate_flow(self.t_interval)
                        delta_heading_next = np.zeros(len(self.heading_next), dtype=np.float32).tolist()

                    time_last = time.time()

                    front_steer, v = self.E2E.step(self.x_next, self.y_next, self.v_next, delta_heading_next)

                    # flag [0, 0]: 滑行状态
                    control = {'Decision': {
                        'Control': {'Deceleration': 20, 'Torque': 350, 'Dec_flag': 0, 'Tor_flag': 1,
                                    'SteerAngleAim': np.float64(front_steer+1.7),
                                    'VehicleGearAim': 1, 'IsValid': True}}}

                    json_cotrol = json.dumps(control)
                    self.socket_pub.send(json_cotrol.encode('utf-8'))
                    self.time_decision = time.time() - self.time_in
                    self.Time.append(time.time() - self.time_initial)
                    state_ego_dict_real, state_ego_dict, element_ori_real,element_ori = self.E2E.get_info()
                    self.State_other['x_other']=self.x_next
                    self.State_other['y_other'] = self.y_next
                    self.State_other['v_other'] = self.v_next
                    self.State_other['heading_next'] =self.heading_next
                    self.Decision['SteerAngle'] = front_steer
                    self.Decision['VehicleSpeed'] = v

                    with self.lock:
                        self.Info_List[0] = self.step
                        self.Info_List[1] = self.Time.copy()
                        self.Info_List[2] = self.Decision.copy()
                        self.Info_List[3] = State_ego.copy()
                        self.Info_List[4] = self.State_other.copy()

                    self.step += 1

                    if self.if_save:
                        # with open(self.save_path + '/record.txt', 'a') as file_handle:
                        #     file_handle.write(str("保存时间：" + datetime.now().strftime("%Y%m%d_%H%M%S")))
                        #     file_handle.write('\n')
                        if self.Decision != {} and State_ego != {} and self.State_other!={}:
                            # 记录自车控制量
                            # print("Decision", Decision)
                            file_handle.write("Decision ")
                            for k1, v1 in self.Decision.items():
                                file_handle.write(k1 + ":" + str(v1) + ", ")
                                if k1 == 'VehicleSpeed':
                                    file_handle.write('\n')

                            # 记录自车信息
                            # print("State_ego", State_ego)
                            file_handle.write("State_ego ")
                            for k2, v2 in State_ego.items():
                                file_handle.write(k2 + ":" + str(v2) + ", ")
                                if k2 == 'BrkOn':
                                    file_handle.write('\n')

                            #记录他车信息
                            file_handle.write("State_other ")
                            for k3, v3 in self.State_other.items():
                                file_handle.write(k3 + ":" + str(v3) + ", ")
                                if k3 == 'heading_next':
                                    file_handle.write('\n')

                            # 记录时间
                            # print("time_receive_radar", time_receive_radar)
                            file_handle.write("Time:" + str(self.Time) +"time_decision:"+str(self.time_decision)+
                                              "time_receive_gps："+str(time_receive_gps)+"time_receive_can："+str(time_receive_can)+"time_receive_radar："
                                              +str(time_receive_radar)+'\n')
                            #记录新增信息
                            file_handle.write("state_ego_dict_real:" + str(state_ego_dict_real) + '\n')
                            file_handle.write("state_ego_dict:" + str(state_ego_dict) + '\n')
                            file_handle.write("element_ori_real:" + str(element_ori_real) + '\n')
                            file_handle.write("element_ori:" + str(element_ori) + '\n')

# torque maximum: 1023 Nm
# factor: 1
# decelaration maximun: -5m/s^2
# factor: 0.05