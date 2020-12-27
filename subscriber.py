import zmq
import json
import time
import math
import numpy as np

def load_map():
    lane_width = 3.5
    a = np.loadtxt('./map/roadMap_2.txt')
    road_info = a[:,1:4]
    for i in range(len(road_info[:,0])):
        if road_info[i,2] < 260:
            stop_point = i
            break

    road_info = road_info[0:stop_point,:]

    lane_list=[]
    lane_center_list = []
    for i in range(3):
        lane_list.append(np.zeros([len(road_info[:,0]),2],dtype=np.float64))
    for i in range(2):
        lane_center_list.append(np.zeros([len(road_info[:,0]),2],dtype=np.float64))

    lane_center_list[0][:,0] = road_info[:,0]
    lane_center_list[0][:,1] = road_info[:,1]
    road_angle = road_info[:,2]
    road_angle_rad = (road_angle - 270)*math.pi/180

    lane_list[0][:,0] = lane_center_list[0][:,0] + 0.5*lane_width*np.sin(road_angle_rad)
    lane_list[0][:,1] =lane_center_list[0][:,1]+ 0.5*lane_width*np.cos(road_angle_rad)

    lane_list[1][:,0] = lane_center_list[0][:,0] - 0.5*lane_width*np.sin(road_angle_rad)
    lane_list[1][:,1] =lane_center_list[0][:,1] - 0.5*lane_width*np.cos(road_angle_rad)

    lane_list[2][:,0] = lane_center_list[0][:,0] - 1.5*lane_width*np.sin(road_angle_rad)
    lane_list[2][:,1] = lane_center_list[0][:,1] - 1.5*lane_width*np.cos(road_angle_rad)

    lane_center_list[1][:,0] = lane_center_list[0][:,0] - 1.*lane_width*np.sin(road_angle_rad)
    lane_center_list[1][:,1] = lane_center_list[0][:,1] - 1.*lane_width*np.cos(road_angle_rad)

    return lane_list, lane_center_list, road_angle

class Subscriber():
    def __init__(self,shared_list,Info_List,receive_index,lock):
        self.shared_list = shared_list
        self.Info_List = Info_List
        self.receive_index_shared = receive_index
        self.receive_index = 0
        self.time_start_can = 0.
        self.time_start_gps = 0.
        self.lock=lock
        self.lane_list, self.lane_center_list, self.road_angle = load_map()
        self.position_index = 0
        self.x_bias = 1.4
        self.radar_x = []
        self.radar_y = []
        self.radar_v = []

        context = zmq.Context()
        # can
        self.socket_can = context.socket(zmq.SUB)
        self.socket_can.connect("tcp://127.0.0.1:6974")  # 上车
        self.socket_can.setsockopt(zmq.SUBSCRIBE, "".encode('utf-8'))  # 接收所有消息

        # gps
        self.socket_gps = context.socket(zmq.SUB)
        self.socket_gps.connect("tcp://127.0.0.1:6666")  # 上车
        self.socket_gps.setsockopt(zmq.SUBSCRIBE, "".encode('utf-8'))  # 接收所有消息

    def _get_modified_X_Y(self,ego_x,ego_y,ego_angle):
        for i in range(max(self.position_index-100,0),len(self.road_angle)-1,1):
            if ego_x<=self.lane_list[1][i, 0] and ego_x>=self.lane_list[1][i+1, 0]:
                self.position_index = i
                break
            if i == len(self.road_angle)-2:
                self.position_index = i
        delta_angle = -(ego_angle - self.road_angle[self.position_index])
        x_modified = ego_x - self.x_bias * math.cos(delta_angle*math.pi/180)
        # print("math.cos(delta_angle*math.pi/180)",-math.cos(delta_angle*math.pi/180))
        y_modified = ego_y - self.x_bias * math.sin(delta_angle*math.pi/180)
        # print("math.sin(delta_angle*math.pi/180",-math.sin(delta_angle*math.pi/180))
        return  x_modified,y_modified

    def run(self):
        State_ego = {}
        State_ego['VehicleSPeedAct'] = 0
        State_ego['SteerAngleAct'] = 0
        State_ego['AutoGear'] = 0
        State_ego['GaussX'] = 0
        State_ego['GaussY'] = 0.3 + 0
        State_ego['Heading'] = 1.0 + 0
        State_ego['GpsSpeed'] = 0
        State_ego['NorthVelocity'] = 0
        State_ego['EastVelocity'] = 0
        State_ego['YawRate'] = 0
        State_ego['LongitudinalAcc'] = 0
        State_ego['LateralAcc'] = 0
        State_ego['Longitude'] = 0
        State_ego['Latitude'] = 0
        State_ego['VehicleMode'] = 0
        State_ego['Throttle'] = 0
        State_ego['BrkOn'] = 0
        time_receive_gps = 0
        time_receive_can = 0
        while True:
            try:
                data_gps = self.socket_gps.recv(zmq.NOBLOCK).decode('utf-8')
                if (data_gps != b'null\n'):
                    GpsJson = json.loads(data_gps)
                    x_rear_axle = GpsJson["Gps"]["Gps"]["GaussX"]
                    y_rear_axle = 0.1+GpsJson["Gps"]["Gps"]["GaussY"]
                    State_ego['Heading'] = 1. + GpsJson["Gps"]["Gps"]["Heading"]
                    State_ego['GaussX'], State_ego['GaussY'] = self._get_modified_X_Y(x_rear_axle,y_rear_axle,State_ego['Heading'])
                    State_ego['GpsSpeed'] = GpsJson["Gps"]["Gps"]["GpsSpeed"]
                    State_ego['NorthVelocity'] = GpsJson["Gps"]["Gps"]["NorthVelocity"]
                    State_ego['EastVelocity'] = GpsJson["Gps"]["Gps"]["EastVelocity"]
                    State_ego['YawRate'] = -GpsJson["Gps"]["Gps"]["YawRate"]
                    State_ego['LongitudinalAcc'] = GpsJson["Gps"]["Gps"]["LongitudinalAcc"]
                    State_ego['LateralAcc'] = GpsJson["Gps"]["Gps"]["LateralAcc"]
                    State_ego['Longitude'] = GpsJson["Gps"]["Gps"]["Longitude"]
                    State_ego['Latitude'] = GpsJson["Gps"]["Gps"]["Latitude"]
                    time_receive_gps = time.time() - self.time_start_gps
                    self.time_start_gps = time.time()
            except zmq.ZMQError:
                pass
            try:
                data_can = self.socket_can.recv(zmq.NOBLOCK).decode('utf-8')
                CanJson = json.loads(data_can)
                if CanJson["CAN"]["VehicleStatues"]["IsValid"] == True:
                    State_ego['VehicleSPeedAct'] = CanJson["CAN"]["VehicleStatues"]["VehicleSpeedAct"]
                    State_ego['SteerAngleAct'] = -1.7 + CanJson["CAN"]["VehicleStatues"]["SteerAngleAct"]
                    State_ego['AutoGear'] = CanJson["CAN"]["VehicleStatues"]["AutoGear"]
                    State_ego['VehicleMode'] = CanJson["CAN"]["VehicleStatues"]["VehicleMode"]
                    State_ego['Throttle'] = CanJson["CAN"]["VehicleStatues"]["Throttle"]
                    State_ego['BrkOn'] = CanJson["CAN"]["VehicleStatues"]["BrkOn"]
                    time_receive_can = time.time() - self.time_start_can
                    self.time_start_can = time.time()
            except zmq.ZMQError:
                pass
            self.receive_index +=1
            with self.lock:
                self.shared_list[0] = State_ego.copy()
                self.shared_list[1] = time_receive_gps
                self.shared_list[2] = time_receive_can
            self.receive_index_shared.value = self.receive_index
            if time_receive_can > 0.1 or time_receive_gps > 0.1:
                print("Subscriber!!!!!!!!!!!1",time_receive_can,time_receive_gps)

