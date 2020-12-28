import zmq
import json
import time
import numpy as np


class Subscriber_Radar():
    def __init__(self,shared_list,State_Other_List,lock):
        self.shared_list = shared_list
        self.State_Other_List = State_Other_List
        self.time_start = 0
        self.lock=lock
        self.radar_x = []
        self.radar_y = []
        self.radar_v = []
        context = zmq.Context()
        # radar
        self.socket_radar = context.socket(zmq.SUB)
        self.socket_radar.connect("tcp://127.0.0.1:2323")
        self.socket_radar.setsockopt(zmq.SUBSCRIBE, ''.encode('utf-8'))

    def run(self):
        State_other = {}
        time_receive_radar=0
        while True:
            try:
                data_radar = self.socket_radar.recv(zmq.NOBLOCK).decode('utf-8')
                RadarJson = json.loads(data_radar)
                # print("RadarJson",RadarJson)
                if 'Data'in RadarJson["Radar"]["Radar"]:
                    # print("RadarJson",RadarJson)
                    State_other["x_other"] = list(RadarJson["Radar"]["Radar"]["Data"]["DistLong"].values())
                    State_other["y_other"] = list(RadarJson["Radar"]["Radar"]["Data"]["DistLat"].values())
                    v_Lon = np.array(list(RadarJson["Radar"]["Radar"]["Data"]["VelocityLon"].values()))
                    v_Lat = np.array(list(RadarJson["Radar"]["Radar"]["Data"]["VelocityLat"].values()))
                    State_other["v_other"] = np.sqrt(v_Lon**2+v_Lat**2).tolist()
                    State_other["heading_next"] = list(RadarJson["Radar"]["Radar"]["Data"]["orientation"].values())
                    # print("-----------heading_next------",State_other["heading_next"])
                    time_receive_radar = time.time() - self.time_start
                    self.time_start = time.time()
            except zmq.ZMQError:
                pass
            with self.lock:

                self.shared_list[3] = time_receive_radar
                self.State_Other_List[0]=State_other.copy()
            if time_receive_radar > 0.1:
                print("Subscriber_Radar!!!!!!!!!!!",time_receive_radar)

