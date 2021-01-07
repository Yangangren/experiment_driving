import struct
import time
from collections import OrderedDict

import numpy as np
import zmq


class SubscriberRadar(object):
    def __init__(self, shared_list, lock):
        self.shared_list = shared_list
        self.time_start = 0
        self.lock=lock
        self.radar_x = []
        self.radar_y = []
        self.radar_v = []
        context = zmq.Context()
        # radar
        self.socket_radar = context.socket(zmq.SUB)
        # self.socket_radar.connect("tcp://192.168.8.117:5555")  # "tcp://127.0.0.1:2323"
        self.socket_radar.connect("tcp://10.10.21.151:5555")  # "tcp://127.0.0.1:2323"
        self.socket_radar.setsockopt(zmq.SUBSCRIBE, ''.encode('utf-8'))

    def parse_msg(self, msg):
        data = struct.unpack('64d', msg)
        x_other = [data[i*8+1] - 21277000. for i in range(3)]
        y_other = [data[i*8+2] - 3447700. for i in range(3)]
        v_lon = np.array([data[i*8+3] for i in range(3)])
        v_lat = np.array([data[i*8+4] for i in range(3)])
        v_other = np.sqrt(v_lon ** 2 + v_lat ** 2).tolist()
        phi_other = [data[i*8+7]*180/np.pi for i in range(3)]
        return x_other, y_other, v_other, phi_other

    def run(self):
        state_other = OrderedDict()
        time_receive_radar = 0
        while True:
            try:
                msg = self.socket_radar.recv(zmq.NOBLOCK)
                x_other, y_other, v_other, phi_other = self.parse_msg(msg)
                state_other["x_other"] = x_other
                state_other["y_other"] = y_other
                state_other["v_other"] = v_other
                state_other["phi_other"] = phi_other
                state_other["v_light"] = 0  # todo add v light
                time_receive_radar = time.time() - self.time_start
                self.time_start = time.time()
            except zmq.ZMQError:
                pass
            with self.lock:
                self.shared_list[4] = state_other.copy()
                self.shared_list[5] = time_receive_radar

            if time_receive_radar > 0.1:
                print("Subscriber of radar is more than 0.1s!", time_receive_radar)


def test():
    context = zmq.Context()
    # radar
    socket_radar = context.socket(zmq.SUB)
    socket_radar.connect("tcp://192.168.8.117:5555")  # "tcp://127.0.0.1:2323"
    socket_radar.setsockopt(zmq.SUBSCRIBE, ''.encode('utf-8'))

    msg = socket_radar.recv(zmq.NOBLOCK)
    print(msg)
    data = struct.unpack('64d', msg)
    # id x(21277000) y(3447700) vx, vy, l, w, theta
    print(data)


if __name__ == "__main__":
    test()



