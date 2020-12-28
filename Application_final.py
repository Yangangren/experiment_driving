from __future__ import print_function
import torch
import numpy as np
np.set_printoptions(suppress=True)
import time
from Model import PolicyNet,QNet, PINet
import argparse
#import gym
import matplotlib.pyplot as plt
import math
import gym
import os


def built_parser():
    parser = argparse.ArgumentParser()

    '''Task'''
    parser.add_argument('--element_dim', dest='list', type=int, default=[])
    parser.add_argument('--atom_dim', dest='list', type=int, default=[])
    parser.add_argument('--state_dim', dest='list', type=int, default=[])
    parser.add_argument('--action_dim', type=int, default=[])
    parser.add_argument('--action_high', dest='list', type=float, default=[],action="append")
    parser.add_argument('--action_low', dest='list', type=float, default=[],action="append")
    parser.add_argument("--NN_type", default="mlp", help='mlp or CNN')

    '''general hyper-parameters'''
    parser.add_argument('--critic_lr' , type=float, default=0.00008,help='critic learning rate')
    parser.add_argument('--actor_lr', type=float, default=0.00005, help='actor learning rate')
    parser.add_argument('--end_lr', type=float, default=0.00002, help='learning rate at the end point of annealing')
    parser.add_argument('--tau', type=float, default=0.001, help='learning rate for target')
    parser.add_argument('--gamma', type=float, default=0.99, help='discount factor for rewards (default: 0.99)')
    parser.add_argument('--delay_update', type=int, default=2, help='update interval for policy, target')
    parser.add_argument('--reward_scale', type=float, default=0.2, help='reward = reward_scale * environmental reward ')

    '''hyper-parameters for soft-Q based algorithm'''
    parser.add_argument('--alpha_lr', type=float, default=0.0001,help='learning rate for temperature')
    parser.add_argument('--target_entropy',  default="auto",help="auto or some value such as -2")

    '''hyper-parameters for soft-Q based algorithm'''
    parser.add_argument('--max_step', type=int, default=500, help='maximum length of an episode')
    parser.add_argument('--buffer_size_max', type=int, default=500000, help='replay memory size')
    parser.add_argument('--initial_buffer_size', type=int, default=2000, help='Learner waits until replay memory stores this number of transition')
    parser.add_argument('--batch_size', type=int, default=128)
    parser.add_argument('--num_hidden_cell', type=int, default=128)

    '''other setting'''
    parser.add_argument("--max_train", type=int, default=800000)
    parser.add_argument("--decay_T_max", type=int, default=parser.parse_args().max_train, help='for learning rate annealing')
    parser.add_argument('--load_param_period', type=int, default=20)
    parser.add_argument('--save_model_period', type=int, default=5000)
    parser.add_argument('--init_time', type=float, default=0.00)
    parser.add_argument('--seed', type=int, default=1, help='initial seed (default: 1)')

    '''parallel architecture'''
    parser.add_argument("--num_buffers", type=int, default=3)
    parser.add_argument("--num_learners", type=int, default=5) #note that too many learners may cause bad update for shared network
    parser.add_argument("--num_MBlearners", type=int, default=5) #note that too many learners may cause bad update for shared network
    parser.add_argument("--num_actors", type=int, default=10)

    '''method list'''
    parser.add_argument("--distributional_Q", default=True)
    parser.add_argument("--stochastic_actor", default=True)
    parser.add_argument("--double_Q", default=False)
    parser.add_argument("--double_actor", default=False)
    parser.add_argument("--adaptive_bound", default=False)
    parser.add_argument('--alpha', default="auto", help="auto or some value such as 1")
    parser.add_argument('--TD_bound', type=float, default=20)
    parser.add_argument('--bound',  default=True)
    parser.add_argument('--if_sensor_used', default=True)
    parser.add_argument('--veh_num', default="auto")
    parser.add_argument('--if_PI', default=True)

    return parser.parse_args()


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


class Application():
    def __init__(self, args):
        super(Application, self).__init__()
        seed = args.seed
        np.random.seed(seed)
        torch.manual_seed(seed)
        self.args = args
        self.device = torch.device("cpu")
        self.load_index = self.args.max_train

        self.PI_net = PINet(args).to(self.device)

        self.PI_net.load_state_dict(torch.load('./Net1-0_0816/PI_' + str(self.load_index) + '.pkl',map_location='cpu'))

        self.actor = PolicyNet(args).to(self.device)
        self.actor.load_state_dict(torch.load('./Net1-0_0816/policy1_' + str(self.load_index) + '.pkl',map_location='cpu'))

        # self.Q_net0 = QNet(args).to(self.device)
        # self.Q_net0.load_state_dict(torch.load('./Net1-0_0816/Q1_' + str(self.load_index) + '.pkl',map_location='cpu'))
        self.speed_limit_max = 20 / 3.6
        self.steer_factor = 20
        self.action_multiplier = np.array([math.pi/(9*self.steer_factor),2], dtype=np.float32)
        self.safe_gap = 0.
        self.noise_factor = 0.

        self.step = 0
        self.frequency = 10
        self.lanekeeping_max = 5
        self.lane_number = 2
        self.lane_width = 3.5

        self.car_length = 4.5
        self.car_width = 1.855


        self.road_width = self.lane_number*self.lane_width
        self.detect_rear_dist = 80
        self.detect_front_dist = 100

        self.other_length = 5.169
        self.other_width = 2.392
        self.position_index = 0
        self.position_other = np.zeros(20,dtype=np.int)

        self.max_state_other = np.array([100, self.road_width, self.speed_limit_max, math.pi / 6, 6.5, 2.5],dtype=np.float32)

        self.max_state_ego = np.array(
            [self.speed_limit_max, 1, math.pi / 6, math.pi / 6, math.pi / 6, 4, self.lane_width / 2, self.road_width,
             self.road_width, self.speed_limit_max, self.speed_limit_max, self.lane_number - 1, 5, 2, \
             math.pi / 6, math.pi / 6, math.pi / 6, math.pi / 6, math.pi / 6], dtype=np.float32)




        self.wheel_steer_bound = [-1.5*math.pi,1.5*math.pi]

        self.lane_list, self.lane_center_list, self.road_angle = load_map()

    def _get_dist_to_roadside(self, lane_index, dist_to_center):
        dist_left =  (self.lane_number -1 - lane_index)*self.lane_width + self.lane_width/2 - dist_to_center
        dist_right =  lane_index*self.lane_width + self.lane_width/2 + dist_to_center
        return dist_left, dist_right

    def _get_road_related_info(self,ego_x,ego_y,ego_angle):
        self.ego_x = ego_x
        self.ego_y = ego_y
        if self.step>=1:
            self.lane_index_old = self.lane_index
        for i in range(max(self.position_index-100,0),len(self.road_angle)-1,1):
            if self.ego_x<=self.lane_list[1][i, 0] and self.ego_x>=self.lane_list[1][i+1, 0]:
                self.position_index = i
                if ego_y<=self.lane_list[1][i,1]:
                    lane_index = 1
                else:
                    lane_index = 0
                break
            if i == len(self.road_angle)-2:
                lane_index = 0
                self.position_index = i
        # print("lane_index",lane_index,"road_angle",len(self.road_angle))
        if ego_y>self.lane_center_list[lane_index][self.position_index,1]:
            dist_flag = -1.0
        else:
            dist_flag = 1.0
        dist2center = dist_flag*np.sqrt((ego_x-self.lane_center_list[lane_index][self.position_index,0])**2+(ego_y-self.lane_center_list[lane_index][self.position_index,1])**2)
        dist_left,dist_right = self._get_dist_to_roadside(lane_index,dist2center)
        self.lane_index = lane_index
        self.dist2center = np.float32(dist2center)
        self.dist_left = np.float32(dist_left)
        self.dist_right = np.float32(dist_right)
        self.delta_angle = -(ego_angle - self.road_angle[self.position_index])
        self.current_road_angle = self.road_angle[self.position_index]

        # print(self.lane_index,self.dist2center,self.dist_left ,self.dist_right,self.delta_angle )

    def _get_next_vehicle(self,t_interval):
        lane_list=[]
        for i in range(len(self.x_other)):
            for j in range(max(self.position_other[i]-100,0),len(self.road_angle)-1,1):
                if self.x_other[i]<=self.lane_center_list[0][j, 0] and self.x_other[i]>=self.lane_center_list[0][j+1, 0]:
                    index =j
                    break
            dist_1_0 = np.sqrt((self.x_other[i]-self.lane_center_list[0][index,0])**2+(self.y_other[i]-self.lane_center_list[0][index,1])**2)
            dist_1_1 = np.sqrt((self.x_other[i]-self.lane_center_list[1][index,0])**2+(self.y_other[i]-self.lane_center_list[1][index,1])**2)
            self.position_other[i]=index
            if dist_1_0 < dist_1_1:
                lane_list.append(0)
            else:
                lane_list.append(1)

        x_next=[]
        y_next=[]
        heading_next = []

        for i in range(len(self.x_other)):
            x_next.append(self.x_other[i]-self.v_other[i]*t_interval)
            if len(self.road_angle)-self.position_other[i]<1000:
                x_next[i] = self.lane_center_list[lane_list[i]][0, 0]
                y_next.append(self.lane_center_list[lane_list[i]][0, 1])
                heading_next.append(self.road_angle[0])
                self.position_other[i] = 0
            else:
                y_next.append(self.lane_center_list[lane_list[i]][self.position_other[i], 1])
                heading_next.append(self.road_angle[self.position_other[i]])
                # for j in range(len(self.road_angle) - 1):
                #     if x_next[i]<=self.lane_center_list[lane_list[i]][j, 0] and x_next[i]>=self.lane_center_list[lane_list[i]][j+1, 0]:
                #         y_next.append(self.lane_center_list[lane_list[i]][j, 1])

        return x_next,y_next,self.v_other, heading_next

    def _get_ego_state(self,v=1,v_lat=0,yaw_rate=0,wheel_steer=0,acc=0):
        if self.step == 0:
            self.lanekeeping_time = 4
        else:
            if self.lane_index == self.lane_index_old:
                self.lanekeeping_time += 1/self.frequency
            else:
                self.lanekeeping_time = 0
            self.lanekeeping_time = min(self.lanekeeping_time, self.lanekeeping_max)

        self.state_ego_dict_real = dict(v=v,
                               v_lat=v_lat,
                               yaw_rate=yaw_rate* math.pi/180,
                               heading=self.delta_angle * math.pi/180,
                               steer=wheel_steer/ self.steer_factor * math.pi/180,
                               acc=acc,
                               #v_long=self.ego_dynamics['Longitudinal_speed'],
                               dist2center=self.dist2center,
                               dist2road_bound_1=self.dist_left-self.car_width/2-self.safe_gap,
                               dist2road_bound_2=self.dist_right-self.car_width/2-self.safe_gap,
                               dist2speed_limit=self.speed_limit_max-v,
                               dist2speed_limit_low=v-0,
                               lane=self.lane_index,
                               other_veh_num=self.veh_num,
                               lanekeeping_time = self.lanekeeping_time,
                               future_heading_10 =0,
                               future_heading_20=0,
                               future_heading_30=0,
                               future_heading_40=0,
                               future_heading_50=0,
                              )
        position_noise = self.noise_factor*np.clip(np.random.normal(0, 0.033), -0.1, 0.1)
        heading_noise = self.noise_factor*np.clip(np.random.normal(0, 0.33), -1, 1)

        self.state_ego_dict = dict(v=v,
                               v_lat=v_lat,
                               yaw_rate=yaw_rate* math.pi/180,
                               heading=(self.delta_angle+heading_noise) * math.pi/180,
                               steer=wheel_steer/ self.steer_factor * math.pi/180,
                               acc=acc,
                               #v_long=self.ego_dynamics['Longitudinal_speed'],
                               dist2center=self.dist2center+position_noise,
                               dist2road_bound_1=self.dist_left-self.car_width/2-self.safe_gap - position_noise,
                               dist2road_bound_2=self.dist_right-self.car_width/2-self.safe_gap + position_noise,
                               dist2speed_limit=self.speed_limit_max-v,
                               dist2speed_limit_low=v-0,
                               lane=self.lane_index,
                               other_veh_num=self.veh_num,
                               lanekeeping_time = self.lanekeeping_time,
                               future_heading_10 =0,
                               future_heading_20=0,
                               future_heading_30=0,
                               future_heading_40=0,
                               future_heading_50=0,
                              )

        self.state_ego = np.array(list(self.state_ego_dict.values()),dtype=np.float32)/self.max_state_ego

    def _get_other_info(self,v_ego,x,y,v,heading):
        self.x_other = x
        self.y_other = y
        self.v_other = v
        heading_other = heading
        self.veh_num = 0
        veh_index =[]
        for i in range(len(x)):
            if self.ego_x - x[i] < self.detect_front_dist and x[i]-self.ego_x < self.detect_rear_dist:
                self.veh_num+=1
                veh_index.append(i)
        if self.veh_num!=0:
            self.element_ori = np.zeros([len(veh_index),6],dtype=np.float32)
            self.element_ori_real = self.element_ori.copy()

            for i in range(len(veh_index)):
                other_x =x[veh_index[i]]
                other_y =y[veh_index[i]]
                other_v = v[veh_index[i]]
                other_heading = heading_other[veh_index[i]]
                delta_x = self.ego_x - other_x
                delta_y = self.ego_y - other_y
                dist_ego2other = np.sqrt(delta_x ** 2 + delta_y ** 2)
                if delta_x >= 0:
                    heading_ego2other = np.arctan(delta_y / (delta_x + 1e-6))
                else:
                    heading_ego2other = np.arctan(delta_y / (delta_x - 1e-6)) + math.pi
                if heading_ego2other >= math.pi:
                    heading_ego2other = heading_ego2other - 2 * math.pi
                elif heading_ego2other < -math.pi:
                    heading_ego2other = heading_ego2other + 2 * math.pi
                delta_heading = heading_ego2other-(270*math.pi/180-self.current_road_angle*math.pi/180)
                relate_x = dist_ego2other * np.cos(delta_heading)
                relate_y = dist_ego2other * np.sin(delta_heading)
                self.element_ori_real[i] = np.array([relate_x,
                                                 relate_y,
                                                 other_v-v_ego,
                                                 other_heading,
                                                 self.other_length,
                                                 self.other_width], dtype=np.float32)
                self.element_ori[i] = np.array([relate_x+self.noise_factor*np.clip(np.random.normal(0,0.1),-0.3,0.3),
                                                 relate_y+self.noise_factor*np.clip(np.random.normal(0,0.1),-0.3,0.3),
                                                 other_v-v_ego+self.noise_factor*np.clip(np.random.normal(0,0.1),-0.3,0.3),
                                                 other_heading+self.noise_factor*np.clip(np.random.normal(0,0.05),-0.15,0.15),
                                                 self.other_length+self.noise_factor*np.clip(np.random.normal(0,0.02),-0.06,0.06),
                                                 self.other_width+self.noise_factor*np.clip(np.random.normal(0,0.02),-0.06,0.06)], dtype=np.float32)
        else:
            self.veh_num = 1
            self.element_ori = np.array([[self.detect_front_dist, 0, 0, 0, self.other_length, self.other_width]],dtype=np.float32)
            self.element_ori_real = np.array([[self.detect_front_dist, 0, 0, 0, self.other_length, self.other_width]],dtype=np.float32)

        # f2=plt.figure(0,figsize=(20, 5))
        # plt.ion()
        # for i in range(len(self.lane_list)):
        #     plt.plot(self.lane_list[i][:,0], self.lane_list[i][:,1], color='green', linewidth='2')
        # for i in range(len(self.lane_center_list)):
        #     plt.plot(self.lane_center_list[i][:,0], self.lane_center_list[i][:,1], color='red', linewidth='2')
        # plt.scatter(self.ego_x,self.ego_y, color='red')
        # for i in range(len(x)):
        #     plt.scatter(x[i], y[i], color='blue')
        # ax = plt.gca()
        # ax.set_aspect('equal')
        # ax.invert_xaxis()
        # ax.invert_yaxis()
        # plt.title(['relate_x:' + str(self.element_ori[:,0]) + ' relate_y:' + str(self.element_ori[:,1])+ "  relate_angle:"+str(round(self.delta_angle,2))+\
        #            'lane:' + str(round(self.lane_index,2)) + ' dist2center:' + str(round(self.dist2center,2))+ "  distleft:"+str(round(self.dist_left,2))+\
        #            ' dist_right:' + str(round(self.dist_right,2)) + ' road angle:' + str(round(self.current_road_angle,2))])
        # plt.pause(0.01)
        # f2.clf()
        # plt.figure(figsize=(20, 5))
        # plt.plot(road_info[:,0], road_info[:,2], color='green', linewidth='2')
        # ax = plt.gca()
        # ax.set_aspect('equal')
        # ax.invert_xaxis()
        # plt.show()
        self.element = self.element_ori/self.max_state_other


    def simu(self):
        self.env = gym.make("Experiment-v3")
        time_init = time.time()
        step = 0
        observation = self.env.reset(random=False,sensor_used=self.args.if_sensor_used,veh_num=self.args.veh_num,simu=True)
        self.episode_step = 0
        simu_state_list = []
        while True:
            state_ego = observation[0]
            element = observation[1]
            veh_number = observation[1].shape[0]
            state_other_vector = observation[2]
            simu_state_list.append(observation[3])
            element_tensor = torch.FloatTensor(element.copy()).to(self.device)
            ego_tensor = torch.FloatTensor(state_ego.copy()).to(self.device)
            state_other = torch.sum(self.PI_net.evaluate(element_tensor), dim=0)
            state_tensor = torch.cat([ego_tensor, state_other.detach()])
            self.u, log_prob, _ = self.actor.get_action(state_tensor.unsqueeze(0), True)
            self.u = self.u.squeeze(0)

            observation, self.reward, self.done, _ = self.env.step(self.u)
            self.env.render(mode='human')
            step += 1
            if self.done or step==1000:
                time.sleep(1)
                print('method', self.args.method, 'step',step, 'time', time.time()-time_init)
                print("!!!!!!!!!!!!!!!")
                break

    def control_step(self):
        time_init = time.time()
        element_tensor = torch.FloatTensor(self.element.copy()).to(self.device)
        ego_tensor = torch.FloatTensor(self.state_ego.copy()).to(self.device)
        state_other = torch.sum(self.PI_net.evaluate(element_tensor), dim=0)
        state_tensor = torch.cat([ego_tensor, state_other.detach()])
        self.u, log_prob, _ = self.actor.get_action(state_tensor.unsqueeze(0), True)
        self.u = self.u.squeeze(0)
        self.step+=1
        #print(time.time()-time_init)

        return self.u


class E2E():
    def __init__(self):
        super(E2E, self).__init__()
        args = built_parser()
        args.element_dim = (6,)
        args.atom_dim = 20 * args.element_dim[0]
        args.state_dim = 139
        args.action_dim = 2
        args.action_high = [1., 1.]
        args.action_low = [-1., -2.]
        args.seed = np.random.randint(0, 30)
        self.appl = Application(args)

    def step(self, x, y, v, heading):
        # y=[3447704.599198, 3447698.612461]
        self.appl._get_road_related_info(self.ego_x, self.ego_y, self.heading)
        self.appl._get_other_info(self.ego_v,x,y,v,heading)
        self.appl._get_ego_state(v=self.ego_v, v_lat=self.v_lat, yaw_rate=self.yaw_rate, wheel_steer=self.wheel_steer_angle, acc=self.acc)
        output = self.appl.control_step()
        # TODO：动作是方向盘转角增量和加速度，范围为[-pi/180, pi/180], [-4, 2]
        action = output * self.appl.action_multiplier
        # todo:方向盘的转角增量
        self.delta_wheel_steer = action[0] * self.appl.steer_factor

        self.wheel_steer_out = self.wheel_steer_angle*math.pi/180+self.delta_wheel_steer #np.random.choice([10,20,5])*math.pi/180
        self.wheel_steer_out = max(self.wheel_steer_out, self.appl.wheel_steer_bound[0])
        self.wheel_steer_out = min(self.wheel_steer_out, self.appl.wheel_steer_bound[1])
        self.acc=action[1]

        # if abs(self.acc) <= 0.02:
        #     delta_V = 0
        # elif self.acc > 0.02:
        #     delta_V = min(1/3.6, self.acc)
        # else:
        #     delta_V = min(-0.5/3.6, -self.acc / self.appl.frequency)
        delta_V = np.clip(self.acc, -1/3.6, 1/3.6)
        return self.wheel_steer_out*180/math.pi, np.clip(self.ego_v+delta_V, 0, self.appl.speed_limit_max)

    def get_info(self):
        return self.appl.state_ego_dict_real,self.appl.state_ego_dict, self.appl.element_ori_real,self.appl.element_ori

    def generate_flow(self,t_interval):
        x_next,y_next,v_next,heading_next = self.appl._get_next_vehicle(t_interval)
        return x_next,y_next,v_next, heading_next

    def input(self, info):
        self.ego_x = info['GaussX']
        self.ego_y = info['GaussY']
        self.ego_v = info['GpsSpeed']
        self.heading = info['Heading']
        self.yaw_rate = info['YawRate']
        self.wheel_steer_angle = info['SteerAngleAct']
        self.acc = info['LongitudinalAcc']
        self.v_north = info['NorthVelocity']
        self.v_east = info['EastVelocity']
        self.v_lat =0#self.ego_v*math.sin((self.heading -(math.atan(self.v_north/(abs(self.v_east)+1e-6))*180/math.pi+270))*math.pi/180)

    def simu(self):
        self.appl.simu()


def main():
     ego = dict(GaussX=21277096.450883,
                GaussY=3447698.635432,
                GpsSpeed=2,
                Heading=269,
                NorthVelocity=0,
                EastVelocity=0,
                YawRate=0,
                LongitudinalAcc=0,
                SteerAngleAct=0,
                )

     e2e=E2E()
     e2e.input(ego)

     e2e.step(x=[21276975.449190, 21277025.533504, 21276952.371644, 21276983.186771,21277110.689378],\
              y=[3447700.612461, 3447702.799198, 3447705.798612, 3447701.055639, 3447698.148592],
              v=[5,5,3,5,2],\
              heading=[271.,271.,271.,271.,271.])
     x_next,y_next,v_next,heading = e2e.generate_flow(0.1)
     while True:
         time1=time.time()
         e2e.input(ego)
         angle,acc=e2e.step(x=x_next,y=y_next,v=v_next, heading=heading)
         # angle,acc=e2e.step(x=x_next,y=y_next,v=v_next, heading=0*heading)
         x_next, y_next, v_next,_ = e2e.generate_flow(0.1)
         time2=time.time()

         print(angle, acc, time2-time1)


if __name__ == "__main__":
    os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"
    main()



