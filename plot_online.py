import matplotlib.pyplot as plt
import numpy as np
import math
from math import cos, sin, pi
import time
from utils.misc import TimerStat

CROSSROAD_SIZE = 22
LANE_WIDTH = 3.5
START_OFFSET = 3
LANE_NUMBER = 1
EGO_LENGTH = 4.8
EGO_WIDTH = 2.0
STATE_OTHER_LENGTH = EGO_LENGTH
STATE_OTHER_WIDTH = EGO_WIDTH

def rotate_coordination(orig_x, orig_y, orig_d, coordi_rotate_d):
    """
    :param orig_x: original x
    :param orig_y: original y
    :param orig_d: original degree
    :param coordi_rotate_d: coordination rotation d, positive if anti-clockwise, unit: deg
    :return:
    transformed_x, transformed_y, transformed_d(range:(-180 deg, 180 deg])
    """

    coordi_rotate_d_in_rad = coordi_rotate_d * math.pi / 180
    transformed_x = orig_x * math.cos(coordi_rotate_d_in_rad) + orig_y * math.sin(coordi_rotate_d_in_rad)
    transformed_y = -orig_x * math.sin(coordi_rotate_d_in_rad) + orig_y * math.cos(coordi_rotate_d_in_rad)
    transformed_d = orig_d - coordi_rotate_d
    if transformed_d > 180:
        while transformed_d > 180:
            transformed_d = transformed_d - 360
    elif transformed_d <= -180:
        while transformed_d <= -180:
            transformed_d = transformed_d + 360
    else:
        transformed_d = transformed_d
    return transformed_x, transformed_y, transformed_d

def find_closest_point(path, xs, ys, ratio=6):
    path_len = len(path[0])
    reduced_idx = np.arange(0, path_len, ratio)
    reduced_len = len(reduced_idx)
    reduced_path_x, reduced_path_y = path[0][reduced_idx], path[1][reduced_idx]
    xs_tile = np.tile(np.reshape(xs, (-1, 1)), (1, reduced_len))
    ys_tile = np.tile(np.reshape(ys, (-1, 1)), (1, reduced_len))
    pathx_tile = np.tile(np.reshape(reduced_path_x, (1, -1)), (len(xs), 1))
    pathy_tile = np.tile(np.reshape(reduced_path_y, (1, -1)), (len(xs), 1))

    dist_array = np.square(xs_tile - pathx_tile) + np.square(ys_tile - pathy_tile)

    indexes = np.argmin(dist_array, 1) * ratio
    if len(indexes) > 1:
        indexes = indexes[0]
    points = path[0][indexes], path[1][indexes], path[2][indexes]
    return indexes, points


class Plot():
    def __init__(self,Info_List, lock, task):
        self.Info_List = Info_List
        self.lock = lock
        self.task = task
        self.step_old = -1
        self.acc_timer = TimerStat()
        step = []
        left_construct_traj = np.load('./map/left_construct.npy')
        straight_construct_traj = np.load('./map/straight_construct.npy')
        right_construct_traj = np.load('./map/right_construct.npy')
        left_gps = np.load('./map/left_ref_cut.npy')
        straight_gps = np.load('./map/straight_ref_cut.npy')
        right_gps = np.load('./map/right_ref_cut.npy')
        self.ref_path_all = {'left':left_construct_traj, 'straight': straight_construct_traj, 'right':right_construct_traj,
                    'left_ref':left_gps, 'straight_ref':straight_gps, 'right_ref':right_gps}
        self.ref_path = self.ref_path_all[task]

    def run(self):
        square_length = CROSSROAD_SIZE
        start_offset = START_OFFSET
        extension = 40
        lane_width = LANE_WIDTH
        light_line_width = 3
        dotted_line_style = '--'
        solid_line_style = '-'
        start_time = 0
        v_old = 0.
        # plt.figure(0)
        # plt.ion()
        # plt.cla()
        plt.title("Crossroad")
        ax = plt.axes(xlim=(-square_length / 2 - extension, square_length / 2 + extension),
                      ylim=(-square_length / 2 - extension, square_length / 2 + extension))
        plt.axis("equal")
        plt.axis('off')

        # ax.add_patch(plt.Rectangle((-square_length / 2, -square_length / 2),
        #                            square_length, square_length, edgecolor='black', facecolor='none'))


        def is_in_plot_area(x, y, tolerance=5):
            if -square_length / 2 - extension + tolerance < x < square_length / 2 + extension - tolerance and \
                    -square_length / 2 - extension + tolerance < y < square_length / 2 + extension - tolerance:
                return True
            else:
                return False






        def draw_rotate_rec(x, y, a, l, w, color, linestyle='-'):
            RU_x, RU_y, _ = rotate_coordination(l / 2, w / 2, 0, -a)
            RD_x, RD_y, _ = rotate_coordination(l / 2, -w / 2, 0, -a)
            LU_x, LU_y, _ = rotate_coordination(-l / 2, w / 2, 0, -a)
            LD_x, LD_y, _ = rotate_coordination(-l / 2, -w / 2, 0, -a)
            ax.plot([RU_x + x, RD_x + x], [RU_y + y, RD_y + y], color=color, linestyle=linestyle)
            ax.plot([RU_x + x, LU_x + x], [RU_y + y, LU_y + y], color=color, linestyle=linestyle)
            ax.plot([LD_x + x, RD_x + x], [LD_y + y, RD_y + y], color=color, linestyle=linestyle)
            ax.plot([LD_x + x, LU_x + x], [LD_y + y, LU_y + y], color=color, linestyle=linestyle)

        def plot_phi_line(x, y, phi, color):
            line_length = 5
            x_forw, y_forw = x + line_length * cos(phi * pi / 180.), \
                             y + line_length * sin(phi * pi / 180.)
            plt.plot([x, x_forw], [y, y_forw], color=color, linewidth=0.5)

        while True:
            # start_time = time.time()
            plt.cla()
            plt.axis('off')
            ax.add_patch(plt.Rectangle((-square_length / 2 - extension, -square_length / 2 - extension - start_offset),
                                       square_length + 2 * extension, square_length + 2 * extension + start_offset,
                                       edgecolor='black',
                                       facecolor='none'))

            # ----------arrow--------------
            plt.arrow(lane_width / 2, -square_length / 2 - 10, 0, 5, color='b')
            plt.arrow(lane_width / 2, -square_length / 2 - 10 + 5, -0.5, 0, color='b', head_width=1)
            plt.arrow(lane_width / 2, -square_length / 2 - 10, 0, 5, color='b', head_width=1)
            plt.arrow(lane_width / 2, -square_length / 2 - 10, 0, 5, color='b')
            plt.arrow(lane_width / 2, -square_length / 2 - 10 + 5, 0.5, 0, color='b', head_width=1)

            # ----------horizon--------------
            plt.plot([-square_length / 2 - extension, -square_length / 2], [0, 0], color='black')
            plt.plot([square_length / 2 + extension, square_length / 2], [0, 0], color='black')

            #
            for i in range(1, LANE_NUMBER + 1):
                linestyle = dotted_line_style if i < LANE_NUMBER else solid_line_style
                plt.plot([-square_length / 2 - extension, -square_length / 2], [i * lane_width, i * lane_width],
                         linestyle=linestyle, color='black')
                plt.plot([square_length / 2 + extension, square_length / 2], [i * lane_width, i * lane_width],
                         linestyle=linestyle, color='black')
                plt.plot([-square_length / 2 - extension, -square_length / 2], [-i * lane_width, -i * lane_width],
                         linestyle=linestyle, color='black')
                plt.plot([square_length / 2 + extension, square_length / 2], [-i * lane_width, -i * lane_width],
                         linestyle=linestyle, color='black')

            # ----------vertical----------------
            plt.plot([0, 0], [-square_length / 2 - extension - start_offset, -square_length / 2 - start_offset],
                     color='black')
            plt.plot([0, 0], [square_length / 2 + extension, square_length / 2], color='black')

            #
            for i in range(1, LANE_NUMBER + 1):
                linestyle = dotted_line_style if i < LANE_NUMBER else solid_line_style
                plt.plot([i * lane_width, i * lane_width],
                         [-square_length / 2 - extension - start_offset, -square_length / 2 - start_offset],
                         linestyle=linestyle, color='black')
                plt.plot([i * lane_width, i * lane_width], [square_length / 2 + extension, square_length / 2],
                         linestyle=linestyle, color='black')
                plt.plot([-i * lane_width, -i * lane_width],
                         [-square_length / 2 - extension - start_offset, -square_length / 2 - start_offset],
                         linestyle=linestyle, color='black')
                plt.plot([-i * lane_width, -i * lane_width], [square_length / 2 + extension, square_length / 2],
                         linestyle=linestyle, color='black')

            # ----------Oblique--------------
            plt.plot([LANE_NUMBER * lane_width, square_length / 2],
                     [-square_length / 2 - start_offset, -LANE_NUMBER * lane_width],
                     color='black')
            plt.plot([LANE_NUMBER * lane_width, square_length / 2], [square_length / 2, LANE_NUMBER * lane_width],
                     color='black')
            plt.plot([-LANE_NUMBER * lane_width, -square_length / 2],
                     [-square_length / 2 - start_offset, -LANE_NUMBER * lane_width],
                     color='black')
            plt.plot([-LANE_NUMBER * lane_width, -square_length / 2], [square_length / 2, LANE_NUMBER * lane_width],
                     color='black')



            # ----------------------ref_path--------------------

            plot_ref = ['left','straight','right', 'left_ref','straight_ref','right_ref']  # 'left','straight','right', 'left_ref','straight_ref','right_ref'
            for ref in plot_ref:
                ref_path = self.ref_path_all[ref]
                ax.plot(ref_path[0], ref_path[1])

            ax.plot(self.ref_path[0], self.ref_path[1], color='g')  # todo:


            State_others = self.Info_List[4].copy()
            # plot cars
            for i in range(len(State_others['x_other'])): # TODO:
                veh_x = State_others['x_other'][i]
                veh_y = State_others['y_other'][i]
                veh_phi = State_others['phi_other'][i]
                veh_l = STATE_OTHER_LENGTH
                veh_w = STATE_OTHER_WIDTH
                if is_in_plot_area(veh_x, veh_y):
                    plot_phi_line(veh_x, veh_y, veh_phi, 'black')
                    draw_rotate_rec(veh_x, veh_y, veh_phi, veh_l, veh_w, 'black')

            v_light = State_others['v_light']
            if v_light == 0:
                v_color, h_color = 'black', 'black'     #  'green', 'red'
            elif v_light == 1:
                v_color, h_color = 'black', 'black'
            elif v_light == 2:
                v_color, h_color = 'black', 'black'
            else:
                v_color, h_color = 'black', 'black'


            plt.plot([0, lane_width], [-square_length / 2 - start_offset, -square_length / 2 - start_offset],
                     color=v_color, linewidth=light_line_width)

            plt.plot([-lane_width, 0], [square_length / 2, square_length / 2],
                     color=v_color, linewidth=light_line_width)

            plt.plot([-square_length / 2, -square_length / 2], [0, -lane_width],
                     color=h_color, linewidth=light_line_width)

            plt.plot([square_length / 2, square_length / 2], [lane_width, 0],
                     color=h_color, linewidth=light_line_width)



            State_ego = self.Info_List[3].copy()
            ego_v = State_ego['VehicleSPeedAct']
            ego_steer = State_ego['SteerAngleAct']
            ego_gear = State_ego['AutoGear']
            ego_gps_v = State_ego['GpsSpeed']
            ego_north_v = State_ego['NorthVelocity']
            ego_east_v = State_ego['EastVelocity']
            ego_yaw_rate = State_ego['YawRate']
            ego_long_acc = State_ego['LongitudinalAcc']
            ego_lat_acc = State_ego['LateralAcc']
            ego_throttle = State_ego['Throttle']
            ego_brk = State_ego['BrkOn']
            ego_x = State_ego['GaussX']
            ego_y = State_ego['GaussY']
            ego_longitude = State_ego['Longitude']
            ego_latitude = State_ego['Latitude']
            ego_phi = State_ego['Heading']
            ego_l = EGO_LENGTH
            ego_w = EGO_WIDTH
            plot_phi_line(ego_x, ego_y, ego_phi, 'red')
            draw_rotate_rec(ego_x, ego_y, ego_phi, ego_l, ego_w, 'red')
            time1 = time.time()
            delta_time = time1-start_time
            acc_actual = (ego_v-v_old)/delta_time
            self.acc_timer.push(acc_actual)
            start_time = time.time()
            v_old = ego_v

            indexs, points = find_closest_point(self.ref_path, np.array([ego_x], np.float32),
                                                              np.array([ego_y], np.float32))
            path_x, path_y, path_phi = points[0][0], points[1][0], points[2][0]
            plt.plot(path_x, path_y, 'g.')
            delta_x, delta_y, delta_phi = ego_x - path_x, ego_y - path_y, ego_phi - path_phi

            # plot ego dynamics

            text_x, text_y_start = -110, 60
            ge = iter(range(0, 1000, 4))
            plt.text(text_x, text_y_start - next(ge), 'ego_GaussX: {:.2f}m'.format(ego_x))
            plt.text(text_x, text_y_start - next(ge), 'ego_GaussY: {:.2f}m'.format(ego_y))
            plt.text(text_x, text_y_start - next(ge), 'gps_v: {:.2f}m/s'.format(ego_gps_v))
            plt.text(text_x, text_y_start - next(ge), 'north_v: {:.2f}m/s'.format(ego_north_v))
            plt.text(text_x, text_y_start - next(ge), 'east_v: {:.2f}m/s'.format(ego_east_v))
            plt.text(text_x, text_y_start - next(ge), 'yaw_rate: {:.2f}rad/s'.format(ego_yaw_rate))
            plt.text(text_x, text_y_start - next(ge), r'longitude: ${:.2f}\degree$'.format(ego_longitude))
            plt.text(text_x, text_y_start - next(ge), r'latitude: ${:.2f}\degree$'.format(ego_latitude))
            plt.text(text_x, text_y_start - next(ge), r'long_acc: ${:.2f}m/s^2$'.format(ego_long_acc))
            plt.text(text_x, text_y_start - next(ge), r'lat_acc: ${:.2f}m/s^2$'.format(ego_lat_acc))

            plt.text(text_x, text_y_start - next(ge), 'path_x: {:.2f}m'.format(path_x))
            plt.text(text_x, text_y_start - next(ge), 'path_y: {:.2f}m'.format(path_y))
            plt.text(text_x, text_y_start - next(ge), 'delta_x: {:.2f}m'.format(delta_x))
            plt.text(text_x, text_y_start - next(ge), 'delta_y: {:.2f}m'.format(delta_y))
            plt.text(text_x, text_y_start - next(ge), r'ego_phi: ${:.2f}\degree$'.format(ego_phi))
            plt.text(text_x, text_y_start - next(ge), r'path_phi: ${:.2f}\degree$'.format(path_phi))
            plt.text(text_x, text_y_start - next(ge), r'delta_phi: ${:.2f}\degree$'.format(delta_phi))
            decision = self.Info_List[2].copy()
            decision_steer = decision['SteerAngleAim']
            decision_torque = decision['Torque']
            decision_brkacc = decision['Deceleration']
            decision_Dec_flag = decision['Dec_flag']
            decision_Tor_flag = decision['Tor_flag']
            decision_fwrad = decision['front_wheel_rad']
            decision_ax = decision['a_x']

            # decision: {'Deceleration': decel,  # [m/s^2]
            #            'Torque': torque,  # [N*m]
            #            'Dec_flag': dec_flag,
            #            'Tor_flag': tor_flag,
            #            'SteerAngleAim': steer_wheel_deg,  # [deg]
            #            'front_wheel_rad': front_wheel_rad,  # [rad]
            #            'a_x': a_x}

            text_x, text_y_start = 70, 60
            ge = iter(range(0, 1000, 4))

            # done info
            # plt.text(text_x, text_y_start - next(ge), 'done info: {}'.format(self.done_type))
            plt.text(text_x, text_y_start - next(ge), 'CAN')
            plt.text(text_x, text_y_start - next(ge), 'speed_act: {:.2f}m/s'.format(ego_v))
            plt.text(text_x, text_y_start - next(ge), r'steering_act: ${:.2f}\degree$'.format(ego_steer))
            plt.text(text_x, text_y_start - next(ge), 'throttle: {:.2f}'.format(ego_throttle))
            plt.text(text_x, text_y_start - next(ge), 'brake_on: {:.2f}'.format(ego_brk))
            plt.text(text_x, text_y_start - next(ge), 'gear: {:.2f}'.format(ego_gear))
            plt.text(text_x, text_y_start - next(ge), 'Decision')
            plt.text(text_x, text_y_start - next(ge), r'steer_aim_decision: ${:.2f}\degree$'.format(decision_steer))
            plt.text(text_x, text_y_start - next(ge), 'front_wheel_rad_decision: {:.2f}'.format(decision_fwrad))
            plt.text(text_x, text_y_start - next(ge), 'torque_decision: {:.2f}Nm'.format(decision_torque))
            plt.text(text_x, text_y_start - next(ge), 'torque_flag: {}'.format(decision_Tor_flag))
            plt.text(text_x, text_y_start - next(ge), r'brake_acc_decision: {:.2f}$m/s^2$'.format(decision_brkacc))
            plt.text(text_x, text_y_start - next(ge), 'deceleration_flag: {}'.format(decision_Dec_flag))
            plt.text(text_x, text_y_start - next(ge), r'acc_decision: {:.2f}$m/s^2$'.format(decision_ax))
            plt.text(text_x, text_y_start - next(ge), r'acc_actual: {:.2f}$m/s^2$'.format(self.acc_timer.mean))

            plt.pause(0.01)
            # print(time.time()-start_time)
            # ax.cla()

            # # reward info
            # if self.reward_info is not None:
            #     for key, val in self.reward_info.items():
            #         plt.text(text_x, text_y_start - next(ge), '{}: {:.4f}'.format(key, val))
        # plt.figure(0)
        # plt.ion()
        # lane_list, lane_center_list, road_angle = load_map()
        # plt.figsize = (20, 5)
        # plt.title("X-Y")
        # ax = plt.gca()
        # ax.set_aspect('equal')
        # ax.invert_xaxis()
        # ax.invert_yaxis()
        # for i in range(len(lane_list)):
        #     plt.plot(lane_list[i][:, 0], lane_list[i][:, 1], color='green', linewidth='2')
        # for i in range(len(lane_center_list)):
        #     plt.plot(lane_center_list[i][:, 0], lane_center_list[i][:, 1], color='red', linewidth='2')
        #
        # while True:
        #     Info_List = self.Info_List
        #     self.step = Info_List[0]
        #     if self.step > self.step_old:
        #         self.step_old= self.step
        #         with self.lock:
        #             Time  = Info_List[1]
        #             Decision = Info_List[2].copy()
        #             State_ego = Info_List[3].copy()
        #             State_Other = Info_List[4].copy()
        #
        #         # 轨迹显示
        #         plt.scatter(State_ego['GaussX'], State_ego['GaussY'], color='blue')
        #         ego_show=mpathes.Rectangle([State_ego['GaussX']-4.5/2, State_ego['GaussY']-1.855/2], 4.5, 1.855,270-State_ego['Heading'], fc='magenta', ec='magenta',alpha = 0.1)
        #         ax.add_patch(ego_show)
        #
        #         # plt.plot([State_ego['GaussX']-100,State_ego['GaussX']-100],[State_ego['GaussY']-5,State_ego['GaussY']+5])
        #         # plt.plot([State_ego['GaussX']+80,State_ego['GaussX']+80],[State_ego['GaussY']-5,State_ego['GaussY']+5])
        #         # plt.plot([State_ego['GaussX'] - 100, State_ego['GaussX'] + 80],
        #         #          [State_ego['GaussY'] +5, State_ego['GaussY'] + 5])
        #         # plt.plot([State_ego['GaussX'] -100, State_ego['GaussX'] + 80],
        #         #          [State_ego['GaussY'] - 5, State_ego['GaussY'] -5])
        #
        #         for i in range(len(State_Other['x_other'])):
        #             x =  State_Other['x_other'][i]
        #             y = State_Other['y_other'][i]
        #             heading_next=State_Other['heading_next'][i]
        #             other_show=mpathes.Rectangle([x-5.169/2, y-2.392/2], 5.169, 2.392, 270-heading_next, fc='cadetblue', ec='cadetblue', alpha=0.01)
        #             ax.add_patch(other_show)
        #             plt.scatter(x, y, color='blue')
        #
        #
        #         plt.pause(0.01)
        #         # plt.clf()





def static_plot():
    plot = Plot(None,None,'left')
    plot.run()
    plt.show()


if __name__ == '__main__':
    static_plot()


