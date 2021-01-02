import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from utils.plot_new.plot_utils.load_record import load_data
from plot_online import Plot
# from math import sin, cos, pi
import math
import os

CROSSROAD_SIZE = 22
LANE_WIDTH = 3.5
START_OFFSET = 3
LANE_NUMBER = 1
EGO_LENGTH = 4.8
EGO_WIDTH = 2.0
STATE_OTHER_LENGTH = EGO_LENGTH
STATE_OTHER_WIDTH = EGO_WIDTH


class Single_bird_view_plot(Plot):
    def __init__(self, data_all, task):
        proj_root_dir = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
        self.data_all = data_all
        self.ego_x = data_all['GaussX']
        self.ego_y = data_all['GaussY']
        self.ego_v = data_all['GpsSpeed']
        left_construct_traj = np.load('./map/left_construct.npy')
        straight_construct_traj = np.load('./map/straight_construct.npy')
        right_construct_traj = np.load('./map/right_construct.npy')
        left_gps = np.load('./map/left_ref_cut.npy')
        straight_gps = np.load('./map/straight_ref_cut.npy')
        right_gps = np.load('./map/right_ref_cut.npy')
        self.ref_path_all = {'left': left_construct_traj, 'straight': straight_construct_traj,
                             'right': right_construct_traj,
                             'left_ref': left_gps, 'straight_ref': straight_gps, 'right_ref': right_gps}
        self.ref_path = self.ref_path_all[task]

    def _rotate_coordination(self, orig_x, orig_y, orig_d, coordi_rotate_d):
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



    def draw_rotate_rec(self, x, y, a, l, w, color, linestyle='-'):
        RU_x, RU_y, _ = self._rotate_coordination(l / 2, w / 2, 0, -a)
        RD_x, RD_y, _ = self._rotate_coordination(l / 2, -w / 2, 0, -a)
        LU_x, LU_y, _ = self._rotate_coordination(-l / 2, w / 2, 0, -a)
        LD_x, LD_y, _ = self._rotate_coordination(-l / 2, -w / 2, 0, -a)
        self.ax.plot([RU_x + x, RD_x + x], [RU_y + y, RD_y + y], color=color, linestyle=linestyle)
        self.ax.plot([RU_x + x, LU_x + x], [RU_y + y, LU_y + y], color=color, linestyle=linestyle)
        self.ax.plot([LD_x + x, RD_x + x], [LD_y + y, RD_y + y], color=color, linestyle=linestyle)
        self.ax.plot([LD_x + x, LU_x + x], [LD_y + y, LU_y + y], color=color, linestyle=linestyle)

    def plot_phi_line(self, x, y, phi, color):
        line_length = 5
        x_forw, y_forw = x + line_length * math.cos(phi * math.pi / 180.), \
                         y + line_length * math.sin(phi * math.pi / 180.)
        plt.plot([x, x_forw], [y, y_forw], color=color, linewidth=0.5)

    def single_exp_bird_view(self):
        square_length = CROSSROAD_SIZE
        start_offset = START_OFFSET
        extension = 40
        lane_width = LANE_WIDTH
        light_line_width = 3
        dotted_line_style = '--'
        solid_line_style = '-'
        start_time = 0
        v_old = 0.
        plt.title("Crossroad")
        self.ax = plt.axes(xlim=(-square_length / 2 - extension, square_length / 2 + extension),
                      ylim=(-square_length / 2 - extension, square_length / 2 + extension))
        plt.axis("equal")
        plt.axis('off')

        def is_in_plot_area(x, y, tolerance=5):
            if -square_length / 2 - extension + tolerance < x < square_length / 2 + extension - tolerance and \
                    -square_length / 2 - extension + tolerance < y < square_length / 2 + extension - tolerance:
                return True
            else:
                return False

        self.ax.add_patch(plt.Rectangle((-square_length / 2 - extension, -square_length / 2 - extension - start_offset),
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

        self.ax.plot(self.ref_path[0], self.ref_path[1], color='g', linestyle='--')

        v_color, h_color = 'black', 'black'

        plt.plot([0, lane_width], [-square_length / 2 - start_offset, -square_length / 2 - start_offset],
                 color=v_color, linewidth=light_line_width)

        plt.plot([-lane_width, 0], [square_length / 2, square_length / 2],
                 color=v_color, linewidth=light_line_width)

        plt.plot([-square_length / 2, -square_length / 2], [0, -lane_width],
                 color=h_color, linewidth=light_line_width)

        plt.plot([square_length / 2, square_length / 2], [lane_width, 0],
                 color=h_color, linewidth=light_line_width)

        self.draw_ego_points()


    def draw_veh_points(self, start, stop):
        return None

    def draw_ego_points(self, start=0, stop=-1):
        plt.scatter(self.ego_x[start: stop], self.ego_x[start: stop],
                    marker='o',
                    c=self.ego_v[start: stop], cmap='plasma_r')

    def draw_others_points(self):
        return None





        # plt.plot(21277100 - np.array(X[auto_s:auto_e]), 3447706.99961897 - np.array(Y[auto_s:auto_e]), color='black',
        #          linewidth=2)
        # plt.title("X-Y")
        # ax.set_aspect('equal')
        # # ax.invert_xaxis()
        # # ax.invert_yaxis()
        #
        # # 轨迹显示
        # fig = plt.figure('Trajectory-Color')
        # plt.subplot(2, 1, 1)
        # lane_list, lane_center_list, road_angle = load_map()
        # ax = plt.gca()
        # for i in range(len(lane_list)):
        #     plt.plot(21277100 - lane_list[i][:, 0], 3447706.99961897 - lane_list[i][:, 1], color='green', linewidth='2')
        # for i in range(len(lane_center_list)):
        #     plt.plot(21277100 - lane_center_list[i][:, 0], 3447706.99961897 - lane_center_list[i][:, 1], color='red',
        #              linewidth='2')
        #
        # # car_other
        # other_X_list = []
        # other_Y_list = []
        # for num in range(len(Other_X[0])):
        #     other_X_list.append([])
        #     other_Y_list.append([])
        #     for i in range(auto_s, auto_e, 1):
        #         x_other = Other_X[i][num]
        #         y_other = Other_Y[i][num]
        #         other_X_list[num].append(float(x_other))
        #         other_Y_list[num].append(float(y_other))
        #         # other_X_list[num].append(21277100-float(x_other))
        #         # other_Y_list[num].append(3447700-float(y_other))
        #     plt.scatter(21277100 - np.array(other_X_list[num]), 3447706.99961897 - np.array(other_Y_list[num]), marker='3',
        #                 s=10, c=run_time[auto_s:auto_e], cmap='plasma_r', alpha=0.5)
        #
        # plt.scatter(21277100 - np.array(X[auto_s:auto_e]), 3447706.99961897 - np.array(Y[auto_s:auto_e]), marker='o',
        #             c=run_time[auto_s:auto_e], cmap='plasma_r')
        # plt.figsize = (20, 8)
        # plt.title("X-Y", fontsize=20)
        # ax.set_aspect('equal')
        # # ax.invert_xaxis()
        # # ax.invert_yaxis()
        # ax1 = fig.add_axes([0.15, 0.5, 0.7, 0.02])
        # cmap = mpl.cm.plasma_r
        # norm = mpl.colors.Normalize(vmin=float(run_time[0]), vmax=float(run_time[-1]))
        # bar = mpl.colorbar.ColorbarBase(ax=ax1, cmap=cmap, norm=norm, orientation='horizontal')
        # bar.set_label('Time(s)', fontsize=10)

if __name__ == '__main__':
    data_all, keys_for_data = load_data('left_case0_20210102_165947')
    bird_view_plot = Single_bird_view_plot(data_all,'left')
    bird_view_plot.single_exp_bird_view()
    plt.show()