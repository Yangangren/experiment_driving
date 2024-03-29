import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from utils.plot_new.plot_utils.load_record import load_data
from utils.plot_new.plot_utils.search_index import search_geq, search_automode_time, search_leq
import math
import os
import pandas as pd
import matplotlib.pyplot as ticker
import seaborn as sns

CROSSROAD_SIZE = 22
EXTENSION = 25
LANE_WIDTH = 3.5
START_OFFSET = 3
LANE_NUMBER = 1
EGO_LENGTH = 4.8
EGO_WIDTH = 2.0
STATE_OTHER_LENGTH = EGO_LENGTH
STATE_OTHER_WIDTH = EGO_WIDTH


class Single_bird_view_plot(object):
    def __init__(self, data_all, draw_other_veh='scatter', **kwargs):
        proj_root_dir = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
        self.data_all = data_all
        self.ego_x = data_all['GaussX']
        self.ego_y = data_all['GaussY']
        self.ego_v = data_all['GpsSpeed']
        self.time = data_all['Time']
        self.x_others = data_all['x_other']
        self.y_others = data_all['y_other']
        self.phi_others = data_all['phi_other']
        self.ego_steer = (np.array(data_all['front_wheel_deg']) * 15).tolist()
        def join_path(name):
            return proj_root_dir + name
        left_construct_traj = np.load(join_path('/map/left_construct.npy'))
        straight_construct_traj = np.load(join_path('/map/straight_construct.npy'))
        right_construct_traj = np.load(join_path('/map/right_construct.npy'))
        self.ref_path_all = {'left': left_construct_traj, 'straight': straight_construct_traj,
                             'right': right_construct_traj}
        # self.ref_path = self.ref_path_all[task]
        # self.fig = plt.figure(dpi=200)
        self._preprocess_data()
        self.start_index = 0
        self.stop_index = -1
        self.draw_other_veh = draw_other_veh
        self.root_dir = proj_root_dir
        self.path = None
        if 'path' in kwargs.keys():
            self.path = kwargs['path']

    def _preprocess_data(self, sparse_ratio = 5):
        ego_x = self.ego_x
        data_len = len(ego_x)
        sparse_index = np.arange(0, data_len, sparse_ratio)
        def preprocess_data(data):
            data = np.array(data)
            return data[sparse_index]
        self.sparse_ego_x = preprocess_data(self.ego_x)
        self.sparse_ego_y = preprocess_data(self.ego_y)
        self.sparse_time = preprocess_data(self.time)
        self.sparse_ego_v = preprocess_data(self.ego_v)
        self.sparse_x_others = preprocess_data(self.x_others)
        self.sparse_y_others = preprocess_data(self.y_others)
        self.sparse_phi_others = preprocess_data(self.phi_others)
        self.sparse_ego_steer = preprocess_data(self.ego_steer)


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

    def set_time(self, init_time, stop_time):
        self.start_index, _ = search_geq(self.sparse_time, init_time)
        self.stop_index, _ = search_geq(self.sparse_time, stop_time)
        self.sparse_time -= self.sparse_time[self.start_index]

    def draw_rotate_rec(self, x, y, a, l, w, color, linestyle='-'):
        RU_x, RU_y, _ = self._rotate_coordination(l / 2, w / 2, 0, -a)
        RD_x, RD_y, _ = self._rotate_coordination(l / 2, -w / 2, 0, -a)
        LU_x, LU_y, _ = self._rotate_coordination(-l / 2, w / 2, 0, -a)
        LD_x, LD_y, _ = self._rotate_coordination(-l / 2, -w / 2, 0, -a)
        self.ax.plot([RU_x + x, RD_x + x], [RU_y + y, RD_y + y], color=color, linestyle=linestyle)
        self.ax.plot([RU_x + x, LU_x + x], [RU_y + y, LU_y + y], color=color, linestyle=linestyle)
        self.ax.plot([LD_x + x, RD_x + x], [LD_y + y, RD_y + y], color=color, linestyle=linestyle)
        self.ax.plot([LD_x + x, LU_x + x], [LD_y + y, LU_y + y], color=color, linestyle=linestyle)

    def single_exp_bird_view(self):
        square_length = CROSSROAD_SIZE
        start_offset = START_OFFSET
        extension = EXTENSION
        lane_width = LANE_WIDTH
        light_line_width = 1
        dotted_line_style = '--'
        solid_line_style = '-'
        self.fig = plt.figure(figsize=(6.5, 5), dpi=200)

        self.ax = plt.axes(xlim=(-square_length / 2 - extension, square_length / 2 + extension),
                           ylim=(-square_length / 2 - extension, square_length / 2 + extension))
        plt.axis("equal")
        plt.axis('off')

        # self.ax.add_patch(plt.Rectangle((-square_length / 2 - extension, -square_length / 2 - extension - start_offset),
        #                            square_length + 2 * extension, square_length + 2 * extension + start_offset,
        #                            edgecolor='black',
        #                            facecolor='none'))

        self.ax.add_patch(plt.Rectangle((-square_length / 2 - extension - 10, -square_length / 2 - extension - start_offset),
                                         9.8, square_length + 2 * extension + start_offset,
                                   edgecolor='white',
                                   facecolor='white'))

        self.ax.add_patch(plt.Rectangle((-square_length / 2 - extension, -square_length / 2 - extension - start_offset - 10),
                                         square_length + 2 * extension, 9.8,
                                   edgecolor='white',
                                   facecolor='white'))

        self.ax.add_patch(plt.Rectangle((-square_length / 2 - extension, square_length / 2 + extension),
                                         square_length + 2 * extension, 9.8,
                                   edgecolor='white',
                                   facecolor='white'))
        self.ax.add_patch(plt.Rectangle((square_length / 2 + extension, -square_length / 2 - extension - start_offset),
                                         5., square_length + 2 * extension + start_offset,
                                   edgecolor='white',
                                   facecolor='white'))


        # self.ax.set_title('Bird View')

        # ----------arrow--------------
        # plt.arrow(lane_width / 2, -square_length / 2 - 10, 0, 5, color='b')
        # plt.arrow(lane_width / 2, -square_length / 2 - 10 + 5, -0.5, 0, color='b', head_width=1)
        # plt.arrow(lane_width / 2, -square_length / 2 - 10, 0, 5, color='b', head_width=1)
        # plt.arrow(lane_width / 2, -square_length / 2 - 10, 0, 5, color='b')
        # plt.arrow(lane_width / 2, -square_length / 2 - 10 + 5, 0.5, 0, color='b', head_width=1)

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
        if self.draw_other_veh == 'scatter':
            self.draw_other_vehicles_in_points()
        elif self.draw_other_veh == 'rectangular':
            self.draw_other_vehicles_in_rec()

        ax1 = self.fig.add_axes([0.75, 0.23, 0.04, 0.5])
        cmap = mpl.cm.hsv
        norm = mpl.colors.Normalize(vmin=float(self.sparse_time[self.start_index]), vmax=float(self.sparse_time[self.stop_index]))
        bar = mpl.colorbar.ColorbarBase(ax=ax1, cmap=cmap, norm=norm, orientation='vertical', alpha=0.8)
        bar.set_label('Time(s)', fontsize=10)

        ax2 = self.fig.add_axes([0.3, 0.625, 0.18, 0.18])
        x = self.sparse_time[self.start_index: self.stop_index]
        y = self.sparse_ego_v[self.start_index: self.stop_index]
        df = pd.DataFrame({'x': x, 'y': y})
        df['smo'] = df['y'].rolling(window=8, min_periods=1).mean()
        speed = df.plot(x='x', y='smo', ax=ax2, c='saddlebrown', linewidth=2)
        speed.legend_.remove()
        speed.spines['top'].set_visible(False)
        speed.spines['right'].set_visible(False)
        plt.xlabel('Time(s)', fontsize=10)
        plt.ylabel('Speed(m/s)', fontsize=10)
        plt.yticks([0, 2, 4], ['   0','   2','   4'])
        plt.ylim([0, 4.5])

        # add steering wheel
        ax3 = self.fig.add_axes([0.3, 0.28, 0.18, 0.18])
        ax3.set_facecolor('none')
        x = self.sparse_time[self.start_index: self.stop_index]
        y = self.sparse_ego_steer[self.start_index: self.stop_index]
        df = pd.DataFrame({'x': x, 'y': y})
        df['smo'] = df['y'].rolling(window=8, min_periods=1).mean()
        df['smo'] = df['smo'] / 100
        speed = df.plot(x='x', y='smo', ax=ax3, c='blue', linewidth=2)
        speed.legend_.remove()
        speed.spines['top'].set_visible(False)
        speed.spines['right'].set_visible(False)
        plt.xlabel('Time(s)', fontsize=10)
        plt.ylabel(r'Steer($\times 100^{\circ}$)', fontsize=10)
        ax3.yaxis.set_major_locator(ticker.MaxNLocator(nbins=3, integer=True))
        plt.ylim([-3.6, 3.6])

        if self.path is not None:
            self.save_plot()
            plt.show()
        else:
            plt.show()


    def draw_other_vehicles_in_rec(self, color='black'):
        x_others = self.sparse_x_others[self.stop_index]
        y_others = self.sparse_y_others[self.stop_index]
        phi_others = self.sparse_phi_others[self.stop_index]
        for i in range(len(x_others)):
            if -CROSSROAD_SIZE/2-EXTENSION < x_others[i] < CROSSROAD_SIZE/2 + EXTENSION\
                    and -CROSSROAD_SIZE - EXTENSION < y_others[i] < CROSSROAD_SIZE - EXTENSION:
                self.draw_rotate_rec(x_others[i], y_others[i], phi_others[i], 4.8, 2.0, color)

    def draw_other_vehicles_in_points(self):
        square_length = CROSSROAD_SIZE
        extension = EXTENSION
        def is_in_plot_area(x, y, tolerance=1):
            if -square_length / 2 - extension + tolerance < x < square_length / 2 + extension - tolerance and \
                    -square_length / 2 - extension + tolerance < y < square_length / 2 + extension - tolerance:
                return True
            else:
                return False
        if self.stop_index == -1:
            self.stop_index = len(self.sparse_time) - 1
        for i in range(self.sparse_x_others.shape[1]):
            # judge real stop_index
            for j in range(self.stop_index - self.start_index):
                if is_in_plot_area(self.sparse_x_others[self.start_index + j, i], self.sparse_y_others[self.start_index + j, i]) == False:
                    stop_index = self.start_index + j
                    break
                else:
                    stop_index = self.stop_index
            self.p2 = plt.scatter(self.sparse_x_others[self.start_index: stop_index, i], self.sparse_y_others[self.start_index: stop_index, i],
                        marker='D',
                        alpha=0.5,
                        s=10,
                        c=self.sparse_time[self.start_index: stop_index], cmap='hsv')
        if (self.path[0].split('/')[0] == 'case0') and (self.path[1].split('/')[0] == 'left'):
            plt.legend([self.p1, self.p2], ['ego vehicle','surrounding vehicle'], fontsize=8,
                       loc='upper right', bbox_to_anchor=(0.47, 0.42, 0.34, 0.5))


    def draw_ego_points(self):
        x = self.sparse_ego_x[self.start_index: self.stop_index]
        y = self.sparse_ego_y[self.start_index: self.stop_index]
        c = self.sparse_time[self.start_index: self.stop_index]

        reduce_x = [x[i] for i in range(0, len(x), 1)]
        reduce_y = [y[i] for i in range(0, len(y), 1)]
        reduce_c = [c[i] for i in range(0, len(c), 1)]

        self.p1 = plt.scatter(reduce_x, reduce_y, marker='2', s=20, c=reduce_c, alpha=0.5, cmap='hsv')

    def draw_others_points(self):
        return None

    def save_plot(self):
        # fig_path = self.root_dir + '/record/' + self.path + '/figure/bird_view_fig/'
        # if not os.path.exists(fig_path):
        #     os.mkdir(fig_path)
        fig_path = self.root_dir + '/utils/models/' + model_index + '/record/' + exp_index + '/figure/'
        class_fig_path = fig_path + 'bird_view_fig/'
        if not os.path.exists(fig_path):
            os.mkdir(fig_path)
        if not os.path.exists(class_fig_path):
            os.mkdir(class_fig_path)
        fig_name = fig_path.split('/')[4] + "_" + fig_path.split('/')[7] + '.jpg'
        name = class_fig_path + fig_name
        plt.savefig(name)


if __name__ == '__main__':
    exp_index = 'case2/noise0/10_161557_real'
    model_index = 'straight/experiment-2021-01-07-01-22-31'
    path = (exp_index, model_index)
    data_all, keys_for_data = load_data(model_index, exp_index)
    bird_view_plot = Single_bird_view_plot(data_all, draw_other_veh='scatter', # rectangular, scatter
                                           path=path)
    try:
        automode_start_time, automode_stop_time = search_automode_time(data_all)
        print('Auto mode start: {:.2f}s, stop: {:.2f}s'.format(automode_start_time, automode_stop_time))
        bird_view_plot.set_time(automode_start_time, automode_stop_time)
    except (ValueError):
        print('Not switch into autonomous mode!')

    bird_view_plot.single_exp_bird_view()
