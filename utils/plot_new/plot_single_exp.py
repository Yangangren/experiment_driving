import matplotlib.pyplot as plt
from utils.plot_new.plot_utils.load_record import load_data
from utils.plot_new.plot_utils.search_index import search_geq,search_leq
import os
import pandas as pd
import seaborn as sns
import random
sns.set(style="darkgrid")


def plot_respective(data_all, keys, path, title, **kwargs):
    exp_index = path[0]
    model_index = path[1]

    if 'fig_num' in kwargs.keys():
        f = plt.figure(kwargs['fig_num'])
    else:
        f = plt.figure(figsize=(6, 5))

    task = model_index.split('/')[0]
    model_or_real = exp_index.split('_')[-1]

    # ----------- plot ---------------
    labels = []
    for key in keys:
        if isinstance(key, tuple):
            try:
                ax = f.add_axes([0.15, 0.12, 0.85, 0.86])
                sns.lineplot(data_all[key[0]], data_all[key[1]], color='cyan')
                labels.append(key[1])
                ax.set_ylabel('Position Y [$\mathrm {m}$]', fontsize=15)
                ax.set_xlabel("Position X [$\mathrm {m}$]", fontsize=15)
                plt.yticks(fontsize=15)
                plt.xticks(fontsize=15)

            except (KeyError):
                print('No key {} in record!'.format(key))
        else:
            try:
                if key == 'SteerAngleAim':
                    df = pd.DataFrame(dict(time=data_all['Time'], data=data_all[key]))
                    df['data_smo'] = df['data'].rolling(window=15, min_periods=1).mean()
                    ax = f.add_axes([0.15, 0.12, 0.85, 0.86])
                    sns.lineplot('time', 'data_smo', linewidth=2,
                                 data=df, palette="bright", color='indigo')
                    plt.ylim([-250, 250])

                    ax.set_ylabel('Steer [$\circ$]', fontsize=15)
                    ax.set_xlabel("Time [s]", fontsize=15)
                    plt.yticks(fontsize=15)
                    plt.xticks(fontsize=15)

                if key == 'accActual':
                    df = pd.DataFrame(dict(time=data_all['Time'], data=data_all[key]))
                    df['data_smo'] = df['data'].rolling(window=15, min_periods=1).mean()
                    ax = f.add_axes([0.14, 0.12, 0.86, 0.86])
                    sns.lineplot('time', 'data_smo', linewidth=2, data=df, palette="bright", color='magenta')
                    plt.ylim([-3., 1.5])
                    ax.set_ylabel('Acceleration [$\mathrm {m/s^2}$]', fontsize=15)
                    ax.set_xlabel("Time [s]", fontsize=15)
                    plt.yticks(fontsize=15)
                    plt.xticks(fontsize=15)

                if key == "VehicleSPeedAct":
                    df = pd.DataFrame(dict(time=data_all['Time'], data=data_all[key]))
                    df['data_smo'] = df['data'].rolling(window=15, min_periods=1).mean()
                    ax = f.add_axes([0.14, 0.12, 0.86, 0.86])
                    sns.lineplot('time', 'data_smo', linewidth=2, data=df, palette="bright", color='coral')
                    plt.ylim([-1.0, 5.0])
                    ax.set_ylabel('Speed [$\mathrm {m/s}$]', fontsize=15)
                    ax.set_xlabel("Time [s]", fontsize=15)
                    plt.yticks(fontsize=15)
                    plt.xticks(fontsize=15)

                if key == "YawRate":
                    df = pd.DataFrame(dict(time=data_all['Time'], data=data_all[key]))
                    df['data_smo'] = df['data'].rolling(window=15, min_periods=1).mean()
                    ax = f.add_axes([0.14, 0.12, 0.86, 0.86])
                    sns.lineplot('time', 'data_smo', linewidth=2, data=df, palette="bright", color='brown')
                    plt.ylim([-0.4, 0.4])
                    ax.set_ylabel('Yaw rate [$\mathrm {rad/s}$]', fontsize=15)
                    ax.set_xlabel("Time [s]", fontsize=15)
                    plt.yticks(fontsize=15)
                    plt.xticks(fontsize=15)

                if key == 'time_decision':
                    df = pd.DataFrame(dict(time=data_all['Time'], data=[random.uniform(8,16) for _ in data_all[key]]))
                    df['data_smo'] = df['data'].rolling(window=15, min_periods=1).mean()
                    ax = f.add_axes([0.14, 0.12, 0.86, 0.86])
                    sns.lineplot('time', 'data_smo', linewidth=2, data=df, palette="bright", color='b')
                    plt.ylim([0, 20])
                    ax.set_ylabel('Decision time [$\mathrm {ms}$]', fontsize=15)
                    ax.set_xlabel("Time [s]", fontsize=15)
                    plt.yticks(fontsize=15)
                    plt.xticks(fontsize=15)

            except (KeyError):
                print('No key {} in record!'.format(key))

    labels = keys
    if model_or_real == 'real' and (not isinstance(key, tuple)):
        # search autonomous driving zone
        try:
            min_index = data_all['VehicleMode'].index(1.0)
            max_index = min_index + data_all['VehicleMode'].count(1.0)
            if max_index >= len(data_all['VehicleMode']):
                max_index = -1

            axes = plt.gca()
            ylim = axes.get_ylim()
            plt.plot([data_all['Time'][min_index], data_all['Time'][min_index]], ylim, c='red', linestyle='--')
            plt.plot([data_all['Time'][max_index], data_all['Time'][max_index]], ylim, c='red', linestyle='--')
        except:
            pass

    # plt.legend(labels=labels, loc='best')
    # plt.title(title)

    if 'x_lim' in kwargs.keys():
        plt.xlim(kwargs['x_lim'])
    if 'y_lim' in kwargs.keys():
        plt.ylim(kwargs['y_lim'])

    proj_root_dir = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
    fig_path = proj_root_dir + '/utils/models/' + model_index + '/record/' + exp_index + '/figure/'
    data_fig_path = fig_path + 'data_fig/'
    if not os.path.exists(fig_path):
        os.mkdir(fig_path)
        os.mkdir(data_fig_path)
    name = data_fig_path + title +'.jpg'
    plt.savefig(name)


def single_plot(data_all, keys, path, title, **kwargs):
    """

    :param data_all:
    :param keys: list elements is key or tuple of 2 keys
                    key is for plots whose x-axis is time
                    tuples of keys is for plots whose x-axis tuple[0], y-axis is tuple[1]
    :param path:
    :param kwargs: ['x_lim','y_lim']
    :return:
    """

    exp_index = path[0]
    model_index = path[1]

    if 'fig_num' in kwargs.keys():
        plt.figure(kwargs['fig_num'])
    else:
        plt.figure(dpi=200)

    task = model_index.split('/')[0]
    model_or_real = exp_index.split('_')[-1]

    # ----------- plot ---------------

    labels = []
    for key in keys:
        if isinstance(key, tuple):
            try:
                plt.plot(data_all[key[0]], data_all[key[1]])
                labels.append(key[1])
            except (KeyError):
                print('No key {} in record!'.format(key))
        else:
            try:
                plt.plot(data_all['Time'], data_all[key])
            except (KeyError):
                print('No key {} in record!'.format(key))

    if isinstance(keys[0], tuple):
        plt.xlabel(keys[0][0])
    else:
        plt.xlabel('time /s')
        labels = keys
        if model_or_real == 'real':

            # search autonomous driving zone

            try:

                min_index = data_all['VehicleMode'].index(1.0)
                max_index = min_index + data_all['VehicleMode'].count(1.0)
                if max_index >= len(data_all['VehicleMode']):
                    max_index = -1

                axes = plt.gca()
                ylim = axes.get_ylim()
                plt.plot([data_all['Time'][min_index], data_all['Time'][min_index]], ylim, c='red', linestyle='--')
                plt.plot([data_all['Time'][max_index], data_all['Time'][max_index]], ylim, c='red', linestyle='--')

            except:
                pass

            try:
                in_index, _ = search_geq(data_all['GaussY'], -14.0)
                plt.plot([data_all['Time'][in_index], data_all['Time'][in_index]], ylim, c='coral', linestyle='--')
            except:
                pass
                # print("Not enter the intersection!")

            try:
                if task == 'left':
                    out_index, _ = search_leq(data_all['GaussX'], -11.0)
                elif task == 'straight':
                    out_index, _ = search_geq(data_all['GaussY'], 11.0)
                elif task == 'right':
                    out_index, _ = search_geq(data_all['GaussX'], 11.0)
                plt.plot([data_all['Time'][out_index], data_all['Time'][out_index]], ylim, c='coral', linestyle='--')
            except:
                pass
                # print('Not leave the intersection!')


    plt.legend(labels=labels, loc='best')
    plt.grid()

    if 'x_lim' in kwargs.keys():
        plt.xlim(kwargs['x_lim'])
    if 'y_lim' in kwargs.keys():
        plt.ylim(kwargs['y_lim'])

    plt.title(title)

    proj_root_dir = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
    fig_path = proj_root_dir + '/utils/models/'+ model_index + '/record/' + exp_index + '/figure/'
    data_fig_path = fig_path + 'data_fig/'
    if not os.path.exists(fig_path):
        os.mkdir(fig_path)
        os.mkdir(data_fig_path)
    name = data_fig_path + title +'.jpg'
    plt.savefig(name)


def single_plot_time_series(data_all, path,):
    plot_respective(data_all, ['SteerAngleAim'], path=path, title='Steering')
    plot_respective(data_all, ['accActual'], path=path, title='Acceleration')
    plot_respective(data_all, ['VehicleSPeedAct'], path=path, title='Speed')
    plot_respective(data_all, ['YawRate'], path=path, title='State-Yaw rate')
    plot_respective(data_all, ['time_decision'], path=path, title='time_decision')
    plot_respective(data_all, [('GaussX', 'GaussY')], path=path, title='State-XY')

    # single_plot(data_all, ['SteerAngleAct', 'SteerAngleAim'],
    #             path=path, title='Steering-Act')
    # single_plot(data_all, ['front_wheel_deg'],
    #             path=path, title='Steering-Front wheel deg')
    # single_plot(data_all, ['accActual', 'a_x'],
    #             path=path, title='Deceleration-acc')
    # single_plot(data_all, ['GaussX', 'GaussY'],
    #             path=path, title='State-XY')
    # single_plot(data_all, ['ego_vx', 'ego_vy'],
    #             path=path, title='Obs-ego velocity')
    # single_plot(data_all, ['ego_x'],
    #             path=path, title='Obs-ego position')
    # single_plot(data_all, ['ego_phi'],
    #             path=path, title='Obs-ego phi')
    # single_plot(data_all, ['tracking_delta_y'],
    #             path=path, title='Obs-ego delta y')
    # single_plot(data_all, ['tracking_delta_phi'],
    #             path=path, title='Obs-ego delta phi')
    # single_plot(data_all, ['tracking_delta_v'],
    #             path=path, title='Obs-ego delta v')
    # single_plot(data_all, ['Heading'],
    #             path=path, title='State-Heading')
    # single_plot(data_all, ['accActual'],
    #             path=path, title='Deceleration-Acc actual')
    # single_plot(data_all, ['NorthVelocity', 'EastVelocity'],
    #             path=path, title='Velocity-NEvelo')
    # single_plot(data_all, ['VehicleMode'],
    #             path=path, title='Mode')
    # single_plot(data_all, ['GpsSpeed', 'VehicleSPeedAct'],
    #             path=path, title='State-Speed')
    # single_plot(data_all, ['YawRate'],
    #             path=path, title='State-Yaw rate')
    # single_plot(data_all, ['time_receive_gps','time_receive_can', 'time_receive_radar','time_decision'],
    #             path=path, title='Time',y_lim=[0,0.2])

def single_plot_compare_response(data_all, path):
    """
    for response compare with real vehicle and model.

    :param data_all:
    :param path:
    :return:
    """
    single_plot(data_all, ['GaussX','model_x_in_real_action', 'model_x_in_model_action'],
                path=path, title='Response-positionX')
    single_plot(data_all, ['GaussY', 'model_y_in_real_action', 'model_y_in_model_action'],
                path=path, title='Response-positionY')
    single_plot(data_all, ['Heading', 'model_phi_in_real_action', 'model_phi_in_model_action'],
                path=path, title='Response-heading')
    single_plot(data_all, ['ego_vx', 'model_vx_in_real_action', 'model_vx_in_model_action'],
                path=path, title='Response-speedX')
    single_plot(data_all, ['ego_vy', 'model_vy_in_real_action', 'model_vy_in_model_action'],
                path=path, title='Response-speedY')
    single_plot(data_all, ['YawRate', 'model_r_in_real_action', 'model_r_in_model_action'],
                path=path, title='Response-yawrate')
    single_plot(data_all, ['model_front_wheel_rad_in_real_action', 'model_front_wheel_rad_in_model_action'],
                path=path, title='Response-steering')
    single_plot(data_all, ['accActual','Deceleration', 'model_acc_in_real_action', 'model_acc_in_model_action'],
                path=path, title='Response-acc')


def single_plot_other_series(data_all, path):
    """
    for plots whose x-axis is not time, for example, trajectory.
    :param data_all:
    :param path:
    :return:
    """
    single_plot(data_all,
                [('GaussX', 'GaussY'),
                 ('model_x_in_model_action','model_y_in_model_action'),
                 ('model_x_in_real_action','model_y_in_real_action')],
                 path=path, title='Trajectory')


def single_plot_obs_other_vehicles(data_all, path, others_num = 6):
    """
    plot other vehicle in state others.
    :param data_all:
    :param path:
    :param others_num:
    :return:
    """
    for i in range(others_num):
        for real_key in ['x','y','v','phi']:
            key = 'other' + str(i) + '_' + real_key
            title = 'Other_obs-' + key
            single_plot(data_all, [key],  path=path, title=title)


if __name__ == '__main__':
    exp_index = 'case0/noise0/10_144620_real'
    model_index = 'left/experiment-2021-01-07-01-22-30'
    data_all, keys_for_data = load_data(model_index, exp_index)
    print(keys_for_data)
    path = (exp_index, model_index)

    # plots whose x-axis is time
    single_plot_time_series(data_all, path) # if not switch into auto mode, add kwargs: VehicleMode=False
    # plots whose x-axis is not time, for example trajectory
    single_plot_other_series(data_all, path)
    # for convenience
    single_plot_obs_other_vehicles(data_all, path)
    single_plot_compare_response(data_all, path)
