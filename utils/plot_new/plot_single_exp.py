import matplotlib.pyplot as plt
from utils.plot_new.plot_utils.load_record import load_data
import os

def single_plot(data_all, keys=['Deceleration'], **kwargs):
    if 'fig_num' in kwargs.keys():
        plt.figure(kwargs['fig_num'])
    else:
        plt.figure()
    if kwargs['highlight'] == True:
        min_index = data_all['VehicleMode'].index(1.0)
        max_index = min_index + data_all['VehicleMode'].count(1.0)

    labels = []
    for key in keys:
        if isinstance(key, tuple):
            plt.plot(data_all[key[0]], data_all[key[1]])
            labels.append(key[1])
        else:
            plt.plot(data_all['Time'], data_all[key])

    if isinstance(keys[0], tuple):
        plt.xlabel(keys[0][0])
    else:
        plt.xlabel('time /s')
        labels = keys
        if kwargs['highlight'] == True:
            axes = plt.gca()
            ylim = axes.get_ylim()
            plt.plot([data_all['Time'][min_index], data_all['Time'][min_index]], ylim, c='red', linestyle='--')
            plt.plot([data_all['Time'][max_index], data_all['Time'][max_index]], ylim, c='red', linestyle='--')
    plt.legend(labels=labels, loc='best')
    plt.grid()

    if 'x_lim' in kwargs.keys():
        plt.xlim(kwargs['x_lim'])
    if 'y_lim' in kwargs.keys():
        plt.ylim(kwargs['y_lim'])
    if 'title' in kwargs.keys():
        plt.title(kwargs['title'])
    if 'path' in kwargs.keys():
        proj_root_dir = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
        fig_path = proj_root_dir + '/record/' + kwargs['path'] + '/figure/'
        if not os.path.exists(fig_path):
            os.mkdir(fig_path)
        name = fig_path + kwargs['title'] +'.jpg'
        plt.savefig(name)
    else:
        plt.show()

if __name__ == '__main__':
    exp_index = 'left_case0_20210102_164721'
    data_all, keys_for_data = load_data(exp_index)
    print(keys_for_data)
    single_plot(data_all, ['SteerAngleAct', 'SteerAngleAim'], title='Steering Act', path = exp_index,  highlight=True)
    single_plot(data_all, ['first_out'],  title='first out',  path = exp_index, highlight=True)
    single_plot(data_all, ['Dec_flag','BrkOn'], title='Deceleration flag', path=exp_index, highlight=True)
    single_plot(data_all, ['Deceleration','a_x','BrkOn'], title='Deceleration acc', path=exp_index, highlight=True)
    # single_plot(data_all, ['BrkOn'], title='', path=exp_index, highlight=True)

    # single_plot(data_all, ['v_light'], title='V light', path=exp_index, highlight=True)
    single_plot(data_all, [('GaussX','GaussY')], title='Trajectory', path=exp_index, x_lim=[-11,11], highlight=False)
    single_plot(data_all, ['GaussX', 'GaussY'], title='Position', path=exp_index,  highlight=True)
    single_plot(data_all, ['ego_vx', 'ego_vy'], title='Obs ego velocity', path=exp_index, highlight=True)
    # single_plot(data_all, ['ego_v_y'], title='Obs ego velocity', path=exp_index, highlight=True)

    single_plot(data_all, ['ego_x'], title='Obs ego position', path=exp_index, highlight=True)
    single_plot(data_all, ['ego_phi'], title='Obs ego phi', path=exp_index, highlight=True)
    single_plot(data_all, ['tracking_delta_y'], title='Obs ego delta y', path=exp_index, highlight=True)
    single_plot(data_all, ['tracking_delta_phi'], title='Obs ego delta phi', path=exp_index, highlight=True)
    single_plot(data_all, ['tracking_delta_v'], title='Obs ego delta v', path=exp_index, highlight=True)
    # single_plot(data_all, ['ego_r'], title='Obs ego r', path=exp_index, highlight=True)
    # single_plot(data_all, ['delta_y'], title='Obs ego delta y', path=exp_index, highlight=True)
    # single_plot(data_all, ['delta_phi_rad'], title='Obs ego x', path=exp_index, highlight=True)

    single_plot(data_all, ['Heading'], title='Heading', path=exp_index, highlight=True)
    single_plot(data_all, ['NorthVelocity','EastVelocity'], title='Velocity', path=exp_index, highlight=True)
    single_plot(data_all, ['VehicleMode'] , title='Mode', path=exp_index, highlight=True)
    single_plot(data_all, ['GpsSpeed', 'VehicleSPeedAct'], title='Speed', path=exp_index, highlight=True)
    single_plot(data_all, ['Throttle'], title='Throttle', path=exp_index, highlight=True)
    single_plot(data_all, ['YawRate'], title='Yaw rate', path=exp_index, highlight=True)

    single_plot(data_all, ['AutoGear'], title='AutoGear', path=exp_index, highlight=True)

    # # single_plot(data_all, ['YawRate'])
