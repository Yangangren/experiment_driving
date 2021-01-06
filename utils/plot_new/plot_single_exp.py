import matplotlib.pyplot as plt
from utils.plot_new.plot_utils.load_record import load_data
from utils.plot_new.plot_utils.search_index import search_geq,search_leq
import os


def single_plot(data_all, keys, **kwargs):
    if 'fig_num' in kwargs.keys():
        plt.figure(kwargs['fig_num'])
    else:
        plt.figure(dpi=200)

    if kwargs['highlight'] == True:
        min_index = data_all['VehicleMode'].index(1.0)
        max_index = min_index + data_all['VehicleMode'].count(1.0)
        if max_index >= len(data_all['VehicleMode']):
            max_index = -1


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
            out_index, _ = search_leq(data_all['GaussX'], -11.0)
            plt.plot([data_all['Time'][out_index], data_all['Time'][out_index]], ylim, c='coral', linestyle='--')
            in_index, _ = search_geq(data_all['GaussY'], -14.0)
            plt.plot([data_all['Time'][in_index], data_all['Time'][in_index]], ylim, c='coral', linestyle='--')

    plt.legend(labels=labels, loc='best')
    plt.grid()

    if 'x_lim' in kwargs.keys():
        plt.xlim(kwargs['x_lim'])
    if 'y_lim' in kwargs.keys():
        plt.ylim(kwargs['y_lim'])
    if 'title' in kwargs.keys():
        plt.title(kwargs['title'])
    if 'path' in kwargs.keys():
        model_dir = kwargs['path'][1]
        proj_root_dir = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
        fig_path = proj_root_dir + '/utils/models/'+ model_dir + '/record/' + kwargs['path'][0] + '/figure/'
        data_fig_path = proj_root_dir + '/utils/models/'+ model_dir + '/record/' + kwargs['path'][0] + '/figure/data_fig/'
        if not os.path.exists(fig_path):
            os.mkdir(fig_path)
            os.mkdir(data_fig_path)
        name = data_fig_path + kwargs['title'] +'.jpg'
        plt.savefig(name)
    else:
        plt.show()

def single_plot_time_series(data_all, path, AutoMode = True):
    highlight = AutoMode
    single_plot(data_all, ['SteerAngleAct', 'SteerAngleAim'],
                title='Steering-Act', path=path, highlight=highlight)
    single_plot(data_all, ['front_wheel_deg'],
                title='Steering-Front wheel deg', path=path, highlight=highlight)
    single_plot(data_all, ['accActual', 'a_x'],
                title='Deceleration-acc', path=path, highlight=highlight)
    single_plot(data_all, ['GaussX', 'GaussY'],
                title='State-XY', path=path, highlight=highlight)
    single_plot(data_all, ['ego_vx', 'ego_vy'],
                title='Obs-ego velocity', path=path, highlight=highlight)
    single_plot(data_all, ['ego_x'],
                title='Obs-ego position', path=path, highlight=highlight)
    single_plot(data_all, ['ego_phi'],
                title='Obs-ego phi', path=path, highlight=highlight)
    single_plot(data_all, ['tracking_delta_y'],
                title='Obs-ego delta y', path=path, highlight=highlight)
    single_plot(data_all, ['tracking_delta_phi'],
                title='Obs-ego delta phi', path=path, highlight=highlight)
    single_plot(data_all, ['tracking_delta_v'],
                title='Obs-ego delta v', path=path, highlight=highlight)
    single_plot(data_all, ['Heading'],
                title='State-Heading', path=path, highlight=highlight)
    single_plot(data_all, ['accActual'],
                title='Deceleration-Acc actual', path=path, highlight=highlight)
    single_plot(data_all, ['NorthVelocity', 'EastVelocity'],
                title='Velocity-NEvelo', path=path, highlight=highlight)
    single_plot(data_all, ['VehicleMode'],
                title='Mode', path=path, highlight=highlight)
    single_plot(data_all, ['GpsSpeed', 'VehicleSPeedAct'],
                title='State-Speed', path=path, highlight=highlight)
    single_plot(data_all, ['YawRate'],
                title='State-Yaw rate', path=path, highlight=highlight)
    single_plot(data_all, ['time_receive_gps','time_receive_can', 'time_receive_radar','time_decision'],
                title='Time',y_lim=[0,0.2], path=path, highlight=highlight)

def single_plot_compare_response(data_all, path, AutoMode=True):
    highlight = AutoMode
    single_plot(data_all, ['GaussX','model_x_in_real_action', 'model_x_in_model_action'],
                title='Response-positionX', path=path, highlight=highlight)
    single_plot(data_all, ['GaussY', 'model_y_in_real_action', 'model_y_in_model_action'],
                title='Response-positionY', path=path, highlight=highlight)
    single_plot(data_all, ['Heading', 'model_phi_in_real_action', 'model_phi_in_model_action'],
                title='Response-heading', path=path, highlight=highlight)
    single_plot(data_all, ['ego_vx', 'model_vx_in_real_action', 'model_vx_in_model_action'],
                title='Response-speedX', path=path, highlight=highlight)
    single_plot(data_all, ['ego_vy', 'model_vy_in_real_action', 'model_vy_in_model_action'],
                title='Response-speedY', path=path, highlight=highlight)
    single_plot(data_all, ['YawRate', 'model_r_in_real_action', 'model_r_in_model_action'],
                title='Response-yawrate', path=path, highlight=highlight)
    single_plot(data_all, ['model_front_wheel_rad_in_real_action', 'model_front_wheel_rad_in_model_action'],
                title='Response-steering', path=path, highlight=highlight)
    single_plot(data_all, ['accActual','Deceleration', 'model_acc_in_real_action', 'model_acc_in_model_action'],
                title='Response-acc', path=path, highlight=highlight)




def single_plot_other_series(data_all, path):
    single_plot(data_all, [('GaussX', 'GaussY'),('model_x_in_model_action','model_y_in_model_action'),('model_x_in_real_action','model_y_in_real_action')], title='Trajectory', path=path,  highlight=False)

def single_plot_other_vehicles(data_all, path, highlight=True):
    for i in range(6):
        for real_key in ['delta_x','delta_y','delta_v','delta_phi']:
            key = 'other' + str(i) + '_' + real_key
            title = 'Other_obs-' + key
            single_plot(data_all, [key], title=title,  path=path, highlight=highlight)


if __name__ == '__main__':
    exp_index = 'case0/noise1/06_171041_real'
    model_index = 'left/experiment-2021-01-06-14-33-09'
    data_all, keys_for_data = load_data(model_index, exp_index)
    print(keys_for_data)
    path = (exp_index, model_index)

    single_plot_time_series(data_all, path, AutoMode=True) # if not switch into auto mode, add kwargs: VehicleMode=False
    single_plot_other_series(data_all, path)
    # single_plot_other_vehicles(data_all, exp_index, highlight=False)
    single_plot_compare_response(data_all, path, AutoMode=True)
