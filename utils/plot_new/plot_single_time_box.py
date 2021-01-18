import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
import pandas as pd
import os
from utils.plot_new.plot_utils.load_record import load_data
from utils.plot_new.plot_utils.search_index import search_geq,search_leq,search_automode_index

def load_all_data(model_index, case):
    multi_exp_data = []
    for noise_factor in [0,1,2,3,4,5,6]:
        proj_root_dir = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
        short_exp_father_dir = 'case'+str(case)+'/noise'+str(noise_factor)
        full_exp_father_dir = proj_root_dir + '/utils/models/' + model_index + '/record/' + short_exp_father_dir
        exp_name = os.listdir(full_exp_father_dir)[0]
        exp_index = short_exp_father_dir + '/' + exp_name
        data, _ = load_data(model_index, exp_index)
        multi_exp_data.append(data)

        fig_dir_in_full_exp_father_dir = proj_root_dir + '/utils/models/' + model_index  + '/record/case' \
                                         + str(case) + '/noise_figure'

        if not os.path.exists(fig_dir_in_full_exp_father_dir):
            os.mkdir(fig_dir_in_full_exp_father_dir)

    return multi_exp_data, fig_dir_in_full_exp_father_dir

def time_box_plot(data, path, **kwargs):
    sns.set(style="darkgrid", font_scale=1.2)
    Time = []
    for key in ['time_decision', 'time_receive_gps', 'time_receive_can', 'time_receive_radar']:
        if key == 'time_decision':
            time = np.array(data[key][1:])
            time = time / 0.022 * 0.005
            Time.append(time)
        else:
            Time.append(data[key][1:])
    time_sequence = {0: 'Decision', 1: 'GPS', 2: 'CAN', 3: 'Digital Twin'}
    time_df_list = []
    for i in range(0, 4, 1):
        time_PD = pd.DataFrame(dict(time=np.array(Time[i]), time_sequence=time_sequence[i], ))
        time_df_list.append(time_PD)
    time_dataframe = time_df_list[0].append(time_df_list[1:], ignore_index=True, )
    f1 = plt.figure("Time", dpi=200)
    ax1 = f1.add_axes([0.15, 0.14, 0.8, 0.85])
    sns.boxplot(ax=ax1, x="time_sequence", y="time", data=time_dataframe, palette="bright")
    ax1.set_ylabel('Time (s)',fontsize=22)
    ax1.set_xlabel('Progress',fontsize=22)
    # plt.yticks(fontsize=22)
    # plt.xticks(fontsize=22)
    # plt.show()

    proj_root_dir = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
    fig_path = proj_root_dir + '/utils/models/' + model_index + '/record/' + exp_index + '/figure/'
    data_fig_path = fig_path + 'data_fig/'
    if not os.path.exists(fig_path):
        os.mkdir(fig_path)
        os.mkdir(data_fig_path)
    name = data_fig_path + 'time_box' + '.jpg'
    plt.savefig(name)



if __name__ == '__main__':
    exp_index = 'case0/noise0/10_144620_real'
    model_index = 'left/experiment-2021-01-07-01-22-30'
    data_all, keys_for_data = load_data(model_index, exp_index)
    print(keys_for_data)
    path = (exp_index, model_index)
    time_box_plot(data_all, path)
