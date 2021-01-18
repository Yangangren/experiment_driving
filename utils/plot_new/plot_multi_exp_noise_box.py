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

def noise_box_plot(data, key, case, **kwargs):
    df_list = []
    Noise=[0.0,1.0,2.0,3.0,4.0,5.0,6.0]
    for i in range(0, 7, 1):
        # min_index, max_index = search_automode_index(data[i])
        min_index, _ = search_geq(data[i]['ego_y'], -30)
        max_index, _ = search_leq(data[i]['ego_x'], -30)
        PD = pd.DataFrame(dict(YawRate=np.array(data[i][key][min_index:max_index]).squeeze(), Noise=Noise[i], ))
        df_list.append(PD)
    YawRate_dataframe = df_list[0].append(df_list[1:], ignore_index=True, )
    sns.set(style="darkgrid", font_scale=1.3)
    f2 = plt.figure(figsize=[10,5])
    ax2 = f2.add_axes([0.16, 0.14, 0.83, 0.85])
    title = 'case' + str(case)
    ax2.set_title(title)
    sns.boxplot(ax=ax2, x="Noise", y="YawRate", data=YawRate_dataframe, palette="bright",
                order=np.arange(0, 6.1, 1.0))
    y_label = {'a_x':r'Acceleration $(m/s^2)$',
               'SteerAngleAct':r'Steering Angle $(\degree)$',
               'tracking_delta_phi':r'Heading angle error $(\degree)$',
               'tracking_delta_y':r'Tracking error $(m)$'}
    ax2.set_ylabel(y_label[key], fontsize=22)
    ax2.set_xlabel('Noise Level', fontsize=22)
    # plt.yticks(fontsize=12)
    # plt.xticks(fontsize=12)

    if 'path' in kwargs.keys():
        fig_name = kwargs['path'] + '/' + key + '.jpg'
        plt.savefig(fig_name)
    else:
        plt.show()



if __name__ == '__main__':
    # IMPORTANT:
    # support only one experiment in one noise directory, e.g., only exists 10_144620_real in noise0/.
    # pls delete redundant experiment directory.
    model_index = 'left/experiment-2021-01-07-01-22-30'
    for case in [0]:
        for key in ['tracking_delta_phi','tracking_delta_y','SteerAngleAct','a_x']:
            multi_exp_data, fig_dir = load_all_data(model_index, case)
            noise_box_plot(multi_exp_data, key, case, path = fig_dir)
