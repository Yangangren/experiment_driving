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
        min_index, max_index = search_automode_index(data[i])
        PD = pd.DataFrame(dict(YawRate=np.array(data[i][key][min_index:max_index]).squeeze(), Noise=Noise[i], ))
        df_list.append(PD)
    YawRate_dataframe = df_list[0].append(df_list[1:], ignore_index=True, )
    sns.set(style="darkgrid")
    f2 = plt.figure()
    ax2 = f2.add_axes([0.13, 0.14, 0.86, 0.85])
    title = 'case' + str(case)
    ax2.set_title(title)
    sns.boxplot(ax=ax2, x="Noise", y="YawRate", data=YawRate_dataframe, palette="bright",
                order=np.arange(0, 6.1, 1.0))
    y_label = {'a_x':'Acceleration',
               'SteerAngleAct':'Steering Angle',
               'tracking_delta_phi':'Heading angle error',
               'tracking_delta_y':'Tracking error'}
    ax2.set_ylabel(y_label[key], fontsize=12)
    plt.yticks(fontsize=12)
    plt.xticks(fontsize=12)

    if 'path' in kwargs.keys():
        fig_name = kwargs['path'] + '/' + key + '.jpg'
        plt.savefig(fig_name)
    else:
        plt.show()



if __name__ == '__main__':
    model_index = 'left/experiment-2021-01-07-01-22-30'
    for case in [0,1,2]:
        for key in ['tracking_delta_phi','tracking_delta_y','SteerAngleAct','a_x']:
            multi_exp_data, fig_dir = load_all_data(model_index, case)
            noise_box_plot(multi_exp_data, key, case, path = fig_dir)
