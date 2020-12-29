from __future__ import print_function
import numpy as np
import time
import matplotlib as mpl
import math
import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns
from functools import reduce
import copy

sns.set(style="darkgrid")
METHOD_IDX_TO_METHOD_NAME = {0: 'DSAC', 1: 'SAC', 2: 'Double-Q SAC', 3: 'TD4',
                             4: 'TD3', 5: 'DDPG', 6: 'Single-Q SAC'}



def make_a_figure_of_n_runs_for_average_performance(env_name, run_numbers, method_numbers, init_run=0, init_method=0):
    # make a total dataframe
    df_list = []
    init_run = init_run
    init_method = init_method
    last_number_of_each_run = []
    for run_idx_ in range(init_run, init_run + run_numbers, 1):
        last_number_of_each_method = []
        for method_idx in range(init_method, init_method + method_numbers, 1):

            time = np.load('./' + env_name + '-run' + str(run_idx_) + '/method_' + str(method_idx) + '/result/time.npy')[2:22]/20

            method_name = METHOD_IDX_TO_METHOD_NAME[method_idx]
            method_name = [method_name] * time.shape[0]

            df_for_this_run_and_method = pd.DataFrame(dict(
                                                           Algorithms=method_name,
                                                            time=time,))
            df_list.append(df_for_this_run_and_method)
        last_number_of_each_run.append(last_number_of_each_method)

    total_dataframe = df_list[0].append(df_list[1:], ignore_index=True) \
        if run_numbers > 1 or method_numbers > 1 else df_list[0]

    f1 = plt.figure(1)
    ax1 = f1.add_axes([0.095, 0.14, 0.9,0.85])
    sns.boxplot(ax=ax1, x="Algorithms", y="time", data=total_dataframe,palette="bright",
                order=[METHOD_IDX_TO_METHOD_NAME[0],
                            METHOD_IDX_TO_METHOD_NAME[1],
                            METHOD_IDX_TO_METHOD_NAME[2],
                            METHOD_IDX_TO_METHOD_NAME[6],
                            METHOD_IDX_TO_METHOD_NAME[3],
                            METHOD_IDX_TO_METHOD_NAME[4],
                            METHOD_IDX_TO_METHOD_NAME[5]])

    ax1.set_ylabel('Wall-clock Time per 1000 Iterations [s]', fontsize=12)
    ax1.set_xticklabels((METHOD_IDX_TO_METHOD_NAME[0],
                         METHOD_IDX_TO_METHOD_NAME[1],
                         METHOD_IDX_TO_METHOD_NAME[2],
                         METHOD_IDX_TO_METHOD_NAME[6],
                         METHOD_IDX_TO_METHOD_NAME[3],
                         METHOD_IDX_TO_METHOD_NAME[4],
                         METHOD_IDX_TO_METHOD_NAME[5],), rotation=-20, fontsize=12)
    plt.yticks(fontsize=12)
    plt.xticks(fontsize=12)


def test_value_plot():
    for run in range(4):
        for i in range(4):
            iteration = np.array(range(60))

            n_episodes_info_history = []
            for _ in iteration:
                estimated_q = []
                true_q = []
                for _ in range(10):
                    len = np.random.randint(100, 700)
                    estimated_q.append(np.random.random(len))
                    true_q.append(np.random.random(len))
                n_episodes_info_history.append(dict(n_episode_evaluated_Q_list=np.array(estimated_q),
                                                    n_episode_true_gamma_return_list=np.array(true_q)))

            np.save('./' + 'test_data' + '-run' + str(run) + '/method_' + str(i) + '/result/iteration_evaluation.npy',
                    np.array(iteration))
            np.save(
                './' + 'test_data' + '-run' + str(run) + '/method_' + str(i) + '/result/n_episodes_info_history.npy',
                np.array(n_episodes_info_history))


    env_name = 'test_data'



if __name__ == '__main__':
    env_name = "Ant-v2"

    run_numbers = 5
    method_numbers = 7
    init_run = 0
    init_method = 0

    make_a_figure_of_n_runs_for_average_performance(env_name, run_numbers, method_numbers, init_run=init_run,
                                                    init_method=init_method)
    plt.show()
