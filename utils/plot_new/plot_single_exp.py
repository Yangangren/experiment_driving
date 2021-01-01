import matplotlib.pyplot as plt
from utils.plot_new.plot_utils.load_record import load_data


def single_plot(load_dir, keys=['Deceleration']):
    data_all = load_data(load_dir)
    plt.figure()
    for key in keys:
        plt.plot(data_all['Time'], data_all[key]) # data_all['Time']
    plt.legend(labels=keys,loc='best')
    plt.xlabel('time /s')
    plt.show()

if __name__ == '__main__':
    single_plot('left_case0_20210101_152459')
