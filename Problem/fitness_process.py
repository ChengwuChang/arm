import numpy as np
import matplotlib.pyplot as plt
import os
nowpath = os.path.abspath(__file__)
fileDirectory = os.path.dirname(nowpath)
parentDirectory = os.path.dirname(fileDirectory)
data_path = os.path.join(parentDirectory, 'data')


def save(reward, name, times, type):
    x = reward
    target_path = os.path.join(data_path, name)
    fit = np.load(os.path.join(target_path, '%s_fit_val(%d).npy' % (name, times)))

    if fit.size == 0:
        fit = np.append(fit, x)
    else:
        last = fit[-1]
        if type:
            if x > last:
                fit = np.append(fit, x)
            else:
                fit = np.append(fit, last)
        else:
            if x < last:
                fit = np.append(fit, x)
            else:
                fit = np.append(fit, last)
    np.save(os.path.join(target_path, '%s_fit_val(%d).npy' % (name, times)), fit)


def clear(name, times):
    a = []
    for i in name:
        target_path = os.path.join(data_path, i)
        if not os.path.isdir(target_path):
            target_path = os.path.join(data_path, i)
            os.mkdir(target_path)
        np.save(os.path.join(target_path, '%s_fit_val(%d).npy' % (i, times)), a)


def show(name, times):
    for i in name:
        target_path = os.path.join(data_path, i)
        data1 = np.load(os.path.join(target_path, '%s_fit_val(%d).npy' % (i, times)))
        plt.plot(data1, label=i)
    # plt.title('NiaPy Learning Curve')
    plt.legend()
    plt.show()

def show_all(name, num):
    for i in name:
        Average_data_list = []
        sorted_list = []
        for t in range(num):
            target_path = os.path.join(data_path, i)
            data1 = np.load(os.path.join(target_path, '%s_fit_val(%d).npy' % (i, 0)))
            data2 = np.load(os.path.join(target_path, '%s_fit_val(%d).npy' % (i, 1)))
            data3 = np.load(os.path.join(target_path, '%s_fit_val(%d).npy' % (i, 2)))
            data4 = np.load(os.path.join(target_path, '%s_fit_val(%d).npy' % (i, 2)))
            data5 = np.load(os.path.join(target_path, '%s_fit_val(%d).npy' % (i, 2)))
            data6 = np.load(os.path.join(target_path, '%s_fit_val(%d).npy' % (i, 5)))
            data7 = np.load(os.path.join(target_path, '%s_fit_val(%d).npy' % (i, 6)))
            data8 = np.load(os.path.join(target_path, '%s_fit_val(%d).npy' % (i, 7)))
            data9 = np.load(os.path.join(target_path, '%s_fit_val(%d).npy' % (i, 8)))
            data10 = np.load(os.path.join(target_path, '%s_fit_val(%d).npy' % (i, 9)))
            data11 = np.load(os.path.join(target_path, '%s_fit_val(%d).npy' % (i, 10)))
            data12 = np.load(os.path.join(target_path, '%s_fit_val(%d).npy' % (i, 11)))
            data13 = np.load(os.path.join(target_path, '%s_fit_val(%d).npy' % (i, 12)))
            data14 = np.load(os.path.join(target_path, '%s_fit_val(%d).npy' % (i, 13)))
            data15 = np.load(os.path.join(target_path, '%s_fit_val(%d).npy' % (i, 14)))
            data16 = np.load(os.path.join(target_path, '%s_fit_val(%d).npy' % (i, 15)))
            data17 = np.load(os.path.join(target_path, '%s_fit_val(%d).npy' % (i, 16)))
            data18 = np.load(os.path.join(target_path, '%s_fit_val(%d).npy' % (i, 17)))
            data19 = np.load(os.path.join(target_path, '%s_fit_val(%d).npy' % (i, 18)))
            data20 = np.load(os.path.join(target_path, '%s_fit_val(%d).npy' % (i, 19)))
            data_list = [data1, data2, data3, data4, data5, data6, data7, data8, data9, data10, data11, data12, data13,
                        data14, data15, data16, data17, data18, data19, data20]#
            Average_data_list.append(data_list[t])
        Average_list = [sum(e) / len(e) for e in zip(*Average_data_list)]
        sorted_list = sorted(Average_list, reverse=True)
        print(Average_list)
        print(len(Average_list))
        plt.plot(sorted_list, label=i)#
    plt.title('Learning Curve')
    plt.legend()
    plt.show()