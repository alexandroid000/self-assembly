import numpy as np
import matplotlib.pyplot as plt
import math as m
from os.path import join


def first_order(write_graph, save_as):
    f = open(save_as + "_interpolated_trajectory.txt")

    trajectory_x = [[],[],[]]
    trajectory_y = [[],[],[]]

    for lines in f:
        l = lines.strip()
        l = l.split(',')
        for i in range(3):
            trajectory_x[i].append(float(l[2*i]))
            trajectory_y[i].append(float(l[2*i+1]))
    f.close()

    windings = [[],[],[]]

    past_x0 = trajectory_x[0][0]
    past_y0 = trajectory_y[0][0]
    past_x1 = trajectory_x[1][0]
    past_y1 = trajectory_y[1][0]
    past_x2 = trajectory_x[2][0]
    past_y2 = trajectory_y[2][0]

    max_delta = 0
    sum_delta = 0
    frame_num = 0

    past_arg = [np.arctan2(past_y2-past_y1, past_x2-past_x1), np.arctan2(past_y0-past_y2, past_x0-past_x2), np.arctan2(past_y1-past_y0, past_x1-past_x0)]
    cum_delta = [[0],[0],[0]]

    for i in range(1, len(trajectory_x[0])):
        new_x0 = trajectory_x[0][i]
        new_y0 = trajectory_y[0][i]
        new_x1 = trajectory_x[1][i]
        new_y1 = trajectory_y[1][i]
        new_x2 = trajectory_x[2][i]
        new_y2 = trajectory_y[2][i]

        new_arg = [np.arctan2(new_y2-new_y1, new_x2-new_x1), np.arctan2(new_y0-new_y2, new_x0-new_x2), np.arctan2(new_y1-new_y0, new_x1-new_x0)]

        delta = [0,0,0]

        for j in range(3):
            if new_arg[j] > m.pi/2. and past_arg[j] < -m.pi/2.:
                delta = new_arg[j] - past_arg[j] - 2*m.pi
            elif new_arg[j] < -m.pi/2. and past_arg[j] > m.pi/2.:
                delta = new_arg[j] - past_arg[j] + 2*m.pi
            else:
                delta = new_arg[j] - past_arg[j]
            cum_delta[j].append((cum_delta[j][-1]+delta/(2*m.pi)))
            if abs(delta) > max_delta:
                max_delta = abs(delta)
                frame_num = i
        sum_delta += abs(delta)
        past_arg = new_arg

    print max_delta, sum_delta / (len(trajectory_x[0])*3.), frame_num

    filename = save_as + "_winding_angles.txt"

    f = open(filename,'w')
    for i in range(len(cum_delta[0])):
        string_to_write = ''
        for j in range(3):
            string_to_write+=str(cum_delta[j][i]) + ','
        string_to_write+= '\n'
        f.write(string_to_write)
    f.close()

    if write_graph:
        filename = save_as + "_Winding_Numbers.pdf"
        plt.plot(cum_delta[0],'b')
        plt.plot(cum_delta[1],'g')
        plt.plot(cum_delta[2],'r')
        plt.savefig(filename)
        plt.show()

    return 1


def second_order(write_graph,save_as):
    filename = save_as  + "_winding_angles.txt"
    f = open(filename)

    winding = []
    for lines in f:
        l = lines.strip()
        l = l.split(',')
        winding.append([float(l[0]), float(l[1]), float(l[2])])
    f.close()
    double_winding = [[0],[0],[0]]

    past = [winding[0][0], winding[0][1], winding[0][2]]
    past_theta = [0,0,0]
    max_delta = [0, 0]

    for i in range(1, len(winding)):
        new = [winding[i][0], winding[i][1], winding[i][2]]
        nums = [[1,2],[2,0],[0,1]]
        new_theta = [np.arctan2(new[nums[j][1]], new[nums[j][0]]) for j in range(3)]
        delta = [0,0,0]
        for j in range(3):    
            if new_theta[j] > m.pi/2. and past_theta[j] < -m.pi/2.:
                delta = new_theta[j] - past_theta[j] - 2*m.pi
            elif new_theta[j] < -m.pi/2. and past_theta[j] > m.pi/2.:
                delta = new_theta[j] - past_theta[j] + 2*m.pi
            else:
                delta = new_theta[j] - past_theta[j]
            double_winding[j].append(double_winding[j][-1] + delta * (new[nums[j][0]]**2 + new[nums[j][1]]**2)/2.)
        past_theta = new_theta
        past = new

    if write_graph:
        plt.plot(double_winding[0], 'b')
        plt.plot(double_winding[1], 'g')
        plt.plot(double_winding[2], 'r')
        filename = save_as + "_Double_Winding.pdf"
        plt.savefig(filename)
        plt.show()

    filename = save_as + "_double_winding.txt"

    f = open(filename,'w')
    for i in range(len(double_winding[0])):
        string_to_write = ''
        for j in range(3):
            string_to_write+=str(double_winding[j][i]) + ','
        string_to_write+= '\n'
        f.write(string_to_write)

    return 1

