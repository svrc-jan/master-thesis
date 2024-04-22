#!/usr/bin/env python3

import time
import sys
import os
import subprocess

import numpy as np
import matplotlib.pyplot as plt


def tail_file(file):
    process = subprocess.Popen(["tail", "-f", file, "-s", "0.1"], stdout=subprocess.PIPE)
    while True:
        output = process.stdout.readline()
        if output == b'' and process.poll() is not None:
            break
        if output:
            if output != b'':
                yield output.decode('ascii').strip()


def read_file(file):
    with open(file) as f:
        while True:
            line = f.readline().strip()
            if line == '':
                break

            yield line

if __name__ == '__main__':

    # to run GUI event loop

    plt.rcParams['keymap.save'].remove('s')
    pos = np.empty((0, 4))

    fig_sz = 10
    n_hist = 200


    scale = 1.5


    figure, (ax, ax2) = plt.subplots(figsize=(fig_sz, fig_sz + 5), nrows=2, gridspec_kw={'height_ratios': [fig_sz, 4]})
    
    ax.set_xlim(-scale*fig_sz, scale*fig_sz)
    ax.set_ylim(-scale*fig_sz, scale*fig_sz)


    line, = ax.plot([0], [0])
    quiv = ax.quiver([0], [0], [0.03], [0.03], color='red', pivot='mid')

    
    
    # ax2.set_ylim(0, 2*scale*fig_sz)

    ax2.set_ylim(-3.5, 3.5)
    ax2.set_xlim(-n_hist, 0)

    line2, = ax2.plot([0], [0])

    plt.ion()
    plt.show()

    t = 0
    
    dt = 0.01

    for data in read_file(sys.argv[1]):

        if data != '':
            data = data.split(',')
            if data[0] == 'dt':
                dt = float(data[1])
                print(dt)

            elif data[0] == 'pos':
                p = [float(x) for x in data[2:]]


                pos = np.concatenate((pos, np.array(p)[None, :]), axis=0)


                x, y, z, a = tuple(pos[-1,:4].tolist())

                line.set_xdata(np.clip(pos[:, 0], -scale*fig_sz + 0.01, scale*fig_sz - 0.01))
                line.set_ydata(np.clip(pos[:, 1], -scale*fig_sz + 0.01, scale*fig_sz - 0.01))

                

                quiv.set_offsets([(x, y)])
                quiv.set_UVC([0.03*np.cos(a)], [0.03*np.sin(a)])


            plt.pause(dt*0.1)

            
            t = t + 1
        
    
    plt.close('all')


    