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
            print(line)
            if line == '':
                break

            yield line

if __name__ == '__main__':

    # to run GUI event loop

    plt.rcParams['keymap.save'].remove('s')
    pos = np.empty((0, 8))

    fig_sz = 10
    n_hist = 200


    scale = 0.2


    figure, (ax, ax2) = plt.subplots(figsize=(fig_sz, fig_sz + 5), nrows=2, gridspec_kw={'height_ratios': [fig_sz, 4]})
    
    ax.set_xlim(-scale*fig_sz, scale*fig_sz)
    ax.set_ylim(-scale*fig_sz, scale*fig_sz)


    line, = ax.plot([0], [0])
    quiv = ax.quiver([0], [0], [0.03], [0.03], color='red', pivot='mid')

    
    
    # ax2.set_ylim(0, 2*scale*fig_sz)

    ax2.set_ylim(-3.5, 3.5)
    ax2.set_xlim(-n_hist, 0)

    line2, = ax2.plot([0], [0])
    line3, = ax2.plot([0], [0])

    plt.ion()
    plt.show()

    t = 0
    
    
    for data in read_file(sys.argv[1]):
        data = data.replace('\x00', '')
        data = data.replace(' ', '')


        if data != '':
            data = [float(x) for x in data.split(',')]

            pos = np.concatenate((pos, np.array(data[:])[None, :]), axis=0)


            x, y, z, a = tuple(pos[-1,:4].tolist())

            line.set_xdata(np.clip(pos[:, 0], -scale*fig_sz + 0.01, scale*fig_sz - 0.01))
            line.set_ydata(np.clip(pos[:, 1], -scale*fig_sz + 0.01, scale*fig_sz - 0.01))
            

            quiv.set_offsets([(x, y)])
            quiv.set_UVC([0.03*np.cos(a)], [0.03*np.sin(a)])

            # ax.set_xlim((x-10, x+10))
            # ax.set_ylim((y-10, y+10))

            
            line2.set_ydata(pos[max(0, t - n_hist):, 3])
            line2.set_xdata(np.arange(max(0, t - n_hist), t+1) - t)

            line3.set_ydata(pos[max(0, t - n_hist):, 7] + 2)
            line3.set_xdata(np.arange(max(0, t - n_hist), t+1) - t)
            


            # ax.set_ylim(z-10, z+10)


            plt.pause(0.001)

            
            t = t + 1
        
    
    plt.close('all')


    