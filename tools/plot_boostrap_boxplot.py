import os
import sys

import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

cvut_blue = (0/255, 101/255, 189/255)
cvut_orange = (224/255, 82/255, 6/255)
cvut_green = (162/255, 173/255, 0/255)


plt.rcParams.update({
    "text.usetex": True,
    "font.family": "sans-serif",
    "font.sans-serif": "Helvetica",
})

fig, ax = plt.subplots()

par_file = '/home/jsv/CVUT/master-thesis/data/ident_boostrap.csv'
df = pd.read_csv(par_file, header=None, names=['lambda_hor', 'lambda_ver', 'lambda_ang', 'offset_ang'])

values = [1.065, 0.447, 0.955, -0.171]

ax.boxplot([df[v] for i, v in enumerate (['lambda_hor', 'lambda_ver', 'lambda_ang', 'offset_ang'])], medianprops=dict(color=cvut_blue))
    
ax.set_ylim([-0.8, 1.6])

pos = np.arange(4) + 1
upper_labels = [f'({str(round(s, 3))})' for s in values]
for tick, label in zip(range(4), ax.get_xticklabels()):
    k = tick % 2
    ax.text(pos[tick], .95, upper_labels[tick],
             transform=ax.get_xaxis_transform(),
             horizontalalignment='center')

ax.set_xticklabels(['$\\lambda_h$ [m/s]', '$\\lambda_v$ [m/s]', '$\\lambda_a$ [m/s]', '$\\theta_0$ [rad]'])
ax.set_xlabel('parameter')
ax.set_ylabel('values')
ax.set_title('Parameter estimate boxplot')

plt.tight_layout()
plt.savefig('plots/bootstrap.pdf')
