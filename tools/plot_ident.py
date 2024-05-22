import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# plt.rcParams['text.usetex'] = True
plt.rcParams.update({
    "text.usetex": True,
    "font.family": "sans-serif",
    "font.sans-serif": "Helvetica",
})


def get_traj(file):
	df = pd.read_csv(file, names=('tag', 'ts', 'v1', 'v2', 'v3', 'v4'))

	df_s = df[df.tag == 'pos']
	df_s = df_s.rename(columns={'v1' : 'x', 'v2' : 'y', 'v3' : 'z', 'v4' : 'a'})

	df_t = df[df.tag == 'target']
	df_t = df_t.rename(columns={'v1' : 'x', 'v2' : 'y', 'v3' : 'z', 'v4' : 'a'})


	df_u = df[df.tag == 'input']
	df_u = df_u.rename(columns={'v1' : 'roll', 'v2' : 'pitch', 'v3' : 'yaw', 'v4' : 'throttle'})

	x, y, z, a = [df_s[v].to_numpy() for v in ['x', 'y', 'z', 'a']]
	roll, pitch, yaw, throttle = [df_u[v].to_numpy() for v in ['roll', 'pitch', 'yaw', 'throttle']]
	
	s = np.stack([x, y, z, a]).T
	u = np.stack([roll, pitch, yaw, throttle]).T


	return (s, u)




run = '065'

obs_tr, _ = get_traj(f"logs_ident/{run}.log")
est_tr, _ = get_traj(f"logs_ident/est/{run}.log")


cvut_blue = (0/255, 101/255, 189/255)
cvut_orange = (224/255, 82/255, 6/255)
cvut_green = (162/255, 173/255, 0/255)

fig_w = 7
fig_h = 4	

fig_r = fig_h/fig_w
fig_pad = 0.1

fig = plt.figure('ident', figsize=(fig_w, fig_h))
ax1 = fig.add_axes([fig_pad*fig_r, fig_pad, (1-2*fig_pad)*fig_r, 1-2*fig_pad])
ax2 = fig.add_axes([fig_r+0.5*fig_pad, 0.5+0.6*fig_pad, 1-(1+fig_pad)*fig_r, 0.5-1.6*fig_pad])
ax3 = fig.add_axes([fig_r+0.5*fig_pad, fig_pad, 1-(1+fig_pad)*fig_r, 0.5-1.6*fig_pad])

ax1.set_xlabel("$x$ [m]")
ax1.set_ylabel("$y$ [m]")

ln1, = ax1.plot(obs_tr[:, 0], obs_tr[:, 1], c=cvut_blue)
ln2, = ax1.plot(est_tr[:, 0], est_tr[:, 1], c=cvut_orange)

ax2.set_xlabel("$k$ [-]")
ax2.set_ylabel("$z$ [m]")

ax2.plot(obs_tr[:, 2], c=cvut_blue)
ax2.plot(est_tr[:, 2], c=cvut_orange)

ax3.set_xlabel("$k$ [-]")
ax3.set_ylabel("$\\theta$ [rad]")

ax3.plot(obs_tr[:, 3], c=cvut_blue)
ax3.plot(est_tr[:, 3], c=cvut_orange)

lines = [ln1, ln2]
labels = ('observation', 'estimated state')
fig.legend(lines, labels, loc='lower center', ncol=3, fancybox=True, bbox_to_anchor=(0.5, -0.1))
fig.suptitle("Parameter identification state estimates")

plt.savefig('plots/ident.pdf', bbox_inches="tight")






