# %%
import numpy as np
import matplotlib.pyplot as plt
import scipy.stats as stats
from matplotlib import cm

# %%
d, i = np.loadtxt('tools/match_traj.csv', delimiter=',')
i = i/np.max(i)

# %%
t = np.sort(np.unique(i))
x = np.linspace(0, np.quantile(d, 0.7), 200)

t_sig = 0.1
x_sig = 0.05

dt = t[:, None] - i[None, :]
dx = x[:, None] - d[None, :]

pt = stats.norm.pdf(dt, 0, t_sig)
px = stats.norm.pdf(dx, 0, x_sig)

p = pt @ px.T
# p = p/np.sum(p, axis=0)[None, :]

p = p/np.sum(p, axis=1)[:, None]


# %%
fig, ax = plt.subplots(subplot_kw={"projection": "3d"})

x_mesh, t_mesh = np.meshgrid(x, t)

print(x_mesh.shape, t_mesh.shape, p.shape)

ax.plot_surface(x_mesh, t_mesh, p, cmap=cm.coolwarm,
                       linewidth=0, antialiased=False)

plt.show()


