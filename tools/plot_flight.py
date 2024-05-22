import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# plt.rcParams['text.usetex'] = True
plt.rcParams.update({
	"text.usetex": True,
	"font.family": "sans-serif",
	"font.sans-serif": "Helvetica",
})



cvut_blue = (0/255, 101/255, 189/255)
cvut_orange = (224/255, 82/255, 6/255)
cvut_green = (162/255, 173/255, 0/255)
cvut_gray = (155/255, 155/255, 155/255, 0.5)
shadow_color = (0, 0, 0, 0.35)

def set_axes_equal(ax):
	"""
	Make axes of 3D plot have equal scale so that spheres appear as spheres,
	cubes as cubes, etc.

	Input
	  ax: a matplotlib axis, e.g., as output from plt.gca().
	"""

	x_limits = ax.get_xlim3d()
	y_limits = ax.get_ylim3d()
	z_limits = ax.get_zlim3d()

	x_range = abs(x_limits[1] - x_limits[0])
	x_middle = np.mean(x_limits)
	y_range = abs(y_limits[1] - y_limits[0])
	y_middle = np.mean(y_limits)
	z_range = abs(z_limits[1] - z_limits[0])
	z_middle = np.mean(z_limits)

	# The plot bounding box is a sphere in the sense of the infinity
	# norm, hence I call half the max range the plot radius.
	plot_radius = 0.5*max([x_range, y_range, z_range])

	ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
	ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
	ax.set_zlim3d([0, 2*plot_radius])

def get_unique_targets(df_t):
	targets = []
	for _, row in df_t.iterrows():
		tar = row[2:].to_numpy()
		if len(targets) == 0:
			targets.append(tar)

		elif np.any(tar !=  targets[-1]):
			targets.append(tar)

	return targets

def get_traj(file, clip = None):
	df = pd.read_csv(file, names=('tag', 'ts', 'v1', 'v2', 'v3', 'v4'))

	df_s = df[df.tag == 'pos']
	df_s = df_s.rename(columns={'v1' : 'x', 'v2' : 'y', 'v3' : 'z', 'v4' : 'a'})

	df_t = df[df.tag == 'target']
	df_t = df_t.rename(columns={'v1' : 'x', 'v2' : 'y', 'v3' : 'z', 'v4' : 'a'})


	df_u = df[df.tag == 'input']
	df_u = df_u.rename(columns={'v1' : 'roll', 'v2' : 'pitch', 'v3' : 'yaw', 'v4' : 'throttle'})

	if clip is not None:
		df_s = df_s.head(clip)
		df_t = df_t.head(clip)
		df_u = df_u.head(clip)

	x, y, z, a = [df_s[v].to_numpy() for v in ['x', 'y', 'z', 'a']]
	roll, pitch, yaw, throttle = [df_u[v].to_numpy() for v in ['roll', 'pitch', 'yaw', 'throttle']]
	
	
	s = np.stack([x, y, z, a]).T
	u = np.stack([roll, pitch, yaw, throttle]).T

	t = get_unique_targets(df_t)

	return (s, u, t)



def plot_shadow(ax, x, y, c, ls="-"):

	z = np.zeros_like(x)
	
	return ax.plot(x, y, z, color=c,linestyle=ls)

def plot_flight(log, name, clip = None):

	s, u , t  = get_traj(f"{log}.log", clip)
	t = np.stack(t, axis=0)

	fig = plt.figure()
	ax = fig.add_subplot(projection='3d')


	ln2, = ax.plot(t[:,0], t[:, 1], t[:, 2], color=cvut_orange, linestyle="--")
	ln1, = ax.plot(s[:,0], s[:, 1], s[:, 2], color=cvut_blue)

	plot_shadow(ax, t[:,0], t[:, 1], shadow_color, "--")
	plot_shadow(ax, s[:,0], s[:, 1], shadow_color)
	set_axes_equal(ax)


	lines = [ln1, ln2]
	labels = ('states', 'reference')
	ax.set_title(f'{name.capitalize()} trajectory')
	ax.set_xlabel('$x$ [m]')
	ax.set_ylabel('$y$ [m]')
	ax.set_zlabel('$z$ [m]')



	# plt.show()
	fig.set_figheight(5)
	fig.set_figwidth(5)
	plt.savefig(f'plots/{name}_flight.pdf')



plot_flight('logs_square/002', 'square')
plot_flight('logs/mpc6/circle1_1', 'arc', clip=2200)
plot_flight('logs/mpc5/big_square1_2', 'diamond')





