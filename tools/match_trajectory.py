#!/usr/bin/env python3

import sys
import glob

import pandas as pd
import numpy as np
import scipy
import scipy.spatial
import scipy.stats

import matplotlib.pyplot as plt

def get_unique_targets(df_t):
	targets = []
	for _, row in df_t.iterrows():
		tar = row[2:].to_numpy()
		if len(targets) == 0:
			targets.append(tar)

		elif np.any(tar !=  targets[-1]):
			targets.append(tar)

	return targets

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

	t = get_unique_targets(df_t)

	return (s, u, t)


def discretize_trajectory(targets, delta=0.02):
	traj = []
	for i in range(len(targets) - 1):
		diff = targets[i+1] - targets[i]
		norm = np.linalg.norm(diff[:3])
		
		n_steps = int(np.ceil(norm/delta))

		steps = np.arange(n_steps)[:, None]/n_steps * diff[None, :] + targets[i][None, :]

		traj.append(steps)

	traj.append(targets[-1][None, :])

	traj = np.concatenate(traj, axis=0)

	return traj

def match_trajectory(s, t_d):
	distance = scipy.spatial.distance_matrix(s[:, :2], t_d[:, :2])
	idx = np.argmin(distance, axis=1)

	return idx

def straighten_idx(idx):
	i = 0
	while (i < len(idx) - 1):
		if (idx[i] > idx[i+1]):
			idx[i], idx[i+1] = idx[i+1], idx[i]
			if (i > 0):
				i -= 1

		else:
			i += 1

	return idx

if __name__ == '__main__':
	d = []
	u = []
	i = []

	t_len = None
	for log in glob.glob('logs_square/*.log'):
		s, u_, t = get_traj(log)

		t_d = discretize_trajectory(t, 0.02)

		assert(t_len == len(t_d) or t_len is None)
		t_len = len(t_d)
	
		t_idx = match_trajectory(s, t_d)
		# t_idx = straighten_idx(t_idx)

		diff = (s - t_d[t_idx, :]).astype(np.double)
		dist = np.linalg.norm(diff, axis=1)

		d.append(dist)
		u.append(np.linalg.norm(u_[:,:4], axis=1))
		i.append(t_idx)


	d = np.concatenate(d)
	u = np.concatenate(u)
	i = np.concatenate(i)

	np.savetxt('data/match_traj.csv', (d, u, i), delimiter=',')


