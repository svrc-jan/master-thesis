#!/usr/bin/env python3

import os
import sys

import pandas as pd
import numpy as np

import re

logs_folder = "/home/jsv/CVUT/master-thesis/logs/ident/filt"
target_folder = "/home/jsv/CVUT/master-thesis/logs_ident"


def split_traj(file, u_delay=30, min_l=500):
	df = pd.read_csv(file, names=('tag', 'ts', 'v1', 'v2', 'v3', 'v4'))

	df_s = df[df.tag == 'pos']
	df_s = df_s.rename(columns={'v1' : 'x', 'v2' : 'y', 'v3' : 'z', 'v4' : 'a'})


	df_u = df[df.tag == 'input']
	df_u = df_u.rename(columns={'v1' : 'roll', 'v2' : 'pitch', 'v3' : 'yaw', 'v4' : 'throttle'})

	x, y, z, a = [df_s[v].to_numpy() for v in ['x', 'y', 'z', 'a']]
	roll, pitch, yaw, throttle = [df_u[v].to_numpy() for v in ['roll', 'pitch', 'yaw', 'throttle']]


	x, y, z, a = [v[u_delay:] for v in [x, y, z, a]]
	L = min([len(v) for v in [x, y, z, a]])

	roll, pitch, yaw, throttle = [v[:L] for v in [roll, pitch, yaw, throttle]]

	
	N = L//min_l
	slc = np.arange(0, N)*(L/N)
	slc = [int(x) for x in slc] + [L]

	tr = []

	for i in range(N):
		tr.append([v[slc[i]:slc[i+1]] for v in [x, y, z, a, roll, pitch, yaw, throttle]])

	return tr


def save_traj(traj, file_name):
	x, y, z, a, roll, pitch, yaw, throttle = traj

	with open(file_name, "w") as file:
		for t in range(len(x)):
			file.write("pos,{},{:.2f},{:.2f},{:.2f},{:.2f}\n".format(t, x[t], y[t], z[t], a[t]))
			file.write("input,{},{:.2f},{:.2f},{:.2f},{:.2f}\n".format(t, roll[t], pitch[t], yaw[t], throttle[t]))


if __name__ == '__main__':
	files_names = [ f"{logs_folder}/{f}" for f in os.listdir(logs_folder) if os.path.isfile(f"{logs_folder}/{f}")]

	u_delay = 30
	min_l = 300

	traj = sum([split_traj(file, u_delay, min_l) for file in files_names], [])
		
	os.system("rm {}/*.log -f".format(target_folder))

	for log_i, tr in enumerate(traj):
		save_traj(tr, "{}/{:03d}.log".format(target_folder, log_i+1))
