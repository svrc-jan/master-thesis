#!/usr/bin/env python3

import os

import pandas as pd
import numpy as np


est_log = "/home/jsv/CVUT/master-thesis/tools/stats.log"
logs_folder = "/home/jsv/CVUT/master-thesis/logs/ident/filt"


def split_traj(file, u_delay=30, min_l=500):
	df = pd.read_csv(file, names=('tag', 'ts', 'v1', 'v2', 'v3', 'v4'))

	df_s = df[df.tag == 'pos']
	df_s = df_s.rename(columns={'v1' : 'x', 'v2' : 'y', 'v3' : 'z', 'v4' : 'a'})


	df_u = df[df.tag == 'input']
	df_u = df_u.rename(columns={'v1' : 'roll', 'v2' : 'pitch', 'v3' : 'yaw', 'v4' : 'throttle'})

	x, y, z, a = [df_s[v].to_numpy() for v in ['x', 'y', 'z', 'a']]
	roll, pitch, yaw, throttle = [df_u[v].to_numpy() for v in ['roll', 'pitch', 'yaw', 'throttle']]


	shift = 30

	x, y, z, a = [v[shift:] for v in [x, y, z, a]]
	L = min([len(v) for v in [x, y, z, a]])

	roll, pitch, yaw, throttle = [v[:L] for v in [roll, pitch, yaw, throttle]]

	
	N = L//min_l
	slc = np.arange(0, N)*(L/N)
	slc = [int(x) for x in slc] + [L]

	print(slc)




files_names = [ f"{logs_folder}/{f}" for f in os.listdir(logs_folder) if os.path.isfile(f"{logs_folder}/{f}")]

u_delay = 30
min_l = 500

split_traj(files_names[0], u_delay, min_l)

