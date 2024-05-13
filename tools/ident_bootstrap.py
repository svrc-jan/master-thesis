#!/usr/bin/env python3

import os
import sys

import pandas as pd
import numpy as np

import re

est_log = "/home/jsv/CVUT/master-thesis/tools/stats.log"
logs_folder = "/home/jsv/CVUT/master-thesis/logs/ident/filt"
target_folder = "/home/jsv/CVUT/master-thesis/logs/bootstrap"

est_command = '/home/jsv/CVUT/master-thesis/build/run_model_ident \
			   /home/jsv/CVUT/master-thesis/config/ident_bootstrap.json \
			   2> /dev/null | grep "par est"'

par_log_file = '/home/jsv/CVUT/master-thesis/tools/idend_boostrap.log'


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

def get_param_est():
	est_str = os.popen(est_command).read()
	par_str = re.findall(r"[-+]?(?:\d*\.*\d+)", est_str)

	return [float(x) for x in par_str]


if __name__ == '__main__':
	files_names = [ f"{logs_folder}/{f}" for f in os.listdir(logs_folder) if os.path.isfile(f"{logs_folder}/{f}")]

	u_delay = 30
	min_l = 300

	traj = sum([split_traj(file, u_delay, min_l) for file in files_names], [])
	n_traj = len(traj)

	if not os.path.isdir("{}/est".format(target_folder)):
		os.mkdir("{}/est".format(target_folder))

	if (len(sys.argv) < 2):
		exit(1)

	n_iter = int(sys.argv[1])
	with open(par_log_file, "a") as par_log:
		
		for it in range(n_iter):
			os.system("rm {}/*.log -f".format(target_folder))

			for log_i, tr_i in enumerate(np.random.choice(n_traj, n_traj, replace=True)):
				save_traj(traj[tr_i], "{}/{:03d}.log".format(target_folder, log_i))

			par = get_param_est()
			par_log.write("{:.4f},{:.4f},{:.4f},{:.4f}\n".format(*par))
			par_log.flush()
			print("it {}/{}, par est: {:.4f}, {:.4f}, {:.4f}, {:.4f}".format(it+1, n_iter, *par))
