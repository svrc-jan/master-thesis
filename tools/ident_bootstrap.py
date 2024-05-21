#!/usr/bin/env python3

import os
import sys
import glob
import re
import shutil

import numpy as np

target_folder = "/home/jsv/CVUT/master-thesis/logs_ident"

est_command = '/home/jsv/CVUT/master-thesis/build/run_model_ident \
			   /home/jsv/CVUT/master-thesis/config/ident_bootstrap.json \
			   2> /dev/null | grep "par est"'

par_log_file = '/home/jsv/CVUT/master-thesis/data/ident_bootstrap.csv'


def get_all_logs(dir):
	result = [y for x in os.walk(dir) for y in glob.glob(os.path.join(x[0], '*.log'))]
	return result

def get_param_est():
	est_str = os.popen(est_command).read()
	par_str = re.findall(r"[-+]?(?:\d*\.*\d+)", est_str)

	return [float(x) for x in par_str]

def clear_dir(dir):
	for f in glob.glob(f'{dir}/*.log'):
		os.remove(f)

if __name__ == '__main__':
	log_files = sorted(glob.glob(f'{target_folder}/*.log'))

	n_logs = len(log_files)

	if not os.path.isdir("{}/bootstrap".format(target_folder)):
		os.mkdir("{}/bootstrap".format(target_folder))

	if not os.path.isdir("{}/bootstrap/est".format(target_folder)):
		os.mkdir("{}/bootstrap/est".format(target_folder))

	if (len(sys.argv) < 2):
		exit(1)

	n_iter = int(sys.argv[1])
	with open(par_log_file, "a") as par_log:
		
		for it in range(n_iter):
			clear_dir("{}/bootstrap".format(target_folder))

			for i, j in enumerate(np.random.choice(n_logs, n_logs, replace=True)):
				shutil.copy(log_files[j], '{}/bootstrap/{:03d}.log'.format(target_folder,i+1))

			par = get_param_est()
			par_log.write("{:.4f},{:.4f},{:.4f},{:.4f}\n".format(*par))
			par_log.flush()
			print("it {}/{}, par est: {:.4f}, {:.4f}, {:.4f}, {:.4f}".format(it+1, n_iter, *par))
