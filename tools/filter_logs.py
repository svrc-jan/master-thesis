#!/usr/bin/env python3

import sys
import os
import shutil
import glob

import pandas as pd
import numpy as np
import scipy
import scipy.spatial

import matplotlib.pyplot as plt

from match_trajectory import get_traj, discretize_trajectory

log_dir = 'logs'
target_dir = 'logs_square'

def get_all_logs(dir):
	result = [y for x in os.walk(dir) for y in glob.glob(os.path.join(x[0], 'square*.log'))]
	return result

def is_valid_log(log_file, n_tar=5, max_traj_dist=0.3, max_end_dist=0.2):
	s, u, t = get_traj(log_file)

	if len(t) != n_tar:
		return False

	if np.linalg.norm(s[-1, :2] - t[-1][:2]) > max_end_dist:
		return False

	t_d = discretize_trajectory(t)
	dist = scipy.spatial.distance_matrix(s[:, :2], t_d[:, :2])
	if np.max(np.min(dist, axis=1)) > max_traj_dist:
		return False

	return True

def clear_dir(dir):
	for f in glob.glob(f'{dir}/*.log'):
		os.remove(f)


if __name__ == '__main__':
	log_count = 1

	clear_dir(target_dir)

	for log in get_all_logs(log_dir):
		if (is_valid_log(log)):
			shutil.copy(log, '{}/{:03d}.log'.format(target_dir, log_count))
			log_count += 1


	for log in get_all_logs(target_dir):
		s, _, _ = get_traj(log)
		
		plt.plot(s[:,0], s[:, 1])
		plt.show()

