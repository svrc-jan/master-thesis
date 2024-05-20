#!/usr/bin/env python3

import sys

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def state_eq(s, u, p):
	ds = np.empty_like(s)

	ds[0] = p[0]*(np.cos(s[3]+p[3])*u[1] + np.sin(s[2]+p[3])*u[0])
	ds[1] = p[0]*(np.sin(s[3]+p[3])*u[1] - np.cos(s[2]+p[3])*u[0])
	ds[2] = p[1]*u[3]
	ds[3] = p[2]*u[2]

	return ds

def get_step_eq(dt, p):
	def step_eq(s, u):
		return s + dt*state_eq(s, u, p)
	
	return step_eq

def get_traj(file):
	df = pd.read_csv(file, names=('tag', 'ts', 'v1', 'v2', 'v3', 'v4'))

	df_s = df[df.tag == 'pos']
	df_s = df_s.rename(columns={'v1' : 'x', 'v2' : 'y', 'v3' : 'z', 'v4' : 'a'})


	df_u = df[df.tag == 'input']
	df_u = df_u.rename(columns={'v1' : 'roll', 'v2' : 'pitch', 'v3' : 'yaw', 'v4' : 'throttle'})

	x, y, z, a = [df_s[v].to_numpy() for v in ['x', 'y', 'z', 'a']]
	roll, pitch, yaw, throttle = [df_u[v].to_numpy() for v in ['roll', 'pitch', 'yaw', 'throttle']]
	
	s = np.stack([x, y, z, a])
	u = np.stack([roll, pitch, yaw, throttle])

	return (s, u)


def predict(s, u, f, u_len):
	s_pred = s[:, u_len:]


	for t in range(u_len):
		u_pred = u[:,t:t+s_pred.shape[1]]
		s_pred = f(s_pred, u_pred)

	return s_pred


if __name__ == '__main__':
	dt = 0.02
	p = [1.04, 0.4471, 0.9554, -0.1]
	u_delay = 30
	s, u  = get_traj(sys.argv[1])
	
	f = get_step_eq(dt, p)
	
	s_pred = predict(s, u, f, u_delay)


	plt.plot(s[0, :], s[1, :])
	plt.plot(s_pred[0, :], s_pred[1,:])

	plt.show()

	t = np.arange(s.shape[1])

	plt.plot(t, s[0])
	plt.plot(u_delay + t[u_delay:], s_pred[0])


	s_a = s[:, u_delay:]
	u_a = u[:, :s_a.shape[1]]

	dx = dt*p[0]*(np.cos(s_a[3]+ p[3])*u_a[1] + np.sin(s_a[3] + p[3])*u_a[0])
	plt.plot(u_delay + t[u_delay:], np.cumsum(dx) - 2)
	plt.show()


	plt.plot(t, s[1])
	plt.plot(u_delay + t[u_delay:], s_pred[1])

	dy = dt*p[0]*(np.sin(s_a[3]+ p[3])*u_a[1] - np.cos(s_a[3] + p[3])*u_a[0])
	plt.plot(t[u_delay:], np.cumsum(dy)-1)
	plt.show()


	