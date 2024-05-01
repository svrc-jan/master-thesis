#!/usr/bin/env python3
import os
import sys

import numpy as np
import matplotlib.pyplot as plt


def read_file(file):
	with open(file) as f:
		while True:
			line = f.readline().strip()
			if line == '':
				break

			yield line


def plot_file(win, file_path):
	_, fig, ax1, ax2, ax3 = win

	pos = np.empty((0, 4))

	for data in read_file(file_path):
		if data != '':
			data = data.split(',')
			if data[0] == 'pos':
				p = [float(x) for x in data[2:]]
				if (len(p)) == 4:
					pos = np.concatenate((pos, np.array(p)[None, :4]), axis=0)
				
			# if data[0] == 'target':
			# 	p = [float(x) for x in data[1:]]

			# 	ax1.scatter(p[1], p[2], c = 4)
			# 	ax2.scatter(p[0], p[3], c = 4)
			# 	ax3.scatter(p[0], p[4], c = 4)


	ax1.plot(pos[:,0], pos[:,1])
	ax1.axis('equal')
	
	ax2.plot(pos[:,2])
	ax2.set_title("z")
	
	ax3.plot(pos[:,3])
	ax3.set_title("a")


def make_win(file_name):
	fig_w = 10
	fig_h = 6	

	fig_r = fig_h/fig_w
	fig_pad = 0.07

	fig = plt.figure(file_name, figsize=(fig_w, fig_h))
	ax1 = fig.add_axes([fig_pad*fig_r, fig_pad, (1-2*fig_pad)*fig_r, 1-2*fig_pad])
	ax2 = fig.add_axes([fig_r, 0.5+0.5*fig_pad, 1-(1+fig_pad)*fig_r, 0.5-1.5*fig_pad])
	ax3 = fig.add_axes([fig_r, fig_pad, 1-(1+fig_pad)*fig_r, 0.5-1.5*fig_pad])

	w = (file_name, fig, ax1, ax2, ax3)

	return w

if __name__ == '__main__':
	max_win = 10

	wins = []

	for arg in sys.argv[1:]:
		if arg.isnumeric():
			max_win = int(arg)
		
		elif os.path.isdir(arg):
			
			for file_name in os.listdir(arg):
				file_path = os.path.join(arg, file_name)
	
				if os.path.isfile(file_path):
					win_match = [w for w in wins if w[0] == file_name]
					if win_match:
						w = win_match[0]
					else:
						if len(wins) >= max_win:
							continue

						w = make_win(file_name)
						wins.append(w)
					
					plot_file(w, file_path)
		
		elif os.path.isfile(arg):
			win_match = [w for w in wins if wins[0] == arg]
			if win_match:
				w = win_match[0]
			else:
				if len(wins) >= max_win:
					continue

				w = make_win(os.path.basename(arg))
				wins.append(w)
			
			plot_file(w, arg)
	
	plt.show()