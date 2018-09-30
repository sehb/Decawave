import matplotlib.pyplot as plt 
import matplotlib.animation as animation
from matplotlib import style

fig = plt.figure(figsize=(12,12), dpi=70)
ax1 = fig.add_subplot(1,1,1)
plt_range = 8
plt_alpha = 0.1
spot_interval = 200


def animate(i):
	data = open('position.txt', 'r').read()
	lines = data.split('\n')
	dw_xpt = []
	dw_ypt = []
	dw_cur = 0
	vc_xpt = []
	vc_ypt = []
	vc_cur = 0
	
	dw_spot_x = []
	dw_spot_y = []
	dw_spot_t = []
	vc_spot_x = []
	vc_spot_y = []
	vc_spot_t = []

	line_num = 0
	for line in lines:
		line_num = line_num + 1
		if len(line) > 1:
			device, x, y, t = line.split(',')
			x = float(x)
			y = float(y)
			t = float(t)

			if device == 'dw':
				dw_xpt.append(x)
				dw_ypt.append(y)
				dw_cur = t

			if device == 'vc':
				vc_xpt.append(x)
				vc_ypt.append(y)
				vc_cur = t

			if line_num % spot_interval == 0:
				dw_spot_x.append(dw_xpt[-1])
				dw_spot_y.append(dw_ypt[-1])
				dw_spot_t.append(dw_cur)
				vc_spot_x.append(vc_xpt[-1])
				vc_spot_y.append(vc_ypt[-1])
				vc_spot_t.append(vc_cur)

	ax1.clear()
	ax1.set_ylim([-2,plt_range])
	ax1.set_xlim([-2,plt_range])
	ax1.grid(b=True)
	ax1.scatter(dw_xpt, dw_ypt, marker='o', c='g', alpha=plt_alpha, linewidth=0)
	ax1.scatter(vc_xpt, vc_ypt, marker='o', c='r', alpha=plt_alpha, linewidth=0)

	ax1.scatter(dw_spot_x, dw_spot_y, marker='x', c='g', alpha=1)
	for i, txt in enumerate(dw_spot_t):
		ax1.annotate("{:.4f}".format(txt), (dw_spot_x[i], dw_spot_y[i]))

	ax1.scatter(vc_spot_x, vc_spot_y, marker='x', c='r', alpha=1)
	for i, txt in enumerate(vc_spot_t):
		ax1.annotate("{:.4f}".format(txt), (vc_spot_x[i], vc_spot_y[i]))

ani = animation.FuncAnimation(fig, animate, interval=100)
plt.show()