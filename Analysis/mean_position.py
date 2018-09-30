import numpy as np

data = open('position.txt', 'r').read()
lines = data.split('\n')
for line in lines:
	dw_xpt = []
	dw_ypt = []
	vc_xpt = []
	vc_ypt = []
	for line in lines:
		if len(line) > 1:
			device, x, y, t = line.split(',')
			x = float(x)
			y = float(y)
			if device == 'dw':
				dw_xpt.append(x)
				dw_ypt.append(y)
			if device == 'vc':
				vc_xpt.append(x)
				vc_ypt.append(y)


print('Decawave: size={} [{},{}]'.format(
	len(dw_xpt), np.mean(dw_xpt), np.mean(dw_ypt)))

print('Vicon: size={} [{},{}]'.format(
	len(vc_xpt), np.mean(vc_xpt), np.mean(vc_ypt)))