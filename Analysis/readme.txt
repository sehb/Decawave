

Usage:

ROS nodes
	logger.py is subscribed to 
		/decaPos 
	and
		/vrpn_client_node/box/pose

	IP of vicon is on the PC cmd, ipconfig



setting origin by finding static position:
	set origin in logger.py to [0,0] for self.vc_origin and self.dw_origin
	run logger.py for a few seconds
	run mean_position.py to get the mean
	in logger.py, replace self.vc_origin and self.dw_origin with the new mean


realtime plotting
	make sure vicon and decawave are publishing messages
	open 2 terminals
	run rtplot.py in the 1st terminal
	run logger.py in the 2nd terminal
