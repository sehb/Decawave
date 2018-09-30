import rospy
import numpy as np
from time import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from geometry_msgs.msg import Point, PoseStamped


PRINT_POS_FORMAT = "{: <8} |x:{: 8.3f}|y:{: 8.3f}|z:{: 8.3f}"
UPDATE_RATE = 20. 

VRPN_SUBSCRIBER = ""
DECA_SUBSCRIBER = ""

class Plotter():
	def __init__(self):

		self.vc_pos = {}
		self.dw_pos = {}
		self.dw_origin = [4.32343120686,4.47541080897] # manually set
		self.vc_origin = [0.223364139319,-0.129551625864] # manually set

		f = open("position.txt","w+") # empty the file
		f.close()

		self.vc_prev_time = time()
		self.dw_prev_time = time()
		self.start_t = time()


	def vc_cb(self, data):
		t = time()
		if t-self.dw_prev_time > 1./UPDATE_RATE:
			self.vc_pos = {
				'x': data.pose.position.x-self.vc_origin[0],
				'y': data.pose.position.y-self.vc_origin[1],
				'z': data.pose.position.z
			}
			print(PRINT_POS_FORMAT.format(
				"Vicon",
				self.vc_pos['x'],
				self.vc_pos['y'],
				self.vc_pos['z']))

			with open("position.txt", "a") as f:
				f.write("vc,{},{},{}\n".format(self.vc_pos['x'], self.vc_pos['y'], t-self.start_t))

			self.prev_time = t

	def dw_cb(self, data):
		t = time()
		if t-self.dw_prev_time > 1./UPDATE_RATE:
			self.dw_pos = {
				'x': data.x-self.dw_origin[0],
				'y': data.y-self.dw_origin[1],
				'z': data.z
			}
			print(PRINT_POS_FORMAT.format(
				"Decawave",
				self.dw_pos['x'],
				self.dw_pos['y'],
				self.dw_pos['z']))

			if self.dw_pos['x']>10 or self.dw_pos['y']>10:
				print('WARNING: out of range')

			with open("position.txt", "a") as f:
				f.write("dw,{},{},{}\n".format(self.dw_pos['x'], self.dw_pos['y'], t-self.start_t))

			self.prev_time = t

		
	def listener(self):
		
		rospy.init_node('plotter')
		rospy.Subscriber("/vrpn_client_node/box/pose", PoseStamped, self.vc_cb)
		rospy.Subscriber("/decaPos", Point, self.dw_cb)
		rospy.spin()



if __name__ == "__main__":
	P = Plotter()
	P.listener()

	
