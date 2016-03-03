from numpy import *

class RobotState:
	def __init__(self, pos=array([0, 0]), yaw = 0):
		self.pos = pos
		self.yaw = yaw
