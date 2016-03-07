from numpy import *


###############
## CONSTANTS ##
###############
class Constants:
	wheelNumSides = 6
	wheelSideLength = 8 # cm
	tagLength = 9 #cm


##################
## COMMON ENUMS ##
##################

class Directions:
	PosX = 0
	NegX = 1
	PosY = 2
	NegY = 3

	Names = ["PosX", "NegX", "PosY", "NegY"]

	@staticmethod
	def opposite(direction):
		if (direction == DirectionsPosX):
			return Directions.NegX
		if (direction == DirectionsNegX):
			return Directions.PosX
		if (direction == DirectionsPosY):
			return Directions.NegY
		if (direction == DirectionsNegY):
			return Directions.PosY
	
class RobotState:
	def __init__(self, pos=array([0, 0]), yaw = 0):
		self.pos = array(pos)
		self.yaw = yaw


class Mode:
	SIMULATION = 0
	ACTUAL = 1

