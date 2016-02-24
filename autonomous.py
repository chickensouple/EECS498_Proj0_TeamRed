from numpy import *
from coordinateFrames import *
from switch import *
from directions import *
import operator

class AutonomousPlanner:
  def __init__(self, robSim, coordinateFrames): 
    #Get reference to the simgle robSim instance
    # self.robSim = robSim
    self.switch = Switch(robSim)
    self.coordinateFrames = coordinateFrames;
    self.directions = dict()
    self.directions[Directions.PosX] = array([1, 0])
    self.directions[Directions.NegX] = array([-1, 0])
    self.directions[Directions.PosY] = array([0, 1])
    self.directions[Directions.NegY] = array([0, -1])

  def updateFilter (self):
  	pass
    #Run filter computation, update d,r in robSim
    
  def plan(self, waypoints):
  	"""
  		plans the next step of the robot
  		waypoints are in real coordinates
  	"""

  	if (self.switch.inMotion()):
  		return
  	if (len(waypoints) <= 1):
  		return

  	# it seems that the waypoint x's are negative of what
  	# they are supposed to be
  	tempWaypoint = copy(waypoints[1])
  	tempWaypoint[0] = -tempWaypoint[0]

  	currPos = self.coordinateFrames.convertArbitraryToReal(self.switch.getPos())
  	directionVec = tempWaypoint - currPos
  	print("Target Waypoint: " + str(waypoints[1]))
  	print("Current Pos: " + str(currPos))
  	print("DirectionVec: " + str(directionVec))

  	directionDot = dict()
  	for direction, vec in self.directions.iteritems():
  		directionDot[direction] = dot(vec, directionVec)

  	# gets the direction with the largest dot product
  	direction = max(directionDot.iteritems(), key=operator.itemgetter(1))[0]
  	print Directions.Names[direction]
  	self.move(direction)

  def move(self, direction):
  	if (direction == Directions.PosX):
  		self.switch.movePosX()
  	elif (direction == Directions.NegX):
  		self.switch.moveNegX()
  	elif (direction == Directions.PosY):
  		self.switch.movePosY()
  	elif (direction == Directions.NegY):
  		self.switch.moveNegY()

