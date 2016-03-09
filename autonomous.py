from numpy import *
from common import *
import operator
import pdb

class AutonomousPlanner:
  def __init__(self, core): 
    #Get reference to the single robSim instance
    self.core = core
    self.directions = dict()
    self.directions[Directions.PosX] = array([1, 0])
    self.directions[Directions.NegX] = array([-1, 0])
    self.directions[Directions.PosY] = array([0, 1])
    self.directions[Directions.NegY] = array([0, -1])
    
  def plan(self, waypoints):
    """
      plans the next step of the robot
      waypoints are in real coordinates
    """

    if (self.core.inMotion()):
      return
    if (len(waypoints) <= 1):
      return

    currPos = array(self.core.particleFilter.getState().pos)
    directionVec = array(waypoints[1]) - currPos
    # print("Target Waypoint: " + str(waypoints[1]))
    # print("Current Pos: " + str(currPos))
    # print("DirectionVec: " + str(directionVec))

    directionDot = dict()
    for direction, vec in self.directions.iteritems():
      directionDot[direction] = dot(vec, directionVec)

    # gets the direction with the largest dot product
    direction = max(directionDot.iteritems(), key=operator.itemgetter(1))[0]
    self.move(direction)

  def move(self, direction):
    if (direction == Directions.PosX):
      self.core.movePosX()
    elif (direction == Directions.NegX):
      self.core.moveNegX()
    elif (direction == Directions.PosY):
      self.core.movePosY()
    elif (direction == Directions.NegY):
      self.core.moveNegY()

