# -*- coding: utf-8 -*-
"""
Created on Thu Sep  4 20:31:13 2014

@author: shrevzen-home
"""
from gzip import open as opengz
from json import dumps as json_dumps
from numpy import asfarray, dot, c_, newaxis, mean, exp, sum, sqrt
from numpy.linalg import svd, inv
from numpy.random import randn
from waypointShared import *
from joy import *

from pdb import set_trace as DEBUG
from coordinateFrames import *
from common import *

MSG_TEMPLATE = {
       0: [[502, 251], [479, 272], [508, 296], [530, 274]],
       1: [[469, 347], [471, 311], [434, 305], [431, 339]],
       2: [[466, 226], [437, 209], [420, 231], [450, 248]],
       3: [[362, 204], [339, 224], [365, 245], [387, 224]],
       4: [[334, 336], [370, 326], [359, 294], [324, 301]],
       22: [[425, 145], [424, 169], [455, 173], [455, 149]],
       23: [[556, 302], [594, 308], [588, 277], [553, 272]],
       24: [[284, 258], [294, 229], [261, 221], [250, 249]],
       25: [[432, 424], [431, 384], [391, 380], [392, 419]],
       26: [[559, 399], [564, 440], [606, 444], [599, 403]],
       27: [[546, 163], [546, 187], [578, 193], [576, 170]],
       28: [[212, 397], [253, 395], [254, 355], [214, 358]],
       29: [[319, 129], [290, 124], [283, 145], [313, 153]]
  }
  
def tags2list( dic ):
    """ 
    Convert a dictionary of tags into part of a list for JSON serialization
    
    INPUT:
      dic -- dictionary mapping tag id to 4x2 corner location array
      
    OUTPUT:
      list to be concatenated into JSON message
    """
    return [ 
        { 
          u'i' : k, 
          u'p': [ list(row) for row in v ] 
        } 
        for k,v in dic.iteritems()
    ]

def findXing(a,b):
  """
  Find the crossing point of two lines, represented each by a pair of
  points on the line
  
  INPUT:
    a -- 2x2 -- two points, in rows
    b -- 2x2 -- two points, in rows
  
  OUTPUT: c -- 2 -- a point, as an array
  """
  a = asfarray(a)
  b = asfarray(b)
  # The nullspace of this matrix is the projective representation
  # of the intersection of the lines. Each column's nullspace is
  # one of the lines
  X = c_[a[1]-a[0],b[0]-b[1],a[0]-b[0]].T
  if X.ndim is not 2:
    DEBUG()
  Q = svd(X)[0]
  # Last singular vector is basis for nullspace; convert back from
  # projective to Cartesian representation
  q = Q[:2,2]/Q[2,2]
  c = q[0]*(a[1]-a[0])+a[0]
  return c
  
class RobotSimInterface( object ):
  """
  Abstract superclass RobotSimInterface defines the output-facing interface
  of a robot simulation.
  
  Subclasses of this class must implement all of the methods
  """
  def __init__(self, fn=None):
    """
    INPUT:
      fn -- filename / None -- laser log name to use for logging simulated 
          laser data. None logged if name is None
          
    ATTRIBUTES:
      tagPos -- 4x2 float array -- corners of robot tag
      laserAxis -- 2x2 float array -- two points along axis of laser
      waypoints -- dict -- maps waypoint tag numbers to 4x2 float 
          arrays of the tag corners
    """
    # Initialize dummy values into robot and arena state
    self.tagPos = asfarray(MSG_TEMPLATE[ ROBOT_TAGID[0]])
    self.laserAxis = dot([[1,1,0,0],[0,0,1,1]],self.tagPos)/2
    self.waypoints = { tid : asfarray(MSG_TEMPLATE[tid]) for tid in waypoints }

    ### Initialize internal variables
    # Two points on the laser screen
    self.laserScreen = asfarray([[-1,-1],[1,-1]])
    # Cache for simulated TagStreamer messages
    self._msg = None
    # Output for simulated laser data
    if not fn:
      self.out = None
    else:
      self.out = opengz(fn,"w")
    
  def refreshState( self ):
    """<<pure>> refresh the value of self.tagPos and self.laserAxis"""
    print "<<< MUST IMPLEMENT THIS METHOD >>>"
    
  def getTagMsg( self ):
    """
    Using the current state, generate a TagStreamer message simulating
    the robot state
    """
    # Cache a list of the corner tags. They don't move
    if self._msg is None:
      self._msg = tags2list({ tid : asfarray(MSG_TEMPLATE[tid]) for tid in corners})
    # Collect robot and waypoint locations
    state = { ROBOT_TAGID[0] : self.tagPos }
    state.update( self.waypoints )
    # Combine all into a list
    msg = tags2list(state) + self._msg
    # Serialize
    return json_dumps(msg)

  def logLaserValue( self, now ):
    """
    Using the current state, generate a fictitious laser pointer reading
    INPUT:
      now -- float -- timestamp to include in the message
    
    OUTPUT: string of human readable message (not what is in log)      
    """
    x = findXing( self.laserScreen, self.laserAxis )
    if self.out:
      self.out.write("%.2f, 1, %d, %d\n" % (now,n+1,x[0],x[1]))
    return "Laser: %d,%d " % tuple(x)


class RedRobotSim( RobotSimInterface ):
  def __init__(self, core, *args, **kw):
    RobotSimInterface.__init__(self, *args, **kw)

    # robot simulation constants
    self.wheelXNoise = 0.1 # cm
    self.wheelYNoise = 0.1 # cm
    self.yawNoise = 0.002 # radians
    self.xYawBias = 0.001 # radians
    self.yYawBias = -0.001 # radians

    self.slipBias = 0.2 # cm
    
    # state
    self.pos = [0, 0]; # cm (x, y)
    self.yaw = 0; # radians

    self.core = core

    # initializing coordinate frames
    cameraPts = array([sum(MSG_TEMPLATE[tid], axis=0)/4 for tid in corners])
    cameraPts = concatenate((cameraPts, [[1]]*8), axis=1)
    self.core.coordinateFrames.calculateRealToCameraTransformation(cameraPts, ref)

    self.drawRobotCorners()

  def drawRobotCorners(self):
    """
    Redraws the corners of the tag based on 
    self.pos, self.yaw
    and Constants.tagLength
    """
    L = Constants.tagLength
    borderPts = [[L/2, L/2],
      [L/2, -L/2],
      [-L/2, -L/2],
      [-L/2, L/2]]
    tagDiffs = zeros([4, 2])
    for i in range(len(tagDiffs)):
      tagDiffs[i] = CoordinateFrames.rotateCCW(borderPts[i], self.yaw)
    realTagPos = tagDiffs + self.pos
    for i in range(len(realTagPos)):
      self.tagPos[i] = self.core.coordinateFrames.convertRealToCamera(realTagPos[i])

  def moveX( self, dist ):
    """
    Move forward some distance in the X direction
    dist is the number of sides the wheel will turn
    must be an integer
    """
    self.pos[0] += (int(dist) * Constants.wheelSideLength) + (randn() * self.wheelXNoise)
    self.pos[0] -= (dist / abs(dist)) * abs(randn()) * self.slipBias
    self.yaw += (randn() * self.yawNoise) + self.xYawBias
    self.drawRobotCorners()

  def moveY(self, dist):
    """
    Move forward some distance in the Y direction
    dist is the number of sides the wheel will turn
    must be an integer
    """
    self.pos[1] += (int(dist) * Constants.wheelSideLength) + (randn() * self.wheelYNoise)
    self.pos[1] -= (dist / abs(dist)) * abs(randn()) * self.slipBias
    self.yaw += (randn() * self.yawNoise) + self.yYawBias
    self.drawRobotCorners()

  def moveTo(self, loc):
    self.pos = loc
    self.drawRobotCorners()
    
  def refreshState( self ):
    """
    Make state ready for use by client.
    
    ALGORITHM:
    Since the entire robot state is captured by the location of the
    robot tag corners, update the laser axis from the robot tag location 
    """
    self.laserAxis = dot([[1,1,0,0],[0,0,1,1]],self.tagPos)/2
    da = dot([1,-1],self.laserAxis)
    self.laserAxis[1] += randn(2) * sqrt(sum(da*da)) * 0.01
    