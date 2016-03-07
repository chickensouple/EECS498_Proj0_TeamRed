from joy import *
from motorPlan import *
from common import *
from time import *
from coordinateFrames import *
from particleFilter import *

class Core(Plan):
  def __init__(self, mode, app, *arg, **kw):
    Plan.__init__(self, app, *arg, **kw)
    self.mode = mode
    self.sim = None


    self.sensor = []
    self.waypoints = []

    # simulation
    self.lastTime = 0

    self.coordinateFrames = CoordinateFrames()
    # used to make sure that the end of the motion can be processed by the filter
    self.startedMotion = False

    self.newSensor = False

    self.particleFilter = ParticleFilter(self)
    self.filterRunning = False

  def setSim(self, sim):
    self.sim = sim

####################
## MOTION
####################

  def movePosX(self):
    if (self.mode == Mode.SIMULATION):
      self.sim.moveX(1)
      self.lastTime = clock()
      self.startedMotion = True

  def moveNegX(self):
    if (self.mode == Mode.SIMULATION):
      self.sim.moveX(-1)
      self.lastTime = clock()
      self.startedMotion = True

  def movePosY(self):
    if (self.mode == Mode.SIMULATION):
      self.sim.moveY(1)
      self.lastTime = clock()
      self.startedMotion = True

  def moveNegY(self):
    if (self.mode == Mode.SIMULATION):
      self.sim.moveY(-1)
      self.lastTime = clock()
      self.startedMotion = True

  def inMotion(self):
    if (self.mode == Mode.SIMULATION):
      return (clock() - self.lastTime < 2)

#########################
## FILTER
#########################
  def pushSensorAndWaypoints(self, sensor, waypoints):
    self.sensor = sensor
    self.waypoints = waypoints
    self.particleFilter.setSensorAndWaypoints(self.sensor, self.waypoints)
    self.newSensor = True

  def startFilter(self):
    self.particleFilter.setSensorAndWaypoints(self.sensor, self.waypoints)
    self.particleFilter.setState(self.waypoints[0], 0)
    self.filterRunning = True

  def stopFilter(self):
    self.filterRunning = False

  def behavior(self):
    while (True):
      if (self.newSensor):
        self.newSensor = False
        if (self.filterRunning):
          self.particleFilter.correct()

      yield


      if (self.startedMotion):
        if (not self.inMotion):
          self.startedMotion = False

      yield

