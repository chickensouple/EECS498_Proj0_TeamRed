from joy import *
from motorPlan import *
from common import *
from time import *
from coordinateFrames import *
from particleFilter import *
from autonomous import *

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

    self.newSensor = False

    self.particleFilter = ParticleFilter(self)
    self.filterRunning = False

    self.autonomousPlanner = AutonomousPlanner(self)


  def setSim(self, sim):
    self.sim = sim

####################
## MOTION
####################

  def movePosX(self):
    if (self.filterRunning):
      self.particleFilter.actionModel(Directions.PosX)

    if (self.mode == Mode.SIMULATION):
      self.sim.moveX(1)
    elif (self.mode == Mode.ACTUAL):
      # self.app.motorPlan.setMotorNum(0)
      if (self.app.motorPlan.isRunning()):
        return
      self.app.motorPlan.setDirection(Directions.PosX)
      self.app.motorPlan.start()

  def moveNegX(self):
    if (self.filterRunning):
      self.particleFilter.actionModel(Directions.NegX)

    if (self.mode == Mode.SIMULATION):
      self.sim.moveX(-1)
    elif (self.mode == Mode.ACTUAL):
      # self.app.motorPlan.setMotorNum(0)
      if (self.app.motorPlan.isRunning()):
        return
      self.app.motorPlan.setDirection(Directions.NegX)
      self.app.motorPlan.start()

  def movePosY(self):
    if (self.filterRunning):
      self.particleFilter.actionModel(Directions.PosY)

    if (self.mode == Mode.SIMULATION):
      self.sim.moveY(1)
    elif (self.mode == Mode.ACTUAL):
      # self.app.motorPlan.setMotorNum(1)
      if (self.app.motorPlan.isRunning()):
        return
      self.app.motorPlan.setDirection(Directions.PosY)
      self.app.motorPlan.start()

  def moveNegY(self):
    if (self.filterRunning):
      self.particleFilter.actionModel(Directions.NegY)

    if (self.mode == Mode.SIMULATION):
      self.sim.moveY(-1)
    elif (self.mode == Mode.ACTUAL):
      # self.app.motorPlan.setMotorNum(1)
      if (self.app.motorPlan.isRunning()):
        return
      self.app.motorPlan.setDirection(Directions.NegY)
      self.app.motorPlan.start()

  def inMotion(self):
    if (self.mode == Mode.SIMULATION):
      return (clock() - self.lastTime < 2)
    elif (self.mode == Mode.ACTUAL):
      return self.app.motorPlan.isRunning()

#########################
## FILTER
#########################
  def setSensorAndWaypoints(self, sensor, waypoints):
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


