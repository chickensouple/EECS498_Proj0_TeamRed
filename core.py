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
    # used to make sure that the end of the motion can be processed by the filter
    self.startedMotion = False

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
    self.startedMotion = True

    if (self.mode == Mode.SIMULATION):
      self.sim.moveX(1)
      self.lastTime = clock()
    elif (self.mode == Mode.ACTUAL):
      # self.app.motorPlan.setMotorNum(0)
      if (not self.app.motorPlan.isRunning()):
        return
      self.app.motorPlan.setDir(MotorDir.X)
      self.app.motorPlan.setAngleIncrement(0.2)
      self.app.motorPlan.start()

  def moveNegX(self):
    if (self.filterRunning):
      self.particleFilter.actionModel(Directions.NegX)
    self.startedMotion = True

    if (self.mode == Mode.SIMULATION):
      self.sim.moveX(-1)
      self.lastTime = clock()
    elif (self.mode == Mode.ACTUAL):
      # self.app.motorPlan.setMotorNum(0)
      if (not self.app.motorPlan.isRunning()):
        return
      self.app.motorPlan.setDir(MotorDir.X)
      self.app.motorPlan.setAngleIncrement(-0.2)
      self.app.motorPlan.start()

  def movePosY(self):
    if (self.filterRunning):
      self.particleFilter.actionModel(Directions.PosY)
    self.startedMotion = True

    if (self.mode == Mode.SIMULATION):
      self.sim.moveY(1)
      self.lastTime = clock()
    elif (self.mode == Mode.ACTUAL):
      # self.app.motorPlan.setMotorNum(1)
      if (not self.app.motorPlan.isRunning()):
        return
      self.app.motorPlan.setDir(MotorDir.Y)
      self.app.motorPlan.setAngleIncrement(0.2)
      self.app.motorPlan.start()

  def moveNegY(self):
    if (self.filterRunning):
      self.particleFilter.actionModel(Directions.NegY)
    self.startedMotion = True

    if (self.mode == Mode.SIMULATION):
      self.sim.moveY(-1)
      self.lastTime = clock()
    elif (self.mode == Mode.ACTUAL):
      # self.app.motorPlan.setMotorNum(1)
      if (not self.app.motorPlan.isRunning()):
        return
      self.app.motorPlan.setDir(MotorDir.Y)
      self.app.motorPlan.setAngleIncrement(-0.2)
      self.app.motorPlan.start()

  def inMotion(self):
    if (self.mode == Mode.SIMULATION):
      return (clock() - self.lastTime < 2)
    elif (self.mode == Mode.ACTUAL):
      return self.app.motorPlan.isRunning()

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


      # if (self.startedMotion):
      #   if (not self.inMotion):
      #     self.startedMotion = False

      # yield

