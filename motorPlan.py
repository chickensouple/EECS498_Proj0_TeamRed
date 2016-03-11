from joy import *
from math498 import *
from common import *

class MotorPlan(Plan):
  def __init__(self, app, *arg, **kw):
    Plan.__init__(self, app, *arg, **kw)
    self.direction = None
    self.angle = 0.2
    self.torque = 0.1

  def setDirection(self, direction):
    self.direction = direction

  def behavior(self):
    # choose motors based on direction
    if (self.direction == Directions.PosX):
      motors = self.app.xMotors
      angle = self.angle
      torque = self.torque
    elif (self.direction == Directions.NegX):
      motors = self.app.xMotors
      angle = -self.angle
      torque = -self.torque
    elif (self.direction == Directions.PosY):
      motors = self.app.yMotors
      angle = self.angle
      torque = self.torque
    elif (self.direction == Direcitons.NegY):
      motors = self.app.yMotors
      angle = -self.angle
      torque = -self.torque


    currAng0 = motors[0].get_xpos()
    targetAng0 = currAng + currAng0

    currAng1 = motors[1].get_xpos()
    targetAng1 = currAng - currAng1

    motors[0].set_torque(torque)
    motors[1].set_torque(-torque)

    threshold = 0.2

    while(abs(targetAng0 - motors[0].get_xpos()) > threshold and
      abs(targetAng1 - motors[1].get_xpos())):
      yield self.forDuration(0.05)


    motors[0].set_torque(0)
    motors[1].set_torque(0)

    motors[0].go_slack()
    motors[1].go_slack()






  # def __init__(self, app, *arg, **kw):
  #   Plan.__init__(self, app, *arg, **kw)
  #   self.motor = 0
  #   self.angle = 0.2
  #   self.motorDir = None

  # def setDir(self, motorDirection):
  #   self.motorDir = motorDirection

  # def setAngleIncrement(self, angle):
  # 	self.angle = angle


  # def behavior(self):
  #   # choose motors based on direction
  #   if (self.motorDir == MotorDir.X):
  #     idx0 = 0
  #     idx1 = 1
  #   elif (self.motorDir == MotorDir.Y):
  #     idx0 = 2
  #     idx1 = 3
  #   else:
  #     return


  #   self.app.motors[idx0].unslack()
  #   self.app.motors[idx1].unslack()

  #   currAng0 = self.app.motors[idx0].get_ang()
  #   targetAng0 = currAng0 + self.angle

  #   # 2nd motor needs to be reverse of first motor
  #   currAng1 = self.app.motors[idx1].get_ang()
  #   targetAng1 = currAng1 - self.angle


  #   wrappedTargetAng0 = wrapNum(targetAng0, -0.5, 0.5)
  #   wrappedTargetAng1 = wrapNum(targetAng1, -0.5, 0.5)


  #   self.app.motors[idx0].set_ang(targetAng0)
  #   self.app.motors[idx1].set_ang(targetAng1)

  #   threshold = 0.02
  #   while (wrappedAngleDiff(wrappedTargetAng0, self.app.motors[idx0].get_ang(), 0.5) > threshold or
  #     wrappedAngleDiff(wrappedTargetAng1, self.app.motors[idx1].get_ang(), 0.5) > threshold):
  #     yield self.forDuration(0.05)


  #   self.app.motors[idx0].slack()
  #   self.app.motors[idx1].slack()


