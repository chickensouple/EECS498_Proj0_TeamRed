from joy import *
from math498 import *

class MotorPlan(Plan):
  def __init__(self, app, *arg, **kw):
    Plan.__init__(self, app, *arg, **kw)
    self.motor = 0;
    self.angle = 0.2

  def setMotorNum(self, num):
  	self.motor = num

  def setAngleIncrement(self, angle):
  	self.angle = angle

  def behavior(self):
    self.app.motors[self.motor].unslack()

    currAng = self.app.motors[self.motor].get_ang()
    targetAng = currAng + self.angle
    self.app.motors[self.motor].set_ang(targetAng)

    wrappedTargetAng = wrapNum(targetAng, -0.5, 0.5)

    threshold = 0.02
    while (wrappedAngleDiff(wrappedTargetAng, self.app.motors[self.motor].get_ang(), 0.5) > threshold):
    	yield self.forDuration(0.05)

    self.app.motors[self.motor].slack()

    yield

