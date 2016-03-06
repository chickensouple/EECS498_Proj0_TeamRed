from joy import *
from math498 import *

class MovePosXPlan(Plan):
  def __init__(self, app, *arg, **kw):
    Plan.__init__(self, app, *arg, **kw)

  def behavior(self):
    self.app.motorX.unslack()

    currAng = self.app.motorX.get_ang()
    targetAng = currAng + 0.2
    self.app.motorX.set_ang(targetAng)

    wrappedTargetAng = wrapNum(targetAng, -0.5, 0.5)

    threshold = 0.02
    while (wrappedAngleDiff(wrappedTargetAng, self.app.motorX.get_ang(), 0.5) > threshold):
    	yield self.forDuration(0.05)

    self.app.motorX.slack()

    yield


class MoveNegXPlan(Plan):
  def __init__(self, app, *arg, **kw):
    Plan.__init__(self, app, *arg, **kw)

  def behavior(self):
    self.app.motorX.unslack()

    currAng = self.app.motorX.get_ang()
    targetAng = currAng - 0.2
    self.app.motorX.set_ang(targetAng)

    wrappedTargetAng = wrapNum(targetAng, -0.5, 0.5)

    threshold = 0.02
    while (wrappedAngleDiff(wrappedTargetAng, self.app.motorX.get_ang(), 0.5) > threshold):
    	yield self.forDuration(0.05)

    self.app.motorX.slack()

    yield