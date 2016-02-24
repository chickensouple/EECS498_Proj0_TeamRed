from joy import *

class RobotMovementPlan( Plan ):
  def __init__(self, app, robSim, maxFreq=10.0, *arg, **binding ):
    self.robSim = robSim
    self.stepSize = 20
    self.stepDur = .1

  def behavior( self ):
    #auto mode switched on
    direction = self.robSim.autonomousPlanner.stepDir()
    if direction == 'X':
      robSim.moveX( self.stepSize )
      yield self.forDuration( self.stepDur )
      
    elif direction == 'Y':
      robSim.moveY( self.stepSize )
      yield self.forDuration( self.stepDur )
      
    yield

    
