from numpy import *
from CoordinateFrames import *

class AutonomousPlanner:
  def __init__( self, robSim ) 
    #Get reference to the simgle robSim instance
    self.robSim = robSim

  def update ( self )
    #Run filter computation, update d,r in robSim
    
  def stepDir ( self ) 
    #Dot product of X,Y component vectors wrt interwaypoint line
    #return X or Y depending on which dot product is greater

