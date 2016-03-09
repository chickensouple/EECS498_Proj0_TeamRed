# file simTagStreamer.py simulates a robot in an arena

from sensorPlanTCP import SensorPlanTCP
from redRobotSim import RedRobotSim
from joy import JoyApp, progress
from joy.decl import *
from waypointShared import WAYPOINT_HOST, APRIL_DATA_PORT
from socket import (
  socket, AF_INET,SOCK_DGRAM, IPPROTO_UDP, error as SocketError,
  )
from math498 import *
from time import clock
from core import *
import pdb

class RobotSimulatorApp( JoyApp ):
  """Concrete class RobotSimulatorApp <<singleton>>
     A JoyApp which runs the RedRobotSim robot model in simulation, and
     emits regular simulated tagStreamer message to the desired waypoint host.
     
     Used in conjection with waypointServer.py to provide a complete simulation
     environment for Project 1
  """    

  def __init__(self,wphAddr=WAYPOINT_HOST,*arg,**kw):
    JoyApp.__init__( self,
      confPath="$/cfg/JoyApp.yml",
      ) 
    self.srvAddr = (wphAddr, APRIL_DATA_PORT)
    self.auto = False;
    
  def onStart( self ):
    # Set up socket for emitting fake tag messages
    s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)
    s.bind(("",0))
    self.sock = s
    # Set up the sensor receiver plan
    self.sensor = SensorPlanTCP(self,server=self.srvAddr[0])
    self.sensor.start()
    self.timeForStatus = self.onceEvery(1)
    self.timeForLaser = self.onceEvery(1/15.0)
    self.timeForFrame = self.onceEvery(1/20.0)
    self.timeForFilter = self.onceEvery(1.0/10.0)
    progress("Using %s:%d as the waypoint host" % self.srvAddr)
    self.T0 = self.now

    # Setup autonomous mode timer
    self.timeForAuto = self.onceEvery(1)
    self.auto = False


    self.core = Core(Mode.SIMULATION, self)
    self.robSim = RedRobotSim(self.core, fn=None)

    self.core.setSim(self.robSim)
    self.core.start()

    self.startedFilter = False

    # self.logFile = open("log.txt", "w")

  def showSensors( self ):
    ts,f,b = self.sensor.lastSensor
    if ts:
      progress( "Sensor: %4d f %d b %d" % (ts-self.T0,f,b)  )
    else:
      progress( "Sensor: << no reading >>" )
    ts,w = self.sensor.lastWaypoints
    if ts:
      progress( "Waypoints: %4d " % (ts-self.T0) + str(w))
    else:
      progress( "Waypoints: << no reading >>" )
  
  def emitTagMessage( self ):
    """Generate and emit and update simulated tagStreamer message"""
    self.robSim.refreshState()
    # Get the simulated tag message
    msg = self.robSim.getTagMsg()
    # Send message to waypointServer "as if" we were tagStreamer
    self.sock.sendto(msg, self.srvAddr)
    
  def onEvent( self, evt ):
    # periodically, show the sensor reading we got from the waypointServer
    if self.timeForStatus(): 
      self.showSensors()
      print("CurrPos(Camera): " + str(self.core.coordinateFrames.convertRealToCamera(self.robSim.pos)))
      print("CurrPos(Real): " + str(self.robSim.pos))
      progress( self.robSim.logLaserValue(self.now) )
      try:
        print("CurrPos(waypts): " + str(self.core.coordinateFrames.convertRealToWaypoint(self.robSim.pos)) + "\t" + str(self.robSim.yaw - self.core.coordinateFrames.getRealToWaypointYaw()))
        print("Estimated Pos(Real): " + str(self.core.particleFilter.getState().pos) + "\t" + str(self.core.particleFilter.getState().yaw))
      except:
        pass


      # logging
      # if (self.core.coordinateFrames.waypoint != None):
      #   rotatedLength = CoordinateFrames.rotateCCW([Constants.tagLength, 0], self.robSim.yaw - self.core.coordinateFrames.getRealToWaypointYaw())
      #   posWaypoint = self.core.coordinateFrames.convertRealToWaypoint(self.robSim.pos)
      #   frontSensor = posWaypoint[1] + rotatedLength[1]
      #   backSensor = posWaypoint[1] - rotatedLength[1]
      #   if (self.sensor.lastSensor[1] > 5 and self.sensor.lastSensor[2] > 5):
      #     self.logFile.write(str([self.sensor.lastSensor[1], self.sensor.lastSensor[2]]))
      #     self.logFile.write(str([frontSensor, backSensor]) + "\n")
      #     print(rotatedLength)
      #     print(str([self.sensor.lastSensor[1], self.sensor.lastSensor[2]]) + str([frontSensor, backSensor]))


      # generate simulated laser readings
    elif self.timeForLaser():
      self.robSim.logLaserValue(self.now)
    # update the robot and simulate the tagStreamer
    if self.timeForFrame(): 
      self.emitTagMessage()

    if self.timeForFilter():
      self.core.pushSensorAndWaypoints(array([self.sensor.lastSensor[1], self.sensor.lastSensor[2]]), 
        self.sensor.lastWaypoints[1])

    if self.timeForAuto() and self.auto:
      self.core.autonomousPlanner.plan(self.sensor.lastWaypoints[1])

    if evt.type == KEYDOWN:
      if evt.key == K_UP:
        self.core.movePosY()
        return progress("(say) Up")
      elif evt.key == K_DOWN:
        self.core.moveNegY()
        return progress("(say) Down")
      elif evt.key == K_LEFT:
        self.core.moveNegX()
        return progress("(say) Left")
      elif evt.key == K_RIGHT:
        self.core.movePosX()
        return progress("(say) Right")
      elif evt.key == K_RETURN:
        self.startedFilter = not self.startedFilter
        if (self.startedFilter):
          self.core.startFilter()
          return progress("(say) Started Filter")
        else:
          self.core.stopFilter()
          return progress("(say) Stopped Filter")
      elif evt.key == K_TAB:
        self.auto = ~self.auto
        if self.auto:
          return progress("(say) Autonomous On")
        else:
          return progress("(say) Autonomous Off")
      else:
        return JoyApp.onEvent(self,evt)
    return # ignoring non-KEYDOWN events


if __name__=="__main__":
  print """
  Running the robot simulator

  Listens on local port 0xBAA (2986) for incoming waypointServer
  information, and also transmits simulated tagStreamer messages to
  the waypointServer. 
  """
  import sys
  if len(sys.argv)>1:
      app=RobotSimulatorApp(wphAddr=sys.argv[1])
  else:
      app=RobotSimulatorApp(wphAddr=WAYPOINT_HOST)
  app.run()



