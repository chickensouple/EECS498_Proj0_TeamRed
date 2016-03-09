from joy import *
from joy.decl import *
import time
from servoWrapper import ServoWrapperMX
from core import *
from sensorPlanTCP import SensorPlanTCP
from waypointShared import WAYPOINT_HOST, APRIL_DATA_PORT
from socket import (
  socket, AF_INET,SOCK_DGRAM, IPPROTO_UDP, error as SocketError,
  )

numMotors = 4

class MainApp(JoyApp):
  """ Main app to run the program that controls our robot """
  def __init__(self, wphAddr, *arg, **kw):
    cfg = dict()
    JoyApp.__init__(self, cfg=cfg, *arg, **kw)
    self.srvAddr = (wphAddr, APRIL_DATA_PORT)

  def onStart(self):

    self.motors = []
    self.motors.append(ServoWrapperMX(self, self.robot.at.Nx28))
    self.motors.append(ServoWrapperMX(self, self.robot.at.Nx0A))

    self.motors.append(ServoWrapperMX(self, self.robot.at.Nx04))
    self.motors.append(ServoWrapperMX(self, self.robot.at.Nx57))

    for motor in self.motors:
      motor.start()

    self.motorPlan = MotorPlan(self)


    self.core = Core(Mode.ACTUAL, self)
    self.core.start()

    self.timeForFilter = self.onceEvery(1.0/10.0)
    self.startedFilter = False
    self.auto = False

    #sensor stuff
    self.sensor = SensorPlanTCP(self, server=self.srvAddr[0])
    self.sensor.start()

    # Setup autonomous mode timer
    self.timeForAuto = self.onceEvery(1/20.0)

  def onEvent(self, evt):
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


robot = {"count": numMotors}
scr = {}

if __name__=="__main__":
  print """
  Running the robot simulator

  Listens on local port 0xBAA (2986) for incoming waypointServer
  information, and also transmits simulated tagStreamer messages to
  the waypointServer. 
  """
  import sys
  if len(sys.argv)>1:
      app=MainApp(robot=robot, scr=scr, wphAddr=sys.argv[1])
  else:
      app=MainApp(robot=robot, scr=scr, wphAddr=WAYPOINT_HOST)
  app.run()


