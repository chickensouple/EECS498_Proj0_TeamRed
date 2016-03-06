from joy import *
from joy.decl import *
import time
from servoWrapper import ServoWrapperMX
from motionPlans import *
from particleFilter import *
from autonomous import *
from sensorPlanTCP import SensorPlanTCP
from waypointShared import WAYPOINT_HOST, APRIL_DATA_PORT
from socket import (
  socket, AF_INET,SOCK_DGRAM, IPPROTO_UDP, error as SocketError,
  )

numMotors = 1;
cameraPts = [[ 115.632,  -84.96,     1.   ],
	[ -10.064,  -84.96,     1.   ],
	[-104.528,  -85.216,    1.   ],
	[-104.912,    2.336,    1.   ],
	[-113.104,   84.,       1.   ],
	[ -14.288,   84.,       1.   ],
	[ 115.632,   84.,       1.   ],
	[ 115.632,    0.8,      1.   ]]

realPts = [[582, 421,   1],
	[572, 289,   1],
	[561, 178,   1],
	[439, 159,   1],
	[301, 137,   1],
	[272, 239,   1],
	[233, 376,   1],
	[411, 401,   1]]

class MainApp(JoyApp):
	""" Main app to run the program that controls our robot """
	def __init__(self, *arg, **kw):
		cfg = dict()
		JoyApp.__init__(self, cfg=cfg, *arg, **kw)
		self.srvAddr = (WAYPOINT_HOST, APRIL_DATA_PORT)

	def onStart(self):
		self.coordinateFrames = CoordinateFrames()
		self.coordinateFrames.calculateTransformation(cameraPts, realPts)

		self.autonomousPlanner = AutonomousPlanner(None, self, self.coordinateFrames)

		self.motorX = ServoWrapperMX(self, self.robot.at.Nx28)
		# self.motorX = ServoWrapperMX(self, self.robot.at.H11)
		self.motorX.start()
		self.movePosXPlan = MovePosXPlan(self)
		self.moveNegXPlan = MoveNegXPlan(self)
		self.particleFilter = ParticleFilter()

		self.startedFilter = False
		self.auto = False

		#sensor stuff
		self.sensor = SensorPlanTCP(self, server=self.srvAddr[0])
		self.sensor.start()

		# Setup autonomous mode timer
		self.timeForAuto = self.onceEvery(1/20.0)


	def onEvent(self, evt):
		if self.auto and self.timeForAuto():
			self.autonomousPlanner.plan(self.sensor.lastWaypoints[1])

		if evt.type == KEYDOWN:
			if evt.key == K_UP:
				pass
				# self.movePosYPlan.start()
			elif evt.key == K_DOWN:
				pass
				# self.moveNegYPlan.start()
			elif evt.key == K_RIGHT:
				self.movePosXPlan.start()
			elif evt.key == K_LEFT:
				self.moveNegXPlan.start()
			elif evt.key == K_RETURN:
				self.startedFilter = not self.startedFilter
				if (self.startedFilter):
					self.particleFilter.setState(self.sensor.lastWaypoints[1][0], 0)
			elif evt.key == K_TAB:
				self.auto = not self.auto
				if self.auto:
					return progress("(say) Autonomous On")
				else:
					return progress("(say) Autonomous Off")


robot = {"count": numMotors}
scr = {}

app = MainApp(robot=robot, scr=scr)
app.run()
