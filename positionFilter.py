from directions import *
from state import *
from constants import *
from numpy.random import randn, uniform
from numpy import *
from numpy.linalg import *
from coordinateFrames import *
from joy import *
import pdb


class PositionFilterState:
    IDLE = 0
    STARTED_MOVING = 1
    MOVING = 2

class PositionFilter(Plan):
  def __init__(self, app, *arg, **kw):
    Plan.__init__(self, app, *arg, **kw)
    self.state = PositionFilterState.IDLE
    self.moveReady = False
    self.sensorReady = False
    self.sensor = []
    self.waypoints = []
    self.direction = None
    self.coordinateFrames = CoordinateFrames()
    self.tagLength = tagLength
    self.wheelSideLength = wheelSideLength

    # self.states = []
    # self.validStates = []
    self.state = RobotState()

  def setState(self, pos, yaw):
    # self.states = [RobotState(pos, yaw - self.coordinateFrames.getRealToWaypointYaw())]
    # self.validStates = [RobotState(pos, yaw - self.coordinateFrames.getRealToWaypointYaw())]
    self.state = RobotState(pos, yaw - self.coordinateFrames.getRealToWaypointYaw())

  def setControlInput(self, direction):
    self.moveReady = True
    self.direction = direction

  def setSensorAndWaypoints(self, sensor, waypoints):
    #TODO: add low pass filter to sensor
    self.sensor = sensor
    if (len(waypoints) != len(self.waypoints)):
      self.coordinateFrames.calculateRealToWaypointTransformation(waypoints[0], waypoints[1])
      self.setState([0, 0], -self.coordinateFrames.getRealToWaypointYaw())
    self.waypoints = waypoints
    self.sensorReady = True

  def getState(self):
    return RobotState(self.coordinateFrames.convertWaypointToReal(self.state.pos), 
      self.state.yaw + self.coordinateFrames.getRealToWaypointYaw())
    # return RobotState(self.state.pos, 
    #   self.state.yaw + self.coordinateFrames.getRealToWaypointYaw())

  def behavior(self):
    self.state = PositionFilterState.IDLE
    self.moveReady = False
    self.sensorReady = False
    while True:
        if (self.moveReady):
          self.moveReady = False
          self.state = self.actionModel(self.state, self.direction)

        # if (self.state == PositionFilterState.STARTED_MOVING):
        #     self.state = PositionFilterState.MOVING
        # elif (self.state == PositionFilterState.MOVING):
        #     if (not self.app.autonomousPlanner.switch.inMotion()):
        #         self.state = PositionFilterState.IDLE
        #         self.moveReady = True

        # if (self.moveReady and self.sensorReady):
        #     self.moveReady = False
        #     self.sensorReady = False

        #     # applying action model
        #     newStates = []
        #     for state in self.states:
        #         newStates.append(self.actionModel(state, self.direction))

        #     realSensor = PositionFilter.convertSensor(self.sensor)
        #     solutions = self.generateSolutions(realSensor)

        #     pdb.set_trace()

        #     self.validStates = []
        #     self.states = []

        #     r = sum(state.pos[0] for state in newStates) / len(newStates)
        #     for solution in solutions:
        #         newState = RobotState([r, solution[0]], solution[1])
        #         self.states.append(newState)
        #         for state in newStates:
        #             # comparing d
        #             posDiff = abs(solution[0] - state.pos[1])
        #             # comparing yaw
        #             yawDiff = abs(solution[1] - state.yaw)

        #             if (yawDiff < 0.2 and posDiff < 10):
        #                 self.validStates.append(newState)
        #     if (len(self.validStates) == 0):
        #       self.validStates.append(RobotState([r, solutions[0][0]], solutions[0][1]))

        yield self.forDuration(0.05)

  def generateSolutions(self, sensor):
    solutions = []

    f = sensor[0]
    b = sensor[1]

    if (b == -1):
      solutions.append([f, -self.coordinateFrames.getRealToWaypointYaw()])
      return solutions

    d1 = (f+b)/2
    theta1 = atan2((f-b), self.tagLength)

    d2 = (f-b)/2
    theta2 = atan2((f+b), self.tagLength)

    solutions.append([d1, theta1])
    solutions.append([-d1, -theta1])

    if (not floatClose(f, 0) and not floatClose(b, 0)):
      solutions.append([d2, theta2])
      solutions.append([-d2, -theta2])

    return solutions



  def actionModel(self, state, direction):
    dirVec = [0., 0.]
    if direction == (Directions.PosX):
      dirVec[0] = 1
    elif direction == (Directions.NegX):
      dirVec[0] = -1
    elif direction == (Directions.PosY):
      dirVec[1] = 1
    elif direction == (Directions.NegY):
      dirVec[1] = -1

    dirVec = array(dirVec) * self.wheelSideLength


    # yaw = state.yaw + self.coordinateFrames.getRealToWaypointYaw()
    yaw = self.coordinateFrames.getRealToWaypointYaw()

    rotatedDirVec = CoordinateFrames.rotateCCW(dirVec, -yaw)
    ret = RobotState(state.pos + rotatedDirVec, state.yaw)
    return ret


  @staticmethod
  def convertSensor(sensor):
    # converts sensor values into real distances
    # model for sensor is
    # y: real dist
    # x: sensor dist
    # y = 0.0007762568 * x^2 - 0.4104391031*x + 55.0759770855
    ret =  0.0007762568 * sensor * sensor - \
      0.4104391031 * sensor + 55.0759770855

    if (sensor[0] < 5):
      ret[0] = -1
    if (sensor[1] < 5):
      ret[1] = -1
    return ret



