from numpy import *
from numpy.linalg import lstsq, svd, inv
from math498 import *

class CoordinateFrames:
	def __init__(self):
		pass
		self.lineDist = 0
		self.lineDev = 0

	def calculateTransformation(self, cameraPts, realPts):
		# T turns cameraPts into realPts
		self.T = fitHomography(cameraPts, realPts)
		self.T_inverse = inv(self.T)

	def convertCameraToReal(self, arbitraryPt):
		pt = array([arbitraryPt[0], arbitraryPt[1], 1])
		newPt = applyHomography(self.T, pt)
		return array([newPt[0], newPt[1]])

	def convertRealToCamera(self, realPt):
		pt = array([realPt[0], realPt[1], 1])
		newPt = applyHomography(self.T_inverse, pt)
		return array([newPt[0], newPt[1]])

	@staticmethod
	def rotateCCW(pt, yaw):
		"""
		Rotates a point by yaw number of radians counter clockwise
		"""
		cosTheta = cos(yaw)
		sinTheta = sin(yaw)
		x = cosTheta * pt[0] - sinTheta * pt[1]
		y = sinTheta * pt[0] + cosTheta * pt[1]
		return array([x, y])
