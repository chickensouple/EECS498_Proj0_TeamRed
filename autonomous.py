from numpy import *

class AutonomousPlanner:
	def __init__(self):
		pass
		self.lineDist = 0
		self.lineDev = 0

	def calculateTransformation(self, sensorPts, realPts):
		size = len(sensorPts)

		y = zeros(size * 2)
		H = zeros([size * 2, 6])
		count = 0
		for key, sensorPt in sensorPts.iteritems():
			realPt = realPts[key]
			y[2 * count] = realPt[0]
			y[(2 * count) + 1] = realPt[1]
			
			H[2*count,0] = sensorPt[0]
			H[2*count,1] = sensorPt[1]
			H[2*count,2] = 1
			H[(2*count)+1,3] = sensorPt[0]
			H[(2*count)+1,4] = sensorPt[1]
			H[(2*count)+1,5] = 1
			count += 1

		coeffs = linalg.lstsq(H, transpose(y))[0]
		self.T = array([[coeffs[0], coeffs[1], coeffs[2]],
			[coeffs[3], coeffs[4], coeffs[5]],
			[0, 0, 1]])
		self.T_inverse = linalg.inv(self.T)
		self.R = array([[coeffs[0], coeffs[1]],
			[coeffs[3], coeffs[4]]])

		scale = average(sum(self.R**2, axis=0))
		self.R_inverse = (1. / (scale)) * transpose(self.R)
		print(dot(self.R, self.R_inverse))

	def convertArbitraryToReal(self, sensorPt):
		pt = array([sensorPt[0], sensorPt[1], 1])
		rotated = dot(self.T, pt)
		return array([rotated[0], rotated[1]])

	def convertRealToArbitrary(self, realPt):
		pt = array([realPt[0], realPt[1], 1])
		rotated = dot(self.T_inverse, pt)
		return array([rotated[0], rotated[1]])

	def rotateArbitraryToReal(self, sensorPt):
		return dot(self.R, sensorPt)

	def rotateRealToArbitrary(self, realPt):
		return dot(self.R_inverse, realPt)


	def getNextStep( self, robSim, sensorData ):
