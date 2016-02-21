from numpy import *

class AutonomousPlanner:
	def __init__(self):
		pass

	def calculateRotation(self, sensorPts, realPts):
		# constructing y
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







