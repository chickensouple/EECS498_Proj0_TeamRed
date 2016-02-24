from numpy import *
from numpy.linalg import lstsq, svd, inv


def skew( v ):
	"""
	Convert a 3-vector to a skew matrix such that 
	dot(skew(x),y) = cross(x,y)

	The function is vectorized, such that:
	INPUT:
	v -- N... x 3 -- input vectors
	OUTPUT:
	N... x 3 x 3  

	For example:
	>>> skew([[1,2,3],[0,0,1]])
	array([[[ 0,  3, -2],
	    [-3,  0,  1],
	    [ 2, -1,  0]],
	<BLANKLINE>
	   [[ 0,  1,  0],
	    [-1,  0,  0],
	    [ 0,  0,  0]]])
	"""
	v = asarray(v).T
	z = zeros_like(v[0,...])
	return array([
		[ z, -v[2,...], v[1,...]],
		[v[2,...], z, -v[0,...] ],
		[-v[1,...], v[0,...], z ] ]).T


def fitHomography(x, y ):
	"""Fit a homography mapping points x to points y"""
	x = asarray(x)
	assert x.shape == (len(x),3)
	y = asarray(y)
	assert y.shape == (len(y),3)
	S = skew(y)
	plan = [ kron(s,xi) for s,xi in zip(S,x) ]
	#plan.append([[0]*8+[1]])
	A = concatenate( plan, axis=0 )
	U,s,V = svd(A)
	res = V[-1,:].reshape(3,3)
	return res.T

class CoordinateFrames:
	def __init__(self):
		pass
		self.lineDist = 0
		self.lineDev = 0

	def calculateTransformation(self, sensorPts, realPts):
		size = len(sensorPts)

		y = zeros(size * 2)
		H = zeros([size * 2, 6])
		count = 0
		# for key, sensorPt in sensorPts.iteritems():
		for (sensorPt, realPt) in zip(sensorPts, realPts):
			# realPt = realPts[key]

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


	def convertArbitraryToReal(self, arbitraryPt):
		pt = array([arbitraryPt[0], arbitraryPt[1], 1])
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

