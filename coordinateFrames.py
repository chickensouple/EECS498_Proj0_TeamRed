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

def applyHomography(H, x):
	y = dot(x, H)
	y = y / y[2]
	return y

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
