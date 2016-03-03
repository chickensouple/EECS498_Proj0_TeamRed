from numpy import *
from numpy.linalg import lstsq, svd, inv, norm

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


def lineToPtDist(linePt1, linePt2, pt):
	"""
	calculates the distance between a line and a point
	where the line is defined by two points along it (linePt1, linePt2)
	"""
	#            | (linePt2 - linePt1) X (linePt1 - pt) |
	# distance = ----------------------------------------
	#                     | (linePt2 - linePt1) |

	linePt1 = array(linePt1)
	linePt2 = array(linePt2)
	pt = array(pt)
	num = linalg.norm(cross(linePt2 - linePt1, linePt1 - pt))
	den = linalg.norm(linePt2 - linePt1)
	return num / den

