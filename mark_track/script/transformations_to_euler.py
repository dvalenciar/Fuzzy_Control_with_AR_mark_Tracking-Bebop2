
import numpy 
import math



# epsilon for testing whether a number is close to zero
_EPS = numpy.finfo(float).eps * 4.0

# axis sequences for Euler angles
_NEXT_AXIS = [1, 2, 0, 1]

# map axes strings to/from tuples of inner axis, parity, repetition, frame
_AXES2TUPLE = {
    'sxyz': (0, 0, 0, 0), 'sxyx': (0, 0, 1, 0), 'sxzy': (0, 1, 0, 0),
    'sxzx': (0, 1, 1, 0), 'syzx': (1, 0, 0, 0), 'syzy': (1, 0, 1, 0),
    'syxz': (1, 1, 0, 0), 'syxy': (1, 1, 1, 0), 'szxy': (2, 0, 0, 0),
    'szxz': (2, 0, 1, 0), 'szyx': (2, 1, 0, 0), 'szyz': (2, 1, 1, 0),
    'rzyx': (0, 0, 0, 1), 'rxyx': (0, 0, 1, 1), 'ryzx': (0, 1, 0, 1),
    'rxzx': (0, 1, 1, 1), 'rxzy': (1, 0, 0, 1), 'ryzy': (1, 0, 1, 1),
    'rzxy': (1, 1, 0, 1), 'ryxy': (1, 1, 1, 1), 'ryxz': (2, 0, 0, 1),
    'rzxz': (2, 0, 1, 1), 'rxyz': (2, 1, 0, 1), 'rzyz': (2, 1, 1, 1)}

_TUPLE2AXES = dict((v, k) for k, v in _AXES2TUPLE.items())


def quaternion_to_array(q):

	#q_array = numpy.array([q.w, q.x, q.y, q.z])
	q_array = numpy.array([q.w, q.x, q.y, q.z])
	return q_array

def quaternion_matrix(quaternion):


	q = numpy.array(quaternion, dtype=numpy.float64, copy=True)
	n = numpy.dot(q, q)
	if n < _EPS:
		return numpy.identity(4)
	q *= math.sqrt(2.0 / n)
	q = numpy.outer(q, q)
	return numpy.array([
		[1.0-q[2, 2]-q[3, 3],     q[1, 2]-q[3, 0],     q[1, 3]+q[2, 0], 0.0],
		[    q[1, 2]+q[3, 0], 1.0-q[1, 1]-q[3, 3],     q[2, 3]-q[1, 0], 0.0],
		[    q[1, 3]-q[2, 0],     q[2, 3]+q[1, 0], 1.0-q[1, 1]-q[2, 2], 0.0],
		[                0.0,                 0.0,                 0.0, 1.0]])

def euler_from_matrix(matrix, axes='sxyz'):

	try:
		firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
	except (AttributeError, KeyError):
		_TUPLE2AXES[axes]  # validation
		firstaxis, parity, repetition, frame = axes

	i = firstaxis
	j = _NEXT_AXIS[i+parity]
	k = _NEXT_AXIS[i-parity+1]

	M = numpy.array(matrix, dtype=numpy.float64, copy=False)[:3, :3]
	if repetition:
		sy = math.sqrt(M[i, j]*M[i, j] + M[i, k]*M[i, k])
		if sy > _EPS:
			ax = math.atan2( M[i, j],  M[i, k])
			ay = math.atan2( sy,       M[i, i])
			az = math.atan2( M[j, i], -M[k, i])
		else:
			ax = math.atan2(-M[j, k],  M[j, j])
			ay = math.atan2( sy,       M[i, i])
			az = 0.0
	else:
		cy = math.sqrt(M[i, i]*M[i, i] + M[j, i]*M[j, i])
		if cy > _EPS:
			ax = math.atan2( M[k, j],  M[k, k])
			ay = math.atan2(-M[k, i],  cy)
			az = math.atan2( M[j, i],  M[i, i])
		else:
			ax = math.atan2(-M[j, k],  M[j, j])
			ay = math.atan2(-M[k, i],  cy)
			az = 0.0

	if parity:
		ax, ay, az = -ax, -ay, -az
	if frame:
		ax, az = az, ax
	return ax, ay, az

def get_yaw_from_quaternion(quaternion):

	q_array = quaternion_to_array(quaternion)
	rot_mat = quaternion_matrix(q_array)
	euler = euler_from_matrix(rot_mat, 'rxyz')

	#yaw_deg = euler[2] / numpy.pi * 180.0
	#return yaw_deg


	return euler


