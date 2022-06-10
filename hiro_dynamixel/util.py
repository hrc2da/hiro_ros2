import sys
from io import StringIO
from rclpy.parameter import Parameter
import numpy as np

vertical = np.array([0, 1])
horizontal = np.array([1, 0])

black = 0, 0, 0
gray = 200, 200, 200
white = 255, 255, 255
blue = 180, 180, 255
red = 255, 100, 100
green = 100, 255, 100

def degrees(rad):
    return rad * 180/np.pi

def radians(deg):
    return deg * np.pi/180

def normalize(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::

            >>> angle_between((1, 0, 0), (0, 1, 0))
            1.5707963267948966
            >>> angle_between((1, 0, 0), (1, 0, 0))
            0.0
            >>> angle_between((1, 0, 0), (-1, 0, 0))
            3.141592653589793
    """
    v1_u = normalize(v1)
    v2_u = normalize(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

def sigangle(v1, v2):
    """Signed angle between two vectors (-ve if v1 is CCW from v2)"""
    v1_u = normalize(v1)
    v2_u = normalize(v2)
    ang = np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))
    # Right-angle to v2
    perp = [v2_u[1], -v2_u[0]]
    # Check handedness
    if np.dot(v1_u, perp) < 0:
        return -ang
    else:
        return ang

def rotate(v, angle):
    """Rotate a vector v; angle in radians"""
    mat = np.matrix([
        [np.cos(angle), -np.sin(angle)],
        [np.sin(angle),  np.cos(angle)]])
    return np.array(v * mat).squeeze()

def printVec(vec):
    """Pretty-print a floating point vector/np array"""
    print(prettyVec(vec))

def prettyVec(vec):
    """Pretty-format a floating point vector/np array"""
    out = StringIO.StringIO()
    out.write('[')
    for i in xrange(len(vec)):
        out.write('{0:.2f}'.format(vec[i]))
        if i < len(vec) - 1:
            out.write(', ')
    out.write(']')
    str = out.getvalue()
    out.close()
    return str

def setup_params(obj,params):
    obj.params = dict()
    for param in params:
        name = param.name.split('.')[-1]
        if param.type_== Parameter.Type.INTEGER:
            val = param.get_parameter_value().integer_value
            setattr(obj,name,val)
        elif param.type_ == Parameter.Type.DOUBLE:
            val = param.get_parameter_value().double_value
            setattr(obj,name,val)
        elif param.type_ == Parameter.Type.DOUBLE_ARRAY:
            val = param.get_parameter_value().double_array_value
            setattr(obj,name,val)
        elif param.type_ == Parameter.Type.INTEGER_ARRAY:
            val = param.get_parameter_value().integer_array_value
            setattr(obj,name,val)
        else:
            val = param.get_parameter_value().string_value
            setattr(obj,name,val)
        obj.params[name] = val

def elbow_servo_pose(angle_from_vertical_degrees):
    return 150 - angle_from_vertical_degrees # [0,90]-->[150,60]

def shoulder_servo_pose(angle_from_vertical_degrees):
    return 200 - angle_from_vertical_degrees # [0,90]-->[200,110]

def swing_servo_pose(angle_in_degrees):
    return 150-angle_in_degrees

def wrist_servo_pose(angle_in_degrees):
    return angle_in_degrees-180 # center around zero


def base_angle_encoder(deg):
    return int(1024*deg/300.0)

def shoulder_angle_encoder(deg):
    return int(1024*deg/300.0)

def elbow_angle_encoder(deg):
    return int(1024*deg/300.0)

def wrist_angle_encoder(deg):
    return int(1024*deg/300.0)