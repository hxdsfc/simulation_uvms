"""Quaternion helpers; quaternion order is always w, x, y, z."""
import numpy as np
import mujoco
from .kinematics import finite_vector


def quaternion(value):
    q = finite_vector(value,4,"quaternion wxyz")
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        raise ValueError("Quaternion must be nonzero")
    return q/norm


def matrix_quaternion(matrix):
    result = np.empty(4)
    mujoco.mju_mat2Quat(result,np.asarray(matrix,dtype=float).ravel())
    return result


def rotation_error(desired, actual):
    # Desired * conjugate(actual): rotation error expressed in the reference frame.
    a,b = quaternion(desired),quaternion(actual)
    b = b*np.array([1,-1,-1,-1])
    q = np.r_[a[0]*b[0]-a[1:]@b[1:], a[0]*b[1:]+b[0]*a[1:]+np.cross(a[1:],b[1:])]
    if q[0] < 0:
        q = -q
    length = np.linalg.norm(q[1:])
    return 2*q[1:] if length < 1e-10 else q[1:]*(2*np.arctan2(length,q[0])/length)


def slerp(start,end,fraction):
    start,end = quaternion(start),quaternion(end)
    dot = float(start@end)
    if dot < 0:
        end,dot = -end,-dot
    dot = np.clip(dot,-1,1)
    if dot > 0.9995:
        return quaternion(start+(end-start)*fraction)
    angle = np.arccos(dot)
    return (np.sin((1-fraction)*angle)*start+np.sin(fraction*angle)*end)/np.sin(angle)
