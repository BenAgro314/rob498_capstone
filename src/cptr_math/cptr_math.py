# Math utilities library for captor
import numpy as np
from numpy.linalg import matrix_power, inv, norm
from scipy.spatial.transform import Rotation as R


def interpolate_transform(T_start, T_end, n_steps=1, interp_dist=None):
    """ interpolates SE3 poses between transforms T_start and T_end

    Args:
        T_start (4x4 np.array): initial pose
        T_end (4x4 np.array): final pose
        n_steps (int): granularization of interpolation
        interp_dist (float): interpolate poses by intermediate distance [m] rather than steps

    Returns:
        queue: list of interpolated poses - including start and end
    """

    iter = n_steps
    if interp_dist:
        distance = norm(T_end[0:3,:3]-T_start[0:3,:3], ord=2)
        iter = distance//interp_dist

    queue = [T_start]
    for a in range(1,iter):
        T_interp = matrix_power(T_end @ inv(T_start), a/iter) @ T_start
        queue.append(T_interp)
    queue.append(T_end)
    return queue


def get_position(transform):
    """_summary_

    Args:
        transform (4x4 np.array): _description_

    Returns:
        (3x1 np.array): _description_
    """
    return transform[0:3,:3]

def get_rotation(transform, as_vector=True):
    """get rotation/orientation from transform/pose in se3. toggle
       return type as vector or as matrix.

    Args:
        transform (4x4 np.array): 4x4 np.array in SE3 representing transform
        as_vector (bool, optional): Set return type.Defaults to True.
 
    Returns:
        _type_: rotation as rotvec (3x1) or as rotation matrix (3x3)
    """
    r = R.from_matrix(transform[0:3,0:3])
    if as_vector:
        return r.as_rotvec()
    return r.as_matrix()

def gen_transform(pose_vec):
    """generate transfomr from pose vector

    Args:
        pose (6x1 np.array): pose vector [x,y,z,phi,theta,psi]

    Returns:
        transform: 4x4 nparray representing pose in SE3
    """
    transform = np.identity(4)
    r = R.from_rotvec(pose_vec[3;6])
    transform[0:3,0:3] = r.as_matrix()
    transform[0:3,3]   = pose_vec[0:3]
    return transform
