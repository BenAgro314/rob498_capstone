# Math utilities library for captor
import numpy as np
from numpy.linalg import matrix_power, inv, norm


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


