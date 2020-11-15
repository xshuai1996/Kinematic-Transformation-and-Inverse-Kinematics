import numpy as np


def fk(angles, link_lengths):
    """
    Computes the forward kinematics of a planar, n-joint robot arm.
    :param angles: list of angle values for each joint, in radians.
    :param link_lengths: list of lengths for each link in the robot arm.
    :returns: The end effector position with respect to the base
        frame (the frame at the first joint) as a numpy array with dtype
        np.float64
    """
    T_chain = np.identity(4, dtype=np.float64)
    for i in range(len(angles)):
        T = np.array([[np.cos(angles[i]), -np.sin(angles[i]), 0, link_lengths[i] * np.cos(angles[i])],
                      [np.sin(angles[i]), np.cos(angles[i]),  0, link_lengths[i] * np.sin(angles[i])],
                      [0,                 0,                  1, 0],
                      [0,                 0,                  0, 1]], dtype=np.float64)
        T_chain = T_chain @ T
    return [T_chain[0, 3], T_chain[1, 3], T_chain[2, 3]]
