import numpy as np


def line_sphere_intersection(p1, p2, c, r):
    """
    Implementation of the line-sphere intersection algorithm described in wiki
    https://en.wikipedia.org/wiki/Line-sphere_intersection
    :param p1: start of line segment
    :param p2: end of line segment
    :param c: sphere center
    :param r: sphere radius
    :returns: discriminant of the line-sphere intersection formula
    """
    d = np.sqrt(np.sum(np.square(np.subtract(p1, p2))))
    u = np.subtract(p2, p1) / d
    o = p1
    return np.square(u @ (o - c)) - (np.sum(np.square(o - c)) - pow(r, 2))