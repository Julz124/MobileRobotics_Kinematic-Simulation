import numpy as np
import math as m


import numpy as np

def rot(theta):
    return np.array(
        [[np.cos(theta),-np.sin(theta)],
        [np.sin(theta), np.cos(theta)]]
    )

def rotx(theta):
    return np.array(
        [[1, 0, 0],
        [0, np.cos(theta), -np.sin(theta)],
        [0, np.sin(theta), np.cos(theta)]]
    )

def roty(theta):
    return np.array(
        [[np.cos(theta), 0, np.sin(theta)],
        [0, 1, 0],
        [-np.sin(theta), 0, np.cos(theta)]]
    )

def rotz(theta):
    return np.array(
        [[np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta), np.cos(theta), 0],
        [0, 0, 1]]
    )

def rot2trans(r):
    dim = r.shape[0]
    trans_matrix = np.eye(dim + 1)
    trans_matrix[:dim, :dim] = r
    return trans_matrix

def trans(t):
    dim = len(t)
    trans_matrix = np.eye(dim + 1)
    trans_matrix[:dim, -1] = t
    return trans_matrix