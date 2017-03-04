import numpy as np


def gs_coefficient(v1, v2):
    return np.dot(v2, v1) / np.dot(v1, v1)


def multiply(coefficient, v):
    return v*coefficient


def proj(v1, v2):
    return multiply(gs_coefficient(v1, v2), v1)


def gs(X):
    Y = []
    for i in range(len(X)):
        temp_vec = X[i]
        for inY in Y:
            proj_vec = proj(inY, X[i])
            temp_vec = temp_vec - proj_vec
        Y.append(temp_vec)
    return Y

