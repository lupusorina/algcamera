import numpy as np
import math


def verify_orthogonality(a, b):
    ortho = 0
    for i in range(len(a)):
        ortho = ortho + a[i] * b[i]
    err = 0.000001
    if ortho < err:
        print("The two vectors are orthogonal")
    else:
        print("the two vectors are NOT orthogonal")


def cross_product(a, b):
    v = np.array([0.0, 0.0, 0.0])
    v[0] = a[1]*b[2] - a[2]*b[1]
    v[1] = a[0]*b[2] - a[2]*b[0]
    v[2] = a[0]*b[1] - a[1]*b[0]
    return v


def matrix_vector_multipl(a, b):
    result = np.array([0.0, 0.0, 0.0, 0.0])
    for i in range(0, len(a)):
        sum = 0.0
        for j in range(0, len(a)):
            sum = sum + a[i][j] * b[j]
        result[i] = sum
    return result


def matrix_matrix_multipl(a, b):
    result = np.array([[0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]])
    for i in range(0, len(a)):
        for j in range(0, len(a[0])):
           for k in range(0, len(b)):
            result[i][j] += a[i][k] * b[k][j]

    return result

def generate_translation_mx(dx, dy, dz):
    translation_matrix = np.array([[1, 0, 0, dx], [0, 1, 0, dy], [0, 0, 1, dz], [0, 0, 0, 1]])
    return translation_matrix


def rotation_x(theta):
    theta_rad = math.radians(theta)
    c = round(math.cos(theta_rad), 2)
    s = round(math.sin(theta_rad), 2)

    rot_x_mx = np.array([[1, 0, 0, 0],
                         [0, c, -s, 0],
                         [0, s,  c, 0],
                         [0, 0,  0, 1]])
    return rot_x_mx


def rotation_y(theta):
    theta_rad = math.radians(theta)
    c = round(math.cos(theta_rad), 2)
    s = round(math.sin(theta_rad), 2)

    rot_y_mx = np.array([[c, 0, s, 0],
                         [0, 1, 0, 0],
                         [-s, 0, c, 0],
                         [0, 0, 0, 1]])
    return rot_y_mx


def rotation_z(theta):
    theta_rad = math.radians(theta)
    c = round(math.cos(theta_rad), 2)
    s = round(math.sin(theta_rad), 2)

    rot_z_mx = np.array([[c, -s, 0, 0],
                         [s, c, 0, 0],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]])
    return rot_z_mx

