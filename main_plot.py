import gram_schmidt
import numpy as np
import matrix_utils as mx_ut
import matplotlib.pyplot as plt
import util_plot as plt_ut

# PARAMETERS
# Position of the camera relative to the centre
dx_cam = -24  # cm
dy_cam = 0
dz_cam = 24  # cm


def plot_tilted_generated_frame_of_satellite(ax):
    origin = 0 + 10
    ax.quiver(origin, origin, origin, n[0], n[1], n[2], color='r')
    ax.quiver(origin, origin, origin, u[0], u[1], u[2], color='g')
    ax.quiver(origin, origin, origin, v[0], v[1], v[2], color='b')

# Axis is in the center of the ball bearing
def plot_inertial_frame(ax):
    x_arw = plt_ut.Arrow3D([0,x_inertial[0]],[0,x_inertial[1]],[0,x_inertial[2]], arrowstyle="->", color="r", lw = 2, mutation_scale=10)
    y_arw = plt_ut.Arrow3D([0,y_inertial[0]],[0,y_inertial[1]],[0,y_inertial[2]], arrowstyle="->", color="g", lw = 2, mutation_scale=10)
    z_arw = plt_ut.Arrow3D([0,z_inertial[0]],[0,z_inertial[1]],[0,z_inertial[2]], arrowstyle="->", color="b", lw = 2, mutation_scale=10)

    plt_ut.add_to_plot(ax, x_arw, y_arw, z_arw)

def rot_mx():
    # Convention of Rotation: 1) z-axis 2) x-axis 3) y axis
    rot_z_angle = 180  #degrees
    rot_matrix_z = mx_ut.rotation_z(rot_z_angle)
    rot_y_angle = 150  #degrees
    rot_matrix_y = mx_ut.rotation_y(rot_y_angle)

    rot_matrix = mx_ut.matrix_matrix_multipl(rot_matrix_y, rot_matrix_z)
    return rot_matrix


def transform_matrix_inertial_camera():
    transform_matrix = mx_ut.matrix_matrix_multipl(mx_ut.generate_translation_mx(dx_cam, dy_cam, dz_cam), rot_mx())
    return transform_matrix


def generate_and_plot_camera_frame(ax):
    x_camera = mx_ut.matrix_vector_multipl(transform_matrix_inertial_camera(), x_inertial)
    y_camera = mx_ut.matrix_vector_multipl(transform_matrix_inertial_camera(), y_inertial)
    z_camera = mx_ut.matrix_vector_multipl(transform_matrix_inertial_camera(), z_inertial)

    origin_camera = mx_ut.matrix_vector_multipl(mx_ut.generate_translation_mx(dx_cam, dy_cam, dz_cam), origin_inertial)

    x_arw_camera = plt_ut.Arrow3D([origin_camera[0],x_camera[0]], 
                                    [origin_camera[1],x_camera[1]], 
                                    [origin_camera[2],x_camera[2]], 
                                    arrowstyle="->", color="r", lw = 2, mutation_scale=10)

    y_arw_camera = plt_ut.Arrow3D([origin_camera[0],y_camera[0]], [origin_camera[1],y_camera[1]], [origin_camera[2],y_camera[2]], arrowstyle="->", color="g", lw = 2, mutation_scale=10)
    z_arw_camera = plt_ut.Arrow3D([origin_camera[0],z_camera[0]], [origin_camera[1],z_camera[1]], [origin_camera[2],z_camera[2]], arrowstyle="->", color="b", lw = 2, mutation_scale=10)

    plt_ut.add_to_plot(ax, x_arw_camera, y_arw_camera, z_arw_camera)


ax = plt_ut.generate_plot()

# Generate the inertial frame
unit = 6
x_inertial = np.array([unit, 0, 0, 1])
y_inertial = np.array([0, unit, 0, 1])
z_inertial = np.array([0, 0, unit, 1])
origin_inertial = np.array([0, 0, 0, 1])


# INPUT data from the C++ algorithm
n = [0.47*6, -0.04*6, 0.87*6]
u = [3, 0, 0]  # arbitrary chosen

# Generate the other two vectors perpendicular to n using Gram-Schmidt Algorithm
combined_vectors = np.array([n, u])
u = (gram_schmidt.gs(combined_vectors)[1])
v = mx_ut.cross_product(n, u)
plot_tilted_generated_frame_of_satellite(ax)


plot_inertial_frame(ax)
generate_and_plot_camera_frame(ax)


plt.show()

