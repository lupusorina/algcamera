import gram_schmidt
import matplotlib
import numpy as np
import matrix_utils as mx_ut
import util_plot as plt_ut
from matplotlib.widgets import Slider
import math
import camera as cam
import matplotlib.pyplot as plt
# PARAMETERS OF THE ENVIRONMENT (EXTRINSIC)

# Position of the camera relative
# to the centre (inertial frame)
dx_cam = -24  # cm
dy_cam = 0 # cm
dz_cam = 40  # cm

angle_camera = 40 # degrees
scale = 10

# PART 1: Transformation from
# INERTIAL FRAME to CAMERA FRAME

def update_camera_plot(val):
    angle = math.floor(sangle.val)
    angle_camera = math.floor(angle)
    ax.clear()
    plot_camera_frame(ax, angle_camera)


def rot_mx(angle_camera):
    # Convention of Rotation: 1) z-axis 2) x-axis 3) y axis
    rot_z_angle = 180  #degrees
    rot_matrix_z = mx_ut.rotation_z(rot_z_angle)
    rot_y_angle = 90 + angle_camera  # degrees
    rot_matrix_y = mx_ut.rotation_y(rot_y_angle)
    rot_matrix = mx_ut.matrix_matrix_multipl(rot_matrix_y, rot_matrix_z)
    return rot_matrix

def transform_matrix_inertial_to_camera(angle_camera):
    transform_matrix = mx_ut.matrix_matrix_multipl(mx_ut.generate_translation_mx(dx_cam, dy_cam, dz_cam), rot_mx(angle_camera))
    return transform_matrix



# Axis is in the center of the ball bearing
def plot_inertial_frame(ax):
    x_arw = plt_ut.Arrow3D([0,x_inertial[0]],[0,x_inertial[1]],[0,x_inertial[2]], **plt_ut.arr_prop_red)
    y_arw = plt_ut.Arrow3D([0,y_inertial[0]],[0,y_inertial[1]],[0,y_inertial[2]], **plt_ut.arr_prop_green)
    z_arw = plt_ut.Arrow3D([0,z_inertial[0]],[0,z_inertial[1]],[0,z_inertial[2]], **plt_ut.arr_prop_blue)
    plt_ut.add_to_plot(ax, x_arw, y_arw, z_arw)


def generate_camera_frame(inertial_vector):
    camera_point = mx_ut.matrix_vector_multipl(transform_matrix_inertial_to_camera(angle_camera), inertial_vector)
    return camera_point

def plot_camera_frame(ax, angle_camera):
    x_camera = generate_camera_frame(x_inertial)
    y_camera = generate_camera_frame(y_inertial)
    z_camera = generate_camera_frame(z_inertial)
    origin_camera = generate_camera_frame(origin_inertial)
    x_arw_camera = plt_ut.Arrow3D([origin_camera[0], x_camera[0]],[origin_camera[1], x_camera[1]],
                                [origin_camera[2], x_camera[2]], **plt_ut.arr_prop_red)

    y_arw_camera = plt_ut.Arrow3D([origin_camera[0], y_camera[0]], [origin_camera[1], y_camera[1]],
                                [origin_camera[2], y_camera[2]], **plt_ut.arr_prop_green)

    z_arw_camera = plt_ut.Arrow3D([origin_camera[0], z_camera[0]], [origin_camera[1], z_camera[1]],
                                [origin_camera[2], z_camera[2]], **plt_ut.arr_prop_blue)
    plt_ut.add_to_plot(ax, x_arw_camera, y_arw_camera, z_arw_camera)



file = open('output.txt', 'w')
file.write('Rotation Matrix Inertial - to - Camera \n')

fig = plt.figure(1)
ax = plt_ut.generate_plot(fig)

# Generate the inertial frame
unit = 6
x_inertial = np.array([unit, 0, 0, 1])
y_inertial = np.array([0, unit, 0, 1])
z_inertial = np.array([0, 0, unit, 1])
origin_inertial = np.array([0, 0, 0, 1])

# INPUT data from the C++ algorithm
n = [0.47, -0.04, 0.87]
n_hom = [n[0], n[1], n[2], 1]
print (n_hom)
u = [1, 0, 0]  # arbitrary chosen


# Generate the other two vectors perpendicular to n using Gram-Schmidt Algorithm
combined_vectors = np.array([n, u])
u = (gram_schmidt.gs(combined_vectors)[1])

u_hom = [u[0], u[1], u[2], 1]
v = mx_ut.cross_product(n, u)

v_hom = [v[0], v[1], v[2], 1]

axangle = plt.axes([0.25, 0.05, 0.65, 0.03])
sangle = Slider(axangle, 'Camera Angle (deg)', 0, 90, valinit=30, valfmt='%1d')
sangle.on_changed(update_camera_plot)

plot_camera_frame(ax, angle_camera)
plot_inertial_frame(ax)
file.write(str(transform_matrix_inertial_to_camera(angle_camera)))


# PART 1 bis: Transformation viceversa from
# CAMERA FRAME to INERTIAL FRAME

def generate_inertial_frame_from_camera_frame(inv_matrix_camera_inertial):
    xcam = generate_camera_frame(x_inertial)
    ycam = generate_camera_frame(y_inertial)
    zcam = generate_camera_frame(z_inertial)
    org_cam = generate_camera_frame(origin_inertial)

    x_inert = mx_ut.matrix_vector_multipl(inv_matrix_camera_inertial, xcam)
    y_inert = mx_ut.matrix_vector_multipl(inv_matrix_camera_inertial, ycam)
    z_inert = mx_ut.matrix_vector_multipl(inv_matrix_camera_inertial, zcam)
    orig_inert = mx_ut.matrix_vector_multipl(inv_matrix_camera_inertial, org_cam)


file.write('\n')
file.write('\n')
file.write('Rotation Matrix Camera - to - Inertial (computed as inverse matrix) \n') 

inv_matrix_camera_inertial = mx_ut.inverse_matrix(transform_matrix_inertial_to_camera(angle_camera))
file.write(str(inv_matrix_camera_inertial))
generate_inertial_frame_from_camera_frame(inv_matrix_camera_inertial)


# PART 2: Solving Linear Equation
# c0 + landa * d = p0 + alpha * u + belta * v [camera coordinates]

# TERM 1: c0
# c0 represents the position of the camera
# c0 in camera coordinates is (0,0,0,1)
# c0 in world coordinates is

print("Camera position")
c0_world = generate_camera_frame(origin_inertial)
c0_camera = np.array([0.0, 0.0, 0.0, 1.0])


# TERM 2: d
# d represents the direction vector starting
# at the camera center in the direction
# of the point of interest (e.g. center of the smaller ellipses)

def direction_vector_world(origin, point, color):
    result = mx_ut.matrix_vector_multipl(inv_matrix_camera_inertial, point)
    xx = result[0] - generate_camera_frame(origin)[0]
    yy = result[1] - generate_camera_frame(origin)[1]
    zz = result[2] - generate_camera_frame(origin)[2]
    origin_camera = generate_camera_frame(origin)
    direction = np.array([xx, yy, zz, 1])

    d = plt_ut.Arrow3D([origin_camera[0], xx], [origin_camera[1], yy],
                    [origin_camera[2], zz], **color)

    ax.add_artist(d)
    return(direction)

# INPUT TEST
elipse1 = np.array([cam.X_elip_mm[0], cam.Y_elip_mm[0], cam.FOCUS, 1])
elipse2 = np.array([cam.X_elip_mm[1], cam.Y_elip_mm[1], cam.FOCUS, 1])
elipse3 = np.array([cam.X_elip_mm[2], cam.Y_elip_mm[2], cam.FOCUS, 1])

direction1 = direction_vector_world(origin_inertial, elipse1, plt_ut.arr_prop_y)
direction2 = direction_vector_world(origin_inertial, elipse2, plt_ut.arr_prop_y)
direction3 = direction_vector_world(origin_inertial, elipse3, plt_ut.arr_prop_y)


# TERM 3: P0
# P0 represents the center of the main ellipse
# it is computer by starting in the centre of the sphere
# and moving up in the direction of the n vector

sphere_radius = 10 # cm

def plane_vectors(n):
    n_world = n
    module_n_world = math.sqrt(n_world[0] * n_world[0] + n_world[1]*n_world[1] + n_world[2]*n_world[2])
    n_norm_world_x = n_world[0]/(module_n_world)
    n_norm_world_y= n_world[1]/(module_n_world)
    n_norm_world_z = n_world[2]/(module_n_world)

    n_normalized_world = np.array([n_norm_world_x, n_norm_world_y, n_norm_world_z, 1])

    P0_world = n_normalized_world * sphere_radius;
    P0_world[3] = 1

    P0 = plt_ut.Arrow3D([0, P0_world[0]], [0, P0_world[1]],
                    [0, P0_world[2]], **plt_ut.arr_prop_y)
    ax.add_artist(P0)

    return P0_world

# Verify P0 satisfies sphere equation
def sphere_eq_verification(P0_world):
    result = P0_world[0]*P0_world[0] + P0_world[1]*P0_world[1] + P0_world[2]*P0_world[2]
    if result == sphere_radius*sphere_radius:
        return True
    else:
        return False

P0_world = plane_vectors(n_hom)
print(sphere_eq_verification(P0_world))


# u
u_world = np.array([0.0, 0.0, 0.0, 0.0])
v_world = np.array([0.0, 0.0, 0.0, 0.0])

for i in range(len(u_hom)):
    u_world[i] = u_hom[i] * scale + P0_world[i]
    v_world[i] = v_hom[i] * scale + P0_world[i]




d = plt_ut.Arrow3D([P0_world[0], u_world[0]], [P0_world[1], u_world[1]],
                    [P0_world[2], u_world[2]], **plt_ut.arr_prop_green)
ax.add_artist(d)


dd = plt_ut.Arrow3D([P0_world[0], v_world[0]], [P0_world[1], v_world[1]],
                    [P0_world[2], v_world[2]], **plt_ut.arr_prop_red)
ax.add_artist(dd)


x = np.linspace(P0_world[0] - 5, P0_world[0]+5, 10)
y = np.linspace(P0_world[1] - 5, P0_world[1]+5, 10)
X,Y = np.meshgrid(x,y)
z = (-n[0]*x - n[1]*y + sphere_radius )/n[2];

surf = ax.plot_surface(X, Y, z, rstride=1,cstride=1,alpha=0,linewidth=0.5, edgecolors='r')

file.close()

plt.show()

