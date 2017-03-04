import gram_schmidt
import numpy as np
import matrix_utils as mx_ut
import util_plot as plt_ut
import camera as cam
import matplotlib.pyplot as plt
from numpy.linalg import inv
plt.switch_backend('WebAgg')

# PARAMETERS OF THE ENVIRONMENT (EXTRINSIC)
# Position of the camera relative
# to the centre (inertial frame) in the axis of the inertial frame
dx_cam = -24  # cm
dy_cam = 0 # cm
dz_cam = 30 # cm

angle_camera = 30 # degrees
scale = 10


def Point(x, y, z):
    return np.array([x, y, z, 1])

def Vector(x, y, z):
    return np.array([x, y, z, 0])


##### PLOTTINGS
def plot_plane(point, normal_vector):
    x = np.linspace(P0_world[0] - 5, P0_world[0] + 5, 10)
    y = np.linspace(P0_world[1] - 5, P0_world[1] + 5, 10)
    X,Y = np.meshgrid(x,y)
    d = point.dot(normal_vector)
    Z = (-normal_vector[0] * x - normal_vector[1] * y + d) / normal_vector[2]

    ax.plot_surface(X, Y, Z,
                    rstride=1, cstride=1, alpha=0,
                    linewidth=0.5, edgecolors='r')


def plot_vector(v, origin=[0,0,0], style=plt_ut.arr_prop_red):
    arrow = plt_ut.Arrow3D([origin[0], origin[0] + v[0]],
                             [origin[1], origin[1] + v[1]],
                             [origin[2], origin[2] + v[2]],
                             **style)
    ax.add_artist(arrow)


def plot_frames(ax):
    plot_vector(e1_inertial, origin_inertial, plt_ut.arr_prop_red)
    plot_vector(e2_inertial, origin_inertial, plt_ut.arr_prop_green)
    plot_vector(e3_inertial, origin_inertial, plt_ut.arr_prop_blue)
    camera_e1_inertial = transform_inertial_to_camera(camera_e1_camera)
    camera_e2_inertial = transform_inertial_to_camera(camera_e2_camera)
    camera_e3_inertial = transform_inertial_to_camera(camera_e3_camera)
    camera_origin_inertial = transform_inertial_to_camera(camera_origin_camera)
    plot_vector(camera_e1_inertial, camera_origin_inertial, plt_ut.arr_prop_red)
    plot_vector(camera_e2_inertial, camera_origin_inertial, plt_ut.arr_prop_green)
    plot_vector(camera_e3_inertial, camera_origin_inertial, plt_ut.arr_prop_blue)


#### Transformations
def rot_mx_camera_to_inertial(angle_camera):
    # steps of rotations that move the inertial frame into the camera frame
    # -> rotation transforms a vector in the camera frame to
    #    a vector in the inertial frame
    # Convention of Rotation: 1) z-axis 2) x-axis 3) y axis
    rot_z_angle = 180  # degrees
    rot_matrix_z = mx_ut.rotation_z(rot_z_angle)
    rot_y_angle = (90 + angle_camera)  # degrees
    rot_matrix_y = mx_ut.rotation_y(rot_y_angle)
    rot_matrix = mx_ut.matrix_matrix_multipl(rot_matrix_y, rot_matrix_z)
    return rot_matrix

def translation_mx_camera_to_inertial(dx, dy, dz):
    translation_matrix = np.array([[1, 0, 0, dx],
                                   [0, 1, 0, dy],
                                   [0, 0, 1, dz],
                                   [0, 0, 0, 1]])
    return translation_matrix

def rot_and_translation_mx_camera_to_inertial(angle_camera):
    transform_matrix = mx_ut.matrix_matrix_multipl(translation_mx_camera_to_inertial(dx_cam, dy_cam, dz_cam),
                                   rot_mx_camera_to_inertial(angle_camera))
    return transform_matrix


def transform_camera_to_inertial(camera):
    inv_matrix_camera_inertial = mx_ut.inverse_matrix(rot_and_translation_mx_camera_to_inertial(angle_camera))
    return mx_ut.matrix_vector_multipl(inv_matrix_camera_inertial, camera)


def transform_inertial_to_camera(inertial):
    return mx_ut.matrix_vector_multipl(rot_and_translation_mx_camera_to_inertial(angle_camera), inertial)


def generate_inertial_frame_from_camera_frame(inv_matrix_camera_inertial):
    xcam = transform_inertial_to_camera(e1_inertial)
    ycam = transform_inertial_to_camera(e2_inertial)
    zcam = transform_inertial_to_camera(e3_inertial)
    print(xcam, ycam, zcam)
    org_cam = transform_inertial_to_camera(origin_inertial)
    print(org_cam)
    x_inert = mx_ut.matrix_vector_multipl(inv_matrix_camera_inertial, xcam)
    y_inert = mx_ut.matrix_vector_multipl(inv_matrix_camera_inertial, ycam)
    z_inert = mx_ut.matrix_vector_multipl(inv_matrix_camera_inertial, zcam)
    print(x_inert, y_inert, z_inert)
    orig_inert = mx_ut.matrix_vector_multipl(inv_matrix_camera_inertial, org_cam)



# INPUT TESTING POINT
fig = plt.figure(1)
ax = plt_ut.generate_plot(fig)


# Generate the inertial frame
unit = 6
e1_inertial = Vector(unit, 0, 0)
e2_inertial = Vector(0, unit, 0)
e3_inertial = Vector(0, 0, unit)
origin_inertial = Point(0, 0, 0)

camera_e1_camera = Vector(unit, 0, 0)
camera_e2_camera = Vector(0, unit, 0)
camera_e3_camera = Vector(0, 0, unit)
camera_origin_camera = Point(0, 0, 0)
# INPUT data from the C++ algorithm

n_world_input = Vector(0.2, 0.1, -1)
n_camera = transform_inertial_to_camera(n_world_input) # replace with camera ellipse normal algorithm
print(n_camera)
plot_frames(ax)

# PART 1 bis: Transformation viceversa from
# CAMERA FRAME to INERTIAL FRAME


inv_matrix_camera_inertial = mx_ut.inverse_matrix(rot_and_translation_mx_camera_to_inertial(angle_camera))
generate_inertial_frame_from_camera_frame(inv_matrix_camera_inertial)



# PART 2: Solving Linear Equation
# c0 + landa * d = p0 + alpha * u + beta * v [camera coordinates]

# TERM 1: c0
# c0 represents the position of the camera
# c0 in camera coordinates is (0,0,0,1)
# c0 in world coordinates is (-24, 0, 40)

print("Camera position")
c0_world = transform_inertial_to_camera(origin_inertial)
print(c0_world)
# TERM 2: d
# d represents the direction vector starting
# at the camera center in the direction
# of the point of interest (e.g. center of the smaller ellipses)

# INPUT TEST
elipse1 = np.array([cam.X_elip_cm[0], cam.Y_elip_cm[0], cam.FOCUS, 0])
elipse2 = np.array([cam.X_elip_cm[1], cam.Y_elip_cm[1], cam.FOCUS, 0])
elipse3 = np.array([cam.X_elip_cm[2], cam.Y_elip_cm[2], cam.FOCUS, 0])

direction1 = transform_camera_to_inertial(elipse1)
print("direction1")
print(direction1)
direction2 = transform_camera_to_inertial(elipse2)
direction3 = transform_camera_to_inertial(elipse3)

plot_vector(direction1, transform_inertial_to_camera(origin_inertial), style=plt_ut.arr_prop_y)
plot_vector(direction2, transform_inertial_to_camera(origin_inertial))
plot_vector(direction3, transform_inertial_to_camera(origin_inertial))

# TERM 3: P0
# P0 represents the center of the main ellipse
# it is computed by starting in the centre of the sphere
# and moving up in the direction of the n vector

sphere_radius = 10 # cm
def gets_center_drawing_plane(n):
    if np.dot(n, e3_inertial) < 0:
        n = -n  # make sure n points up
    norm = np.linalg.norm(n)
    P0_world = origin_inertial + sphere_radius * n/norm
    plot_vector(P0_world, origin_inertial, style=plt_ut.arr_prop_y)
    return P0_world



# Verify P0 satisfies sphere equation
def sphere_eq_verification(P0_world):
    result = P0_world[0] * P0_world[0] + P0_world[1] * P0_world[1] + P0_world[2] * P0_world[2]
    if result == sphere_radius*sphere_radius:
        return True
    else:
        return False


n_world = transform_camera_to_inertial(n_camera)
print("n_world",n_world)
P0_world = gets_center_drawing_plane(n_world)
plot_plane(P0_world[0:3], n_world[0:3])


def find_vector_bases(n,u):
    combined_vectors = np.array([n, u])
    u = (gram_schmidt.gs(combined_vectors)[1])
    v = mx_ut.cross_product(n, u)
    return u, v


u_arbitrary = Vector(1, 0, 0)  # arbitrary chosen
n_world = mx_ut.normalize(n_world)
u_world, v_world = find_vector_bases(n_world[0:3], u_arbitrary[0:3])
u_world = mx_ut.normalize(u_world)
v_world = mx_ut.normalize(v_world)



plot_vector(u_world * 6, P0_world, plt_ut.arr_prop_red)
plot_vector(v_world * 6, P0_world, plt_ut.arr_prop_green)
plot_vector(n_world * 6, P0_world, plt_ut.arr_prop_blue)


def generate_matrix_from_direction_u_v(d,u,v):
    duv = np.array([[direction1[0], -u_world[0], -v_world[0]],
                [direction1[1], -u_world[1], -v_world[1]],
                [direction1[2], -u_world[2], -v_world[2]]])
    duv_inv = mx_ut.inverse_matrix_lg(duv)
    return duv_inv


def solve_lin_eq(P0C0, duv_inv):
    landa = duv_inv[0][0]*P0C0[0] + duv_inv[0][1]*P0C0[1] + duv_inv[0][2]*P0C0[2]
    alpha = duv_inv[1][0]*P0C0[0] + duv_inv[1][1]*P0C0[1] + duv_inv[1][2]*P0C0[2]
    beta = duv_inv[2][0]*P0C0[0] + duv_inv[2][1]*P0C0[1] + duv_inv[2][2]*P0C0[2]
    return landa, alpha, beta

def calculate_interaction_point(d, landa, c):
    return c + landa * d


# Point Test 1
duv_inv = generate_matrix_from_direction_u_v(direction1, u_world, v_world)
P0C0 = P0_world[0:3] - c0_world[0:3]
landa, alpha, beta = solve_lin_eq(P0C0, duv_inv)

point = calculate_interaction_point(direction1, landa, c0_world)
ax.scatter(point[0], point[1], point[2], c='r', marker='^')
print("Point is: ", point)
plot_vector(direction1*landa, transform_inertial_to_camera(origin_inertial), style=plt_ut.arr_prop_y)


print ("Landa", landa)
print ("Alpha", alpha)
print ("Beta", beta)



plt.show()

