import matplotlib
matplotlib.use('WebAgg')
import gram_schmidt
import numpy as np
import matrix_utils as mx_ut
import util_plot as plt_ut
import camera as cam
import matplotlib.pyplot as plt

# PARAMETERS OF THE ENVIRONMENT (EXTRINSIC)
# Position of the camera relative
# to the centre (inertial frame) in the axis of the inertial frame
dx_cam = -24        # cm
dy_cam = 0          # cm
dz_cam = 30         # cm
angle_camera = 45   # degrees (measured relative to XoY plane)
scale = 10
sphere_radius = 10  # cm


def Point(x, y, z):
    return np.array([x, y, z, 1])


def Vector(x, y, z):
    return np.array([x, y, z, 0])


# PLOTTINGS
def plot_plane(point, normal_vector):
    x = np.linspace(point[0] - 5, point[0] + 5, 10)
    y = np.linspace(point[1] - 5, point[1] + 5, 10)
    X, Y = np.meshgrid(x, y)
    d = point.dot(normal_vector)
    Z = (-normal_vector[0] * x - normal_vector[1] * y + d) / normal_vector[2]
    ax.plot_surface(X, Y, Z, linewidth=0.5, edgecolors='r', alpha=0)


def plot_vector(v, origin=[0, 0, 0], style=plt_ut.arr_red):
    arrow = plt_ut.Arrow3D([origin[0], origin[0] + v[0]],
                           [origin[1], origin[1] + v[1]],
                           [origin[2], origin[2] + v[2]], **style)
    ax.add_artist(arrow)


def plot_frames(ax, frame_inertial, frame_camera):
    plot_vector(frame_inertial[0], frame_inertial[3], plt_ut.arr_red)
    plot_vector(frame_inertial[1], frame_inertial[3], plt_ut.arr_green)
    plot_vector(frame_inertial[2], frame_inertial[3], plt_ut.arr_blue)

    camera_e1_inertial = transform_camera_to_inertial(frame_camera[0])
    camera_e2_inertial = transform_camera_to_inertial(frame_camera[1])
    camera_e3_inertial = transform_camera_to_inertial(frame_camera[2])
    camera_orig_inertial = transform_camera_to_inertial(frame_camera[3])

    plot_vector(camera_e1_inertial, camera_orig_inertial, plt_ut.arr_red)
    plot_vector(camera_e2_inertial, camera_orig_inertial, plt_ut.arr_green)
    plot_vector(camera_e3_inertial, camera_orig_inertial, plt_ut.arr_blue)


# Transformations
def rot_mx_camera_to_inertial(angle_camera):
    # Active transformations
    # steps of rotations that move the inertial frame into the camera frame
    # -> rotation transforms a vector in the camera frame to
    #    a vector in the inertial frame
    # Convention of Rotation: 1) z-axis 2) x-axis 3) y axis
    rot_z_angle = 180  # degrees
    rot_matrix_z = mx_ut.rotation_z(rot_z_angle)
    rot_y_angle = 90 + angle_camera  # degrees
    rot_matrix_y = mx_ut.rotation_y(rot_y_angle)
    return mx_ut.matrix_matrix_multipl(rot_matrix_y, rot_matrix_z)


def translation_mx_camera_to_inertial(dx, dy, dz):
    return np.array([[1, 0, 0, dx],
                     [0, 1, 0, dy],
                     [0, 0, 1, dz],
                     [0, 0, 0, 1]])


def rot_and_translation_mx_camera_to_inertial(angle_camera):
    return mx_ut.matrix_matrix_multipl(
                 translation_mx_camera_to_inertial(dx_cam, dy_cam, dz_cam),
                 rot_mx_camera_to_inertial(angle_camera))


def transform_inertial_to_camera(camera):
    inv_matrix_camera_inertial = mx_ut.inverse_matrix(
        rot_and_translation_mx_camera_to_inertial(angle_camera))
    return mx_ut.matrix_vector_multipl(inv_matrix_camera_inertial, camera)


def transform_camera_to_inertial(inertial):
    return mx_ut.matrix_vector_multipl(
        rot_and_translation_mx_camera_to_inertial(angle_camera), inertial)


def generate_frame(unit):
    e1 = Vector(unit, 0, 0)
    e2 = Vector(0, unit, 0)
    e3 = Vector(0, 0, unit)
    origin = Point(0, 0, 0)
    return e1, e2, e3, origin


def gets_center_drawing_plane(n, e3_inertial, orig_inertial):
    if np.dot(n, e3_inertial) < 0:
        n = -n  # make sure n points up
    norm = np.linalg.norm(n)
    P0_world = orig_inertial + sphere_radius * n/norm
    plot_vector(P0_world, orig_inertial, style=plt_ut.arr_yellow)
    return P0_world


def sphere_eq_verification(P0_world):
    product = P0_world[0]**2 + P0_world[1]**2 + P0_world[2]**2
    if product == sphere_radius**2:
        return True
    else:
        return False


def find_vector_bases(n, u):
    combined_vectors = np.array([n, u])
    u = (gram_schmidt.gs(combined_vectors)[1])
    v = np.cross(n, u)
    return u, v


def generate_matrix_from_d_u_v(d, u, v):

    duv = np.array([[d[0], -u[0], -v[0]],
                    [d[1], -u[1], -v[1]],
                    [d[2], -u[2], -v[2]]])
    return mx_ut.inverse_matrix_lg(duv)


def calculate_intersection_point(d, landa, c):
    return c + landa * d


def solve(d, u, v, P0_world, c0_world):
    duv_inv = generate_matrix_from_d_u_v(d, u, v)
    [landa, alpha, beta] = duv_inv.dot(P0_world[0:3] - c0_world[0:3])
    point = calculate_intersection_point(d, landa, c0_world)
    return landa, alpha, beta, point

def input_from_algorithm(n_camera, center_small_ellip):

    # PART 1: Generate & Plot camera and inertial frame
    unit = 6
    e1_inertial, e2_inertial, e3_inertial, orig_inertial = generate_frame(unit)
    frame_inertial = np.array([e1_inertial, e2_inertial,
                               e3_inertial, orig_inertial])

    cam_e1_cam, cam_e2_cam, cam_e3_cam, cam_orig_cam = generate_frame(unit)
    frame_camera = np.array([cam_e1_cam, cam_e2_cam, cam_e3_cam, cam_orig_cam])
    plot_frames(ax, frame_inertial, frame_camera)

    # PART 2: Determine coefficients of the Linear Equation
    # c0 + landa * d = p0 + alpha * u + beta * v

    # TERM 1: c0 - position of the camera in camera coordinats
    # c0 in world coordinates is (-24, 0, 40)

    c0_world = cam_orig_inertial = transform_camera_to_inertial(cam_orig_cam)

    # TERM 2: d ( direction_center_elipse)
    # d represents the direction vector starting at the camera center in the
    # direction of the point of interest (e.g. center of the smaller ellipses)

    dir_center_elipse_world = transform_camera_to_inertial(center_small_ellip)

    plot_vector(dir_center_elipse_world * unit, cam_orig_inertial,
                style=plt_ut.arr_yellow)

    # TERM 3: P0
    # P0 represents the center of the main ellipse. It is computed by starting
    # in the centre of the sphere, moving up in the direction of the n vector

    n_world = transform_camera_to_inertial(n_camera)
    P0_world = gets_center_drawing_plane(n_world, e3_inertial, orig_inertial)
    try:
        sphere_eq_verification(P0_world)
    except False:
        print("Sphere equation is not fulfilled")
    plot_plane(P0_world[0:3], n_world[0:3])

    # TERM 4, 5: u and v (the two orthogonal vectors in the plane)
    # Determined using Gram-Schmidt algorithm
    u_arbitrary = Vector(1, 0, 0)  # arbitrary chosen
    n_world = mx_ut.normalize(n_world)
    u_world, v_world = find_vector_bases(n_world[0:3], u_arbitrary[0:3])
    u_world = mx_ut.normalize(u_world)
    v_world = mx_ut.normalize(v_world)
    plot_vector(u_world * scale, P0_world, plt_ut.arr_red)
    plot_vector(v_world * scale, P0_world, plt_ut.arr_green)
    plot_vector(n_world * scale, P0_world, plt_ut.arr_blue)

    # PART 3: Solve Linear Equation
    landa, alpha, beta, pt = solve(dir_center_elipse_world, u_world, v_world,
                                    P0_world, c0_world)

    ax.scatter(pt[0], pt[1], pt[2], c='r', marker='^')
    plot_vector(dir_center_elipse_world * landa, cam_orig_inertial,
                style=plt_ut.arr_yellow)

    # Test start with the point generate and get the X,Y (cm)
    direction_pt_world = pt - c0_world
    direction_pt_camera = transform_inertial_to_camera(direction_pt_world)

    return pt


fig = plt.figure(1)
ax = plt_ut.generate_plot(fig)

# INPUT TESTING POINT
n_world_input = Vector(0.2, 0.1, -1)
n_camera = transform_inertial_to_camera(n_world_input)
elipse_center_cam = Vector(cam.X_elip_cm[0], cam.Y_elip_cm[0], cam.FOCUS)
point_world = input_from_algorithm(n_camera, elipse_center_cam)
print(point_world)
plt.show()
