import numpy as np
import matplotlib.pyplot as plt

# PARAMETERS OF THE CAMERA (INTRINSIC)

IMG_SIZE_HEIGHT_PX = 480   # PX
IMG_SIZE_WIDTH_PX = 744    # PX
PIXEL_SIZE = 0.006         # mm
IMG_SIZE_HEIGHT_MM = IMG_SIZE_HEIGHT_PX * PIXEL_SIZE
IMG_SIZE_WIDTH_MM = IMG_SIZE_WIDTH_PX * PIXEL_SIZE
FOCUS = 0.6                # cm


# Transformation FROM PIXELS to MM
def tranf_image_coord_px_mm(x, y):
    x_centered = x - IMG_SIZE_WIDTH_PX / 2
    y_centered = y - IMG_SIZE_HEIGHT_PX / 2
    return x_centered * PIXEL_SIZE, y_centered * PIXEL_SIZE


# INPUT DATA FOR TEST
# Position of the centers of the elipses

X_elip_px = [356.155,  352, 342.237, 420.086, 383.095, 385.717, 365.972, 381.199]
Y_elip_px = [187.559, 129.87, 153.799, 150.596, 141.909, 205.973, 213.578, 209.188, 189.199]

X_elip_mm = np.zeros(len(X_elip_px))
Y_elip_mm = np.zeros(len(Y_elip_px))
X_elip_cm = X_elip_mm / 10
Y_elip_cm = Y_elip_mm / 10

for i in range(0, len(X_elip_mm)):
    X_elip_mm[i], Y_elip_mm[i] = tranf_image_coord_px_mm(X_elip_px[i], Y_elip_px[i])
    X_elip_cm[i] = X_elip_mm[i] / 10
    Y_elip_cm[i] = Y_elip_mm[i] / 10


if __name__ == 'main':
    plt.figure(2)
    plt.subplot(1, 2, 1)
    plt.plot(X_elip_px, Y_elip_px)
    plt.yscale('linear')
    plt.title('Image coordinates (px)')
    plt.grid(True)
    plt.ylim((0, IMG_SIZE_HEIGHT_PX))
    plt.xlim((0, IMG_SIZE_WIDTH_PX))

    plt.subplot(1, 2, 2)
    plt.plot(X_elip_cm, Y_elip_cm)
    plt.yscale('linear')
    plt.title('Image coordinates (mm)')
    plt.ylim((0, IMG_SIZE_HEIGHT_MM))
    plt.xlim((0, IMG_SIZE_WIDTH_MM))
    plt.grid(True)
    plt.show()
