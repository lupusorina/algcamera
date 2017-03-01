import numpy as np
import matplotlib.pyplot as plt
import matrix_utils

# PARAMETERS OF THE CAMERA (INTRINSIC)

IMG_SIZE_HEIGHT_PX = 480 # PX
IMG_SIZE_WIDTH_PX = 744 # PX
PIXEL_SIZE = 0.006 # mm
IMG_SIZE_HEIGHT_MM = IMG_SIZE_HEIGHT_PX * PIXEL_SIZE
IMG_SIZE_WIDTH_MM = IMG_SIZE_WIDTH_PX * PIXEL_SIZE
FOCUS = 6 # mm

# Transformation FROM PIXELS to MM

def tranf_image_coord_px_mm(input):
    return (input * PIXEL_SIZE)


# INPUT DATA FOR TEST
# Position of the centers of the elipses

X_elip_px = [375.992, 353.51, 327.265, 311.674, 316.17, 338.743, 365.972, 381.199]
Y_elip_px = [166.052, 152.426, 156.293, 175.538, 199.192, 213.578, 209.188, 189.199]

# plot with various axes scales
plt.figure(2)

plt.subplot(1, 2, 1)
plt.plot(X_elip_px, Y_elip_px)
plt.yscale('linear')
plt.title('Image coordinates (px)')
plt.grid(True)
plt.ylim((0 ,IMG_SIZE_HEIGHT_PX))
plt.xlim((0 ,IMG_SIZE_WIDTH_PX))

X_elip_mm = np.zeros(len(X_elip_px))
Y_elip_mm = np.zeros(len(Y_elip_px))

for i in range(0, len(X_elip_mm)):
    X_elip_mm[i] = tranf_image_coord_px_mm(X_elip_px[i])
    Y_elip_mm[i] = tranf_image_coord_px_mm(Y_elip_px[i]) 


plt.subplot(1, 2, 2)
plt.plot(X_elip_mm, Y_elip_mm)
plt.yscale('linear')
plt.title('Image coordinates (mm)')
plt.ylim((0 ,IMG_SIZE_HEIGHT_MM))
plt.xlim((0 ,IMG_SIZE_WIDTH_MM))
plt.grid(True)





plt.show()