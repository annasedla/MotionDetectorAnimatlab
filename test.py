import pylab as plt
import numpy as np
from matplotlib.path import Path
import math

width, height=64, 64

# This index runs from n = 0 to 63 (one for each angular bin)
n = 63

theta = np.linspace((2*math.pi)*(float(n)/64), (2*math.pi)*float(n+1)/64, 5)
print((2*math.pi)*(n/64))
print((2*math.pi)*(n+1)/64)
xCenter = 32;
yCenter = 32;
radius = 32;
x = radius * np.cos(theta) + xCenter;
y = radius * np.sin(theta) + yCenter;
origin = np.array([xCenter, yCenter])
c = np.column_stack((x,y))
c = np.row_stack((c,origin))
poly_path=Path(c)

x, y = np.mgrid[:height, :width]
coors=np.hstack((x.reshape(-1, 1), y.reshape(-1,1))) # coors.shape is (4000000,2)

mask = poly_path.contains_points(coors)
z = mask.reshape(height, width)
z_final = np.where(z == False, 0, 1)  # z_final is the matrix of 1s and 0s, where 1 is a pixel in the angular bin and 0 is not included
num_of_pixels_in_bin = np.count_nonzero(z_final == 1)
print("This bin has",num_of_pixels_in_bin,"pixels!")


# Here we are just plotting the result so you see the angular bin (yellow) and the other pixels outside this bin (purple)
plt.imshow(z_final)
plt.show()

# The idea is that you run the index n above from 0 to 63. Each one corresponds to one of the angular bins (these matrices
# are referred to as "masks" because they select out the relevant pixels in that angular bin only).
# Use the z_final matrix and use numpy.multiply function to multiply the angular bin mask times your image.
# Next take the sume of all pixel intensities in the resultant matrix and divide by num_of_pixels_in_bin to get your
# average value over that angular bin to input to your neural network! Repeat this for each bin (n = 0, ..., 63)
