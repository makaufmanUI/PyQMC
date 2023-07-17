"""
### 2D Calibration: Make Calculations

Read magnetic sensor data (pair of `X`, `Y` coordinates) from a file, then:

    1) Calculate the ellipse that best fits the data, using the least squares method.
    2) Calculate the affine transformation matrix from the ellipse to the circle with the radius equal to the ellipse major axis.
    3) Output a Gnuplot script which generates a graph with:
        - the input data points;
        - the fitting ellipse;
        - the affine transformation circle;
        - the position of an example point before and after the transformation;
        - the affine transformation circle, centered at the origin;

#### Web References
    - Fitting an Ellipse to a Set of Data Points
        http://nicky.vanforeest.com/misc/fitEllipse/fitEllipse.html
    - Circle affine transformation
        https://math.stackexchange.com/questions/619037/circle-affine-transformation
    - How to fit a 2D ellipse to given points
        https://stackoverflow.com/questions/47873759/how-to-fit-a-2d-ellipse-to-given-points
    - Fitting an ellipse to a set of data points in python (nan values in axes)
        https://stackoverflow.com/questions/39693869/fitting-an-ellipse-to-a-set-of-data-points-in-python
"""

import sys
import json
import os.path
import numpy as np
from numpy.linalg import eig, inv



if len(sys.argv) > 1:
    RAW_DATA_FILE = sys.argv[1]
else:
    print(u"Usage: %s {raw_data_file}" % (os.path.basename(sys.argv[0])))
    sys.exit(1)



# Take a point of the ellipse to show affine transformation to circle.
# This is the counter-clockwise angle from the X positive axis.
if len(sys.argv) > 2:
    TEST_PHI = 2 * np.pi * float(sys.argv[2]) / 100.0
else:
    TEST_PHI = np.pi * 0.3



# Measured values range.
SENSOR_MIN_VAL = -32768
SENSOR_MAX_VAL =  32767



# Name of the image generated by Gnuplot.
if RAW_DATA_FILE[-4] == u'.':
    OUTPUT_PNG = RAW_DATA_FILE[0:-4] + u'.png'
else:
    OUTPUT_PNG = RAW_DATA_FILE + u'.png'



def read_data_file():
    """
    Reads a file with `x`, `y` data lines. 
    
    Returns two lists with `x` and `y` values, plus the min/max values for `x` and `y`.
    """
    global SENSOR_MAX_VAL, SENSOR_MIN_VAL
    min_x = min_y = SENSOR_MAX_VAL
    max_x = max_y = SENSOR_MIN_VAL
    x = []
    y = []
    with open(RAW_DATA_FILE, 'r') as f:
        for line in f:
            values = line.strip().split()
            data_x = float(values[0])
            data_y = float(values[1])
            x.append(data_x)
            y.append(data_y)
            if data_x < min_x:
                min_x = data_x
            if data_x > max_x:
                max_x = data_x
            if data_y < min_y:
                min_y = data_y
            if data_y > max_y:
                max_y = data_y
    return x, y, min_x, min_y, max_x, max_y



def fit_ellipse(x, y, use_abs=True):
    """
    Returns the best fit ellipse from two `numpy.ndarray` (multidimensional arrays) of vertices.
    """
    x = x[:, np.newaxis]
    y = y[:, np.newaxis]
    D =  np.hstack((x*x, x*y, y*y, x, y, np.ones_like(x)))
    S = np.dot(D.T, D)
    C = np.zeros([6,6])
    C[0, 2] = C[2, 0] = 2; C[1, 1] = -1
    E, V = eig(np.dot(inv(S), C))
    if use_abs:
        n = np.argmax(np.abs(E))
    else:
        # Use this if semi axes are invalid (sqrt of negative).
        n = np.argmax(E)
    a = V[:, n]
    return a



def ellipse_center(a):
    """
    Returns the coordinates of the ellipse center.
    """
    b,c,d,f,g,a = a[1]/2, a[2], a[3]/2, a[4]/2, a[5], a[0]
    num = b*b-a*c
    x0=(c*d-b*f)/num
    y0=(a*f-b*d)/num
    return np.array([x0, y0])



def ellipse_semi_axes_length(a):
    """
    Returns the length of both semi-axes of the ellipse.
    """
    b,c,d,f,g,a = a[1]/2, a[2], a[3]/2, a[4]/2, a[5], a[0]
    up = 2*(a*f*f+c*d*d+g*b*b-2*b*d*f-a*c*g)
    down1=(b*b-a*c)*( (c-a)*np.sqrt(1+4*b*b/((a-c)*(a-c)))-(c+a))
    down2=(b*b-a*c)*( (a-c)*np.sqrt(1+4*b*b/((a-c)*(a-c)))-(c+a))
    if (up/down1) >= 0 and (up/down2) >= 0:
        res1=np.sqrt(up/down1)
        res2=np.sqrt(up/down2)
    else:
        res1 = None
        res2 = None
    return np.array([res1, res2])



def ellipse_angle_of_rotation(a):
    """
    Returns the rotation angle (in radians) of the ellipse axes.
    
    A positive angle means counter-clockwise rotation.
    """
    b,c,d,f,g,a = a[1]/2, a[2], a[3]/2, a[4]/2, a[5], a[0]
    return 0.5*np.arctan(2*b/(a-c))



def affine_matrix(a, b, phi, to_origin=False):
    """
    Matrix for affine transformation from ellipse to circle.
    """
    if a >= b:
        # Affine transformation to circle with R = A (major axis).
        ab_ratio = float(a) / float(b)
        cos_phi = np.cos(phi)
        sin_phi = np.sin(phi)
    else:
        # Swap A and B axis: transformation to circle with R = B (major axis).
        ab_ratio = float(b) / float(a)
        cos_phi = np.cos(phi+np.pi/2)
        sin_phi = np.sin(phi+np.pi/2)
    # R1 and R2: matrix to rotate the ellipse orthogonal to the axes and back.
    # T1 and T2: matrix to translate the ellipse to the origin and back.
    # D: matrix to scale ellipse to circle.
    R1 = np.array([[cos_phi,  sin_phi, 0], [-sin_phi, cos_phi, 0], [0, 0, 1]], dtype=float)
    R2 = np.array([[cos_phi, -sin_phi, 0], [sin_phi,  cos_phi, 0], [0, 0, 1]], dtype=float)
    T1 = np.array([[1,          0,   -cx], [0,        1,     -cy], [0, 0, 1]], dtype=float)
    T2 = np.array([[1,          0,    cx], [0,        1,      cy], [0, 0, 1]], dtype=float)
    D  = np.array([[1,          0,     0], [0, ab_ratio,       0], [0, 0, 1]], dtype=float)
    if to_origin:
        # Transformation shifted to axes origin.
        return np.matmul(np.matmul(np.matmul(R2, D), R1), T1)
    else:
        # Transformation centered with the ellipse.
        return np.matmul(np.matmul(np.matmul(np.matmul(T2, R2), D), R1), T1)



# Read data from file.
x, y, min_x, min_y, max_x, max_y = read_data_file()
MAX_SCALE = int(max(abs(min_x), abs(max_x), abs(min_y), abs(max_y)) / 500.0 * 750.0)



# Convert lists x and y into Numpy N-dimensional arrays.
x_arr = np.fromiter(x, np.float)        # type: ignore
y_arr = np.fromiter(y, np.float)        # type: ignore



# Calculate the ellipse which best fits the data.
warning = ''
ellipse = fit_ellipse(x_arr, y_arr)
[cx, cy] = ellipse_center(ellipse)
[a, b] = ellipse_semi_axes_length(ellipse)
phi = ellipse_angle_of_rotation(ellipse)
# If semi axes are invalid, try a different method.
if a == None or b == None:
    warning = "Invalid semi axes detected: using fit_ellipse() without np.abs()."
    ellipse = fit_ellipse(x_arr, y_arr, use_abs=False)
    [cx, cy] = ellipse_center(ellipse)
    [a, b] = ellipse_semi_axes_length(ellipse)
    phi = ellipse_angle_of_rotation(ellipse)



# Calculate the coordinates of semi-axes vertices.
ax = cx + a * np.cos(phi)
ay = cy + a * np.sin(phi)
bx = cx + b * np.cos(phi + np.pi/2)
by = cy + b * np.sin(phi + np.pi/2)



# Calculate the affine transformation matrix:
# centered with the best fitting ellipse...
M = affine_matrix(a, b, phi, to_origin=False)
# centered on the origin...
M1 = affine_matrix(a, b, phi, to_origin=True)



# Take P1: an example point to be transformed.
x1 = cx + a*np.cos(TEST_PHI)*np.cos(phi) - b*np.sin(TEST_PHI)*np.sin(phi)
y1 = cy + a*np.cos(TEST_PHI)*np.sin(phi) + b*np.sin(TEST_PHI)*np.cos(phi)
P1 = np.array([x1, y1, 1], dtype=float)
P1.shape = (3,1)



# Calculate P2: the affine transformation of the example point.
P2 = np.matmul(M, P1)
x2 = P2[0, 0]
y2 = P2[1, 0]



# Output the Gnuplot recipe.
GNUPLOT_SCRIPT = u"""#!/usr/bin/gnuplot
#
# The calibration matrix (affine transformation with offset to origin):
#
# %s
#
# The same matrix, as a Python array:
#
# sensor.calibration = %s
#
# %s
#
input_data = "%s"
set output "%s"
circle_size = %d * 0.02
raw_data_color = "#28e828"
ellipse_color = "#38a838"
affine_offset_color = "#d0d0d0"
affine_centered_color = "#c020c0"
set term png size 1200, 1200 font "Helvetica,18"
set style line 100 lc rgb raw_data_color lw 1
set style line 300 lc rgb ellipse_color lw 3
set style line 400 lc rgb affine_offset_color lw 3
set style line 500 lc rgb affine_centered_color lw 3
set style fill  transparent solid 0.50
set title "QMC5883L Magnetic Sensor X-Y Plane Calibration"
set size ratio 1
set xzeroaxis
set yzeroaxis
set xrange [-%d:%d]
set yrange [-%d:%d]
set label 40 center at graph 0.5,char 1.5 \\
    "Ellipse center (x, y) = (%d, %d), Semi-axis (a, b) = (%d, %d), Rotation = %.1f°"
set bmargin 5
set object 20 ellipse center %.2f,%.2f size %.2f,%.2f angle %.2f \\
    front fillstyle empty border lc rgb ellipse_color lw 3
set object 10 circle center %.2f,%.2f size %.2f \\
    front fillstyle empty border lc rgb affine_offset_color lw 3
set object 30 circle center 0,0 size %.2f \\
    front fillstyle empty border lc rgb affine_centered_color lw 3
plot input_data using 1:2:(circle_size) with circles linestyle 100 \\
        title "Raw Data", \\
    "<echo '%.2f %.2f %.2f %.2f\\n%.2f %.2f %.2f %.2f'" \\
        using 1:2:($3-$1):($4-$2) with vectors nohead linestyle 300 \\
        title "Best Fit Ellipse", \\
    "<echo '%.2f %.2f %.2f %.2f\\n%.2f %.2f %.2f %.2f'" \\
        using 1:2:($3-$1):($4-$2) with vectors nohead linestyle 400 \\
        title "Affine Transformation from Ellipse to Circle", \\
    "<echo '%.2f %.2f\\n%.2f %.2f'" \\
        using 1:2:(circle_size) with circles linestyle 400 \\
        title "Transformation: Example Point", \\
    "<echo '0 0 %.2f %.2f'" \\
        using 1:2:($3-$1):($4-$2) with vectors nohead linestyle 500 \\
        title "Transformation Circle: Offset to Origin", \\
    "<echo '%.2f %.2f'" \\
        using 1:2:(circle_size) with circles linestyle 500 \\
        title "Example Point: Offset to Origin"
"""


print((GNUPLOT_SCRIPT % (str(M1).replace("\n", "\n# "),  # Affine Matrix (split on multiple comment lines)
      json.dumps(M1.tolist()),              # Affine matrix (JSON Python string)
      warning,                              # Warning message, if any.
      RAW_DATA_FILE,
      OUTPUT_PNG,
      MAX_SCALE, MAX_SCALE, MAX_SCALE, MAX_SCALE, MAX_SCALE,
      cx, cy, a, b, np.degrees(phi),        # Label
      cx, cy, a*2, b*2, np.degrees(phi),    # Best Fit Ellipse
      cx, cy, max(a, b),                    # Affine Circle
      max(a, b),                            # Centered Affine Circle
      cx, cy, ax, ay, cx, cy, bx, by,       # Semi-axis A and B
      cx, cy, x1, y1, cx, cy, x2, y2,       # Example point + transformed (vector)
      x1, y1, x2, y2,                       # Example point + transformed (dot)
      x2-cx, y2-cy,                         # Example point transformed and centered (vector)
      x2-cx, y2-cy)                         # Example point (dot)
).encode('utf-8'))

