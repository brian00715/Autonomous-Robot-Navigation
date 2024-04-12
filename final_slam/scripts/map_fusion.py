import cv2
import numpy as np

# Read the two PGM maps
map1 = cv2.imread('../maps/carto_map_2d.pgm', cv2.IMREAD_GRAYSCALE)
map2 = cv2.imread('../maps/fast_lio_map.pgm', cv2.IMREAD_GRAYSCALE)

# Processing Cartographer Map
_, map1 = cv2.threshold(map1, 200, 255, cv2.THRESH_BINARY)


# Processing Fast LIO Map
edges = cv2.Canny(map2, 100, 200)
kernel = np.ones((5, 5), np.uint8)
dilated_edges = cv2.dilate(edges, kernel)
lines = cv2.HoughLines(edges, 1, np.pi / 180, threshold=100)

# Finding the rotation angle
bottom_line = None
bottom_y = -float('inf')
for line in lines:
    rho, theta = line[0]
    a = np.cos(theta)
    b = np.sin(theta)
    x0 = a * rho
    y0 = b * rho
    y1 = int(y0 + 1000 * (a))
    if y1 > bottom_y:
        bottom_y = y1
        bottom_line = line
angle = np.degrees(bottom_line[0][1])
adjusted_angle = 360 - angle

# Rotating Fast LIO Map
rows, cols = map2.shape
M = cv2.getRotationMatrix2D((cols / 2, rows / 2), adjusted_angle, 1)
map2 = cv2.warpAffine(map2, M, (cols, rows))
border_size = 5
map2 = cv2.copyMakeBorder(map2, border_size, border_size, border_size, border_size, cv2.BORDER_CONSTANT, value=0)


# Map Fusion
map2 = cv2.resize(map2, (map1.shape[1], map1.shape[0]))
# cv2.imwrite('adjusted_map.png', map2)
alpha = 0.5  # Blend weight, adjustable
fusion_map = cv2.addWeighted(map1, alpha, map2, 1-alpha, 0)
_, fusion_map = cv2.threshold(fusion_map, 200, 255, cv2.THRESH_BINARY)

# Saving the fused map
cv2.imwrite('../maps/fusion_map.pgm', fusion_map)