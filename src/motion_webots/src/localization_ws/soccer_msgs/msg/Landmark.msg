# A single field landmark detection, back-projected to the ground plane.
# Class ids follow the landmark_geometry / YOLO dataset ordering:
#   0 = L, 1 = T, 2 = X, 3 = goalpost, 4 = center_circle
uint16 class_id
float32 confidence           # detector confidence [0..1]
float32 u                    # box reference pixel (junction center / post base)
float32 v
geometry_msgs/Point p_cam    # landmark position in the camera optical frame [m]
geometry_msgs/Point p_base   # landmark position in base_link (ground, z~0) [m]
float64[4] covariance_2x2    # 2x2 row-major [xx, xy, yx, yy] of p_base xy [m^2]
bool valid_range             # false if beyond the trusted range for this class
