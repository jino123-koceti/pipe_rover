# LaserLine.msg
# Line laser profiler data (120 degree FOV fan beam)

std_msgs/Header header

# Laser identification
uint8 laser_id                    # 0: single, 1: left, 2: right

# Raw detection in image space
geometry_msgs/Point32[] image_points    # Detected laser points in image coordinates (u, v, 0)
uint32 num_points                       # Number of detected points

# 3D reconstructed line points (camera frame)
geometry_msgs/Point32[] points_3d       # 3D points from triangulation
float32[] depths                        # Depth values for each point

# Line fitting results
geometry_msgs/Point32 line_start        # Line segment start point (3D)
geometry_msgs/Point32 line_end          # Line segment end point (3D)
float32 line_length                     # Total line length in meters

# Quality metrics
float32 detection_confidence            # 0.0 ~ 1.0
float32 line_fit_error                  # RMS error of line fitting
bool is_valid                           # True if valid detection

# Laser hardware status
bool laser_on                           # GPIO state
float32 intensity                       # Laser intensity (0-255)
