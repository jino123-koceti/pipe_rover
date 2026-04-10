# GeometryConstraint.msg
# Factor graph constraint data for GTSAM optimization

std_msgs/Header header

# Constraint type
uint8 CONSTRAINT_NONE = 0
uint8 CONSTRAINT_PLANE = 1
uint8 CONSTRAINT_CYLINDER = 2
uint8 CONSTRAINT_IMU = 3
uint8 CONSTRAINT_VISUAL = 4
uint8 CONSTRAINT_LIDAR = 5
uint8 CONSTRAINT_WHEEL_ODOM = 6
uint8 constraint_type

# Keyframe information
uint64 keyframe_id                      # Associated keyframe ID
uint64 prev_keyframe_id                 # Previous keyframe (for between factors)

# Measurement data
geometry_msgs/Pose measured_pose        # Measured pose (if applicable)
geometry_msgs/Twist measured_velocity   # Measured velocity (if applicable)

# Plane constraint data
float32[4] plane_params                 # Plane coefficients [a, b, c, d]

# Cylinder constraint data
float32 cylinder_radius                 # Measured pipe radius
geometry_msgs/Vector3 cylinder_axis     # Measured pipe axis
geometry_msgs/Point cylinder_center     # Measured pipe center

# Covariance / Noise model
float64[36] covariance                  # 6x6 covariance matrix (row-major)
float32 information_weight              # Scalar weight for this constraint

# Quality
float32 confidence                      # Measurement confidence (0.0 ~ 1.0)
bool is_outlier                         # Marked as outlier by robust estimation
bool is_active                          # Whether this constraint is active
