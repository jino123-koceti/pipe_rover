# VillSlamStatus.msg
# VILL-SLAM system status message

std_msgs/Header header

# System state
uint8 STATE_INITIALIZING = 0
uint8 STATE_RUNNING = 1
uint8 STATE_LOST = 2
uint8 STATE_PAUSED = 3
uint8 STATE_ERROR = 4
uint8 system_state

# Environment mode
uint8 MODE_CORRIDOR = 1
uint8 MODE_PIPE = 2
uint8 MODE_AUTO = 3
uint8 environment_mode

# SLAM statistics
uint64 num_keyframes                    # Total keyframes in map
uint64 num_landmarks                    # Total landmarks/features
uint64 num_constraints                  # Total factor graph constraints
float32 map_size_mb                     # Current map size in MB

# Current pose (world frame)
geometry_msgs/PoseStamped current_pose
geometry_msgs/TwistStamped current_velocity

# Distance traveled
float32 total_distance                  # Total distance traveled in meters
float32 session_distance                # Distance in current session

# Sensor status (bitmask)
uint16 SENSOR_LIDAR = 1
uint16 SENSOR_IMU = 2
uint16 SENSOR_CAMERA_FRONT = 4
uint16 SENSOR_CAMERA_LEFT = 8
uint16 SENSOR_CAMERA_RIGHT = 16
uint16 SENSOR_LINE_LASER = 32
uint16 SENSOR_WHEEL_ODOM = 64
uint16 SENSOR_GPS = 128
uint16 sensors_active                   # Bitmask of active sensors
uint16 sensors_healthy                  # Bitmask of healthy sensors

# Geometry estimation (pipe mode)
float32 estimated_pipe_radius           # Current pipe radius estimate
float32 pipe_radius_confidence          # Confidence in radius estimate
float32 distance_from_center            # Robot distance from pipe center

# Performance metrics
float32 processing_fps                  # Current processing rate
float32 latency_ms                      # End-to-end latency
float32 cpu_usage_percent               # CPU utilization
float32 memory_usage_mb                 # Memory usage

# Drift estimation
float32 estimated_drift                 # Estimated position drift
float32 loop_closure_count              # Number of loop closures

# Recording status
bool is_recording                       # Data recording active
string recording_path                   # Current recording directory
