# SurfaceSection.msg
# Cross-section data for pipe/corridor wall surface

std_msgs/Header header

# Environment mode
uint8 MODE_UNKNOWN = 0
uint8 MODE_CORRIDOR = 1
uint8 MODE_PIPE = 2
uint8 environment_mode

# Geometry type detected
uint8 GEOMETRY_UNKNOWN = 0
uint8 GEOMETRY_PLANE = 1
uint8 GEOMETRY_CYLINDER = 2
uint8 geometry_type

# Common fields
geometry_msgs/Point center              # Center point of the section (robot frame)
geometry_msgs/Vector3 normal            # Surface normal direction

# Plane parameters (for corridor mode)
float32[4] plane_coefficients           # ax + by + cz + d = 0

# Cylinder parameters (for pipe mode)
float32 radius                          # Pipe radius in meters
float32 radius_variance                 # Uncertainty in radius estimation
geometry_msgs/Vector3 axis_direction    # Cylinder axis direction
float32 eccentricity                    # Deviation from perfect circle (0 = perfect)

# Cross-section points
geometry_msgs/Point32[] section_points  # Points forming the cross-section
uint32 num_section_points

# Quality metrics
float32 confidence                      # Overall detection confidence (0.0 ~ 1.0)
float32 fit_error                       # RMS fitting error
float32 coverage_angle                  # Angular coverage in degrees (max 360)
bool is_valid                           # True if valid detection

# Distance measurements
float32 min_distance                    # Minimum distance to wall
float32 max_distance                    # Maximum distance to wall
float32 mean_distance                   # Mean distance to wall
