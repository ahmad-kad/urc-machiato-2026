# Camera control commands for mast-mounted camera
# Supports pan-tilt-zoom operations for computer vision tasks

std_msgs/Header header

# Command type
uint8 command_type  # 0=absolute, 1=relative, 2=track_target, 3=scan_pattern

# Pan-tilt control (radians)
float32 pan_angle     # Desired pan angle (-pi to pi)
float32 tilt_angle    # Desired tilt angle (-pi/2 to pi/2)
float32 pan_speed     # Pan angular velocity (rad/s)
float32 tilt_speed    # Tilt angular velocity (rad/s)

# Zoom control
float32 zoom_level    # 1.0 = normal, higher values = zoomed in
bool autofocus        # Enable/disable autofocus

# Tracking control (for command_type=2)
geometry_msgs/Point target_position  # Target position in camera frame
float32 tracking_timeout  # Seconds to track before giving up

# Scan pattern control (for command_type=3)
string scan_pattern   # "horizontal", "vertical", "spiral", "raster"
float32 scan_speed    # Scan angular velocity (rad/s)
float32 scan_range    # Total scan angle (radians)

# Safety limits
float32 max_pan_speed   # Maximum allowed pan speed (rad/s)
float32 max_tilt_speed  # Maximum allowed tilt speed (rad/s)

# Command priority and timing
uint8 priority       # 0=normal, 1=high, 2=critical
float32 timeout      # Command timeout in seconds
