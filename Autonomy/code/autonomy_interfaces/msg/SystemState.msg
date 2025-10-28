# System State Message
# Comprehensive state information for the rover state machine

std_msgs/Header header

# Primary state information
string current_state              # Top-level state (BOOT, CALIBRATION, IDLE, etc.)
string substate                   # Mission-specific substate (SCIENCE, DELIVERY, etc.)
string sub_substate              # Detailed substate (SAMPLE_DELIVERY, etc.)

# State metadata
float64 time_in_state            # Seconds spent in current state
float64 state_timeout            # Maximum allowed time in state (0.0 = no timeout)
string previous_state            # Last state before current
builtin_interfaces/Time transition_timestamp  # When current state was entered

# System status
bool is_transitioning            # True if mid-transition
bool preconditions_met           # True if ready for autonomous operations
string[] active_subsystems       # List of currently active subsystems
string[] failed_subsystems       # List of subsystems in error state

# Metadata for context
string mission_phase             # Competition mission context
string operator_id               # Who initiated current state
string state_reason              # Human-readable reason for state

