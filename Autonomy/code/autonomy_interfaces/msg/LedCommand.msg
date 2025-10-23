# LED status signaling command
# Based on URC 2026 competition requirements

std_msgs/Header header

# Status codes (judge-visible signals)
int32 status_code  # 0=off, 1=ready, 2=running, 3=error, 4=emergency, 5=success

# Color specification
float32 red        # 0.0 to 1.0
float32 green      # 0.0 to 1.0
float32 blue       # 0.0 to 1.0

# Pattern specification
string pattern     # "solid", "blinking", "pulsing", "chasing"
float32 frequency  # Hz, for blinking/pulsing patterns
int32 priority     # 0=normal, 1=warning, 2=critical (affects brightness)

# Timing
float32 duration   # seconds, 0.0 = indefinite
bool override      # true to override lower priority commands
