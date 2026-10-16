std_msgs/Header header

# Teensy time of the newest ESC frame used in this message
uint32 teensy_time_ms

# ---------------------------------------------------------------------------
# Per motor. Index 0..3 = motor 1..4
# ---------------------------------------------------------------------------

# True when this motor's ESC gave a valid answer within thruster_data_timeout_ms.
# When false, rpm, voltage and temperature below are the last valid values.
bool[4] valid

# Teensy time of each motor's last valid ESC answer
uint32[4] sample_time_ms

# Latest thruster command received from ROS, 0.0 - 1.0
float32[4] target_command

# Thruster command currently applied by the Teensy, 0.0 - 1.0. It follows
# target_command at once when rising and through the ramp when falling.
float32[4] applied_command

# DShot value currently sent to the ESC: 0 = stop, 48..2047 = throttle
uint16[4] dshot

# Converted values. The scales are parameters of the bridge node.
# Mechanical RPM
float32[4] rpm
# Supply voltage seen by the ESC [V]
float32[4] voltage
# ESC temperature [deg C]
int16[4] temperature

# Raw values exactly as sent by the ESC
# Electrical RPM in units of 100
uint16[4] erpm_raw
# Voltage in units of 0.01 V
uint16[4] voltage_raw

# Telemetry requests without an answer / with a corrupt answer since the bridge started
uint32[4] no_answer_count
uint32[4] corrupt_count

# ---------------------------------------------------------------------------
# Total current of all four motors
# ---------------------------------------------------------------------------

# Current [A], converted from current_adc with the bridge calibration
float32 current

# Raw 12-bit ADC count of the ESC current output (0..4095 = 0..3.3 V),
# mean of the samples received since the previous message
float32 current_adc

# Number of ADC samples averaged into current_adc. 0 = no new sample,
# current and current_adc repeat the previous message.
uint16 current_samples
