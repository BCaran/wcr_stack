std_msgs/Header header

uint32 sequence
uint32 teensy_time_ms

# Direktna signed 24-bit HX711 očitanja
int32 front_raw_adc
int32 rear_raw_adc

# Sirova očitanja nakon oduzimanja tare offseta
int32 front_raw_tared
int32 rear_raw_tared

# Calibrated values in grams
float32 front_grams
float32 rear_grams
float32 total_grams

#True when Teensy is not accepting new msgs
bool communication_timeout

# Teensy error bitmask; 0 mean no error
uint16 error_flags