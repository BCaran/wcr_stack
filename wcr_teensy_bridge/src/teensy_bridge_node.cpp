#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <wcr_interfaces/msg/load_cell_data.hpp>
#include <wcr_interfaces/msg/thruster_data.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <functional>
#include <iomanip>
#include <limits>
#include <mutex>
#include <poll.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <thread>
#include <unordered_map>
#include <unistd.h>
#include <vector>

using namespace std::chrono_literals;

namespace
{
constexpr std::size_t kThrusterCount = 4;
constexpr std::size_t kMaximumSerialLineLength = 512;

// Thruster commands are fractions of full throttle, 0.0 .. 1.0. The Teensy
// works in thousandths, so three decimals are sent on the serial link.
constexpr double kThrusterCommandResolution = 0.001;

// Status field of the ESC frame.
constexpr uint32_t kEscStatusNoAnswer = 0U;
constexpr uint32_t kEscStatusValid = 1U;
constexpr uint32_t kEscStatusCorrupt = 2U;

// Fraction with the three decimals the Teensy resolves, e.g. 0.250.
std::string formatFraction(double value)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3) << value;
  return stream.str();
}

uint16_t crc16Ccitt(const std::string & data)
{
  uint16_t crc = 0xFFFF;

  for (const unsigned char byte : data) {
    crc ^= static_cast<uint16_t>(byte) << 8U;

    for (int bit = 0; bit < 8; ++bit) {
      if ((crc & 0x8000U) != 0U) {
        crc = static_cast<uint16_t>((crc << 1U) ^ 0x1021U);
      } else {
        crc <<= 1U;
      }
    }
  }

  return crc;
}

std::string framePayload(const std::string & payload)
{
  std::ostringstream stream;
  stream << payload << '*' << std::uppercase << std::hex << std::setw(4)
         << std::setfill('0') << crc16Ccitt(payload) << '\n';
  return stream.str();
}

std::vector<std::string> splitCsv(const std::string & text)
{
  std::vector<std::string> fields;
  std::stringstream stream(text);
  std::string field;

  while (std::getline(stream, field, ',')) {
    fields.push_back(field);
  }

  return fields;
}

speed_t baudToTermios(int baud_rate)
{
  switch (baud_rate) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
#ifdef B230400
    case 230400: return B230400;
#endif
#ifdef B460800
    case 460800: return B460800;
#endif
#ifdef B921600
    case 921600: return B921600;
#endif
    default:
      throw std::invalid_argument("Unsupported baud_rate for termios: " +
                                  std::to_string(baud_rate));
  }
}

bool parseUnsigned(const std::string & text, uint32_t & value)
{
  try {
    std::size_t consumed = 0;
    const unsigned long parsed = std::stoul(text, &consumed, 10);
    if (consumed != text.size() || parsed > std::numeric_limits<uint32_t>::max()) {
      return false;
    }
    value = static_cast<uint32_t>(parsed);
    return true;
  } catch (...) {
    return false;
  }
}

bool parseSigned(const std::string & text, int32_t & value)
{
  try {
    std::size_t consumed = 0;
    const long parsed = std::stol(text, &consumed, 10);
    if (consumed != text.size() ||
      parsed < std::numeric_limits<int32_t>::min() ||
      parsed > std::numeric_limits<int32_t>::max())
    {
      return false;
    }
    value = static_cast<int32_t>(parsed);
    return true;
  } catch (...) {
    return false;
  }
}

bool parseFloat(const std::string & text, float & value)
{
  try {
    std::size_t consumed = 0;
    const float parsed = std::stof(text, &consumed);
    if (consumed != text.size() || !std::isfinite(parsed)) {
      return false;
    }
    value = parsed;
    return true;
  } catch (...) {
    return false;
  }
}
}  // namespace

class TeensyBridgeNode : public rclcpp::Node
{
public:
  TeensyBridgeNode()
  : Node("teensy_bridge")
  {
    declareParameters();
    readParameters();
    validateParametersOrThrow();

    load_cell_publisher_ = create_publisher<wcr_interfaces::msg::LoadCellData>(
      "/wcr/load_cells", rclcpp::SensorDataQoS());

    thruster_data_publisher_ = create_publisher<wcr_interfaces::msg::ThrusterData>(
      "/wcr/thrusters/data", rclcpp::SensorDataQoS());

    pwm_subscription_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/wcr/thrusters/duty_cycle", rclcpp::QoS(10),
      std::bind(&TeensyBridgeNode::pwmCallback, this, std::placeholders::_1));

    tare_service_ = create_service<std_srvs::srv::Trigger>(
      "/wcr/load_cells/tare",
      std::bind(
        &TeensyBridgeNode::tareServiceCallback, this,
        std::placeholders::_1, std::placeholders::_2));

    parameter_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&TeensyBridgeNode::parameterCallback, this, std::placeholders::_1));

    openSerialPort();
    running_.store(true);
    serial_thread_ = std::thread(&TeensyBridgeNode::serialWorker, this);

    sendCalibrationConfiguration();
    sendPwmConfiguration();

    const auto period = std::chrono::duration<double>(1.0 / command_rate_hz_);
    command_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&TeensyBridgeNode::commandTimerCallback, this));

    const auto thruster_data_period =
      std::chrono::duration<double>(1.0 / thruster_data_rate_hz_);
    thruster_data_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(thruster_data_period),
      std::bind(&TeensyBridgeNode::thrusterDataTimerCallback, this));

    RCLCPP_INFO(
      get_logger(),
      "Teensy bridge started on %s at %d baud. Thruster command (0.0..1.0): /wcr/thrusters/pwm, "
      "thruster data: /wcr/thrusters/data at %.1f Hz, load cells: /wcr/load_cells, "
      "tare: /wcr/load_cells/tare",
      serial_port_name_.c_str(), baud_rate_, thruster_data_rate_hz_);
  }

  ~TeensyBridgeNode() override
  {
    running_.store(false);
    if (serial_thread_.joinable()) {
      serial_thread_.join();
    }

    if (serial_fd_ >= 0) {
      close(serial_fd_);
      serial_fd_ = -1;
    }
  }

private:
  struct AckResult
  {
    bool success{false};
    std::string message;
  };

  // Latest telemetry of one ESC, as received in ESC frames.
  struct EscState
  {
    bool has_valid{false};
    std::chrono::steady_clock::time_point last_valid_time{};
    uint32_t sample_time_ms{0U};
    uint16_t erpm_raw{0U};
    uint16_t voltage_raw{0U};
    int16_t temperature{0};
    float applied_command{0.0F};
    float target_command{0.0F};
    uint16_t dshot{0U};
    uint32_t no_answer_count{0U};
    uint32_t corrupt_count{0U};
  };

  void declareParameters()
  {
    declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
    declare_parameter<int>("baud_rate", 115200);

    // HX711 counts per gram. Negative values are valid.
    declare_parameter<double>("front_calibration_factor", 1.0);
    declare_parameter<double>("rear_calibration_factor", 1.0);

    // Thruster commands are fractions of full throttle, 0.0 .. 1.0.
    declare_parameter<double>("thruster_min_command", 0.0);
    declare_parameter<double>("thruster_max_command", 1.0);
    declare_parameter<double>("startup_thruster_command", 0.0);
    declare_parameter<int>("teensy_command_timeout_ms", 500);

    // Controlled decrease: one step every interval. step=0 disables ramping.
    // 0.001 every 9 ms is the slope of the old firmware (1/180 every 50 ms).
    declare_parameter<double>("ramp_down_step_command", 0.001);
    declare_parameter<int>("ramp_down_interval_ms", 9);

    // Thruster data: publish rate and how old an ESC answer may be to count as valid.
    declare_parameter<double>("thruster_data_rate_hz", 50.0);
    declare_parameter<int>("thruster_data_timeout_ms", 500);

    // Conversion of the raw Teensy values.
    //   rpm     = erpm_raw * 100 / motor_pole_pairs
    //   voltage = voltage_raw * esc_voltage_scale
    //   current = current_adc * current_amps_per_adc_count + current_offset_amps
    declare_parameter<int>("motor_pole_pairs", 7);
    declare_parameter<double>("esc_voltage_scale", 0.01);
    declare_parameter<double>("current_amps_per_adc_count", 0.04438);
    declare_parameter<double>("current_offset_amps", -0.666);

    declare_parameter<int>("ros_command_timeout_ms", 500);
    declare_parameter<double>("command_rate_hz", 20.0);
    declare_parameter<int>("tare_service_timeout_ms", 10000);
  }

  void readParameters()
  {
    serial_port_name_ = get_parameter("serial_port").as_string();
    baud_rate_ = static_cast<int>(get_parameter("baud_rate").as_int());

    front_calibration_factor_ = get_parameter("front_calibration_factor").as_double();
    rear_calibration_factor_ = get_parameter("rear_calibration_factor").as_double();

    thruster_min_command_ = get_parameter("thruster_min_command").as_double();
    thruster_max_command_ = get_parameter("thruster_max_command").as_double();
    startup_thruster_command_ = get_parameter("startup_thruster_command").as_double();
    teensy_command_timeout_ms_ =
      static_cast<int>(get_parameter("teensy_command_timeout_ms").as_int());
    ramp_down_step_command_ = get_parameter("ramp_down_step_command").as_double();
    ramp_down_interval_ms_ =
      static_cast<int>(get_parameter("ramp_down_interval_ms").as_int());

    ros_command_timeout_ms_ =
      static_cast<int>(get_parameter("ros_command_timeout_ms").as_int());
    command_rate_hz_ = get_parameter("command_rate_hz").as_double();
    tare_service_timeout_ms_ =
      static_cast<int>(get_parameter("tare_service_timeout_ms").as_int());

    thruster_data_rate_hz_ = get_parameter("thruster_data_rate_hz").as_double();
    thruster_data_timeout_ms_ =
      static_cast<int>(get_parameter("thruster_data_timeout_ms").as_int());
    motor_pole_pairs_ = static_cast<int>(get_parameter("motor_pole_pairs").as_int());
    esc_voltage_scale_ = get_parameter("esc_voltage_scale").as_double();
    current_amps_per_adc_count_ = get_parameter("current_amps_per_adc_count").as_double();
    current_offset_amps_ = get_parameter("current_offset_amps").as_double();

    requested_pwm_.fill(static_cast<float>(startup_thruster_command_));
  }

  void validateParametersOrThrow() const
  {
    if (serial_port_name_.empty()) {
      throw std::invalid_argument("serial_port must not be empty");
    }

    static_cast<void>(baudToTermios(baud_rate_));

    if (!validCalibrationFactor(front_calibration_factor_) ||
      !validCalibrationFactor(rear_calibration_factor_))
    {
      throw std::invalid_argument(
              "front_calibration_factor and rear_calibration_factor must be finite and non-zero");
    }

    const std::string thruster_error = thrusterConfigurationError(
      thruster_min_command_, thruster_max_command_, startup_thruster_command_,
      ramp_down_step_command_);
    if (!thruster_error.empty()) {
      throw std::invalid_argument(thruster_error);
    }

    const std::string conversion_error = conversionError(
      motor_pole_pairs_, esc_voltage_scale_, current_amps_per_adc_count_,
      current_offset_amps_, thruster_data_timeout_ms_);
    if (!conversion_error.empty()) {
      throw std::invalid_argument(conversion_error);
    }

    if (!std::isfinite(thruster_data_rate_hz_) || thruster_data_rate_hz_ <= 0.0 ||
      thruster_data_rate_hz_ > 1000.0)
    {
      throw std::invalid_argument("thruster_data_rate_hz must be in (0, 1000]");
    }

    if (teensy_command_timeout_ms_ < 50 || teensy_command_timeout_ms_ > 10000) {
      throw std::invalid_argument("teensy_command_timeout_ms must be in [50, 10000]");
    }

    if (ramp_down_interval_ms_ < 1 || ramp_down_interval_ms_ > 10000) {
      throw std::invalid_argument("ramp_down_interval_ms must be in [1, 10000]");
    }

    if (ros_command_timeout_ms_ < 50 || ros_command_timeout_ms_ > 10000) {
      throw std::invalid_argument("ros_command_timeout_ms must be in [50, 10000]");
    }

    if (!std::isfinite(command_rate_hz_) || command_rate_hz_ <= 0.0 ||
      command_rate_hz_ > 200.0)
    {
      throw std::invalid_argument("command_rate_hz must be in (0, 200]");
    }

    if (tare_service_timeout_ms_ < 1000 || tare_service_timeout_ms_ > 30000) {
      throw std::invalid_argument("tare_service_timeout_ms must be in [1000, 30000]");
    }
  }

  static bool validCalibrationFactor(double factor)
  {
    return std::isfinite(factor) && std::abs(factor) > 1.0e-12;
  }

  // Returns an empty string when the thruster limits and ramp step are usable.
  static std::string thrusterConfigurationError(
    double min_command, double max_command, double startup_command, double ramp_down_step)
  {
    if (!std::isfinite(min_command) || !std::isfinite(max_command) ||
      min_command < 0.0 || max_command > 1.0 || min_command >= max_command)
    {
      return "Thruster command limits must satisfy 0.0 <= min < max <= 1.0";
    }

    if (!std::isfinite(startup_command) || startup_command < min_command ||
      startup_command > max_command)
    {
      return "startup_thruster_command must be within the command limits";
    }

    // The Teensy resolves 0.001. A smaller non-zero step would mean "no ramp" there.
    if (!std::isfinite(ramp_down_step) || ramp_down_step > 1.0 ||
      (ramp_down_step != 0.0 && ramp_down_step < kThrusterCommandResolution))
    {
      return "ramp_down_step_command must be 0.0 (no ramp) or in [0.001, 1.0]";
    }

    return "";
  }

  // Returns an empty string when the conversion parameters are usable.
  static std::string conversionError(
    int pole_pairs, double voltage_scale, double amps_per_adc_count, double offset_amps,
    int data_timeout_ms)
  {
    if (pole_pairs < 1 || pole_pairs > 100) {
      return "motor_pole_pairs must be in [1, 100]";
    }

    if (!std::isfinite(voltage_scale) || voltage_scale <= 0.0) {
      return "esc_voltage_scale must be finite and positive";
    }

    if (!std::isfinite(amps_per_adc_count) || !std::isfinite(offset_amps)) {
      return "current_amps_per_adc_count and current_offset_amps must be finite";
    }

    if (data_timeout_ms < 10 || data_timeout_ms > 10000) {
      return "thruster_data_timeout_ms must be in [10, 10000]";
    }

    return "";
  }

  void openSerialPort()
  {
    serial_fd_ = open(serial_port_name_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
      throw std::runtime_error(
              "Cannot open " + serial_port_name_ + ": " + std::strerror(errno));
    }

    termios tty{};
    if (tcgetattr(serial_fd_, &tty) != 0) {
      const std::string error = std::strerror(errno);
      close(serial_fd_);
      serial_fd_ = -1;
      throw std::runtime_error("tcgetattr failed: " + error);
    }

    cfmakeraw(&tty);
    const speed_t speed = baudToTermios(baud_rate_);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
    tty.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
    tty.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
    tty.c_cflag &= static_cast<tcflag_t>(~PARENB);
    tty.c_cflag &= static_cast<tcflag_t>(~CSIZE);
    tty.c_cflag |= CS8;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
      const std::string error = std::strerror(errno);
      close(serial_fd_);
      serial_fd_ = -1;
      throw std::runtime_error("tcsetattr failed: " + error);
    }

    tcflush(serial_fd_, TCIOFLUSH);
  }

  uint32_t nextCommandSequence()
  {
    return command_sequence_.fetch_add(1U);
  }

  void enqueuePayload(const std::string & payload, bool coalesce_pwm = false)
  {
    const std::string frame = framePayload(payload);
    std::lock_guard<std::mutex> lock(tx_mutex_);

    if (coalesce_pwm) {
      tx_queue_.erase(
        std::remove_if(
          tx_queue_.begin(), tx_queue_.end(),
          [](const std::string & queued_frame) {
            return queued_frame.rfind("PWM,", 0) == 0;
          }),
        tx_queue_.end());
    }

    constexpr std::size_t kMaximumQueueSize = 100;
    if (tx_queue_.size() >= kMaximumQueueSize) {
      tx_queue_.pop_front();
    }
    tx_queue_.push_back(frame);
  }

  bool writeAll(const std::string & frame)
  {
    std::size_t written = 0;

    while (written < frame.size() && running_.load()) {
      const ssize_t result = write(
        serial_fd_, frame.data() + written, frame.size() - written);

      if (result > 0) {
        written += static_cast<std::size_t>(result);
        continue;
      }

      if (result < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
        RCLCPP_ERROR(get_logger(), "Serial write failed: %s", std::strerror(errno));
        return false;
      }

      pollfd descriptor{};
      descriptor.fd = serial_fd_;
      descriptor.events = POLLOUT;
      static_cast<void>(poll(&descriptor, 1, 20));
    }

    return written == frame.size();
  }

  void serialWorker()
  {
    std::string receive_buffer;
    receive_buffer.reserve(8192);
    std::array<char, 4096> temporary_buffer{};

    while (running_.load()) {
      std::deque<std::string> local_queue;
      {
        std::lock_guard<std::mutex> lock(tx_mutex_);
        local_queue.swap(tx_queue_);
      }

      for (const auto & frame : local_queue) {
        if (!writeAll(frame)) {
          break;
        }
      }

      pollfd descriptor{};
      descriptor.fd = serial_fd_;
      descriptor.events = POLLIN;

      const int poll_result = poll(&descriptor, 1, 10);
      if (poll_result < 0) {
        if (errno != EINTR) {
          RCLCPP_ERROR(get_logger(), "Serial poll failed: %s", std::strerror(errno));
        }
        continue;
      }

      if (poll_result == 0 || (descriptor.revents & POLLIN) == 0) {
        continue;
      }

      const ssize_t bytes_read = read(
        serial_fd_, temporary_buffer.data(), temporary_buffer.size());

      if (bytes_read < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
          RCLCPP_ERROR(get_logger(), "Serial read failed: %s", std::strerror(errno));
        }
        continue;
      }

      receive_buffer.append(temporary_buffer.data(), static_cast<std::size_t>(bytes_read));

      std::size_t newline_position = std::string::npos;
      while ((newline_position = receive_buffer.find('\n')) != std::string::npos) {
        std::string line = receive_buffer.substr(0, newline_position);
        receive_buffer.erase(0, newline_position + 1);

        if (!line.empty() && line.back() == '\r') {
          line.pop_back();
        }
        if (!line.empty()) {
          processSerialLine(line);
        }
      }

      if (receive_buffer.size() > kMaximumSerialLineLength) {
        RCLCPP_WARN(get_logger(), "Discarding oversized/incomplete serial frame");
        receive_buffer.clear();
      }
    }
  }

  bool extractAndValidatePayload(const std::string & line, std::string & payload) const
  {
    const std::size_t separator = line.rfind('*');
    if (separator == std::string::npos || line.size() - separator - 1 != 4) {
      return false;
    }

    payload = line.substr(0, separator);
    const std::string crc_text = line.substr(separator + 1);

    try {
      std::size_t consumed = 0;
      const unsigned long received_crc = std::stoul(crc_text, &consumed, 16);
      return consumed == crc_text.size() &&
             static_cast<uint16_t>(received_crc) == crc16Ccitt(payload);
    } catch (...) {
      return false;
    }
  }

  void processSerialLine(const std::string & line)
  {
    std::string payload;
    if (!extractAndValidatePayload(line, payload)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Discarding serial frame with invalid format or CRC");
      return;
    }

    const auto fields = splitCsv(payload);
    if (fields.empty()) {
      return;
    }

    if (fields[0] == "DATA") {
      processDataFrame(fields);
    } else if (fields[0] == "ESC") {
      processEscFrame(fields);
    } else if (fields[0] == "ACK") {
      processAckFrame(fields);
    } else if (fields[0] == "ERR") {
      processErrorFrame(fields);
    } else if (fields[0] == "STATUS") {
      RCLCPP_INFO(get_logger(), "Teensy status: %s", payload.c_str());
    }
  }

  void processDataFrame(const std::vector<std::string> & fields)
  {
    // DATA,sequence,millis,front_adc,rear_adc,front_tared,rear_tared,
    // front_g,rear_g,communication_timeout,error_flags
    // The thruster commands are in the ESC frame.
    if (fields.size() != 11) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Unexpected DATA field count: %zu", fields.size());
      return;
    }

    uint32_t sequence = 0;
    uint32_t teensy_time_ms = 0;
    int32_t front_raw_adc = 0;
    int32_t rear_raw_adc = 0;
    int32_t front_raw_tared = 0;
    int32_t rear_raw_tared = 0;
    float front_grams = 0.0F;
    float rear_grams = 0.0F;
    uint32_t communication_timeout = 0;
    uint32_t error_flags = 0;

    if (!parseUnsigned(fields[1], sequence) ||
      !parseUnsigned(fields[2], teensy_time_ms) ||
      !parseSigned(fields[3], front_raw_adc) ||
      !parseSigned(fields[4], rear_raw_adc) ||
      !parseSigned(fields[5], front_raw_tared) ||
      !parseSigned(fields[6], rear_raw_tared) ||
      !parseFloat(fields[7], front_grams) ||
      !parseFloat(fields[8], rear_grams) ||
      !parseUnsigned(fields[9], communication_timeout) ||
      !parseUnsigned(fields[10], error_flags) ||
      communication_timeout > 1U ||
      error_flags > std::numeric_limits<uint16_t>::max())
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Invalid DATA values");
      return;
    }

    auto message = wcr_interfaces::msg::LoadCellData();
    message.header.stamp = now();
    message.header.frame_id = "load_cell_frame";
    message.sequence = sequence;
    message.teensy_time_ms = teensy_time_ms;
    message.front_raw_adc = front_raw_adc;
    message.rear_raw_adc = rear_raw_adc;
    message.front_raw_tared = front_raw_tared;
    message.rear_raw_tared = rear_raw_tared;
    message.front_grams = front_grams;
    message.rear_grams = rear_grams;
    message.total_grams = front_grams + rear_grams;

    message.communication_timeout = communication_timeout != 0U;
    message.error_flags = static_cast<uint16_t>(error_flags);

    if (message.error_flags != last_teensy_error_flags_) {
      if (message.error_flags == 0U) {
        RCLCPP_INFO(get_logger(), "Teensy reports that all monitored conditions are OK");
      } else {
        RCLCPP_WARN(
          get_logger(), "Teensy error flags changed: 0x%04X",
          static_cast<unsigned int>(message.error_flags));
      }
      last_teensy_error_flags_ = message.error_flags;
    }

    load_cell_publisher_->publish(message);
  }

  void processEscFrame(const std::vector<std::string> & fields)
  {
    // ESC,sequence,millis,motor,status,temp,voltage,current,consumption,erpm,
    //     applied,target,dshot,current_adc
    // One frame per telemetry request. temp..erpm are exactly as the ESC sent
    // them and are 0 unless status is 1 (valid answer). applied and target are
    // the thruster commands of this motor as fractions 0.000..1.000.
    if (fields.size() != 14) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Unexpected ESC field count: %zu", fields.size());
      return;
    }

    uint32_t sequence = 0;
    uint32_t teensy_time_ms = 0;
    uint32_t motor = 0;
    uint32_t status = 0;
    uint32_t temperature = 0;
    uint32_t voltage_raw = 0;
    uint32_t esc_current_raw = 0;
    uint32_t consumption_raw = 0;
    uint32_t erpm_raw = 0;
    float applied_command = 0.0F;
    float target_command = 0.0F;
    uint32_t dshot = 0;
    uint32_t current_adc = 0;

    if (!parseUnsigned(fields[1], sequence) ||
      !parseUnsigned(fields[2], teensy_time_ms) ||
      !parseUnsigned(fields[3], motor) ||
      !parseUnsigned(fields[4], status) ||
      !parseUnsigned(fields[5], temperature) ||
      !parseUnsigned(fields[6], voltage_raw) ||
      !parseUnsigned(fields[7], esc_current_raw) ||
      !parseUnsigned(fields[8], consumption_raw) ||
      !parseUnsigned(fields[9], erpm_raw) ||
      !parseFloat(fields[10], applied_command) ||
      !parseFloat(fields[11], target_command) ||
      !parseUnsigned(fields[12], dshot) ||
      !parseUnsigned(fields[13], current_adc) ||
      motor < 1U || motor > kThrusterCount || status > kEscStatusCorrupt ||
      temperature > 255U || voltage_raw > std::numeric_limits<uint16_t>::max() ||
      erpm_raw > std::numeric_limits<uint16_t>::max() ||
      applied_command < 0.0F || applied_command > 1.0F ||
      target_command < 0.0F || target_command > 1.0F ||
      dshot > 2047U || current_adc > 4095U)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Invalid ESC values");
      return;
    }

    std::lock_guard<std::mutex> lock(thruster_mutex_);
    EscState & esc = esc_states_[motor - 1U];

    // The commands, the DShot value and the current sample come from the
    // Teensy itself, so they are usable whatever the ESC answered.
    esc.applied_command = applied_command;
    esc.target_command = target_command;
    esc.dshot = static_cast<uint16_t>(dshot);
    current_adc_sum_ += current_adc;
    ++current_adc_samples_;
    newest_esc_teensy_time_ms_ = teensy_time_ms;
    received_esc_frame_ = true;

    if (status == kEscStatusValid) {
      esc.has_valid = true;
      esc.last_valid_time = std::chrono::steady_clock::now();
      esc.sample_time_ms = teensy_time_ms;
      esc.erpm_raw = static_cast<uint16_t>(erpm_raw);
      esc.voltage_raw = static_cast<uint16_t>(voltage_raw);
      esc.temperature = static_cast<int16_t>(temperature);
    } else if (status == kEscStatusNoAnswer) {
      ++esc.no_answer_count;
    } else {
      ++esc.corrupt_count;
    }
  }

  // Publishes the newest data of all four motors at thruster_data_rate_hz.
  void thrusterDataTimerCallback()
  {
    int pole_pairs;
    double voltage_scale;
    double amps_per_adc_count;
    double offset_amps;
    int data_timeout_ms;
    {
      std::lock_guard<std::mutex> lock(configuration_mutex_);
      pole_pairs = motor_pole_pairs_;
      voltage_scale = esc_voltage_scale_;
      amps_per_adc_count = current_amps_per_adc_count_;
      offset_amps = current_offset_amps_;
      data_timeout_ms = thruster_data_timeout_ms_;
    }

    auto message = wcr_interfaces::msg::ThrusterData();

    {
      std::lock_guard<std::mutex> lock(thruster_mutex_);
      if (!received_esc_frame_) {
        return;  // nothing from the Teensy yet
      }

      const auto steady_now = std::chrono::steady_clock::now();
      message.teensy_time_ms = newest_esc_teensy_time_ms_;

      for (std::size_t index = 0; index < kThrusterCount; ++index) {
        const EscState & esc = esc_states_[index];
        const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
          steady_now - esc.last_valid_time).count();

        message.valid[index] = esc.has_valid && age_ms <= data_timeout_ms;
        message.sample_time_ms[index] = esc.sample_time_ms;
        message.target_command[index] = esc.target_command;
        message.applied_command[index] = esc.applied_command;
        message.dshot[index] = esc.dshot;
        message.rpm[index] = static_cast<float>(
          static_cast<double>(esc.erpm_raw) * 100.0 / static_cast<double>(pole_pairs));
        message.voltage[index] = static_cast<float>(
          static_cast<double>(esc.voltage_raw) * voltage_scale);
        message.temperature[index] = esc.temperature;
        message.erpm_raw[index] = esc.erpm_raw;
        message.voltage_raw[index] = esc.voltage_raw;
        message.no_answer_count[index] = esc.no_answer_count;
        message.corrupt_count[index] = esc.corrupt_count;
      }

      // Average the current samples that arrived since the previous message.
      if (current_adc_samples_ > 0U) {
        last_current_adc_mean_ =
          static_cast<double>(current_adc_sum_) / static_cast<double>(current_adc_samples_);
      }
      message.current_samples = static_cast<uint16_t>(
        std::min<uint32_t>(current_adc_samples_, std::numeric_limits<uint16_t>::max()));
      message.current_adc = static_cast<float>(last_current_adc_mean_);
      current_adc_sum_ = 0U;
      current_adc_samples_ = 0U;
    }

    // A reading below the zero point of the calibration is reported as 0 A.
    message.current = static_cast<float>(std::max(
        0.0, static_cast<double>(message.current_adc) * amps_per_adc_count + offset_amps));

    message.header.stamp = now();
    message.header.frame_id = "thruster_frame";
    thruster_data_publisher_->publish(message);
  }

  void processAckFrame(const std::vector<std::string> & fields)
  {
    if (fields.size() != 4) {
      return;
    }

    uint32_t sequence = 0;
    if (!parseUnsigned(fields[2], sequence)) {
      return;
    }

    if (fields[1] == "TARE") {
      {
        std::lock_guard<std::mutex> lock(ack_mutex_);
        ack_results_[sequence] = AckResult{fields[3] == "OK", fields[3]};
      }
      ack_condition_.notify_all();
    } else {
      RCLCPP_DEBUG(
        get_logger(), "Teensy ACK: command=%s sequence=%u status=%s",
        fields[1].c_str(), sequence, fields[3].c_str());
    }
  }

  void processErrorFrame(const std::vector<std::string> & fields)
  {
    if (fields.size() != 4) {
      return;
    }

    uint32_t sequence = 0;
    if (!parseUnsigned(fields[2], sequence)) {
      return;
    }

    if (fields[1] == "TARE") {
      {
        std::lock_guard<std::mutex> lock(ack_mutex_);
        ack_results_[sequence] = AckResult{false, fields[3]};
      }
      ack_condition_.notify_all();
    }

    RCLCPP_ERROR(
      get_logger(), "Teensy error: command=%s sequence=%u reason=%s",
      fields[1].c_str(), sequence, fields[3].c_str());
  }

  void pwmCallback(const std_msgs::msg::Float32MultiArray::SharedPtr message)
  {
    if (message->data.size() != kThrusterCount) {
      RCLCPP_ERROR(
        get_logger(), "Expected exactly %zu thruster command values, received %zu",
        kThrusterCount, message->data.size());
      return;
    }

    for (const float value : message->data) {
      if (!std::isfinite(value)) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Ignoring thruster command with a non-finite value");
        return;
      }
    }

    std::array<float, kThrusterCount> new_pwm{};
    bool clamped = false;

    {
      std::lock_guard<std::mutex> lock(configuration_mutex_);
      for (std::size_t index = 0; index < kThrusterCount; ++index) {
        const double value = static_cast<double>(message->data[index]);
        const double safe_value = std::clamp(value, thruster_min_command_, thruster_max_command_);
        new_pwm[index] = static_cast<float>(safe_value);
        clamped = clamped || safe_value != value;
      }
    }

    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      requested_pwm_ = new_pwm;
      last_ros_command_time_ = std::chrono::steady_clock::now();
      received_ros_command_ = true;
    }

    if (clamped) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "At least one thruster command was clamped to configured limits");
    }
  }

  void commandTimerCallback()
  {
    std::array<float, kThrusterCount> pwm{};
    bool source_stale = true;
    int ros_timeout_ms = 500;

    {
      std::lock_guard<std::mutex> lock(configuration_mutex_);
      ros_timeout_ms = ros_command_timeout_ms_;
    }

    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      if (!received_ros_command_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "No ROS thruster command received yet; not overwriting Teensy thruster outputs");
        return;
      }

      const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - last_ros_command_time_).count();
      source_stale = age > ros_timeout_ms;

      // Once at least one real ROS command has arrived, keep resending the last
      // request. A stale topic is reported but does not force startup PWM.
      pwm = requested_pwm_;
    }

    if (source_stale) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Thruster command topic is stale; holding and resending the last command values");
    }

    const uint32_t sequence = nextCommandSequence();
    std::ostringstream payload;
    payload << "PWM," << sequence;
    for (const float value : pwm) {
      payload << ',' << formatFraction(static_cast<double>(value));
    }
    payload << ',' << (source_stale ? 1 : 0);

    enqueuePayload(payload.str(), true);
  }

  void tareServiceCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    const uint32_t sequence = nextCommandSequence();
    enqueuePayload("TARE," + std::to_string(sequence));

    int tare_timeout_ms = 10000;
    {
      std::lock_guard<std::mutex> config_lock(configuration_mutex_);
      tare_timeout_ms = tare_service_timeout_ms_;
    }

    std::unique_lock<std::mutex> lock(ack_mutex_);
    const bool received = ack_condition_.wait_for(
      lock, std::chrono::milliseconds(tare_timeout_ms),
      [this, sequence]() {return ack_results_.find(sequence) != ack_results_.end();});

    if (!received) {
      response->success = false;
      response->message = "Timed out waiting for Teensy tare acknowledgement";
      return;
    }

    const AckResult result = ack_results_.at(sequence);
    ack_results_.erase(sequence);
    response->success = result.success;
    response->message = result.success ? "Both load cells were tared" : result.message;
  }

  rcl_interfaces::msg::SetParametersResult parameterCallback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;

    double new_front_factor;
    double new_rear_factor;
    double new_pwm_min;
    double new_pwm_max;
    double new_startup_pwm;
    int new_teensy_timeout;
    double new_ramp_down_step;
    int new_ramp_down_interval;
    int new_ros_timeout;
    int new_tare_timeout;
    int new_data_timeout;
    int new_pole_pairs;
    double new_voltage_scale;
    double new_amps_per_adc_count;
    double new_offset_amps;

    {
      std::lock_guard<std::mutex> lock(configuration_mutex_);
      new_front_factor = front_calibration_factor_;
      new_rear_factor = rear_calibration_factor_;
      new_pwm_min = thruster_min_command_;
      new_pwm_max = thruster_max_command_;
      new_startup_pwm = startup_thruster_command_;
      new_teensy_timeout = teensy_command_timeout_ms_;
      new_ramp_down_step = ramp_down_step_command_;
      new_ramp_down_interval = ramp_down_interval_ms_;
      new_ros_timeout = ros_command_timeout_ms_;
      new_tare_timeout = tare_service_timeout_ms_;
      new_data_timeout = thruster_data_timeout_ms_;
      new_pole_pairs = motor_pole_pairs_;
      new_voltage_scale = esc_voltage_scale_;
      new_amps_per_adc_count = current_amps_per_adc_count_;
      new_offset_amps = current_offset_amps_;
    }

    bool calibration_changed = false;
    bool pwm_configuration_changed = false;

    for (const auto & parameter : parameters) {
      const std::string & name = parameter.get_name();

      if (name == "serial_port" || name == "baud_rate" || name == "command_rate_hz" ||
        name == "thruster_data_rate_hz")
      {
        result.reason = name + " cannot be changed while the node is running";
        return result;
      }

      try {
        if (name == "front_calibration_factor") {
          new_front_factor = parameter.as_double();
          calibration_changed = true;
        } else if (name == "rear_calibration_factor") {
          new_rear_factor = parameter.as_double();
          calibration_changed = true;
        } else if (name == "thruster_min_command") {
          new_pwm_min = parameter.as_double();
          pwm_configuration_changed = true;
        } else if (name == "thruster_max_command") {
          new_pwm_max = parameter.as_double();
          pwm_configuration_changed = true;
        } else if (name == "startup_thruster_command") {
          new_startup_pwm = parameter.as_double();
          pwm_configuration_changed = true;
        } else if (name == "teensy_command_timeout_ms") {
          new_teensy_timeout = static_cast<int>(parameter.as_int());
          pwm_configuration_changed = true;
        } else if (name == "ramp_down_step_command") {
          new_ramp_down_step = parameter.as_double();
          pwm_configuration_changed = true;
        } else if (name == "ramp_down_interval_ms") {
          new_ramp_down_interval = static_cast<int>(parameter.as_int());
          pwm_configuration_changed = true;
        } else if (name == "ros_command_timeout_ms") {
          new_ros_timeout = static_cast<int>(parameter.as_int());
        } else if (name == "tare_service_timeout_ms") {
          new_tare_timeout = static_cast<int>(parameter.as_int());
        } else if (name == "thruster_data_timeout_ms") {
          new_data_timeout = static_cast<int>(parameter.as_int());
        } else if (name == "motor_pole_pairs") {
          new_pole_pairs = static_cast<int>(parameter.as_int());
        } else if (name == "esc_voltage_scale") {
          new_voltage_scale = parameter.as_double();
        } else if (name == "current_amps_per_adc_count") {
          new_amps_per_adc_count = parameter.as_double();
        } else if (name == "current_offset_amps") {
          new_offset_amps = parameter.as_double();
        }
      } catch (const rclcpp::ParameterTypeException & exception) {
        result.reason = exception.what();
        return result;
      }
    }

    if (!validCalibrationFactor(new_front_factor) ||
      !validCalibrationFactor(new_rear_factor))
    {
      result.reason = "Calibration factors must be finite and non-zero";
      return result;
    }

    const std::string thruster_error = thrusterConfigurationError(
      new_pwm_min, new_pwm_max, new_startup_pwm, new_ramp_down_step);
    if (!thruster_error.empty()) {
      result.reason = thruster_error;
      return result;
    }

    const std::string conversion_error = conversionError(
      new_pole_pairs, new_voltage_scale, new_amps_per_adc_count, new_offset_amps,
      new_data_timeout);
    if (!conversion_error.empty()) {
      result.reason = conversion_error;
      return result;
    }

    if (new_teensy_timeout < 50 || new_teensy_timeout > 10000 ||
      new_ros_timeout < 50 || new_ros_timeout > 10000)
    {
      result.reason = "Command timeouts must be in [50, 10000] ms";
      return result;
    }

    if (new_ramp_down_interval < 1 || new_ramp_down_interval > 10000) {
      result.reason = "ramp_down_interval_ms must be in [1, 10000]";
      return result;
    }

    if (new_tare_timeout < 1000 || new_tare_timeout > 30000) {
      result.reason = "tare_service_timeout_ms must be in [1000, 30000]";
      return result;
    }

    {
      std::lock_guard<std::mutex> lock(configuration_mutex_);
      front_calibration_factor_ = new_front_factor;
      rear_calibration_factor_ = new_rear_factor;
      thruster_min_command_ = new_pwm_min;
      thruster_max_command_ = new_pwm_max;
      startup_thruster_command_ = new_startup_pwm;
      teensy_command_timeout_ms_ = new_teensy_timeout;
      ramp_down_step_command_ = new_ramp_down_step;
      ramp_down_interval_ms_ = new_ramp_down_interval;
      ros_command_timeout_ms_ = new_ros_timeout;
      tare_service_timeout_ms_ = new_tare_timeout;
      thruster_data_timeout_ms_ = new_data_timeout;
      motor_pole_pairs_ = new_pole_pairs;
      esc_voltage_scale_ = new_voltage_scale;
      current_amps_per_adc_count_ = new_amps_per_adc_count;
      current_offset_amps_ = new_offset_amps;
    }

    if (pwm_configuration_changed) {
      std::lock_guard<std::mutex> command_lock(command_mutex_);
      if (!received_ros_command_) {
        requested_pwm_.fill(static_cast<float>(new_startup_pwm));
      }
    }

    if (calibration_changed) {
      sendCalibrationConfiguration();
    }
    if (pwm_configuration_changed) {
      sendPwmConfiguration();
    }

    result.successful = true;
    result.reason = "Parameters accepted";
    return result;
  }

  void sendCalibrationConfiguration()
  {
    double front_factor;
    double rear_factor;
    {
      std::lock_guard<std::mutex> lock(configuration_mutex_);
      front_factor = front_calibration_factor_;
      rear_factor = rear_calibration_factor_;
    }

    const uint32_t sequence = nextCommandSequence();
    std::ostringstream payload;
    payload << std::setprecision(12) << "CAL," << sequence << ','
            << front_factor << ',' << rear_factor;
    enqueuePayload(payload.str());
  }

  void sendPwmConfiguration()
  {
    double pwm_min;
    double pwm_max;
    double startup_pwm;
    int timeout_ms;
    double ramp_down_step;
    int ramp_down_interval;
    {
      std::lock_guard<std::mutex> lock(configuration_mutex_);
      pwm_min = thruster_min_command_;
      pwm_max = thruster_max_command_;
      startup_pwm = startup_thruster_command_;
      timeout_ms = teensy_command_timeout_ms_;
      ramp_down_step = ramp_down_step_command_;
      ramp_down_interval = ramp_down_interval_ms_;
    }

    const uint32_t sequence = nextCommandSequence();
    std::ostringstream payload;
    // CFG,sequence,min,max,startup,timeout_ms,ramp_step,ramp_interval_ms
    // min, max, startup and ramp_step are fractions of full throttle.
    payload << "CFG," << sequence << ',' << formatFraction(pwm_min) << ','
            << formatFraction(pwm_max) << ',' << formatFraction(startup_pwm) << ','
            << timeout_ms << ',' << formatFraction(ramp_down_step) << ','
            << ramp_down_interval;
    enqueuePayload(payload.str());
  }

  std::string serial_port_name_;
  int baud_rate_{115200};

  std::mutex configuration_mutex_;
  double front_calibration_factor_{1.0};
  double rear_calibration_factor_{1.0};
  double thruster_min_command_{0.0};
  double thruster_max_command_{1.0};
  double startup_thruster_command_{0.0};
  int teensy_command_timeout_ms_{500};
  double ramp_down_step_command_{0.001};
  int ramp_down_interval_ms_{9};
  int ros_command_timeout_ms_{500};
  double command_rate_hz_{20.0};
  int tare_service_timeout_ms_{10000};
  double thruster_data_rate_hz_{50.0};
  int thruster_data_timeout_ms_{500};
  int motor_pole_pairs_{7};
  double esc_voltage_scale_{0.01};
  double current_amps_per_adc_count_{0.04438};
  double current_offset_amps_{-0.666};

  int serial_fd_{-1};
  std::atomic<bool> running_{false};
  std::thread serial_thread_;

  std::mutex tx_mutex_;
  std::deque<std::string> tx_queue_;
  std::atomic<uint32_t> command_sequence_{1U};

  std::mutex command_mutex_;
  std::array<float, kThrusterCount> requested_pwm_{};
  std::chrono::steady_clock::time_point last_ros_command_time_{};
  bool received_ros_command_{false};
  uint16_t last_teensy_error_flags_{0U};

  // Thruster telemetry, written by the serial thread and read by the publish timer.
  std::mutex thruster_mutex_;
  std::array<EscState, kThrusterCount> esc_states_{};
  uint64_t current_adc_sum_{0U};
  uint32_t current_adc_samples_{0U};
  double last_current_adc_mean_{0.0};
  uint32_t newest_esc_teensy_time_ms_{0U};
  bool received_esc_frame_{false};

  std::mutex ack_mutex_;
  std::condition_variable ack_condition_;
  std::unordered_map<uint32_t, AckResult> ack_results_;

  rclcpp::Publisher<wcr_interfaces::msg::LoadCellData>::SharedPtr
    load_cell_publisher_;
  rclcpp::Publisher<wcr_interfaces::msg::ThrusterData>::SharedPtr
    thruster_data_publisher_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr pwm_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr tare_service_;
  rclcpp::TimerBase::SharedPtr command_timer_;
  rclcpp::TimerBase::SharedPtr thruster_data_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<TeensyBridgeNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("teensy_bridge"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}