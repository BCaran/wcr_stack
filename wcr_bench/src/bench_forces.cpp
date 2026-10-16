// WCR bench - force sensor node.
//
// Reads raw HX711 counts from the Arduino over USB and publishes three
// views of the same samples:
//
//   /bench/raw          std_msgs/Int32MultiArray     raw ADC counts
//   /bench/raw_tared    std_msgs/Int32MultiArray     counts minus offset
//   /bench/forces       std_msgs/Float32MultiArray   grams
//   /bench/forces_n     std_msgs/Float32MultiArray   newtons
//
// All four carry four values ordered FL, BL, BR, FR, and all four are
// derived from the same sample, so they line up index by index in a bag.
//
// Newtons are grams / 1000 * g. The local value of g is a parameter:
// 9.80665 is the standard value, but if you want force figures accurate
// to better than a part in a thousand, use the local value for the lab.
//
// Services:
//   /bench/tare         std_srvs/Trigger   average N samples, store as offset
//
// Configuration comes from a YAML params file - serial port, calibration
// factors, tare depth. Nothing is compiled in, so recalibrating or moving
// to another machine needs no rebuild.
//
// On sample rate: the HX711 converts at ~10 SPS, so samples arrive about
// 91 ms apart and the topics run at ~11 Hz. A stuck data line makes the
// board's read() return immediately with values that look plausible but
// are garbage. The node watches the board's own millis() for exactly
// that, and warns when the cadence leaves the expected window.

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace
{
constexpr size_t N_CELLS = 4;
constexpr long NO_ANSWER = 99999999L;

const char * CELL_NAMES[N_CELLS] = {"fl", "bl", "br", "fr"};
}  // namespace

class BenchForces : public rclcpp::Node
{
public:
  BenchForces()
  : Node("bench_forces")
  {
    port_ = this->declare_parameter<std::string>("port", "/dev/ttyACM0");
    baudrate_ = this->declare_parameter<int>("baudrate", 115200);
    frame_id_ = this->declare_parameter<std::string>("frame_id", "bench");
    tare_samples_ = this->declare_parameter<int>("tare_samples", 30);
    tare_on_start_ = this->declare_parameter<bool>("tare_on_start", false);
    console_ = this->declare_parameter<bool>("console", true);
    console_period_ms_ = this->declare_parameter<int>("console_period_ms", 500);
    gravity_ = this->declare_parameter<double>("gravity", 9.80665);

    // Expected gap between samples. Defaults suit the HX711's 10 SPS
    // setting (~91 ms measured). At 80 SPS the gap drops to ~12 ms, so
    // these must be widened or the node warns on every sample.
    dt_min_ms_ = this->declare_parameter<double>("dt_min_ms", 40.0);
    dt_max_ms_ = this->declare_parameter<double>("dt_max_ms", 400.0);

    // Serial poll period. Must stay well below the sample period:
    // 10 ms is fine at 10 SPS, but at 80 SPS (~12 ms) it leaves no
    // margin, so 2 ms is the safer default.
    poll_period_ms_ = this->declare_parameter<int>("poll_period_ms", 10);

    // No usable defaults on purpose: a wrong-but-plausible factor is far
    // worse than a loud complaint at startup, so the YAML must supply them.
    for (size_t i = 0; i < N_CELLS; ++i) {
      const std::string name = std::string("calibration.") + CELL_NAMES[i];
      factors_[i] = this->declare_parameter<double>(name, 0.0);
      if (factors_[i] == 0.0) {
        RCLCPP_ERROR(this->get_logger(),
                     "calibration factor '%s' missing or zero - load a "
                     "params file, forces will be meaningless", name.c_str());
      }
    }

    offsets_.fill(0);
    tare_acc_.fill(0);

    pub_raw_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
      "/bench/raw", 50);
    pub_tared_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
      "/bench/raw_tared", 50);
    pub_forces_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "/bench/forces", 50);
    pub_newtons_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "/bench/forces_n", 50);

    srv_tare_ = this->create_service<std_srvs::srv::Trigger>(
      "/bench/tare",
      std::bind(&BenchForces::onTare, this,
                std::placeholders::_1, std::placeholders::_2));

    openPort();

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(poll_period_ms_),
      std::bind(&BenchForces::pollSerial, this));

    RCLCPP_INFO(this->get_logger(),
                "calibration [counts/g]  FL=%.2f  BL=%.2f  BR=%.2f  FR=%.2f",
                factors_[0], factors_[1], factors_[2], factors_[3]);
    RCLCPP_INFO(this->get_logger(),
                "publishing /bench/raw, /bench/raw_tared, /bench/forces [g], "
                "/bench/forces_n [N] as [FL, BL, BR, FR]");

    if (tare_on_start_) startTare();
  }

  ~BenchForces() override { closePort(); }

private:
  // -- serial -----------------------------------------------------------

  static speed_t toBaud(int b)
  {
    switch (b) {
      case 9600:   return B9600;
      case 19200:  return B19200;
      case 38400:  return B38400;
      case 57600:  return B57600;
      case 115200: return B115200;
      case 230400: return B230400;
      default:     return B115200;
    }
  }

  bool openPort()
  {
    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                            "cannot open %s: %s",
                            port_.c_str(), std::strerror(errno));
      return false;
    }

    termios tty{};
    if (::tcgetattr(fd_, &tty) != 0) { closePort(); return false; }

    ::cfmakeraw(&tty);
    const speed_t s = toBaud(baudrate_);
    ::cfsetispeed(&tty, s);
    ::cfsetospeed(&tty, s);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (::tcsetattr(fd_, TCSANOW, &tty) != 0) { closePort(); return false; }

    ::tcflush(fd_, TCIOFLUSH);
    buffer_.clear();
    prev_valid_ = false;
    RCLCPP_INFO(this->get_logger(), "opened %s at %d", port_.c_str(), baudrate_);
    return true;
  }

  void closePort()
  {
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
  }

  void pollSerial()
  {
  
    if (tare_requested_.exchange(false)) startTare();
  
    if (fd_ < 0) { openPort(); return; }

    char chunk[512];
    for (;;) {
      const ssize_t n = ::read(fd_, chunk, sizeof(chunk));
      if (n > 0) { buffer_.append(chunk, static_cast<size_t>(n)); continue; }
      if (n == 0 || errno == EAGAIN || errno == EWOULDBLOCK) break;
      RCLCPP_ERROR(this->get_logger(), "read error: %s, reconnecting",
                   std::strerror(errno));
      closePort();
      return;
    }

    // Consume whole lines only; a partial tail stays buffered.
    size_t nl;
    while ((nl = buffer_.find('\n')) != std::string::npos) {
      std::string line = buffer_.substr(0, nl);
      buffer_.erase(0, nl + 1);
      if (!line.empty() && line.back() == '\r') line.pop_back();
      if (!line.empty()) handleLine(line);
    }

    if (buffer_.size() > 4096) {
      RCLCPP_WARN(this->get_logger(), "no newline in 4 kB, dropping buffer");
      buffer_.clear();
    }
  }

  // -- parsing ----------------------------------------------------------

  void handleLine(const std::string & line)
  {
    if (line.rfind("I,", 0) == 0) {
      RCLCPP_INFO(this->get_logger(), "board: %s", line.substr(2).c_str());
      return;
    }
    if (line.rfind("E,", 0) == 0) {
      RCLCPP_ERROR(this->get_logger(), "board: %s", line.substr(2).c_str());
      return;
    }
    if (line.rfind("D,", 0) != 0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "unrecognised line: %s", line.c_str());
      return;
    }

    unsigned long board_ms = 0;
    long v[N_CELLS] = {0, 0, 0, 0};
    const int got = std::sscanf(line.c_str() + 2, "%lu,%ld,%ld,%ld,%ld",
                                &board_ms, &v[0], &v[1], &v[2], &v[3]);
    if (got != 5) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "malformed data line: %s", line.c_str());
      return;
    }

    for (size_t i = 0; i < N_CELLS; ++i) {
      if (v[i] == NO_ANSWER) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                              "cell %s is not answering", CELL_NAMES[i]);
        return;   // drop the whole sample: a partial one is worse than none
      }
    }

    checkCadence(board_ms);
    publish(v);

    if (tare_pending_) accumulateTare(v);
  }

  // Flags data that cannot be the result of real conversions.
  void checkCadence(unsigned long board_ms)
  {
    if (prev_valid_) {
      const double dt = static_cast<double>(board_ms) -
                        static_cast<double>(prev_millis_);
      if (dt < dt_min_ms_ || dt > dt_max_ms_) {
        ++bad_cadence_;
        if (bad_cadence_ == 1 || bad_cadence_ == 10 ||
            bad_cadence_ == 100 || bad_cadence_ % 500 == 0)
        {
          RCLCPP_WARN(this->get_logger(),
                      "sample cadence %.0f ms outside [%.0f, %.0f] - readings "
                      "may not be real conversions (count: %lu)",
                      dt, dt_min_ms_, dt_max_ms_, bad_cadence_);
        }
      }
    }
    prev_millis_ = board_ms;
    prev_valid_ = true;
  }

  // -- publishing -------------------------------------------------------

  void publish(const long v[N_CELLS])
  {
    std_msgs::msg::MultiArrayDimension dim;
    dim.label = "FL,BL,BR,FR";
    dim.size = N_CELLS;
    dim.stride = N_CELLS;

    std_msgs::msg::Int32MultiArray raw;
    raw.layout.dim.push_back(dim);
    raw.data.resize(N_CELLS);

    std_msgs::msg::Int32MultiArray tared;
    tared.layout.dim.push_back(dim);
    tared.data.resize(N_CELLS);

    std_msgs::msg::Float32MultiArray forces;
    forces.layout.dim.push_back(dim);
    forces.data.resize(N_CELLS);

    std_msgs::msg::Float32MultiArray newtons;
    newtons.layout.dim.push_back(dim);
    newtons.data.resize(N_CELLS);

    std::array<float, N_CELLS> grams{};
    double sum_g = 0.0;
    double sum_n = 0.0;

    for (size_t i = 0; i < N_CELLS; ++i) {
      const long t = v[i] - offsets_[i];
      raw.data[i] = static_cast<int32_t>(v[i]);
      tared.data[i] = static_cast<int32_t>(t);
      grams[i] = (factors_[i] != 0.0)
                 ? static_cast<float>(static_cast<double>(t) / factors_[i])
                 : 0.0f;
      forces.data[i] = grams[i];
      newtons.data[i] = static_cast<float>(grams[i] / 1000.0 * gravity_);
      sum_g += grams[i];
      sum_n += newtons.data[i];
    }

    pub_raw_->publish(raw);
    pub_tared_->publish(tared);
    pub_forces_->publish(forces);
    pub_newtons_->publish(newtons);

    if (console_) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(),
        static_cast<uint64_t>(console_period_ms_),
        "\n"
        "  raw    FL=%9d  BL=%9d  BR=%9d  FR=%9d\n"
        "  tared  FL=%9d  BL=%9d  BR=%9d  FR=%9d\n"
        "  grams  FL=%9.1f  BL=%9.1f  BR=%9.1f  FR=%9.1f   sum=%9.1f g\n"
        "  force  FL=%9.2f  BL=%9.2f  BR=%9.2f  FR=%9.2f   sum=%9.2f N",
        raw.data[0], raw.data[1], raw.data[2], raw.data[3],
        tared.data[0], tared.data[1], tared.data[2], tared.data[3],
        grams[0], grams[1], grams[2], grams[3], sum_g,
        newtons.data[0], newtons.data[1], newtons.data[2], newtons.data[3],
        sum_n);
    }
  }

  // -- tare -------------------------------------------------------------

  void startTare()
  {
    tare_acc_.fill(0);
    tare_count_ = 0;
    tare_pending_ = true;
  }

  void accumulateTare(const long v[N_CELLS])
  {
    for (size_t i = 0; i < N_CELLS; ++i) tare_acc_[i] += v[i];
    ++tare_count_;

    if (tare_count_ >= tare_samples_) {
      for (size_t i = 0; i < N_CELLS; ++i) {
        offsets_[i] = static_cast<long>(tare_acc_[i] / tare_count_);
      }
      tare_pending_ = false;
      RCLCPP_INFO(this->get_logger(),
                  "tared on %d samples - offsets %ld %ld %ld %ld",
                  tare_count_, offsets_[0], offsets_[1],
                  offsets_[2], offsets_[3]);
    }
  }

  void onTare(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (fd_ < 0) {
      response->success = false;
      response->message = "serial port not open";
      return;
    }
    if (tare_pending_ || tare_requested_.load()) {
      response->success = false;
      response->message = "a tare is already in progress";
      return;
    }
    tare_requested_.store(true);
    response->success = true;
    response->message = "averaging " + std::to_string(tare_samples_) +
                        " samples";
  }

  // -- state ------------------------------------------------------------

  std::string port_;
  int baudrate_{115200};
  std::string frame_id_;
  int tare_samples_{30};
  bool tare_on_start_{false};
  bool console_{true};
  int console_period_ms_{500};
  double gravity_{9.80665};
  double dt_min_ms_{40.0};
  double dt_max_ms_{400.0};
  int poll_period_ms_{2};

  std::array<double, N_CELLS> factors_{};
  std::array<long, N_CELLS> offsets_{};

  bool tare_pending_{false};
  std::atomic<bool> tare_requested_{false};
  int tare_count_{0};
  std::array<long long, N_CELLS> tare_acc_{};

  int fd_{-1};
  std::string buffer_;
  unsigned long prev_millis_{0};
  bool prev_valid_{false};
  unsigned long bad_cadence_{0};

  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_raw_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_tared_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_forces_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_newtons_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_tare_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<BenchForces>();
  
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
  exec.add_node(node);
  exec.spin();
  
  rclcpp::shutdown();
  return 0;
}
