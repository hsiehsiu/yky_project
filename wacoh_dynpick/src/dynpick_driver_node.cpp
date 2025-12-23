#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <stdio.h>
#include <fcntl.h>
#include <time.h>
#include <termios.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include <mutex>
#include <condition_variable>
#include <memory>
#include <vector>
#include <chrono>
#include <thread>

#define RESET_COMMAND_TRY   3
#define DATA_LENGTH 27
#define CALIB_DATA_LENGTH 46

class DynpickDriverNode : public rclcpp::Node
{
public:
  using Trigger = std_srvs::srv::Trigger;

  DynpickDriverNode() : Node("dynpick_driver_node")
  {
    RCLCPP_INFO(this->get_logger(), "Starting Dynpick Driver Node...");

    // 1. 參數
    devname_      = this->declare_parameter<std::string>("device", "/dev/ttyUSB0");
    frame_id_     = this->declare_parameter<std::string>("frame_id", "sensor_link");
    rate_         = this->declare_parameter<double>("rate", 1000.0);
    auto_adjust_  = this->declare_parameter<bool>("acquire_calibration", true);
    frq_div_      = this->declare_parameter<int>("frequency_div", 1);

    // 2. ROS 介面先建好，這樣就算 serial 讀不到，也不會卡住
    publisher_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("force", 10);
    tare_service_ = this->create_service<Trigger>(
      "tare",
      std::bind(&DynpickDriverNode::tare_callback, this, std::placeholders::_1, std::placeholders::_2)
    );

    // 3. 設定 timer
    auto period = std::chrono::duration<double>(1.0 / rate_);
    timer_ = this->create_wall_timer(
      period,
      std::bind(&DynpickDriverNode::publish_wrench_data, this)
    );

    // 4. 一開始設定成還沒連到感測器
    fdc_ = -1;
    clock_ = 0;
    offset_reset_ = 0;
    calib_ = {13.0, 13.0, 13.0, 650.0, 650.0, 650.0};  // 預設值，避免除以 0
    connected_ = false;

    // 5. 啟一條背景 thread 去連線，避免 constructor 卡住
    worker_ = std::thread([this](){
      this->init_serial_and_sensor();
    });

    RCLCPP_INFO(this->get_logger(), "Dynpick Driver Node initialized successfully (ROS parts ready).");
  }

  ~DynpickDriverNode()
  {
    running_ = false;
    if (worker_.joinable()) {
      worker_.join();
    }
    if (fdc_ >= 0) {
      close(fdc_);
    }
  }

private:
  // ------------------- 背景初始化 -------------------
  void init_serial_and_sensor()
  {
    // 這條 thread 裡面做可能會卡的事，卡住也不會影響 ROS node 註冊
    while (running_ && !connected_) {
      RCLCPP_INFO(this->get_logger(), "Opening serial port: %s", devname_.c_str());
      int fd = open(devname_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
      if (fd < 0) {
        RCLCPP_WARN(this->get_logger(), "Could not open %s: %s. Retry in 1s.", devname_.c_str(), strerror(errno));
        std::this_thread::sleep_for(std::chrono::seconds(1));
        continue;
      }

      if (SetComAttr(fd) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set COM attributes.");
        close(fd);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        continue;
      }

      // 清掉舊資料
      char trash[255];
      clearSocket(fd, trash, 255);

      // 讀校正（有可能失敗，要容忍）
      if (auto_adjust_) {
        acquire_calibration(fd);
      }

      // 設 divider（也可能失敗，要容忍）
      set_frequency_divider(fd);

      // 都 ok 才正式啟用
      {
        std::lock_guard<std::mutex> lk(m_);
        fdc_ = fd;
        connected_ = true;
      }
      RCLCPP_INFO(this->get_logger(), "Sensor connected and ready.");
    }
  }

  // ------------------- ROS 迴圈 -------------------
  void publish_wrench_data()
  {
    std::unique_lock<std::mutex> lock(m_);
    if (!connected_ || fdc_ < 0) {
      // 尚未連到感測器，先跳過
      return;
    }

    int fd = fdc_;

    // 有人在呼叫 tare，就送 'O'
    if (offset_reset_ > 0) {
      if (write(fd, "O", 1) == 1) {
        offset_reset_--;
        if (offset_reset_ == 0) {
          cv_.notify_all();
        }
      }
      return;
    }

    // 1. 送 'R'
    ssize_t w = write(fd, "R", 1);
    if (w != 1) {
      RCLCPP_WARN(this->get_logger(), "Failed to write 'R' command.");
      return;
    }

    // 2. 讀資料（這裡改成有 timeout 的版本，不會卡死）
    char str[256] = {0};
    lock.unlock();  // 讀的時候不用鎖著
    bool ok = read_exact_with_timeout(fd, DATA_LENGTH, str, std::chrono::milliseconds(100));
    lock.lock();

    if (!ok) {
      // 讀不到就跳過這次，不要卡
      // RCLCPP_WARN(this->get_logger(), "Timeout reading sensor data.");
      return;
    }

    // 確保結尾
    str[DATA_LENGTH] = '\0';

    int tick;
    unsigned short data[6];
    int parsed = sscanf(str, "%1d%4hx%4hx%4hx%4hx%4hx%4hx",
                        &tick, &data[0], &data[1], &data[2], &data[3], &data[4], &data[5]);
    if (parsed != 7) {
      RCLCPP_WARN(this->get_logger(), "Failed to parse sensor data string: %s", str);
      return;
    }

    auto msg = geometry_msgs::msg::WrenchStamped();
    msg.header.frame_id = frame_id_;
    msg.header.stamp = this->now();
    msg.header.stamp.nanosec = clock_++;

    // Dynpick 14-bit 基準 8192
    msg.wrench.force.x  = (data[0] - 8192.0) / calib_[0];
    msg.wrench.force.y  = (data[1] - 8192.0) / calib_[1];
    msg.wrench.force.z  = (data[2] - 8192.0) / calib_[2];
    msg.wrench.torque.x = (data[3] - 8192.0) / calib_[3];
    msg.wrench.torque.y = (data[4] - 8192.0) / calib_[4];
    msg.wrench.torque.z = (data[5] - 8192.0) / calib_[5];

    publisher_->publish(msg);
  }

  // ------------------- Service -------------------
  void tare_callback(const std::shared_ptr<Trigger::Request> /*req*/,
                     std::shared_ptr<Trigger::Response> res)
  {
    std::unique_lock<std::mutex> lock(m_);
    if (!connected_) {
      res->success = false;
      res->message = "Sensor not connected.";
      return;
    }
    offset_reset_ = RESET_COMMAND_TRY;
    cv_.wait(lock, [this]{ return offset_reset_ <= 0; });
    res->success = true;
    res->message = "Offset reset sent.";
  }

  // ------------------- Serial helpers -------------------
  int SetComAttr(int fdc)
  {
    struct termios term;
    if (tcgetattr(fdc, &term) < 0) {
      return -1;
    }
    bzero(&term, sizeof(term));

    // 你可以改成 B115200 試試
    term.c_cflag = B921600 | CS8 | CLOCAL | CREAD;
    term.c_iflag = IGNPAR;
    term.c_oflag = 0;
    term.c_lflag = 0;
    term.c_cc[VTIME] = 0;
    term.c_cc[VMIN]  = 0;

    return tcsetattr(fdc, TCSANOW, &term);
  }

  bool clearSocket(const int& fdc, char* leftover, int length)
  {
    int len = 0;
    while (len < length) {
      int c = read(fdc, leftover + len, length - len);
      if (c > 0) len += c;
      else break;
    }
    return true;
  }

  void acquire_calibration(int fd)
  {
    write(fd, "p", 1);
    char reply[CALIB_DATA_LENGTH] = {0};
    if (read_exact_with_timeout(fd, CALIB_DATA_LENGTH, reply, std::chrono::milliseconds(200))) {
      reply[CALIB_DATA_LENGTH-1] = '\0';
      float c0, c1, c2, c3, c4, c5;
      if (sscanf(reply, "%f,%f,%f,%f,%f,%f", &c0, &c1, &c2, &c3, &c4, &c5) == 6) {
        calib_[0] = c0; calib_[1] = c1; calib_[2] = c2;
        calib_[3] = c3; calib_[4] = c4; calib_[5] = c5;
        RCLCPP_INFO(this->get_logger(), "Calibration from sensor: Fx %.3f Fy %.3f Fz %.3f Tx %.3f Ty %.3f Tz %.3f",
                    c0, c1, c2, c3, c4, c5);
      } else {
        RCLCPP_WARN(this->get_logger(), "Failed to parse calibration string: %s", reply);
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "Timeout when reading calibration.");
    }
  }

  void set_frequency_divider(int fd)
  {
    if (!(frq_div_ == 1 || frq_div_ == 2 || frq_div_ == 4 || frq_div_ == 8)) {
      RCLCPP_WARN(this->get_logger(), "frequency_div out of range, skip.");
      return;
    }
    char cmd[3];
    snprintf(cmd, sizeof(cmd), "%dF", frq_div_);
    if (write(fd, cmd, 2) != 2) {
      RCLCPP_WARN(this->get_logger(), "Failed to send frequency divider command.");
    } else {
      RCLCPP_INFO(this->get_logger(), "Set the frequency divider to %s", cmd);
    }
    // 這裡就不要再去讀回來驗證了，避免卡住
  }

  // 這是新的：有 timeout 的讀取
  bool read_exact_with_timeout(int fd, int length, char* buffer,
                               std::chrono::milliseconds timeout)
  {
    int got = 0;
    auto start = std::chrono::steady_clock::now();
    while (got < length) {
      int n = read(fd, buffer + got, length - got);
      if (n > 0) {
        got += n;
      } else if (n == 0 || (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      } else {
        return false;
      }

      if (std::chrono::steady_clock::now() - start > timeout) {
        return false;
      }
    }
    return true;
  }

  // ------------------- members -------------------
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr publisher_;
  rclcpp::Service<Trigger>::SharedPtr tare_service_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string devname_;
  std::string frame_id_;
  double rate_;
  bool auto_adjust_;
  int frq_div_;

  int fdc_;
  int clock_;
  std::vector<float> calib_;

  std::mutex m_;
  std::condition_variable cv_;
  int offset_reset_;
  bool connected_ = false;
  bool running_ = true;
  std::thread worker_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DynpickDriverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
