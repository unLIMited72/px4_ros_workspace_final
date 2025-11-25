#include <memory>
#include <string>
#include <chrono>
#include <functional>
#include <cmath>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <px4_interface/msg/px4_status.hpp>

#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/estimator_status_flags.hpp>
#include <px4_msgs/msg/failsafe_flags.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>

#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using px4_interface::msg::PX4Status;

class PX4StatusNode : public rclcpp::Node
{
public:
  PX4StatusNode() : Node("px4_status_node")
  {
    this->declare_parameter<std::string>("px4_ns", "px4_0");
    ns_ = this->get_parameter("px4_ns").as_string();

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    battery_sub_ = this->create_subscription<BatteryStatus>(
      "/" + ns_ + "/fmu/out/battery_status_v1", qos,
      std::bind(&PX4StatusNode::battery_callback, this, std::placeholders::_1));

    status_sub_ = this->create_subscription<VehicleStatus>(
      "/" + ns_ + "/fmu/out/vehicle_status_v1", qos,
      std::bind(&PX4StatusNode::status_callback, this, std::placeholders::_1));

    ekf_sub_ = this->create_subscription<EstimatorStatusFlags>(
      "/" + ns_ + "/fmu/out/estimator_status_flags", qos,
      std::bind(&PX4StatusNode::estimator_status_callback, this, std::placeholders::_1));

    failsafe_sub_ = this->create_subscription<FailsafeFlags>(
      "/" + ns_ + "/fmu/out/failsafe_flags", qos,
      std::bind(&PX4StatusNode::failsafe_callback, this, std::placeholders::_1));

    attitude_sub_ = this->create_subscription<VehicleAttitude>(
      "/" + ns_ + "/fmu/out/vehicle_attitude", qos,
      std::bind(&PX4StatusNode::attitude_callback, this, std::placeholders::_1));

    global_sub_ = this->create_subscription<VehicleGlobalPosition>(
      "/" + ns_ + "/fmu/out/vehicle_global_position", qos,
      std::bind(&PX4StatusNode::global_position_callback, this, std::placeholders::_1));

    local_sub_ = this->create_subscription<VehicleLocalPosition>(
      "/" + ns_ + "/fmu/out/vehicle_local_position_v1", qos,
      std::bind(&PX4StatusNode::heading_callback, this, std::placeholders::_1));

    status_pub_ = this->create_publisher<PX4Status>("/" + ns_ + "/status", 10);

    timer_ = this->create_wall_timer(500ms, std::bind(&PX4StatusNode::timer_callback, this));
  }

private:
  void battery_callback(const BatteryStatus::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    battery_remaining_ = msg->remaining * 100.0;
    battery_ok_ = (battery_remaining_ > 20.0);
  }

  void status_callback(const VehicleStatus::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_px4_timestamp_us_ = msg->timestamp;  
    last_msg_ros_time_ = this->now();        
    connected_ = true;

    armed_ = (msg->arming_state == msg->ARMING_STATE_ARMED);
    preflight_ok_ = msg->pre_flight_checks_pass;
    all_failsafe_ok_ = !msg->failsafe;
    failure_ok_ = (msg->failure_detector_status == msg->FAILURE_NONE);
    power_ok_ = msg->power_input_valid;
  }

  void estimator_status_callback(const EstimatorStatusFlags::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ekf_ok_ =
      msg->cs_tilt_align &&
      msg->cs_yaw_align &&
      msg->cs_gnss_pos &&
      msg->cs_baro_hgt &&
      !msg->cs_mag_fault &&
      !msg->cs_gnss_fault &&
      !msg->cs_rng_fault &&
      !msg->reject_hor_pos &&
      !msg->reject_ver_pos &&
      !msg->reject_yaw;
  }

  void failsafe_callback(const FailsafeFlags::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    detail_failsafe_ok_ =
      !msg->attitude_invalid &&
      !msg->local_position_invalid &&
      !msg->global_position_invalid &&
      !msg->battery_unhealthy &&
      !msg->battery_low_remaining_time &&
      !msg->geofence_breached &&
      !msg->flight_time_limit_exceeded &&
      !msg->fd_critical_failure &&
      !msg->fd_motor_failure &&
      !msg->fd_esc_arming_failure &&
      !msg->fd_imbalanced_prop;
  }

  void attitude_callback(const VehicleAttitude::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    tf2::Quaternion q(msg->q[1], msg->q[2], msg->q[3], msg->q[0]);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    roll_deg_ = roll * 180.0 / M_PI;
    pitch_deg_ = pitch * 180.0 / M_PI;

    if (std::abs(roll_deg_) > 35.0 || std::abs(pitch_deg_) > 35.0) {
      tilt_level_ = 2;  
    } else if (std::abs(roll_deg_) > 25.0 || std::abs(pitch_deg_) > 25.0) {
      tilt_level_ = 1;  
    } else {
      tilt_level_ = 0;
    }
  }

  void global_position_callback(const VehicleGlobalPosition::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_lat_ = msg->lat;
    last_lon_ = msg->lon;
  }

  void heading_callback(const VehicleLocalPosition::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_heading_deg_ = msg->heading * 180.0 / M_PI;
    if (last_heading_deg_ < 0) last_heading_deg_ += 360.0;
  }

  void timer_callback()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto now = this->now();

    if ((now - last_msg_ros_time_).seconds() > 3.0) {
      connected_ = false;
    }

    arm_ready_ = battery_ok_ && power_ok_ && ekf_ok_ && connected_ &&
                 preflight_ok_ && all_failsafe_ok_ && detail_failsafe_ok_ && failure_ok_;

    uint8_t flight_status = PX4Status::FLIGHT_STATUS_NORMAL;

    if (!connected_ || battery_remaining_ < 20.0 || tilt_level_ == 2) {
      flight_status = PX4Status::FLIGHT_STATUS_DANGER;
    } 
    else if (!arm_ready_ || battery_remaining_ < 30.0 || tilt_level_ == 1) {
      flight_status = PX4Status::FLIGHT_STATUS_WARNING;
    }

    PX4Status msg;
    msg.timestamp = last_px4_timestamp_us_;
    msg.heartbeat = connected_;
    msg.battery_percentage = static_cast<float>(battery_remaining_);
    msg.flight_ready = arm_ready_;
    msg.armed = armed_;
    msg.status_in_flight = flight_status;
    msg.drone_id = ns_;
    msg.latitude = last_lat_;
    msg.longitude = last_lon_;
    msg.heading_deg = last_heading_deg_;

    status_pub_->publish(msg);
  }

  rclcpp::Subscription<BatteryStatus>::SharedPtr battery_sub_;
  rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<EstimatorStatusFlags>::SharedPtr ekf_sub_;
  rclcpp::Subscription<FailsafeFlags>::SharedPtr failsafe_sub_;
  rclcpp::Subscription<VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Subscription<VehicleGlobalPosition>::SharedPtr global_sub_;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_sub_;
  rclcpp::Publisher<PX4Status>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mutex_;

  uint64_t last_px4_timestamp_us_{0};
  rclcpp::Time last_msg_ros_time_;

  double battery_remaining_{0.0};
  double roll_deg_{0.0}, pitch_deg_{0.0};
  bool connected_{false}, armed_{false}, arm_ready_{false};
  bool preflight_ok_{false}, all_failsafe_ok_{false}, detail_failsafe_ok_{false};
  bool failure_ok_{false}, power_ok_{false}, battery_ok_{false}, ekf_ok_{false};
  double last_lat_{0.0}, last_lon_{0.0};
  double last_heading_deg_{0.0};

  uint8_t tilt_level_{0};
  std::string ns_{"px4_0"};
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PX4StatusNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
