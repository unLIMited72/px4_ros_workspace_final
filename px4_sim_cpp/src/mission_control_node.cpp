#include <rclcpp/rclcpp.hpp>

#include <px4_interface/msg/mission_plan_for_drone.hpp>
#include <px4_interface/msg/mission_status.hpp>
#include <px4_interface/msg/mission_waypoint.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>

using std::placeholders::_1;

using px4_interface::msg::MissionPlanForDrone;
using px4_interface::msg::MissionStatus;
using px4_interface::msg::MissionWaypoint;

using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleGlobalPosition;
using px4_msgs::msg::VehicleStatus;

class DroneOffboardMissionControl : public rclcpp::Node
{
public:
  DroneOffboardMissionControl()
  : Node("drone_offboard_mission_control")
  {
    // 기본 파라미터
    drone_id_ = this->declare_parameter<std::string>("drone_id", "px4_0");
    px4_ns_   = this->declare_parameter<std::string>("px4_ns", "px4_0");

    cruise_speed_mps_ = this->declare_parameter<double>("cruise_speed_mps", 3.0);
    climb_speed_mps_  = this->declare_parameter<double>("climb_speed_mps", 1.0);

    // 🔽 MAV_SYS_ID / MAV_COMP_ID용 파라미터
    target_system_id_ = this->declare_parameter<int>(
      "target_system_id",
      inferMavSysIdFromNs(px4_ns_)   // px4_0 -> 1, px4_1 -> 2, px4_2 -> 3 ...
    );
    target_component_id_ = this->declare_parameter<int>("target_component_id", 1);

    auto gcs_qos     = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    auto px4_out_qos = rclcpp::SensorDataQoS();
    auto px4_in_qos  = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    mission_plan_sub_ = this->create_subscription<MissionPlanForDrone>(
      "/gcs/mission_plan_for_drone", gcs_qos,
      std::bind(&DroneOffboardMissionControl::onMissionPlanForDrone, this, _1));

    mission_status_sub_ = this->create_subscription<MissionStatus>(
      "/gcs/mission_status", gcs_qos,
      std::bind(&DroneOffboardMissionControl::onMissionStatus, this, _1));

    vehicle_global_pos_sub_ = this->create_subscription<VehicleGlobalPosition>(
      ns("/fmu/out/vehicle_global_position"), px4_out_qos,
      std::bind(&DroneOffboardMissionControl::onVehicleGlobalPosition, this, _1));

    vehicle_status_sub_ = this->create_subscription<VehicleStatus>(
      ns("/fmu/out/vehicle_status_v1"), px4_out_qos,
      std::bind(&DroneOffboardMissionControl::onVehicleStatus, this, _1));

    offboard_mode_pub_ = this->create_publisher<OffboardControlMode>(
      ns("/fmu/in/offboard_control_mode"), px4_in_qos);

    traj_setpoint_pub_ = this->create_publisher<TrajectorySetpoint>(
      ns("/fmu/in/trajectory_setpoint"), px4_in_qos);

    vehicle_command_pub_ = this->create_publisher<VehicleCommand>(
      ns("/fmu/in/vehicle_command"), px4_in_qos);

    mission_status_fb_pub_ = this->create_publisher<MissionStatus>(
      "/gcs/mission_status_feedback", gcs_qos);

    using namespace std::chrono_literals;
    control_timer_ = this->create_wall_timer(
      50ms, std::bind(&DroneOffboardMissionControl::onControlTimer, this));

    RCLCPP_INFO(this->get_logger(),
      "DroneOffboardMissionControl started (drone_id='%s', px4_ns='%s', cruise_speed=%.2f, climb_speed=%.2f, target_system_id=%d, target_component_id=%d)",
      drone_id_.c_str(), px4_ns_.c_str(),
      cruiseSpeed(), climbSpeed(),
      target_system_id_, target_component_id_);
  }

private:
  enum class FlightPhase {
    IDLE = 0,
    CLIMB,
    NAV
  };

  // px4_ns로부터 MAV_SYS_ID 유추: "px4_0" -> 1, "px4_1" -> 2 ...
  int inferMavSysIdFromNs(const std::string & ns) const
  {
    auto pos = ns.find_last_of('_');
    if (pos == std::string::npos) {
      return 1; // 이상하면 그냥 1로
    }
    try {
      int idx = std::stoi(ns.substr(pos + 1)); // "0","1","2" -> 0,1,2
      return idx + 1;                          // -> 1,2,3
    } catch (...) {
      return 1;
    }
  }

  std::string ns(const std::string & topic) const
  {
    if (px4_ns_.empty()) {
      return topic;
    }

    if (topic.empty() || topic[0] != '/') {
      return "/" + px4_ns_ + "/" + topic;
    }

    return "/" + px4_ns_ + topic;
  }

  uint64_t now_u() const
  {
    return this->now().nanoseconds() / 1000;
  }

  double cruiseSpeed() const
  {
    double v = cruise_speed_mps_;
    if (has_plan_ && current_plan_.cruise_speed_mps > 0.0f) {
      v = current_plan_.cruise_speed_mps;
    }
    if (v < 1.0) v = 1.0;
    return v;
  }

  double climbSpeed() const
  {
    double v = climb_speed_mps_;
    if (v <= 0.0) v = 1.0;
    if (v > cruiseSpeed()) v = cruiseSpeed();
    return v;
  }

  double spacingDistance() const
  {
    if (!has_plan_) return 5.0;
    double d = current_plan_.spacing_value;
    if (d <= 0.0) d = 5.0;
    return d;
  }

  double computeClimbTargetAltRel() const
  {
    if (has_plan_ && current_plan_.cruise_altitude_m > 0.0f) {
      return current_plan_.cruise_altitude_m;
    }

    if (has_plan_) {
      for (const auto & wp : current_plan_.waypoints) {
        if (wp.alt > 0.0f) return wp.alt;
      }
    }

    return 5.0;
  }

  void onMissionPlanForDrone(const MissionPlanForDrone::SharedPtr msg)
  {
    if (msg->drone_id != drone_id_) {
      return;
    }

    if (global_state_ == MissionStatus::STATE_ACTIVE || global_state_ == MissionStatus::STATE_EMERGENCY)
    {
      RCLCPP_WARN(this->get_logger(), "[%s] Received plan while ACTIVE/EMERGENCY. Ignored.", drone_id_.c_str());
      return;
    }

    if (msg->mission_id.empty() || msg->waypoints.empty()) {
      RCLCPP_WARN(this->get_logger(), "[%s] Invalid MissionPlanForDrone. Ignore.", drone_id_.c_str());
      return;
    }

    current_plan_ = *msg;
    has_plan_ = true;

    resetMissionState(false);

    RCLCPP_INFO(this->get_logger(), "[%s] Loaded plan id=%s (wps=%zu, cruise_alt=%.2f, cruise_speed=%.2f, spacing=%.2f)",
      drone_id_.c_str(),
      current_plan_.mission_id.c_str(),
      current_plan_.waypoints.size(),
      current_plan_.cruise_altitude_m,
      current_plan_.cruise_speed_mps,
      current_plan_.spacing_value);
  }

  void onMissionStatus(const MissionStatus::SharedPtr msg)
  {
    if (!msg->mission_id.empty() && has_plan_ && msg->mission_id != current_plan_.mission_id)
    {
      return;
    }

    const uint8_t new_state = msg->state;

    if (new_state == MissionStatus::STATE_ACTIVE && global_state_ != MissionStatus::STATE_ACTIVE)
    {
      active_start_time_ = this->now();
      offboard_started_ = false;
      offboard_setpoint_counter_ = 0;

      flight_phase_ = FlightPhase::CLIMB;
      climb_target_alt_rel_ = computeClimbTargetAltRel();

      RCLCPP_INFO(this->get_logger(), "[%s] MissionStatus -> ACTIVE (CLIMB to %.2fm)", drone_id_.c_str(), climb_target_alt_rel_);
    }

    if (new_state == MissionStatus::STATE_EMERGENCY && global_state_ != MissionStatus::STATE_EMERGENCY)
    {
      RCLCPP_WARN(this->get_logger(), "[%s] MissionStatus -> EMERGENCY", drone_id_.c_str());
      emergency_rtl_sent_ = false;
      emergency_complete_reported_ = false;
    }

    if (new_state == MissionStatus::STATE_ABORTED && global_state_ != MissionStatus::STATE_ABORTED)
    {
      RCLCPP_WARN(this->get_logger(), "[%s] MissionStatus -> ABORTED", drone_id_.c_str());
    }

    if (new_state == MissionStatus::STATE_COMPLETED && global_state_ != MissionStatus::STATE_COMPLETED)
    {
      RCLCPP_INFO(this->get_logger(), "[%s] MissionStatus -> COMPLETED", drone_id_.c_str());
    }

    global_state_ = new_state;
    updateFormationInfo(*msg);
  }

  void onVehicleGlobalPosition(const VehicleGlobalPosition::SharedPtr msg)
  {
    last_global_pos_ = *msg;

    if (!home_set_ && msg->lat != 0.0 && msg->lon != 0.0) {
      home_lat_ = msg->lat;
      home_lon_ = msg->lon;
      home_alt_ = msg->alt;
      home_set_ = true;

      RCLCPP_INFO(this->get_logger(), "[%s] Home set: lat=%.7f, lon=%.7f, alt=%.2f", drone_id_.c_str(), home_lat_, home_lon_, home_alt_);
    }
  }

  void onVehicleStatus(const VehicleStatus::SharedPtr msg)
  {
    last_vehicle_status_ = *msg;
    has_vehicle_status_ = true;
  }

  void onControlTimer()
  {
    auto now = this->now();
    if (has_last_control_time_) {
      control_dt_ = std::max(0.001, (now - last_control_time_).seconds());
    }
    last_control_time_ = now;
    has_last_control_time_ = true;

    if (!has_plan_ || !home_set_ || !has_vehicle_status_) {
      return;
    }

    switch (global_state_) {
      case MissionStatus::STATE_ACTIVE: handleActive(); break;
      case MissionStatus::STATE_PAUSED: handleHold(); break;
      case MissionStatus::STATE_EMERGENCY: handleEmergency(); break;
      case MissionStatus::STATE_ABORTED: handleAbort(); break;
      case MissionStatus::STATE_COMPLETED: handleMissionCompleted(); break;
      case MissionStatus::STATE_IDLE:
      default: break;
    }
  }

  void handleActive()
  {
    publishOffboardControlModeVelocity();

    if (!offboard_started_) {
      publishZeroVelocitySetpoint();

      if (++offboard_setpoint_counter_ > 20) {
        enterOffboardAndArmVelocity();
        offboard_started_ = true;
        RCLCPP_INFO(this->get_logger(), "[%s] Offboard started (velocity mode)", drone_id_.c_str());
      }
      return;
    }

    if (mission_completed_) {
      handleMissionCompleted();
      return;
    }

    if (flight_phase_ == FlightPhase::CLIMB) {
      handleClimb();
    } else if (flight_phase_ == FlightPhase::NAV) {
      handleNav();
    } else {
      publishZeroVelocitySetpoint();
    }
  }

  void handleClimb()
  {
    if (!home_set_) {
      publishZeroVelocitySetpoint();
      return;
    }

    double target_alt_amsl = home_alt_ + climb_target_alt_rel_;
    double cur_alt_amsl = last_global_pos_.alt;
    double dz = target_alt_amsl - cur_alt_amsl;

    if (std::fabs(dz) < climb_alt_tolerance_) {
      flight_phase_ = FlightPhase::NAV;
      RCLCPP_INFO(this->get_logger(), "[%s] Climb completed. Switch to NAV.", drone_id_.c_str());
      publishZeroVelocitySetpoint();
      return;
    }

    double v = climbSpeed();
    double vz_ned = (dz > 0.0) ? -v : v;

    TrajectorySetpoint sp{};
    sp.timestamp = now_u();
    sp.position = {
      std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::quiet_NaN()
    };
    sp.velocity = {0.f, 0.f, static_cast<float>(vz_ned)};
    sp.yaw = last_yaw_;

    traj_setpoint_pub_->publish(sp);
  }

  void handleNav()
  {
    if (current_wp_index_ >= current_plan_.waypoints.size()) {
      mission_completed_ = true;
      handleMissionCompleted();
      return;
    }

    const auto & wp = current_plan_.waypoints[current_wp_index_];

    double target_alt_rel =
      (wp.alt > 0.0f) ? wp.alt : computeClimbTargetAltRel();

    double target_x, target_y, target_z;
    llaToNed(wp.lat, wp.lon, home_alt_ + target_alt_rel,
             target_x, target_y, target_z);

    applyAlongTrackSpacing(target_x, target_y, current_wp_index_);

    double cur_x, cur_y, cur_z;
    llaToNed(last_global_pos_.lat, last_global_pos_.lon, last_global_pos_.alt,
             cur_x, cur_y, cur_z);

    double dx = target_x - cur_x;
    double dy = target_y - cur_y;
    double dz = target_z - cur_z;
    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

    const double reach_thresh = 2.0;

    if (dist < reach_thresh) {
      RCLCPP_INFO(this->get_logger(), "[%s] Reached WP %zu (dist=%.2f)", drone_id_.c_str(), current_wp_index_, dist);
      current_wp_index_++;
      if (current_wp_index_ >= current_plan_.waypoints.size()) {
        mission_completed_ = true;
      }
      publishZeroVelocitySetpoint();
      return;
    }

    double ux = dx / dist;
    double uy = dy / dist;
    double uz = dz / dist;
    double v  = cruiseSpeed();

    TrajectorySetpoint sp{};
    sp.timestamp = now_u();
    sp.position = {
      std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::quiet_NaN()
    };
    sp.velocity = {
      static_cast<float>(ux * v),
      static_cast<float>(uy * v),
      static_cast<float>(uz * v)
    };

    if (std::fabs(ux) > 1e-3 || std::fabs(uy) > 1e-3) {
      sp.yaw = static_cast<float>(std::atan2(uy, ux));
    } else {
      sp.yaw = last_yaw_;
    }
    last_yaw_ = sp.yaw;

    traj_setpoint_pub_->publish(sp);
  }

  void handleHold()
  {
    publishOffboardControlModeVelocity();
    publishZeroVelocitySetpoint();
  }

  void handleEmergency()
  {
    if (!emergency_rtl_sent_) {
      RCLCPP_WARN(this->get_logger(), "[%s] EMERGENCY -> RTL (send once)", drone_id_.c_str());
      sendVehicleCommand(VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH);
      publishLocalMissionStatus(MissionStatus::STATE_EMERGENCY);

      emergency_rtl_sent_ = true;
      return;
    }

    if (!has_vehicle_status_ || !home_set_) {
      return;
    }

    const auto & vs = last_vehicle_status_;

    double hx, hy, hz;
    llaToNed(last_global_pos_.lat, last_global_pos_.lon, last_global_pos_.alt, hx, hy, hz);

    double dist_home = std::sqrt(hx*hx + hy*hy);
    bool near_home   = (dist_home < 5.0);
    bool low_alt     = (std::fabs(hz) < 1.0);
    bool disarmed    = (vs.arming_state == VehicleStatus::ARMING_STATE_DISARMED);

    if (!emergency_complete_reported_ && disarmed && near_home && low_alt)
    {
      RCLCPP_INFO(this->get_logger(), "[%s] EMERGENCY RTL completed. Report COMPLETED to GCS.", drone_id_.c_str());

      publishLocalMissionStatus(MissionStatus::STATE_COMPLETED);
      emergency_complete_reported_ = true;

      resetMissionState(true);
    }
  }

  void handleAbort()
  {
    publishOffboardControlModeVelocity();
    publishZeroVelocitySetpoint();
    publishLocalMissionStatus(MissionStatus::STATE_ABORTED);
  }

  void handleMissionCompleted()
  {
    if (!has_plan_ || mission_completion_processed_) return;
    mission_completion_processed_ = true;

    if (current_plan_.landing_mode == MissionPlanForDrone::LANDING_HOME) {
      RCLCPP_INFO(this->get_logger(), "[%s] Mission complete -> RTL(HOME)", drone_id_.c_str());

      sendVehicleCommand(VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH);
      publishLocalMissionStatus(MissionStatus::STATE_COMPLETED);
      resetMissionState(true);

    } else if (current_plan_.landing_mode == MissionPlanForDrone::LANDING_LAST_WAYPOINT) {

      if (!current_plan_.waypoints.empty() && home_set_) {
        const auto & last_wp = current_plan_.waypoints.back();

        double target_alt_rel = (last_wp.alt > 0.0f) ? last_wp.alt : computeClimbTargetAltRel();

        double lx, ly, lz;
        llaToNed(last_wp.lat, last_wp.lon, home_alt_ + target_alt_rel, lx, ly, lz);

        applyLandingOffset(lx, ly);

        double cx, cy, cz;
        llaToNed(last_global_pos_.lat, last_global_pos_.lon, last_global_pos_.alt, cx, cy, cz);

        double dx = lx - cx;
        double dy = ly - cy;
        double dz = lz - cz;
        double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

        publishOffboardControlModeVelocity();

        TrajectorySetpoint sp{};
        sp.timestamp = now_u();
        sp.position = {
          std::numeric_limits<float>::quiet_NaN(),
          std::numeric_limits<float>::quiet_NaN(),
          std::numeric_limits<float>::quiet_NaN()
        };

        if (dist > 1.0) {
          double ux = dx / dist;
          double uy = dy / dist;
          double uz = dz / dist;
          double v  = std::min(cruiseSpeed(), 1.5);
          sp.velocity = {
            static_cast<float>(ux * v),
            static_cast<float>(uy * v),
            static_cast<float>(uz * v)
          };
        } else {
          sp.velocity = {0.f, 0.f, 0.f};
        }

        traj_setpoint_pub_->publish(sp);

        RCLCPP_INFO(this->get_logger(), "[%s] Mission complete -> LAND near last WP (idx=%d)", drone_id_.c_str(), formation_index_);
      }

      sendVehicleCommand(VehicleCommand::VEHICLE_CMD_NAV_LAND);
      publishLocalMissionStatus(MissionStatus::STATE_COMPLETED);
      resetMissionState(true);

    } else {
      RCLCPP_INFO(this->get_logger(), "[%s] Mission complete -> RTL(default)", drone_id_.c_str());

      sendVehicleCommand(VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH);
      publishLocalMissionStatus(MissionStatus::STATE_COMPLETED);
      resetMissionState(true);
    }
  }

  void updateFormationInfo(const MissionStatus & st)
  {
    formation_size_ = 1;
    formation_index_ = 0;

    if (st.drone_ids.empty()) {
      return;
    }

    std::vector<std::string> ids = st.drone_ids;
    std::sort(ids.begin(), ids.end());

    auto it = std::find(ids.begin(), ids.end(), drone_id_);
    if (it == ids.end()) {
      return;
    }

    formation_size_ = static_cast<int>(ids.size());
    formation_index_ = static_cast<int>(std::distance(ids.begin(), it));
  }

  void applyAlongTrackSpacing(double &tx, double &ty, size_t wp_index)
  {
    if (formation_index_ <= 0 || !has_plan_) return;

    double d = spacingDistance() * formation_index_;
    if (d <= 0.0) return;

    double x0 = 0.0, y0 = 0.0, z0 = 0.0;

    if (wp_index > 0 && wp_index < current_plan_.waypoints.size()) {
      const auto & prev = current_plan_.waypoints[wp_index - 1];
      double prev_alt = (prev.alt > 0.0f) ? prev.alt : computeClimbTargetAltRel();
      llaToNed(prev.lat, prev.lon, home_alt_ + prev_alt, x0, y0, z0);
    }

    double vx = tx - x0;
    double vy = ty - y0;
    double norm = std::sqrt(vx*vx + vy*vy);
    if (norm < 1e-3) return;

    double ux = vx / norm;
    double uy = vy / norm;

    tx -= ux * d;
    ty -= uy * d;
  }

  void applyLandingOffset(double &x, double &y)
  {
    if (formation_size_ <= 1 || formation_index_ <= 0) return;

    double d = spacingDistance();
    if (d <= 0.0) d = 3.0;
    d *= formation_index_;

    double angle = (M_PI / 2.0) + (2.0 * M_PI * formation_index_ / formation_size_);

    x += d * std::cos(angle);
    y += d * std::sin(angle);
  }

  void enterOffboardAndArmVelocity()
  {
    sendVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
    sendVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
  }

  void publishOffboardControlModeVelocity()
  {
    OffboardControlMode msg{};
    msg.timestamp = now_u();
    msg.position = false;
    msg.velocity = true;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.thrust_and_torque = false;
    msg.direct_actuator = false;
    offboard_mode_pub_->publish(msg);
  }

  void publishZeroVelocitySetpoint()
  {
    TrajectorySetpoint sp{};
    sp.timestamp = now_u();
    sp.position = {
      std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::quiet_NaN()
    };
    sp.velocity = {0.f, 0.f, 0.f};
    sp.yaw = last_yaw_;
    traj_setpoint_pub_->publish(sp);
  }

  void sendVehicleCommand(uint16_t command,
                          float p1 = 0.0f, float p2 = 0.0f,
                          float p3 = 0.0f, float p4 = 0.0f,
                          float p5 = 0.0f, float p6 = 0.0f,
                          float p7 = 0.0f)
  {
    VehicleCommand cmd{};
    cmd.timestamp = now_u();
    cmd.param1 = p1;
    cmd.param2 = p2;
    cmd.param3 = p3;
    cmd.param4 = p4;
    cmd.param5 = p5;
    cmd.param6 = p6;
    cmd.param7 = p7;
    cmd.command = command;

    // 🔽 여기 중요: 각 인스턴스마다 다른 SYS_ID로 전송
    cmd.target_system    = static_cast<uint8_t>(target_system_id_);
    cmd.target_component = static_cast<uint8_t>(target_component_id_);
    cmd.source_system    = static_cast<uint8_t>(target_system_id_);
    cmd.source_component = static_cast<uint8_t>(target_component_id_);

    cmd.from_external = true;
    vehicle_command_pub_->publish(cmd);
  }

  void llaToNed(double lat, double lon, double alt, double &x, double &y, double &z) const
  {
    static constexpr double R = 6378137.0;

    double dlat = (lat - home_lat_) * M_PI / 180.0;
    double dlon = (lon - home_lon_) * M_PI / 180.0;
    double dalt = alt - home_alt_;

    double north = dlat * R;
    double east  = dlon * R * std::cos(home_lat_ * M_PI / 180.0);

    x = north;
    y = east;
    z = -dalt;
  }

  void publishLocalMissionStatus(uint8_t st)
  {
    if (!has_plan_) return;

    MissionStatus msg;
    msg.mission_id = current_plan_.mission_id;
    msg.state = st;
    msg.drone_ids.clear();
    msg.drone_ids.push_back(drone_id_);

    mission_status_fb_pub_->publish(msg);
  }

  void resetMissionState(bool clear_plan)
  {
    mission_completed_ = false;
    mission_completion_processed_ = false;

    offboard_started_ = false;
    offboard_setpoint_counter_ = 0;

    flight_phase_ = FlightPhase::IDLE;
    current_wp_index_ = 0;

    last_yaw_ = 0.0f;

    emergency_rtl_sent_ = false;
    emergency_complete_reported_ = false;

    formation_index_ = 0;
    formation_size_ = 1;
    global_state_ = MissionStatus::STATE_IDLE;

    if (clear_plan) {
      has_plan_ = false;
    }

    RCLCPP_INFO(this->get_logger(), "[%s] Mission state reset (clear_plan=%d). UI can return to idle.",
      drone_id_.c_str(), clear_plan ? 1 : 0);
  }

  // ===== 멤버 변수 =====
  std::string drone_id_;
  std::string px4_ns_;

  int target_system_id_{1};
  int target_component_id_{1};

  rclcpp::Subscription<MissionPlanForDrone>::SharedPtr   mission_plan_sub_;
  rclcpp::Subscription<MissionStatus>::SharedPtr         mission_status_sub_;
  rclcpp::Subscription<VehicleGlobalPosition>::SharedPtr vehicle_global_pos_sub_;
  rclcpp::Subscription<VehicleStatus>::SharedPtr         vehicle_status_sub_;

  rclcpp::Publisher<OffboardControlMode>::SharedPtr  offboard_mode_pub_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr   traj_setpoint_pub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr       vehicle_command_pub_;
  rclcpp::Publisher<MissionStatus>::SharedPtr        mission_status_fb_pub_;

  rclcpp::TimerBase::SharedPtr control_timer_;

  MissionPlanForDrone current_plan_;
  bool                has_plan_{false};

  VehicleGlobalPosition last_global_pos_{};
  bool                  home_set_{false};
  double                home_lat_{0.0}, home_lon_{0.0}, home_alt_{0.0};

  VehicleStatus         last_vehicle_status_{};
  bool                  has_vehicle_status_{false};

  uint8_t               global_state_{MissionStatus::STATE_IDLE};

  size_t                current_wp_index_{0};

  bool                  mission_completed_{false};
  bool                  mission_completion_processed_{false};

  int                   formation_index_{0};
  int                   formation_size_{1};

  rclcpp::Time          last_control_time_{0,0,RCL_SYSTEM_TIME};
  bool                  has_last_control_time_{false};
  double                control_dt_{0.05};

  rclcpp::Time          active_start_time_{0,0,RCL_SYSTEM_TIME};

  bool        offboard_started_{false};
  int         offboard_setpoint_counter_{0};
  double      cruise_speed_mps_{3.0};
  double      climb_speed_mps_{1.0};
  double      climb_target_alt_rel_{5.0};
  float       last_yaw_{0.0f};
  FlightPhase flight_phase_{FlightPhase::IDLE};

  const double climb_alt_tolerance_ = 0.5;

  bool emergency_rtl_sent_{false};
  bool emergency_complete_reported_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DroneOffboardMissionControl>());
  rclcpp::shutdown();
  return 0;
}
