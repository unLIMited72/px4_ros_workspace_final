#include <rclcpp/rclcpp.hpp>

#include <px4_interface/msg/mission_plan.hpp>
#include <px4_interface/msg/mission_command.hpp>
#include <px4_interface/msg/mission_waypoint.hpp>
#include <px4_interface/msg/mission_plan_for_drone.hpp>
#include <px4_interface/msg/mission_status.hpp>

#include <string>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <algorithm>

using std::placeholders::_1;
using namespace px4_interface::msg;

class MissionPlannerNode : public rclcpp::Node
{
public:
  MissionPlannerNode()
  : Node("mission_planner_node")
  {
    using rclcpp::QoS;
    auto qos = QoS(rclcpp::KeepLast(10)).reliable();

    mission_plan_sub_ = this->create_subscription<MissionPlan>(
      "/gcs/mission_plan", qos,
      std::bind(&MissionPlannerNode::onMissionPlan, this, _1));

    mission_command_sub_ = this->create_subscription<MissionCommand>(
      "/gcs/mission_command", qos,
      std::bind(&MissionPlannerNode::onMissionCommand, this, _1));

    mission_plan_for_drone_pub_ = this->create_publisher<MissionPlanForDrone>(
      "/gcs/mission_plan_for_drone", qos);

    mission_status_pub_ = this->create_publisher<MissionStatus>(
      "/gcs/mission_status", qos);

    mission_status_fb_sub_ = this->create_subscription<MissionStatus>(
      "/gcs/mission_status_feedback", qos,
      std::bind(&MissionPlannerNode::onMissionStatusFeedback, this, _1));

    state_ = MissionStatus::STATE_IDLE;

    RCLCPP_INFO(this->get_logger(), "MissionPlannerNode started.");
  }

private:

  void onMissionPlan(const MissionPlan::SharedPtr msg)
  {
    bool is_new_mission = false;

    if (!has_plan_ || msg->mission_id.empty()) {
      is_new_mission = true;
    } else if (msg->mission_id != current_plan_.mission_id) {
      is_new_mission = true;
    }

    if (is_new_mission) {
      handleNewMissionPlan(*msg);
    } else {
      handleUpdatedMissionPlan(*msg);
    }
  }

  void handleNewMissionPlan(const MissionPlan & msg)
  {
    if (msg.mission_id.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received MissionPlan with empty mission_id. Ignoring.");
      return;
    }

    if (msg.waypoints.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received MissionPlan with no waypoints. Ignoring.");
      return;
    }

    current_plan_ = msg;
    normalizePlanOptions(current_plan_);

    current_drone_ids_ = uniqueDroneIds(current_plan_.drone_ids);
    if (current_drone_ids_.empty()) {
      RCLCPP_WARN(this->get_logger(), "MissionPlan has no valid drone_ids. Ignoring.");
      has_plan_ = false;
      return;
    }

    has_plan_ = true;
    state_ = MissionStatus::STATE_IDLE;
    drone_states_.clear();

    RCLCPP_INFO(this->get_logger(), "NEW MissionPlan: id=%s, drones=%zu, wps=%zu",
      current_plan_.mission_id.c_str(),
      current_drone_ids_.size(),
      current_plan_.waypoints.size());

    publishMissionPlanForAllDrones();
    publishMissionStatus();
  }

  void handleUpdatedMissionPlan(const MissionPlan & msg)
  {
    if (!has_plan_) {
      handleNewMissionPlan(msg);
      return;
    }

    current_plan_ = msg;
    normalizePlanOptions(current_plan_);
    current_drone_ids_ = uniqueDroneIds(current_plan_.drone_ids);

    RCLCPP_INFO(this->get_logger(), "UPDATED MissionPlan: id=%s, drones=%zu, wps=%zu (state=%u)",
      current_plan_.mission_id.c_str(),
      current_drone_ids_.size(),
      current_plan_.waypoints.size(),
      state_);

    publishMissionPlanForAllDrones();
    publishMissionStatus();
  }

  void onMissionCommand(const MissionCommand::SharedPtr msg)
  {
    if (!has_plan_) {
      RCLCPP_WARN(this->get_logger(), "Received MissionCommand(cmd=%u) but no active MissionPlan. Ignoring.", msg->command);
      return;
    }

    if (!msg->mission_id.empty() &&
        msg->mission_id != current_plan_.mission_id)
    {
      RCLCPP_WARN(this->get_logger(), "MissionCommand mission_id(%s) != current(%s). Ignoring.",
        msg->mission_id.c_str(), current_plan_.mission_id.c_str());
      return;
    }

    switch (msg->command) {
      case MissionCommand::CMD_START:            handleCmdStart();    break;
      case MissionCommand::CMD_PAUSE:            handleCmdPause();    break;
      case MissionCommand::CMD_RESUME:           handleCmdResume();   break;
      case MissionCommand::CMD_EMERGENCY_RETURN: handleCmdEmergency();break;
      case MissionCommand::CMD_ABORT:
      default:                                   handleCmdAbort();    break;
    }
  }

  void handleCmdStart()
  {
    if (!has_plan_) {
      RCLCPP_WARN(this->get_logger(), "CMD_START but no MissionPlan.");
      return;
    }

    if (state_ == MissionStatus::STATE_ACTIVE) {
      RCLCPP_INFO(this->get_logger(), "CMD_START but mission already ACTIVE. Ignoring.");
      return;
    }

    state_ = MissionStatus::STATE_ACTIVE;
    drone_states_.clear();

    RCLCPP_INFO(this->get_logger(), "Mission START: id=%s, drones=%zu",
      current_plan_.mission_id.c_str(),
      current_drone_ids_.size());

    publishMissionPlanForAllDrones();
    publishMissionStatus();
  }

  void handleCmdPause()
  {
    if (state_ != MissionStatus::STATE_ACTIVE) {
      RCLCPP_WARN(this->get_logger(), "CMD_PAUSE but mission not ACTIVE. Ignoring.");
      return;
    }
    state_ = MissionStatus::STATE_PAUSED;
    RCLCPP_INFO(this->get_logger(), "Mission PAUSED.");
    publishMissionStatus();
  }

  void handleCmdResume()
  {
    if (state_ != MissionStatus::STATE_PAUSED) {
      RCLCPP_WARN(this->get_logger(), "CMD_RESUME but mission not PAUSED. Ignoring.");
      return;
    }
    state_ = MissionStatus::STATE_ACTIVE;
    RCLCPP_INFO(this->get_logger(), "Mission RESUMED.");
    publishMissionStatus();
  }

  void handleCmdEmergency()
  {
    if (!has_plan_) {
      RCLCPP_WARN(this->get_logger(), "CMD_EMERGENCY_RETURN but no MissionPlan.");
      return;
    }

    state_ = MissionStatus::STATE_EMERGENCY;
    RCLCPP_WARN(this->get_logger(), "Mission EMERGENCY_RETURN: all drones should RTL.");
    publishMissionStatus();
  }

  void handleCmdAbort()
  {
    if (!has_plan_) {
      RCLCPP_WARN(this->get_logger(), "CMD_ABORT but no MissionPlan.");
      return;
    }

    state_ = MissionStatus::STATE_ABORTED;
    RCLCPP_WARN(this->get_logger(), "Mission ABORTED: all drones stop mission.");
    publishMissionStatus();
  }

  void onMissionStatusFeedback(const MissionStatus::SharedPtr msg)
  {
    if (!has_plan_) return;
    if (!msg->mission_id.empty() &&
        msg->mission_id != current_plan_.mission_id)
    {
      return;
    }

    for (const auto & id : msg->drone_ids) {
      if (id.empty()) continue;
      if (std::find(current_drone_ids_.begin(), current_drone_ids_.end(), id)
          == current_drone_ids_.end()) {
        continue;
      }
      drone_states_[id] = msg->state;
    }

    recalcGlobalStateFromDroneStates();
    publishMissionStatus();
  }

  void recalcGlobalStateFromDroneStates()
  {
    if (!has_plan_ || current_drone_ids_.empty()) {
      state_ = MissionStatus::STATE_IDLE;
      return;
    }

    bool any_emg = false, any_abort = false;
    bool any_active = false, any_paused = false;
    bool all_completed = !current_drone_ids_.empty();

    for (const auto & id : current_drone_ids_) {
      uint8_t s = state_;

      auto it = drone_states_.find(id);
      if (it != drone_states_.end()) {
        s = it->second;
      }

      if (s == MissionStatus::STATE_EMERGENCY) any_emg = true;
      if (s == MissionStatus::STATE_ABORTED)  any_abort = true;
      if (s == MissionStatus::STATE_ACTIVE)   any_active = true;
      if (s == MissionStatus::STATE_PAUSED)   any_paused = true;
      if (s != MissionStatus::STATE_COMPLETED) all_completed = false;
    }

    if (any_emg) {
      state_ = MissionStatus::STATE_EMERGENCY;
    } else if (any_abort) {
      state_ = MissionStatus::STATE_ABORTED;
    } else if (all_completed) {
      state_ = MissionStatus::STATE_COMPLETED;
      RCLCPP_INFO(this->get_logger(), "All drones COMPLETED. MissionStatus -> COMPLETED.");
    } else if (any_active) {
      state_ = MissionStatus::STATE_ACTIVE;
    } else if (any_paused) {
      state_ = MissionStatus::STATE_PAUSED;
    }
  }


  void publishMissionPlanForAllDrones()
  {
    if (!has_plan_) return;
    if (current_drone_ids_.empty()) return;
    if (current_plan_.waypoints.empty()) return;

    std::vector<std::string> drone_ids = current_drone_ids_;
    std::sort(drone_ids.begin(), drone_ids.end());

    for (const auto & drone_id : drone_ids) {
      MissionPlanForDrone out;
      out.mission_id = current_plan_.mission_id;
      out.drone_id   = drone_id;
      out.waypoints  = current_plan_.waypoints;
      out.cruise_altitude_m = current_plan_.cruise_altitude_m;
      out.cruise_speed_mps  = current_plan_.cruise_speed_mps;
      out.landing_mode      = current_plan_.landing_mode;

      out.spacing_type       = MissionPlan::SPACING_DISTANCE;
      out.spacing_value      = current_plan_.spacing_value;
      out.sequential_launch  = true;
      out.order_by_id        = true;
      out.heading_to_next_wp = true;

      mission_plan_for_drone_pub_->publish(out);
    }
  }

  void publishMissionStatus()
  {
    MissionStatus st;
    if (has_plan_) {
      st.mission_id = current_plan_.mission_id;
      st.drone_ids  = current_drone_ids_;
    } else {
      st.mission_id = "";
      st.drone_ids.clear();
    }
    st.state = state_;

    mission_status_pub_->publish(st);
  }

  void normalizePlanOptions(MissionPlan & plan)
  {
    plan.spacing_type       = MissionPlan::SPACING_DISTANCE;
    if (plan.spacing_value <= 0.0f) {
      plan.spacing_value = 5.0f;
    }
    plan.sequential_launch  = true;
    plan.order_by_id        = true;
    plan.heading_to_next_wp = true;

    if (plan.cruise_altitude_m <= 0.0f) {
      plan.cruise_altitude_m = 40.0f;
    }
    if (plan.cruise_speed_mps <= 0.0f) {
      plan.cruise_speed_mps = 5.0f;
    }
  }

  std::vector<std::string> uniqueDroneIds(const std::vector<std::string> & in)
  {
    std::unordered_set<std::string> s;
    std::vector<std::string> out;
    out.reserve(in.size());
    for (const auto & id : in) {
      if (id.empty()) continue;
      if (s.insert(id).second) {
        out.push_back(id);
      }
    }
    return out;
  }

  rclcpp::Subscription<MissionPlan>::SharedPtr      mission_plan_sub_;
  rclcpp::Subscription<MissionCommand>::SharedPtr   mission_command_sub_;
  rclcpp::Publisher<MissionPlanForDrone>::SharedPtr mission_plan_for_drone_pub_;
  rclcpp::Publisher<MissionStatus>::SharedPtr       mission_status_pub_;
  rclcpp::Subscription<MissionStatus>::SharedPtr    mission_status_fb_sub_;

  MissionPlan              current_plan_;
  std::vector<std::string> current_drone_ids_;
  bool                     has_plan_{false};
  uint8_t                  state_{MissionStatus::STATE_IDLE};

  std::unordered_map<std::string, uint8_t> drone_states_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
