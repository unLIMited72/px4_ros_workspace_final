#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <px4_interface/msg/gyro_control.hpp>
#include <nlohmann/json.hpp>

using px4_interface::msg::GyroControl;
using json = nlohmann::json;

class GcsGyroGateway : public rclcpp::Node
{
public:
  GcsGyroGateway()
  : Node("gcs_gyro_gateway")
  {
    auto gcs_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    sub_ = this->create_subscription<std_msgs::msg::String>(
      "/gcs/gyro_control_raw", gcs_qos,
      std::bind(&GcsGyroGateway::onRaw, this, std::placeholders::_1));

    pub_ = this->create_publisher<GyroControl>("/gcs/gyro_control", gcs_qos);

    RCLCPP_INFO(this->get_logger(), "gcs_gyro_gateway started.");
  }

private:
  void onRaw(const std_msgs::msg::String::SharedPtr msg)
  {
    try {
      auto j = json::parse(msg->data);

      const std::string drone_id = j.at("drone_id").get<std::string>();
      const std::string cmd_str  = j.at("command").get<std::string>();

      GyroControl out{};
      out.drone_id = drone_id;

      if (cmd_str == "TAKEOFF") {
        out.command = GyroControl::COMMAND_TAKEOFF;
        out.target_altitude_m = j.value("target_altitude_m", 5.0f);

      } else if (cmd_str == "LAND") {
        out.command = GyroControl::COMMAND_LAND;

      } else if (cmd_str == "CONTROL") {
        out.command = GyroControl::COMMAND_CONTROL;
        out.vx_mps  = j.value("vx_mps", 0.0f);
        out.vy_mps  = j.value("vy_mps", 0.0f);
        out.yaw_deg = j.value("yaw_deg", 0.0f);

      } else {
        RCLCPP_WARN(this->get_logger(), "Unknown gyro command: %s", cmd_str.c_str());
        return;
      }

      pub_->publish(out);
    }
    catch (const std::exception &e) {
      RCLCPP_WARN(this->get_logger(), "Failed to parse gyro_control_raw: %s", e.what());
    }
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::Publisher<GyroControl>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GcsGyroGateway>());
  rclcpp::shutdown();
  return 0;
}
