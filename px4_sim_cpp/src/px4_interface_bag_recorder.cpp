#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <mutex>
#include <chrono>
#include <filesystem>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

#include <px4_interface/msg/gyro_control.hpp>
#include <px4_interface/msg/instance_event.hpp>
#include <px4_interface/msg/instance_list.hpp>
#include <px4_interface/msg/mission_command.hpp>
#include <px4_interface/msg/mission_plan.hpp>
#include <px4_interface/msg/mission_plan_for_drone.hpp>
#include <px4_interface/msg/mission_status.hpp>
#include <px4_interface/msg/mission_waypoint.hpp>
#include <px4_interface/msg/px4_status.hpp>
#include <px4_interface/msg/ui_status.hpp>

using namespace std::chrono_literals;

using px4_interface::msg::GyroControl;
using px4_interface::msg::InstanceEvent;
using px4_interface::msg::InstanceList;
using px4_interface::msg::MissionCommand;
using px4_interface::msg::MissionPlan;
using px4_interface::msg::MissionPlanForDrone;
using px4_interface::msg::MissionStatus;
using px4_interface::msg::PX4Status;
using px4_interface::msg::UIStatus;

class Px4InterfaceBagRecorder : public rclcpp::Node
{
public:
  Px4InterfaceBagRecorder()
  : Node("px4_interface_bag_recorder")
  {
    base_dir_       = declare_parameter<std::string>("base_dir", "bags");
    bag_prefix_     = declare_parameter<std::string>("bag_prefix", "px4_if");
    max_duration_s_ = declare_parameter<double>("max_duration_s", 1800.0);
    max_size_mb_    = declare_parameter<double>("max_size_mb", 1024.0);
    use_wall_time_  = declare_parameter<bool>("use_wall_time", true);

    std::error_code ec;
    std::filesystem::create_directories(base_dir_, ec);
    if (ec) {
      RCLCPP_WARN(get_logger(),
                  "Failed to create base_dir '%s': %s",
                  base_dir_.c_str(), ec.message().c_str());
    }

    open_new_bag();

    add_topic<GyroControl>         ("/gcs/gyro_control",             0);
    add_topic<MissionPlan>         ("/gcs/mission_plan",             0);
    add_topic<MissionCommand>      ("/gcs/mission_command",          0);
    add_topic<MissionPlanForDrone> ("/gcs/mission_plan_for_drone",   0);
    add_topic<MissionStatus>       ("/gcs/mission_status",           0);
    add_topic<MissionStatus>       ("/gcs/mission_status_feedback",  0);
    add_topic<InstanceEvent>       ("/gcs/instance_event",           0);
    add_topic<InstanceList>        ("/gcs/instance_list",            0);
    add_topic<UIStatus>            ("/gcs/ui_status",              500);

    instance_list_sub_ = this->create_subscription<InstanceList>(
      "/gcs/instance_list",
      rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
      std::bind(&Px4InterfaceBagRecorder::onInstanceList, this, std::placeholders::_1)
    );

    RCLCPP_INFO(get_logger(),
      "px4_interface_bag_recorder started. base_dir='%s', prefix='%s', "
      "max_duration_s=%.1f, max_size_mb=%.1f",
      base_dir_.c_str(), bag_prefix_.c_str(),
      max_duration_s_, max_size_mb_);
  }

private:
  struct TopicInfo
  {
    std::string type_name;
    uint64_t    min_interval_ns{0};
    bool        registered{false};
    rclcpp::Time last_time{0,0,RCL_ROS_TIME};
  };

  template<typename MsgT>
  void add_topic(const std::string & topic_name, uint64_t min_interval_ms)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (topics_.count(topic_name) > 0) {
      return;
    }

    TopicInfo info;
    info.type_name = rosidl_generator_traits::name<MsgT>();
    info.min_interval_ns =
      (min_interval_ms > 0) ? (min_interval_ms * 1000000ull) : 0ull;
    info.registered = false;
    info.last_time = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    topics_[topic_name] = info;

    auto qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable();

    auto sub = this->create_subscription<MsgT>(
      topic_name,
      qos,
      [this, topic_name](const typename MsgT::SharedPtr msg)
      {
        this->handle_message<MsgT>(topic_name, msg);
      });

    subscriptions_.push_back(sub);

    RCLCPP_INFO(get_logger(),
      "Subscribed for recording: %s (%s), min_interval_ms=%llu",
      topic_name.c_str(), info.type_name.c_str(),
      static_cast<unsigned long long>(min_interval_ms));
  }

  void onInstanceList(const InstanceList::SharedPtr msg)
  {
    std::unordered_set<std::string> ids(msg->drone_ids.begin(), msg->drone_ids.end());
    std::vector<std::string> to_add;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (const auto & id : ids) {
        if (id.empty()) continue;
        std::string topic = "/" + id + "/status";
        if (topics_.find(topic) == topics_.end()) {
          to_add.push_back(topic);
        }
      }
    }

    for (const auto & topic : to_add) {
      add_topic<PX4Status>(topic, 500);
    }
  }

  template<typename MsgT>
  void handle_message(const std::string & topic_name,
                      const typename MsgT::SharedPtr & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    const rclcpp::Time now = use_wall_time_ ? rclcpp::Clock(RCL_SYSTEM_TIME).now() : this->now();

    auto it = topics_.find(topic_name);
    if (it == topics_.end()) {
      return;
    }
    TopicInfo & info = it->second;

    if (info.min_interval_ns > 0 && info.last_time.nanoseconds() > 0) {
      const int64_t diff = now.nanoseconds() - info.last_time.nanoseconds();
      if (diff >= 0) {
        const uint64_t dt = static_cast<uint64_t>(diff);
        if (dt < info.min_interval_ns) {
          return;
        }
      }
    }

    if (!info.registered) {
      rosbag2_storage::TopicMetadata meta;
      meta.name = topic_name;
      meta.type = info.type_name;
      meta.serialization_format = "cdr";
      writer_.create_topic(meta);
      info.registered = true;

      RCLCPP_INFO(get_logger(),
        "Registered topic in bag: %s (%s)",
        meta.name.c_str(), meta.type.c_str());
    }

    rclcpp::Serialization<MsgT> serializer;
    rclcpp::SerializedMessage serialized;
    serializer.serialize_message(msg.get(), &serialized);

    auto bag_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
    bag_msg->topic_name = topic_name;
    bag_msg->time_stamp = now.nanoseconds();

    bag_msg->serialized_data = std::make_shared<rcutils_uint8_array_t>(serialized.get_rcl_serialized_message());

    writer_.write(bag_msg);

    info.last_time = now;
    written_bytes_ += bag_msg->serialized_data->buffer_length;

    check_rotation(now);
  }

  void open_new_bag()
  {
    if (bag_opened_) {
      try {
        writer_.close();
      } catch (const std::exception & e) {
        RCLCPP_WARN(get_logger(), "Error closing previous bag: %s", e.what());
      }
    }

    auto st = std::chrono::system_clock::now();
    std::time_t tt = std::chrono::system_clock::to_time_t(st);
    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &tt);
#else
    localtime_r(&tt, &tm);
#endif

    char buf[32];
    std::snprintf(buf, sizeof(buf),
                  "%04d%02d%02d_%02d%02d%02d",
                  tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                  tm.tm_hour, tm.tm_min, tm.tm_sec);

    std::string uri = base_dir_ + "/" + bag_prefix_ + "_" + buf;
    if (bag_index_ > 0) {
      uri += "_" + std::to_string(bag_index_);
    }

    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = uri;
    storage_options.storage_id = "sqlite3";
    storage_options.max_bagfile_size = 0;
    storage_options.max_bagfile_duration = 0;

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    writer_.open(storage_options, converter_options);

    bag_start_time_ = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    written_bytes_  = 0;
    bag_opened_     = true;
    bag_index_++;

    RCLCPP_INFO(get_logger(), "Opened new bag: %s", uri.c_str());
  }

  void check_rotation(const rclcpp::Time & now)
  {
    if (!bag_opened_) return;

    bool rotate = false;

    if (max_duration_s_ > 0.0) {
      double elapsed = (now - bag_start_time_).seconds();
      if (elapsed >= max_duration_s_) {
        rotate = true;
      }
    }

    if (!rotate && max_size_mb_ > 0.0) {
      double limit_bytes = max_size_mb_ * 1024.0 * 1024.0;
      if (static_cast<double>(written_bytes_) >= limit_bytes) {
        rotate = true;
      }
    }

    if (rotate) {
      open_new_bag();
    }
  }

  std::mutex mutex_;

  rosbag2_cpp::Writer writer_;
  bool        bag_opened_{false};
  int         bag_index_{0};
  rclcpp::Time bag_start_time_{0,0,RCL_SYSTEM_TIME};
  size_t      written_bytes_{0};

  std::string base_dir_;
  std::string bag_prefix_;
  double      max_duration_s_;
  double      max_size_mb_;
  bool        use_wall_time_;

  std::unordered_map<std::string, TopicInfo> topics_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;

  rclcpp::Subscription<InstanceList>::SharedPtr instance_list_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Px4InterfaceBagRecorder>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
