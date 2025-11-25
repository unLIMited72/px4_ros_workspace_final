#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from px4_interface.msg import PX4Status
from px4_interface.msg import UIStatus
from px4_interface.msg import InstanceList
import time

class PX4StatusAggregator(Node):
    def __init__(self):
        super().__init__("px4_status_aggregator")

        self.qos_profile_ = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.status_pub = self.create_publisher(UIStatus, "/gcs/ui_status", self.qos_profile_)
        self.create_subscription(InstanceList, "/gcs/instance_list", self.instance_list_callback, self.qos_profile_)

        self.status_subs = {}     
        self.drone_ids = []       
        self.timer = self.create_timer(0.5, self.publish_ui_status)

    def instance_list_callback(self, msg: InstanceList):
        current_ids = set(msg.drone_ids)
        old_ids = set(self.status_subs.keys())

        for drone_id in current_ids - old_ids:
            topic_name = f"/{drone_id}/status"
            self.get_logger().info(f"Subscribing to {topic_name}")
            self.status_subs[drone_id] = None
            self.create_subscription(PX4Status, topic_name,
                                     lambda m, id=drone_id: self.status_callback(id, m), self.qos_profile_)

        for drone_id in old_ids - current_ids:
            self.get_logger().info(f"Removing {drone_id}")
            del self.status_subs[drone_id]

        self.drone_ids = sorted(list(current_ids))

    def status_callback(self, drone_id, msg: PX4Status):
        self.status_subs[drone_id] = msg

    def publish_ui_status(self):
        ui_msg = UIStatus()
        ui_msg.timestamp = int(time.time() * 1e6)

        for drone_id in self.drone_ids:
            status = self.status_subs.get(drone_id)
            if not status:
                continue

            ui_msg.drone_ids.append(status.drone_id)
            ui_msg.heartbeats.append(status.heartbeat)
            ui_msg.battery_percentages.append(status.battery_percentage)
            ui_msg.flight_readies.append(status.flight_ready)
            ui_msg.armeds.append(status.armed)
            ui_msg.status_in_flights.append(status.status_in_flight)
            ui_msg.latitudes.append(status.latitude)
            ui_msg.longitudes.append(status.longitude)
            ui_msg.heading_degs.append(status.heading_deg)

        if ui_msg.drone_ids:
            self.status_pub.publish(ui_msg)


def main(args = None):
  rclpy.init(args = args)
  node = PX4StatusAggregator()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()

if __name__ == "__main__":
    main()
