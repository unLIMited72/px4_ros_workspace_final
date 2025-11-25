#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import subprocess, re, time

from px4_interface.msg import InstanceEvent, InstanceList

class PX4InstanceManager(Node): 
  def __init__(self):
    super().__init__("px4_instance_manager")
    self.qos_profile_ = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10
    )
    self.active_ids = set()
    self.event_pub = self.create_publisher(InstanceEvent, '/gcs/instance_event', self.qos_profile_)
    self.list_pub = self.create_publisher(InstanceList, '/gcs/instance_list', self.qos_profile_)
    self.timer = self.create_timer(2.0, self.check_instances)

  def detect_instances(self):
    topics = subprocess.getoutput("ros2 topic list")
    matches = re.findall(r'/(\w*px4_\d+)/status', topics)
    return set(sorted(matches))
  
  def publish_event(self, drone_id, event):
    msg = InstanceEvent()
    msg.drone_id = drone_id
    msg.event = event
    msg.timestamp = int(time.time() * 1e6)
    self.event_pub.publish(msg)
    self.get_logger().info(f"[{event.upper()}] {drone_id}")
  
  def publish_list(self):
    msg = InstanceList()
    msg.drone_ids = sorted(list(self.active_ids))
    msg.timestamp = int(time.time() * 1e6)
    self.list_pub.publish(msg)

  def check_instances(self):
    current_ids = self.detect_instances()

    new_ids = current_ids - self.active_ids
    for drone_id in new_ids:
      self.publish_event(drone_id, "join")

    removed_ids = self.active_ids - current_ids
    for drone_id in removed_ids:
      self.publish_event(drone_id, "leave")

    self.active_ids = current_ids
    self.publish_list()

def main(args = None):
  rclpy.init(args = args)
  node = PX4InstanceManager()
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
