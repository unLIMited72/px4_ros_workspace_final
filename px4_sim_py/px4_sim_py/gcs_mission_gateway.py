#!/usr/bin/env python3
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import String
from px4_interface.msg import MissionPlan, MissionCommand, MissionWaypoint

class GCSMissionGateway(Node):
    def __init__(self):
        super().__init__("gcs_mission_gateway")

        self.qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, depth=10)

        self.plan_sub = self.create_subscription(String, "/gcs/mission_plan_raw", self.plan_callback, self.qos)
        self.cmd_sub = self.create_subscription(String, "/gcs/mission_command_raw", self.command_callback, self.qos)
        self.plan_pub = self.create_publisher(MissionPlan, "/gcs/mission_plan", self.qos)
        self.cmd_pub = self.create_publisher(MissionCommand, "/gcs/mission_command", self.qos)

        self.get_logger().info("GCSMissionGateway started.")

    def plan_callback(self, msg: String):
        self.get_logger().info(f"Received mission_plan_raw: {msg.data}")
        try:
            data = json.loads(msg.data)

            plan = MissionPlan()
            plan.mission_id = data.get("mission_id", "").strip()
            plan.drone_ids = list(data.get("drone_ids", []))

            waypoints = []
            for wp in data.get("waypoints", []):
                w = MissionWaypoint()
                w.seq = int(wp.get("seq", 0))
                w.lat = float(wp.get("lat"))
                w.lon = float(wp.get("lon"))
                w.alt = float(wp.get("alt", 0.0))
                w.hold_time = float(wp.get("hold_time", 0.0))
                waypoints.append(w)
            plan.waypoints = waypoints

            plan.cruise_altitude_m = float(
                data.get("cruise_altitude_m",
                data.get("cruise_altitude", 40.0))
            )
            plan.cruise_speed_mps = float(
                data.get("cruise_speed_mps",
                data.get("cruise_speed", 5.0))
            )

            landing_mode_str = str(data.get("landing_mode", "HOME")).upper()
            if landing_mode_str == "LAST_WAYPOINT":
                plan.landing_mode = MissionPlan.LANDING_LAST_WAYPOINT
            else:
                plan.landing_mode = MissionPlan.LANDING_HOME

            spacing_val = float(data.get("spacing_value", data.get("spacing_distance", 10.0)))
            if spacing_val <= 0.0:
                spacing_val = 5.0

            plan.spacing_type       = MissionPlan.SPACING_DISTANCE
            plan.spacing_value      = spacing_val
            plan.sequential_launch  = True
            plan.order_by_id        = True
            plan.heading_to_next_wp = True

            self.plan_pub.publish(plan)
            self.get_logger().info(
                f"Published MissionPlan: {plan.mission_id} "
                f"(drones={len(plan.drone_ids)}, wps={len(plan.waypoints)}, "
                f"alt={plan.cruise_altitude_m}, v={plan.cruise_speed_mps}, "
                f"spacing={plan.spacing_value})"
            )

        except Exception as e:
            self.get_logger().error(f"Failed to parse mission_plan_raw: {e}")

    def command_callback(self, msg: String):
        self.get_logger().info(f"Received mission_command_raw: {msg.data}")
        try:
            data = json.loads(msg.data)

            mc = MissionCommand()
            mc.mission_id = data.get("mission_id", "").strip()

            cmd_str = str(data.get("command", "")).upper()
            if cmd_str == "START": mc.command = MissionCommand.CMD_START
            elif cmd_str == "PAUSE": mc.command = MissionCommand.CMD_PAUSE
            elif cmd_str == "RESUME": mc.command = MissionCommand.CMD_RESUME
            elif cmd_str in ("EMERGENCY_RETURN", "EMERGENCY"): mc.command = MissionCommand.CMD_EMERGENCY_RETURN
            else: mc.command = MissionCommand.CMD_ABORT

            self.cmd_pub.publish(mc)
            self.get_logger().info(f"Published MissionCommand: mission_id={mc.mission_id}, cmd={mc.command}")

        except Exception as e:
            self.get_logger().error(f"Failed to parse mission_command_raw: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = GCSMissionGateway()
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
