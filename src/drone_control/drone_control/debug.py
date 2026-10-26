#!/usr/bin/env python3
"""
PX4 Debug Script - Check ROS2 connections and PX4 status
Run this script to diagnose connection issues between ROS2 and PX4
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from px4_msgs.msg import VehicleStatus, VehicleCommand
import time

class PX4DebugNode(Node):
    def __init__(self):
        super().__init__('px4_debug')
        
        # QoS profile
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscriber for vehicle status
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        
        # Publisher for vehicle commands
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        
        self.status_received = False
        self.last_status_time = time.time()
        
        # Timer for status check
        self.timer = self.create_timer(2.0, self.check_connection)
        
        self.get_logger().info("PX4 Debug Node started - Checking connection...")
        
    def vehicle_status_callback(self, msg):
        """Callback for vehicle status messages."""
        self.status_received = True
        self.last_status_time = time.time()
        
        # Decode arming state
        arming_states = {
            0: "DISARMED",
            1: "ARMED", 
            2: "ARMED",
            3: "STANDBY_ERROR",
            4: "SHUTDOWN",
            5: "IN_AIR_RESTORE"
        }
        
        # Decode navigation state
        nav_states = {
            0: "MANUAL",
            1: "ALTCTL", 
            2: "POSCTL",
            3: "AUTO_MISSION",
            4: "AUTO_LOITER",
            5: "AUTO_RTL",
            6: "AUTO_ACRO", 
            7: "OFFBOARD",
            8: "STAB",
            9: "RATTITUDE",
            10: "AUTO_TAKEOFF",
            11: "AUTO_LAND",
            12: "AUTO_FOLLOW_TARGET",
            13: "AUTO_PRECLAND",
            14: "ORBIT",
            17: "AUTO_VTOL_TAKEOFF"
        }
        
        arming_state_str = arming_states.get(msg.arming_state, f"UNKNOWN({msg.arming_state})")
        nav_state_str = nav_states.get(msg.nav_state, f"UNKNOWN({msg.nav_state})")
        
        self.get_logger().info(f"Vehicle Status:")
        self.get_logger().info(f"  Arming State: {arming_state_str}")
        self.get_logger().info(f"  Navigation State: {nav_state_str}")
        self.get_logger().info(f"  Pre-flight Check: {'PASS' if msg.pre_flight_checks_pass else 'FAIL'}")
        self.get_logger().info(f"  Failsafe: {'ACTIVE' if msg.failsafe else 'INACTIVE'}")
        self.get_logger().info(f"  System ID: {msg.system_id}")
        self.get_logger().info(f"  Component ID: {msg.component_id}")
        self.get_logger().info("---")
        
    def check_connection(self):
        """Check if we're receiving status messages."""
        current_time = time.time()
        if not self.status_received:
            self.get_logger().error("❌ No vehicle status messages received!")
            self.get_logger().error("   Check if MicroXRCEAgent is running on port 8888")
            self.get_logger().error("   Check if PX4 SITL is running and connected")
        elif current_time - self.last_status_time > 5.0:
            self.get_logger().warn("⚠️  Vehicle status messages stopped!")
        else:
            self.get_logger().info("✅ Vehicle connection active")

def main():
    print("PX4 Debug Script")
    print("================")
    print("This script will check your ROS2 <-> PX4 connection")
    print("Make sure the following are running:")
    print("1. MicroXRCEAgent udp4 -p 8888")
    print("2. PX4 SITL: cd ~/PX4-Autopilot && make px4_sitl gz_x500")
    print("3. QGroundControl (optional)")
    print("")
    
    rclpy.init()
    debug_node = PX4DebugNode()
    
    try:
        rclpy.spin(debug_node)
    except KeyboardInterrupt:
        print("\nShutting down debug node...")
    finally:
        debug_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
