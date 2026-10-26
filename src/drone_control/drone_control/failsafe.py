#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleCommand, SensorGps, VehicleStatus
import time
import threading

class GpsFailsafeRtlTest(Node):
    def __init__(self):
        super().__init__('gps_failsafe_rtl_test')
        
        # Publishers
        self.gps_pub = self.create_publisher(
            SensorGps,
            '/fmu/in/sensor_gps', 
            10)
            
        self.command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10)
            
        # Subscribers (PX4 publishes /fmu/out/* as BEST_EFFORT + TRANSIENT_LOCAL;
        # a default RELIABLE subscriber is incompatible and receives nothing)
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.status_callback,
            px4_qos)
            
        # State variables
        self.is_armed = False
        self.nav_state = 0
        self.rtl_triggered = False
        
        # GPS simulation variables
        self.gps_enabled = True
        
        # Initial GPS position (Zurich, Switzerland)
        self.lat = 47.397742  # degrees
        self.lon = 8.545594   # degrees
        self.alt = 488.0      # metres above MSL
        
        # GPS publishing timer (5 Hz)
        self.gps_timer = self.create_timer(0.2, self.publish_gps)
        
        # Main test sequence timer (runs once after initialization; cancelled in the callback)
        self.test_timer = self.create_timer(5.0, self.run_test_sequence)
        
        self.get_logger().info('GPS Failsafe RTL Test node initialized')
        
    def status_callback(self, msg):
        # Track vehicle status
        self.is_armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)

        # Log state changes
        if (msg.nav_state != self.nav_state
                and msg.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_RTL):
            self.get_logger().info("Vehicle is in RTL mode")
        self.nav_state = msg.nav_state
    
    def publish_gps(self):
        # Only publish if GPS is enabled
        if not self.gps_enabled:
            return
            
        msg = SensorGps()
        msg.timestamp = int(time.time() * 1000000)
        msg.time_utc_usec = int(time.time() * 1000000)
        # PX4 v1.15 SensorGps uses float degrees / metres (older releases used int lat/lon/alt)
        msg.latitude_deg = self.lat
        msg.longitude_deg = self.lon
        msg.altitude_msl_m = self.alt
        msg.altitude_ellipsoid_m = self.alt
        
        # Set good GPS quality
        msg.fix_type = 3            # 3D fix
        msg.eph = 0.5               # Low horizontal position error
        msg.epv = 0.8               # Low vertical position error
        msg.s_variance_m_s = 0.2
        msg.c_variance_rad = 0.1
        msg.vel_m_s = 0.0           # Stationary
        msg.vel_n_m_s = 0.0
        msg.vel_e_m_s = 0.0
        msg.vel_d_m_s = 0.0
        msg.cog_rad = 0.0
        msg.satellites_used = 10    # Good satellite count
        msg.heading = 0.0
        msg.heading_offset = 0.0
        
        self.gps_pub.publish(msg)
        
    def disable_gps(self):
        # Turn OFF GPS
        self.get_logger().info("Turning OFF GPS")
        self.gps_enabled = False
        
    def enable_gps(self):
        # Turn ON GPS
        self.get_logger().info("Turning ON GPS")
        self.gps_enabled = True
        
    def trigger_rtl(self):
        self.get_logger().info('Initiating Return to Home (RTL)')
        
        msg = VehicleCommand()
        msg.timestamp = int(time.time() * 1000000)
        msg.param1 = 0.0
        msg.command = 20    # MAV_CMD_NAV_RETURN_TO_LAUNCH
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 255
        msg.source_component = 1
        msg.from_external = True
        
        self.command_pub.publish(msg)
        self.rtl_triggered = True
        
    def run_test_sequence(self):
        # One-shot: without this the 5 s timer re-fires and repeats the whole sequence forever
        self.test_timer.cancel()
        self.get_logger().info("Starting GPS Failsafe RTL Test Sequence")
        
        # Step 1: Turn OFF GPS
        self.disable_gps()
        self.get_logger().info("GPS turned OFF")
        
        # Wait for system to detect GPS loss
        time.sleep(3)
        
        # Step 2: Trigger RTL failsafe
        self.trigger_rtl()
        self.get_logger().info("RTL command sent")
        
        # Wait for RTL to start processing (simplified - in real implementation would check nav_state)
        time.sleep(5)
        
        # If you want to wait longer for RTL to make progress before re-enabling GPS:
        time.sleep(15)
        
        # Step 3: Turn GPS back ON
        self.enable_gps()
        self.get_logger().info("GPS turned back ON")
        
        self.get_logger().info("Test sequence completed")

def main(args=None):
    rclpy.init(args=args)
    node = GpsFailsafeRtlTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
