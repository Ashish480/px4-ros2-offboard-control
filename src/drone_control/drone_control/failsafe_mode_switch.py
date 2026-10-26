#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
import signal
import sys
from px4_msgs.msg import VehicleCommand, OffboardControlMode, VehicleGlobalPosition, VehicleStatus
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class GPSFailureSimulator(Node):
    def __init__(self):
        super().__init__('gps_failure_simulator')
        
        # Configure QoS profile for PX4 communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, 
            '/fmu/in/vehicle_command', 
            qos_profile)
        
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            qos_profile)
            
        # Subscribers
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile)
            
        self.vehicle_global_position_subscriber = self.create_subscription(
            VehicleGlobalPosition,
            '/fmu/out/vehicle_global_position',
            self.vehicle_global_position_callback,
            qos_profile)

        # Initialize variables
        self.vehicle_status = None
        self.vehicle_global_position = None
        self.gps_failure_active = False
        self.home_position = None
        
        # Timer for publishing offboard control mode
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Setup signal handler for keyboard interrupt
        signal.signal(signal.SIGINT, self.signal_handler)
        
        self.get_logger().info('GPS Failure Simulator started. Press Ctrl+C to exit.')

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg
        
    def vehicle_global_position_callback(self, msg):
        self.vehicle_global_position = msg
        
        # Store home position on first position message
        if self.home_position is None and msg.lat != 0 and msg.lon != 0:
            self.home_position = {'lat': msg.lat, 'lon': msg.lon, 'alt': msg.alt}
            self.get_logger().info(f'Home position set: lat={msg.lat}, lon={msg.lon}, alt={msg.alt}')

    def timer_callback(self):
        # Publish offboard control mode (needed to keep connection active)
        self.publish_offboard_control_mode()
        
    def publish_offboard_control_mode(self):
        # Minimal offboard control mode message to keep connection
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        
        self.offboard_control_mode_publisher.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param3=0.0, param4=0.0, param5=0.0, param6=0.0, param7=0.0):
        """Publish vehicle command"""
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = param3
        msg.param4 = param4
        msg.param5 = param5
        msg.param6 = param6
        msg.param7 = param7
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        
        self.vehicle_command_publisher.publish(msg)
        self.get_logger().info(f'Vehicle command published: {command}')

    def simulate_gps_failure(self):
        """Simulate GPS failure by commanding the vehicle to disable GPS"""
        if self.vehicle_global_position is None:
            self.get_logger().error('No position data available. Make sure the vehicle has a GPS fix.')
            return False
            
        # Store current position before disabling GPS
        if self.home_position is None:
            self.home_position = {
                'lat': self.vehicle_global_position.lat,
                'lon': self.vehicle_global_position.lon,
                'alt': self.vehicle_global_position.alt
            }
            self.get_logger().info(f'Home position set: {self.home_position}')
            
        # Command: MAV_CMD_DO_SET_MODE to set mode to RTL
        # This simulates the PX4 behavior when GPS is lost - we trigger RTL mode directly
        self.get_logger().info('Simulating GPS failure and triggering RTL...')
        self.gps_failure_active = True
        
        # Set mode to RTL (Return to Launch)
        # param1: 1 = set mode
        # param2: 4 = auto mode
        # param3: 5 = sub-mode: RTL (PX4_CUSTOM_SUB_MODE_AUTO_RTL)
        self.publish_vehicle_command(176, 1.0, 4.0, 5.0)  # MAV_CMD_DO_SET_MODE - Set to RTL
        
        return True
        
    def restore_gps(self):
        """Restore GPS functionality"""
        if not self.gps_failure_active:
            return
            
        self.get_logger().info('Restoring GPS functionality...')
        self.gps_failure_active = False
        
        # Switch back to hold mode
        # param1: 1 = set mode
        # param2: 4 = auto mode
        # param3: 3 = auto hold (PX4_CUSTOM_SUB_MODE_AUTO_LOITER)
        self.publish_vehicle_command(176, 1.0, 4.0, 3.0)  # MAV_CMD_DO_SET_MODE - Set to Hold
        
        self.get_logger().info('GPS functionality restored')

    def signal_handler(self, sig, frame):
        """Handle keyboard interrupt"""
        self.get_logger().info('Keyboard interrupt detected')
        if self.gps_failure_active:
            self.restore_gps()
        self.get_logger().info('Exiting...')
        sys.exit(0)

    def spin_for(self, seconds, until=None):
        """Process callbacks for up to `seconds`, returning early once `until()` is true"""
        end_time = time.time() + seconds
        while time.time() < end_time:
            if until is not None and until():
                return
            rclpy.spin_once(self, timeout_sec=0.1)

    def run(self):
        """Run the GPS failure simulation"""
        # Spin (so subscriptions are serviced) until we have position data
        self.spin_for(10.0, until=lambda: self.vehicle_global_position is not None)

        if self.simulate_gps_failure():
            self.get_logger().info('GPS failure simulation started. Running for 10 seconds...')

            # Spin for 10 seconds or until keyboard interrupt
            try:
                self.spin_for(10.0)
                self.restore_gps()
                self.get_logger().info('GPS failure simulation completed.')
            except KeyboardInterrupt:
                self.restore_gps()
                self.get_logger().info('GPS failure simulation interrupted.')

def main(args=None):
    rclpy.init(args=args)
    
    gps_failure_simulator = GPSFailureSimulator()
    
    try:
        # Run the GPS failure simulation
        gps_failure_simulator.run()
        
        # Keep the node running to handle any additional commands
        rclpy.spin(gps_failure_simulator)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        gps_failure_simulator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
