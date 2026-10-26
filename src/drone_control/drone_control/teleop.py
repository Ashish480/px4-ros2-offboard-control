#!/usr/bin/env python3
import sys
import math
import geometry_msgs.msg
import rclpy
import std_msgs.msg

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


msg = """
This node takes keypresses from the keyboard and publishes them
as Twist messages. 
Using the arrow keys and WASD you have Mode 2 RC controls.
W: Up
S: Down
A: Yaw Left
D: Yaw Right
Up Arrow: Pitch Forward
Down Arrow: Pitch Backward
Left Arrow: Roll Left
Right Arrow: Roll Right

Press SPACE to arm/disarm the drone
"""

moveBindings = {
    'w': (0, 0, 1, 0), #Z+
    's': (0, 0, -1, 0),#Z-
    'a': (0, 0, 0, -1), #Yaw+
    'd': (0, 0, 0, 1),#Yaw-
    '\x1b[A' : (0, 1, 0, 0),  #Up Arrow
    '\x1b[B' : (0, -1, 0, 0), #Down Arrow
    '\x1b[C' : (-1, 0, 0, 0), #Right Arrow
    '\x1b[D' : (1, 0, 0, 0),  #Left Arrow
}


speedBindings = {
    # 'q': (1.1, 1.1),
    # 'z': (.9, .9),
    # 'w': (1.1, 1),
    # 'x': (.9, 1),
    # 'e': (1, 1.1),
    # 'c': (1, .9),
}


def getKey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        if key == '\x1b':  # if the first character is \x1b, we might be dealing with an arrow key
            additional_chars = sys.stdin.read(2)  # read the next two characters
            key += additional_chars  # append these characters to the key
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key



def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def vels(speed, turn):
    return 'currently:\tspeed %s\tturn %s ' % (speed, turn)


def main():
    settings = saveTerminalSettings()

    rclpy.init()

    node = rclpy.create_node('teleop_twist_keyboard')

    qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10
    )


    pub = node.create_publisher(geometry_msgs.msg.Twist, '/offboard_velocity_cmd', qos_profile)

    arm_toggle = False
    arm_pub = node.create_publisher(std_msgs.msg.Bool, '/arm_message', qos_profile)

    # Add variables for elliptical path
    ellipse_mode = False
    ellipse_major_axis = 3.0  # meters - longer axis
    ellipse_minor_axis = 1.5  # meters - shorter axis
    ellipse_speed = 0.5      # m/s
    ellipse_height = 1.5     # meters
    ellipse_angle = 0.0      # radians
    ellipse_update_rate = 0.1  # seconds

    speed = 0.5
    turn = .2
    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0
    status = 0.0
    x_val = 0.0
    y_val = 0.0
    z_val = 0.0
    yaw_val = 0.0

    try:
        print(msg)
        # print(vels(speed, turn))
        
        # Add to message about 'o' key
        print("Press 'o' to arm the drone and fly in an elliptical orbit")
        print("Press 'p' to exit elliptical orbit mode")
        
        last_ellipse_update = node.get_clock().now()
        
        while True:
            key = getKey(settings)
            
            # Check for elliptical mode exit
            if key == 'p' and ellipse_mode:
                ellipse_mode = False
                print("Exiting elliptical orbit mode")
                # Reset velocities
                x_val = 0.0
                y_val = 0.0
                z_val = 0.0
                yaw_val = 0.0
            
            # Handle elliptical path mode
            if ellipse_mode:
                current_time = node.get_clock().now()
                if (current_time - last_ellipse_update).nanoseconds / 1e9 >= ellipse_update_rate:
                    # Update ellipse position
                    # For an elliptical orbit, we need to adjust the angular velocity
                    # to maintain constant speed along the path
                    
                    # Calculate the position on the ellipse
                    x_pos = ellipse_major_axis * math.cos(ellipse_angle)
                    y_pos = ellipse_minor_axis * math.sin(ellipse_angle)
                    
                    # Calculate the velocity components needed to move along the ellipse
                    # The derivatives of the position equations give us the velocity components
                    x_vel = -ellipse_major_axis * math.sin(ellipse_angle) * ellipse_speed
                    y_vel = ellipse_minor_axis * math.cos(ellipse_angle) * ellipse_speed
                    
                    # Adjust the angle for the next iteration
                    # The angle increment is adjusted based on the position to maintain approximately
                    # constant speed along the elliptical path
                    semi_perimeter = math.sqrt((x_pos * x_pos) + (y_pos * y_pos))
                    if semi_perimeter > 0.1:  # Avoid division by zero
                        angle_increment = ellipse_speed * ellipse_update_rate / semi_perimeter
                        ellipse_angle += angle_increment
                    else:
                        ellipse_angle += ellipse_speed * ellipse_update_rate / ellipse_minor_axis
                        
                    if ellipse_angle > 2 * math.pi:  # Reset angle after full ellipse
                        ellipse_angle = 0.0
                    
                    # Set new values - convert position-based calculation to velocity commands
                    x_val = x_vel
                    y_val = y_vel
                    z_val = ellipse_height - z_val  # Adjust to maintain target height
                    
                    # Calculate yaw to point in the direction of movement
                    target_yaw = math.atan2(y_vel, x_vel)
                    # Convert to drone's coordinate system if needed
                    yaw_val = target_yaw * 0.5  # Scale down for smoother yaw control
                    
                    last_ellipse_update = current_time
                    
                # Skip normal key processing
                twist = geometry_msgs.msg.Twist()
                twist.linear.x = x_val
                twist.linear.y = y_val
                twist.linear.z = z_val
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = yaw_val
                pub.publish(twist)
                print("ELLIPSE MODE | X:", twist.linear.x, "   Y:", twist.linear.y, 
                      "   Z:", twist.linear.z, "   Yaw:", twist.angular.z,
                      "   Angle:", ellipse_angle)
                continue
                
            # Check for 'o' key to activate elliptical path
            if key == 'o' and not ellipse_mode:
                print("Activating elliptical orbit mode!")
                
                # Arm the drone if not already armed
                if not arm_toggle:
                    arm_toggle = True
                    arm_msg = std_msgs.msg.Bool()
                    arm_msg.data = arm_toggle
                    arm_pub.publish(arm_msg)
                    print("Arming drone...")
                
                # Start the elliptical path mode
                ellipse_mode = True
                ellipse_angle = 0.0
                # Set initial height gain
                z_val = ellipse_height
                last_ellipse_update = node.get_clock().now()
                continue
                
            # Regular key processing
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            
            else:
                x = 0.0
                y = 0.0
                z = 0.0
                th = 0.0
                if (key == '\x03'):
                    break

            if key == ' ':  # ASCII value for space
                arm_toggle = not arm_toggle  # Flip the value of arm_toggle
                arm_msg = std_msgs.msg.Bool()
                arm_msg.data = arm_toggle
                arm_pub.publish(arm_msg)
                print(f"Arm toggle is now: {arm_toggle}")

            twist = geometry_msgs.msg.Twist()
            
            x_val = (x * speed) + x_val
            y_val = (y * speed) + y_val
            z_val = (z * speed) + z_val
            yaw_val = (th * turn) + yaw_val
            twist.linear.x = x_val
            twist.linear.y = y_val
            twist.linear.z = z_val
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = yaw_val
            pub.publish(twist)
            print("X:", twist.linear.x, "   Y:", twist.linear.y, "   Z:", twist.linear.z, "   Yaw:", twist.angular.z)
            

    except Exception as e:
        print(e)

    finally:
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()
