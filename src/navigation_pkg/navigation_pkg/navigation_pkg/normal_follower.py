import rclpy
from rclpy.node import Node          
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import spidev
import sys
import time
from decision_pkg.pick_place import PickPlaceMechanism 

try:
    from decision_pkg.actions import junction_action 
except ImportError:
    def junction_action(logger):
        logger.info("Junction action function called (fallback).")
        return True

# Constants
KP = 0.002
KI = 0.0001
TIMER_PERIOD = 0.05
LINEAR_SPEED = 0.15
BLACK_THRESHOLD = 600  # adjustment for black sensoring
WHITE_THRESHOLD = 150  # adjustment for white sensoring
TARGET_JUNCTION = 3 

# Global variables
integral_error = 0.0
junction_count = 0
state = 'FOLLOWING' 
action_done = False

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1350000

def read_adc(channel):
    adc = spi.xfer2([1, (8 + channel) << 4, 0])
    return ((adc[1] & 3) << 8) + adc[2]

def timer_callback():
    global integral_error, junction_count, state, action_done
    
    # Read sensors
    l_val = read_adc(0)
    c_val = read_adc(1)
    r_val = read_adc(2)
    
    l_black = l_val > BLACK_THRESHOLD
    c_black = c_val > BLACK_THRESHOLD
    r_black = r_val > BLACK_THRESHOLD

    all_white = l_val < WHITE_THRESHOLD and c_val < WHITE_THRESHOLD and r_val < WHITE_THRESHOLD

    message = Twist()

    if l_black and c_black and r_black:
        if state == 'FOLLOWING':
            # Stop the robot to process cube
            publisher.publish(Twist()) 
            
            junction_count += 1
            node.get_logger().info(f"Junction {junction_count} detected! Checking Cube...")

            state = 'JUNCTION_DETECTED'
    
    elif state == 'JUNCTION_DETECTED' and not (l_black and r_black):
        state = 'FOLLOWING'

    if all_white and junction_count >= TARGET_JUNCTION and not action_done:
        state = 'ACTION_EXECUTION'
        
        # Stop the robot
        message = Twist()
        message.linear.x = 0.0
        message.angular.z = 0.0
        publisher.publish(message)
        
        node.get_logger().info("Target Zone Reached! Placing cubes...")

        # Trigger the placement mechanism
        success = mechanism.place_cubes() 
        
        if success:
            action_done = True
            state = 'FINISHED'
            node.get_logger().info("Target Action Complete. Robot stop.")
            return 
        else:
            return 

    if state == 'FOLLOWING':
        error = r_val - l_val
        integral_error += error
        integral_error = max(min(integral_error, 1000), -1000)
        
        steering = (KP * error) + (KI * integral_error)
        
        if max(l_val, c_val, r_val) > BLACK_THRESHOLD:
            message.linear.x = LINEAR_SPEED
            message.angular.z = -float(steering)

        else:
            message.linear.x = 0.05 
            
    publisher.publish(message)

def main(args=None):
    rclpy.init(args=args)
    global node, publisher, mechanism
    node = Node('pi_follower_node')
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)

    mechanism = PickPlaceMechanism(node.get_logger())
    
    timer = node.create_timer(TIMER_PERIOD, timer_callback)
    
    node.get_logger().info("PI Follower Node Activated")
    rclpy.spin(node)

if __name__ == '__main__':
    try:
        main()
    except (KeyboardInterrupt, rclpy.exceptions.ROSInterruptException):
        pass
    finally:
        if 'publisher' in globals():
            publisher.publish(Twist())
        spi.close()
        rclpy.shutdown()