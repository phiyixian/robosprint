import rclpy
from rclpy.node import Node          
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty  

import numpy as np
import cv2
import cv_bridge

try:
    from decision_pkg.actions import junction_action 
except ImportError:
    def junction_action(logger):
        logger.info("Action file not found, but target reached!")
        return True

bridge = cv_bridge.CvBridge()

MIN_AREA = 500
MIN_AREA_TRACK = 5000
LINEAR_SPEED = 0.2
KP = 1.5/100
LOSS_FACTOR = 1.2
TIMER_PERIOD = 0.06
FINALIZATION_PERIOD = 4
MAX_ERROR = 30

lower_bgr_values = np.array([31, 42, 53])
upper_bgr_values = np.array([255, 255, 255])

def crop_size(height, width):
    return (1*height//3, height, width//4, 3*width//4)

image_input = 0
error = 0
just_seen_line = False
just_seen_right_mark = False
should_move = False
right_mark_count = 0
finalization_countdown = None

junction_detected_flag = False 
target_reached = False

def start_follower_callback(request, response):
    global should_move, right_mark_count, finalization_countdown, target_reached
    should_move = True
    right_mark_count = 0
    finalization_countdown = None
    target_reached = False
    return response

def stop_follower_callback(request, response):
    global should_move, finalization_countdown
    should_move = False
    finalization_countdown = None
    return response

def image_callback(msg):
    global image_input
    image_input = bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

def get_contour_data(mask, out):
    global crop_w_start
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    mark = {}
    line = {}

    for contour in contours:
        M = cv2.moments(contour)
        if M['m00'] > MIN_AREA:
            if (M['m00'] > MIN_AREA_TRACK):
                line['x'] = crop_w_start + int(M["m10"]/M["m00"])
                line['y'] = int(M["m01"]/M["m00"])

                cv2.drawContours(out, [contour], -1, (255,255,0), 1) 
                cv2.putText(out, str(M['m00']), (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])),
                    cv2.FONT_HERSHEY_PLAIN, 2, (255,255,0), 2)
            else:
                if (not mark) or (mark['y'] > int(M["m01"]/M["m00"])):
                    mark['y'] = int(M["m01"]/M["m00"])
                    mark['x'] = crop_w_start + int(M["m10"]/M["m00"])

                    cv2.drawContours(out, [contour], -1, (255,0,255), 1) 
                    cv2.putText(out, str(M['m00']), (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])),
                        cv2.FONT_HERSHEY_PLAIN, 2, (255,0,255), 2)

    if mark and line:
        mark_side = "right" if mark['x'] > line['x'] else "left"
    else:
        mark_side = None

    return (line, mark_side)

def timer_callback():
    global error, image_input, just_seen_line, just_seen_right_mark
    global should_move, right_mark_count, finalization_countdown, target_reached, junction_detected_flag

    if type(image_input) != np.ndarray:
        return

    height, width, _ = image_input.shape
    image = image_input.copy()

    global crop_w_start
    crop_h_start, crop_h_stop, crop_w_start, crop_w_stop = crop_size(height, width)

    crop = image[crop_h_start:crop_h_stop, crop_w_start:crop_w_stop]
    mask = cv2.inRange(crop, lower_bgr_values, upper_bgr_values)

    output = image
    line, mark_side = get_contour_data(mask, output[crop_h_start:crop_h_stop, crop_w_start:crop_w_stop])  
    
    message = Twist()

    if line and mark_side:
        if not junction_detected_flag:
            print(f"Junction detected on {mark_side}! Waiting for line command")
            junction_detected_flag = True
    elif not mark_side:
        junction_detected_flag = False

    if mark_side != None:
        if (mark_side == "right") and (finalization_countdown == None) and \
            (abs(error) <= MAX_ERROR) and (not just_seen_right_mark):

            right_mark_count += 1
            print(f"Right Mark Count: {right_mark_count}")

            # Trigger Action at Target (e.g., 3rd right mark)
            if right_mark_count >= 3 and not target_reached:
                print("TARGET ZONE REACHED! Triggering action...")
                should_move = False # Stop for action
                publisher.publish(Twist()) # Force stop
                trigger_target_action(node.get_logger())
                target_reached = True

            if right_mark_count > 1:
                finalization_countdown = int(FINALIZATION_PERIOD / TIMER_PERIOD) + 1
                print("finalize")
            
            just_seen_right_mark = True
    else:
        just_seen_right_mark = False

    # --- PID MOVEMENT ---
    if line:
        x = line['x']
        error = x - width//2
        message.linear.x = LINEAR_SPEED
        just_seen_line = True
        cv2.circle(output, (line['x'], crop_h_start + line['y']), 5, (0,255,0), 7)
    else:
        if just_seen_line:
            just_seen_line = False
            error = error * LOSS_FACTOR
        message.linear.x = 0.0

    message.angular.z = float(error) * -KP
    
    cv2.rectangle(output, (crop_w_start, crop_h_start), (crop_w_stop, crop_h_stop), (0,0,255), 2)
    cv2.imshow("output", output)
    cv2.waitKey(5)

    if finalization_countdown != None:
        if finalization_countdown > 0:
            finalization_countdown -= 1
        elif finalization_countdown == 0:
            should_move = False

    if should_move:
        publisher.publish(message)
    else:
        publisher.publish(Twist())

def main():
    rclpy.init()
    global node, publisher
    node = Node('follower')
    publisher = node.create_publisher(Twist, '/cmd_vel', rclpy.qos.qos_profile_system_default)
    node.create_subscription(Image, 'camera/image_raw', image_callback, rclpy.qos.qos_profile_sensor_data)
    node.create_timer(TIMER_PERIOD, timer_callback)
    node.create_service(Empty, 'start_follower', start_follower_callback)
    node.create_service(Empty, 'stop_follower', stop_follower_callback)
    rclpy.spin(node)

if __name__ == '__main__':
    try:
        main()
    except (KeyboardInterrupt, rclpy.exceptions.ROSInterruptException):
        if 'publisher' in globals():
            publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
        exit()