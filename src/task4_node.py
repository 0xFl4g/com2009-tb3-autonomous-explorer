#!/usr/bin/env python3

import rospy
import math 
from math import sqrt

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from tb3 import Tb3LaserScan

import cv2
from cv_bridge import CvBridge, CvBridgeError 

from sensor_msgs.msg import Image 

class Beaconing():

    def __init__(self):

        rospy.init_node('beaconing_node')

        self.laser_data = None
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.img_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.lidar_scan = Tb3LaserScan()

        self.start_time = rospy.Time.now()
        self.find_turquoise = False
        self.find_green = False
        self.find_blue = False
        self.find_yellow = False
        self.find_purple = False
        self.find_red = False
        self.find_beacon = False
        
        self.startup = True

        # define the robot pose variables and initialise them to zero:
        # variables for the robot's "current position":
        self.x = 0.0
        self.y = 0.0
        self.theta_z = 0.0
        # variables for a "reference position":
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta_z0 = 0.0
        self.ctrl_c = False
        self.rate = rospy.Rate(10)
        self.cvbridge_interface = CvBridge()
        rospy.on_shutdown(self.shutdownhook)

        self.wave=-3

        # Initialize colour detection attributes
        self.cy_turquoise = 0
        self.cy_green = 0
        self.cy_blue = 0
        self.cy_yellow = 0
        self.cy_purple = 0
        self.cy_red = 0
        self.turquoise_detected = False
        self.green_detected = False
        self.blue_detected = False
        self.yellow_detected = False
        self.purple_detected = False
        self.red_detected = False
        self.cz_t = 0
        self.cy_t = 0
        self.cz_g = 0
        self.cy_g = 0
        self.cz_b = 0
        self.cy_b = 0
        self.cz_y = 0
        self.cy_y = 0
        self.cz_p = 0
        self.cy_p = 0
        self.cz_r = 0
        self.cy_r = 0

        # Turn behind to face camera to start zone colour
        vel_msg = Twist()
        start_time = rospy.Time.now().to_sec()
        action_time = rospy.Time.now().to_sec()

        while action_time <= (start_time + 4.7):
            action_time = rospy.Time.now().to_sec()
            vel_msg.angular.z = 0.7
            vel_msg.linear.x = 0
            self.vel_pub.publish(vel_msg)

        vel_msg.angular.z = 0.0
        vel_msg.linear.x = 0
        self.vel_pub.publish(vel_msg)
        self.detect_colour = True

        # Perform initial scan of start zone colour
        colour = "unknown"
        while self.detect_colour:
            if self.cy_turquoise != 0:
                colour = "turquoise"
                self.find_turquoise = True
            elif self.cy_green != 0:
                colour = "green"
                self.find_green = True
            elif self.cy_blue != 0:
                colour = "blue"
                self.find_blue = True
            elif self.cy_yellow != 0:
                colour = "yellow"
                self.find_yellow = True
            elif self.cy_purple != 0:
                colour = "purple"
                self.find_purple = True
            elif self.cy_red != 0:
                colour = "red"
                self.find_red = True

            self.detect_colour = False
            break

        print(f"SEARCH INITIATED: The target beacon colour is {colour}.")

        # Turn to face camera for exploration
        start_time = rospy.Time.now().to_sec()
        action_time = rospy.Time.now().to_sec()

        while action_time <= (start_time + 4.7):
            action_time = rospy.Time.now().to_sec()
            vel_msg.angular.z = -0.6
            vel_msg.linear.x = 0
            self.vel_pub.publish(vel_msg)

        vel_msg.angular.z = 0.0
        vel_msg.linear.x = 0
        self.vel_pub.publish(vel_msg)

        # Move fwd just a bit outside of box
        start_time = rospy.Time.now().to_sec()
        action_time = rospy.Time.now().to_sec()
        while action_time <= (start_time + 1):
            action_time = rospy.Time.now().to_sec()
            vel_msg.angular.z = 0
            vel_msg.linear.x = 0.2
            self.vel_pub.publish(vel_msg)

        vel_msg.angular.z = 0.0
        vel_msg.linear.x = 0
        self.vel_pub.publish(vel_msg)


    @staticmethod
    def get_pose_from_msg(msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        _, _, yaw = euler_from_quaternion([msg.pose.pose.orientation.x,
                                           msg.pose.pose.orientation.y,
                                           msg.pose.pose.orientation.z,
                                           msg.pose.pose.orientation.w])
        yaw = math.degrees(yaw)
        return x, y, yaw

    def laser_callback(self, msg):
        self.laser_data = msg


    def avoid_obstacles(self):
        if self.laser_data is None:
            return None

        vel_msg = Twist()
        
        # Front = 300 to 60, Back = 120 to 240, Left = 60 to 120, Right = 240 to 300
        # Front is split into directly infront and then the fringes either side of directly infront
        direct_front_ranges = self.laser_data.ranges[0:35] + self.laser_data.ranges[330:360]
        outer_front_ranges = self.laser_data.ranges[30:59] + self.laser_data.ranges[300:329]
        right_ranges = self.laser_data.ranges[239:299]
        left_ranges = self.laser_data.ranges[60:119]

        tight_front_ranges = self.laser_data.ranges[0:15] + self.laser_data.ranges[-15:]
        mid_front = self.laser_data.ranges[15:30] + self.laser_data.ranges[-30:-15]
        wide_front = self.laser_data.ranges[-55:-30] + self.laser_data.ranges[30:55]
        # emergency avoidance
        if (min(wide_front) < 0.234) or (min(mid_front) < 0.21) or (min(tight_front_ranges) < 0.2):
            if sum(self.laser_data.ranges[-45:-15]) > sum(self.laser_data.ranges[15:45]):
                vel_msg.angular.z = 0.3
            else:
                vel_msg.angular.z = -0.3
            if min(self.laser_data.ranges[125:235]) < 0.25:
                if (min(self.laser_data.ranges[125:235]) > min(tight_front_ranges + mid_front)):
                    vel_msg.linear.x = -0.025
                else:
                    vel_msg.linear.x = 0.025
            else:
                vel_msg.linear.x = -0.2

        elif min(direct_front_ranges) <= 0.4 or min(outer_front_ranges) <= 0.2 or min(right_ranges) <= 0.1 or min(left_ranges) <= 0.1:
            vel_msg.linear.x = -0.26
            if max(right_ranges) > max(left_ranges):
                vel_msg.angular.z = -0.8
            else:
                vel_msg.angular.z = 0.8
        else:
            vel_msg.linear.x = 0.26
            if self.wave < 0:
                vel_msg.angular.z =  0.8  # Forwards + no rotation
            else:
                vel_msg.angular.z =  -0.8
            self.wave +=1
        
        if self.wave >= 14:
            self.wave = -15

        self.vel_pub.publish(vel_msg)
          

    def odom_callback(self, topic_data: Odometry):
        # obtain relevant topic data: pose (position and orientation):
        pose = topic_data.pose.pose
        position = pose.position
        orientation = pose.orientation

        # obtain the robot's position co-ords:
        pos_x = position.x
        pos_y = position.y

        # convert orientation co-ords to roll, pitch & yaw 
        # (theta_x, theta_y, theta_z):
        (roll, pitch, yaw) = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w], "sxyz"
        )

        # We're only interested in x, y and theta_z
        # so assign these to class variables (so that we can
        # access them elsewhere within our Square() class):
        self.x = pos_x
        self.y = pos_y
        self.theta_z = yaw

        # If this is the first time that the callback_function has run
        # (e.g. the first time a message has been received), then
        # obtain a "reference position" (used to determine how far
        # the robot has moved during its current operation)
        if self.startup:
            # don't initialise again:
            self.startup = False
            # set the reference position:
            self.x0 = self.x
            self.y0 = self.y
            self.theta_z0 = self.theta_z   
                

    def shutdownhook(self):
        # self.robot_controller.stop()
        cv2.destroyAllWindows()
        self.vel_pub.publish(Twist())
        self.ctrl_c = True


    def camera_callback(self, img_data):
        # Convert the image from ROS to OpenCV format
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
            return

        # Reset detected flags each frame
        self.turquoise_detected = False
        self.green_detected = False
        self.blue_detected = False
        self.yellow_detected = False
        self.purple_detected = False
        self.red_detected = False

                #in order:  turquoise,       green,          blue,            yellow,         purple,          red
        lower_thresholds = [(75, 240, 100), (50, 150, 100), (115, 225, 100), (20, 185, 100), (140, 145, 100), (0, 190, 100)] 
        upper_thresholds = [(95, 265, 255), (65, 255, 255), (130, 255, 255), (35, 215, 255), (155, 260, 255), (10, 260, 255)]

        height , width, _ = cv_img.shape
        
        crop_width = 1000
        crop_height = int(height / 5)
        # 150 is an offset compensation. Camera (in simulation) is offset from center of robot.
        crop_x = int((width / 2) - (crop_width / 2)) - 150 
        crop_z0 = height - crop_height - 10
        cropped_img = cv_img[crop_z0:crop_z0+crop_height, crop_x:crop_x+crop_width]

        # Convert the image to HSV color space
        hsv = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)

        # Apply color mask
        for i in range(6):
                if i == 0:
                    self.mask_turquoise = cv2.inRange(hsv, lower_thresholds[0], upper_thresholds[0])
                    self.turquoise = cv2.moments(self.mask_turquoise)
                    self.cy_turquoise = int(self.turquoise['m10'] / (self.turquoise['m00'] + 1e-5))

                elif i == 1:
                    self.mask_green = cv2.inRange(hsv, lower_thresholds[1], upper_thresholds[1])
                    self.green = cv2.moments(self.mask_green)
                    self.cy_green = int(self.green['m10'] / (self.green['m00'] + 1e-5))

                elif i == 2:
                    self.mask_blue = cv2.inRange(hsv, lower_thresholds[2], upper_thresholds[2])
                    self.blue = cv2.moments(self.mask_blue)
                    self.cy_blue = int(self.blue['m10'] / (self.blue['m00'] + 1e-5))
                
                elif i == 3:
                    self.mask_yellow = cv2.inRange(hsv, lower_thresholds[3], upper_thresholds[3])
                    self.yellow = cv2.moments(self.mask_yellow)
                    self.cy_yellow = int(self.yellow['m10'] / (self.yellow['m00'] + 1e-5))

                elif i == 4:
                    self.mask_purple = cv2.inRange(hsv, lower_thresholds[4], upper_thresholds[4])
                    self.purple = cv2.moments(self.mask_purple)
                    self.cy_purple = int(self.purple['m10'] / (self.purple['m00'] + 1e-5))

                elif i == 5:
                    self.mask_red = cv2.inRange(hsv, lower_thresholds[5], upper_thresholds[5])
                    self.red = cv2.moments(self.mask_red)
                    self.cy_red = int(self.red['m10'] / (self.red['m00'] + 1e-5))

        
        # Detect colour and set cy cz circle values
        if self.cy_turquoise != 0 and self.turquoise['m00'] > 100000:
            self.turquoise_detected = True
            mask = cv2.inRange(hsv, (75, 240, 100), (95, 265, 255))

            res = cv2.bitwise_and(cropped_img, cropped_img, mask = mask)

            m = cv2.moments(mask)            
            self.cz_t = m['m01'] / (m['m00'] + 1e-5) 
            self.cy_t = m['m10'] / (m['m00'] + 1e-5)

            cv2.circle(res, (int(self.cy_t), int(self.cz_t)), 10, (255, 0, 0), 2)
     
        elif self.cy_green != 0 and self.green['m00'] > 100000: 
            self.green_detected = True
            mask = cv2.inRange(hsv, (50, 150, 100), (65, 255, 255))

            res = cv2.bitwise_and(cropped_img, cropped_img, mask = mask)

            m = cv2.moments(mask)            
            self.cz_g = m['m01'] / (m['m00'] + 1e-5) 
            self.cy_g = m['m10'] / (m['m00'] + 1e-5)

            cv2.circle(res, (int(self.cy_g), int(self.cz_g)), 10, (255, 0, 0), 2)
    
        elif self.cy_blue != 0 and self.blue['m00'] > 100000:
            self.blue_detected = True
            mask = cv2.inRange(hsv, (115, 225, 100), (130, 255, 255))

            res = cv2.bitwise_and(cropped_img, cropped_img, mask = mask)

            m = cv2.moments(mask)            
            self.cz_b = m['m01'] / (m['m00'] + 1e-5) 
            self.cy_b = m['m10'] / (m['m00'] + 1e-5)

            cv2.circle(res, (int(self.cy_b), int(self.cz_b)), 10, (255, 0, 0), 2)
              
        elif self.cy_yellow != 0 and self.yellow['m00'] > 100000:
            self.yellow_detected = True
            mask = cv2.inRange(hsv, (20, 185, 100), (35, 215, 255))

            res = cv2.bitwise_and(cropped_img, cropped_img, mask = mask)

            m = cv2.moments(mask)            
            self.cz_y = m['m01'] / (m['m00'] + 1e-5) 
            self.cy_y = m['m10'] / (m['m00'] + 1e-5)
            
            cv2.circle(res, (int(self.cy_y), int(self.cz_y)), 10, (255, 0, 0), 2)

        elif self.cy_purple != 0 and self.purple['m00'] > 100000:
            self.purple_detected = True
            mask = cv2.inRange(hsv, (140, 145, 100), (155, 260, 255))

            res = cv2.bitwise_and(cropped_img, cropped_img, mask = mask)

            m = cv2.moments(mask)            
            self.cz_p = m['m01'] / (m['m00'] + 1e-5) 
            self.cy_p = m['m10'] / (m['m00'] + 1e-5)

            cv2.circle(res, (int(self.cy_p), int(self.cz_p)), 10, (255, 0, 0), 2)

        elif self.cy_red != 0 and self.red['m00'] > 100000:
            self.red_detected = True
            mask = cv2.inRange(hsv, (0, 190, 100), (10, 260, 255))

            res = cv2.bitwise_and(cropped_img, cropped_img, mask = mask)

            m = cv2.moments(mask)            
            self.cz_r = m['m01'] / (m['m00'] + 1e-5) 
            self.cy_r = m['m10'] / (m['m00'] + 1e-5)

            cv2.circle(res, (int(self.cy_r), int(self.cz_r)), 10, (255, 0, 0), 2)
        

    def main(self):
        while not self.ctrl_c:

            self.vel_msg = Twist()

            beacon_seen = False

            current_displacement = sqrt((pow(self.x - self.x0, 2)) + pow(self.y - self.y0, 2))

            if current_displacement <= 3:
                start_zone = True
            else: 
                start_zone = False

            if (start_zone == False) and (beacon_seen == False):
                if (self.find_turquoise and self.turquoise_detected) and ((self.lidar_scan.front < 0.6) and ((self.cy_t >= 560 - 100)  and (self.cy_t < 560 + 100 ))):
                        print("TARGET DETECTED: Beaconing initiated.")
                        beacon_seen = True
                        self.find_beacon = True 
                        print("BEACONING COMPLETE: The robot has now stopped.")      
                        break

                elif (self.find_green and self.green_detected) and ((self.lidar_scan.front < 0.6) and ((self.cy_g >= 560 - 100)  and (self.cy_g < 560 + 100 ))):           
                        print("TARGET DETECTED: Beaconing initiated.")
                        beacon_seen = True
                        self.find_beacon = True 
                        print("BEACONING COMPLETE: The robot has now stopped.")      
                        break

                elif (self.find_blue and self.blue_detected) and ((self.lidar_scan.front < 0.6) and ((self.cy_b >= 560 - 100)  and (self.cy_b < 560 + 100 ))):        
                        print("TARGET DETECTED: Beaconing initiated.")
                        beacon_seen = True
                        self.find_beacon = True       
                        print("BEACONING COMPLETE: The robot has now stopped.")
                        break

                elif (self.find_yellow and self.yellow_detected) and ((self.lidar_scan.front < 0.6) and ((self.cy_y >= 560 - 100)  and (self.cy_y < 560 + 100 ))):
                        print("TARGET DETECTED: Beaconing initiated.")
                        beacon_seen = True
                        self.find_beacon = True       
                        print("BEACONING COMPLETE: The robot has now stopped.")
                        break
                
                elif (self.find_purple and self.purple_detected) and ((self.lidar_scan.front < 0.6) and ((self.cy_p >= 560 - 100)  and (self.cy_p < 560 + 100 ))):
                        print("TARGET DETECTED: Beaconing initiated.")
                        beacon_seen = True
                        self.find_beacon = True       
                        print("BEACONING COMPLETE: The robot has now stopped.")
                        break

                elif (self.find_red and self.red_detected) and ((self.lidar_scan.front < 0.6) and ((self.cy_r >= 560 - 100)  and (self.cy_r < 560 + 100 ))):
                        print("TARGET DETECTED: Beaconing initiated.")
                        beacon_seen = True
                        self.find_beacon = True       
                        print("BEACONING COMPLETE: The robot has now stopped.")
                        break

            
            if self.find_beacon: 
                self.vel_pub.publish(Twist())

            else:
                self.avoid_obstacles()
                self.rate.sleep()

if __name__ == '__main__':
        lf_instance = Beaconing()
        try:
            lf_instance.main()
        except rospy.ROSInterruptException:
            pass
 
