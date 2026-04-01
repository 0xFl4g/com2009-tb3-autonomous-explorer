#!/usr/bin/env python3

import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import random

class ObstacleAvoidance:
    def __init__(self):
        rospy.init_node('obstacle_avoidance_node')
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.start_time = rospy.Time.now()
        self.laser_data = None
        self.rate = rospy.Rate(20)
        self.ctrl_c = False

        self.initial_pose_received = False
        self.initial_x = 0
        self.initial_y = 0
        self.initial_yaw = 0

        self.shutdown_time = rospy.Time.now() + rospy.Duration(90)  # Set shutdown time to 90 seconds
        rospy.on_shutdown(self.shutdownhook)

    def laser_callback(self, msg):
        self.laser_data = msg
        minimum_range = min(self.laser_data.ranges)
        maximum_range = max(self.laser_data.ranges)
        formatted_laser_data = "Max Range={:.2f} [m], Min Range={:.2f} [m]".format(maximum_range, minimum_range)
        rospy.loginfo_throttle(1, formatted_laser_data)

    def odom_callback(self, msg):
        self.current_pose = self.get_pose_from_msg(msg)
        if not self.initial_pose_received:
            self.initial_x, self.initial_y, self.initial_yaw = self.get_pose_from_msg(msg)
            self.initial_pose_received = True
        else:
            current_x, current_y, current_yaw = self.get_pose_from_msg(msg)
            current_x -= self.initial_x
            current_y -= self.initial_y
            current_yaw -= self.initial_yaw

            # formatted_odom = "x={:.2f} [m], y={:.2f} [m], yaw={:.1f} [degrees]".format(current_x, current_y, current_yaw)
            # rospy.loginfo_throttle(1, formatted_odom)  # Use loginfo_throttle to maintain 1 Hz logging rate
    
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
        
    def shutdownhook(self):
        rospy.loginfo("Shutting down program...")
        self.vel_pub.publish(Twist())
        rospy.loginfo("Program ended.")
        self.ctrl_c = True

    def avoid_obstacles(self):
        random_noise = random.uniform(-1, 1)

        if self.laser_data is None:
            return None

        vel_msg = Twist()
        
        # Front = 300 to 60, Back = 120 to 240, Left = 60 to 120, Right = 240 to 300
        # Front is split into directly infront and then the fringes either side of directly infront
        direct_front_ranges = self.laser_data.ranges[0:29] + self.laser_data.ranges[330:360]
        outer_front_ranges = self.laser_data.ranges[30:59] + self.laser_data.ranges[300:329]
        # full_front_ranges = direct_front_ranges + outer_front_ranges
        right_ranges = self.laser_data.ranges[255:284]
        left_ranges = self.laser_data.ranges[75:104]
        # full_back_ranges = self.laser_data.ranges[120:239]
        direct_back_ranges = self.laser_data.ranges[160:219]
        # current_x, current_y, current_yaw = self.current_pose
        # maximum_range = self.laser_data.range_max

        # Reversal detection
        if min(direct_back_ranges) <= 0.275:
            vel_msg.linear.x = 0.2
            vel_msg.angular.z = 0
            self.vel_pub.publish(vel_msg)
            rospy.loginfo_throttle(1, "Minimum Behind Distance reached.")

        # Directly in front detection
        elif min(direct_front_ranges) <= 0.4:
            vel_msg.linear.x = -0.15
            if max(right_ranges) > max(left_ranges):
                    vel_msg.angular.z = -0.75
            else:
                vel_msg.angular.z = 0.75
            self.vel_pub.publish(vel_msg)
            rospy.loginfo_throttle(1, "Minimum Frontal Distance reached.")

        # Left detection
        elif min(left_ranges) <= 0.225:
            vel_msg.linear.x = 0
            vel_msg.angular.z = -0.75
            self.vel_pub.publish(vel_msg)
            rospy.loginfo_throttle(1,"Left detected.")

        # Right detection
        elif min(right_ranges) <= 0.225:
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0.75
            self.vel_pub.publish(vel_msg)
            rospy.loginfo_throttle(1,"Right detected.")

        # Peripheral detection
        elif min(outer_front_ranges) <= 0.3:
            vel_msg.linear.x = -0.15
            if max(right_ranges) > max(left_ranges):
                    vel_msg.angular.z = -0.75
            else:
                vel_msg.angular.z = 0.75
            self.vel_pub.publish(vel_msg)
            rospy.loginfo_throttle(1, "Minimum Peripheral Distance reached.")

        # Normal pathing/exploration instructions
        else:
            vel_msg.linear.x = 0.25
            vel_msg.angular.z = 0 + random_noise
            self.vel_pub.publish(vel_msg)
            rospy.loginfo_throttle(1, "Proceeding as planned.")

    def run(self):
        try:
            while not rospy.is_shutdown() and not self.ctrl_c:
                if rospy.Time.now() >= self.shutdown_time:
                    rospy.loginfo("Time is up. Shutting down program...")
                    self.shutdownhook()  # Call the modified shutdownhook method
                    break  # Exit the loop when time is up

                self.avoid_obstacles()
                self.rate.sleep()

        except rospy.ROSInterruptException:
            rospy.loginfo("Obstacle Avoidance script interrupted.")
            rospy.signal_shutdown("Shutting down...")

if __name__ == '__main__':
    obstacle_avoidance = ObstacleAvoidance()
    obstacle_avoidance.run()