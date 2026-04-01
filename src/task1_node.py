#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class FigureEight:
    def __init__(self):
        rospy.init_node('figure_eight_node')

        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.start_time = rospy.Time.now()
        self.initial_pose_received = False
        self.initial_x = 0
        self.initial_y = 0
        self.initial_yaw = 0

        self.rate = rospy.Rate(10)  # Change rate to 10 Hz for smoother control

        self.desired_time_per_loop = 27.0  # Desired time to complete each loop in seconds
        self.transition_time = 1.75  # Time to move straight between loops in seconds

    def odom_callback(self, msg):
        if not self.initial_pose_received:
            self.initial_x, self.initial_y, self.initial_yaw = self.get_pose_from_msg(msg)
            self.initial_pose_received = True
        else:
            current_x, current_y, current_yaw = self.get_pose_from_msg(msg)
            current_x -= self.initial_x
            current_y -= self.initial_y
            current_yaw -= self.initial_yaw

            formatted_odom = "x={:.2f} [m], y={:.2f} [m], yaw={:.1f} [degrees]".format(current_x, current_y, current_yaw)
            rospy.loginfo_throttle(1, formatted_odom)  # Use loginfo_throttle to maintain 1 Hz logging rate

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

    def calculate_velocities(self):
        elapsed_time = rospy.Time.now() - self.start_time
        elapsed_time_seconds = elapsed_time.to_sec()

        vel_msg = Twist()

        circle_circumference = math.pi * 1  # Circumference of the circle with a diameter of 1 meter
        linear_velocity = circle_circumference / self.desired_time_per_loop
        angular_velocity = linear_velocity / 0.5  # 0.5 is the radius of the circle

        if elapsed_time_seconds < self.desired_time_per_loop:
            vel_msg.linear.x = linear_velocity
            vel_msg.angular.z = angular_velocity
        elif elapsed_time_seconds < self.desired_time_per_loop + self.transition_time:
            vel_msg.linear.x = linear_velocity
            vel_msg.angular.z = 0.0  # Move straight during the transition phase
        elif elapsed_time_seconds < 2 * self.desired_time_per_loop + self.transition_time:
            vel_msg.linear.x = linear_velocity
            vel_msg.angular.z = -angular_velocity
        else:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0

        return vel_msg

    def run(self):
        while not rospy.is_shutdown() and not self.initial_pose_received:
            pass

        try:
            while not rospy.is_shutdown():
                vel_msg = self.calculate_velocities()
                self.vel_pub.publish(vel_msg)
                self.rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo("Figure-eight movement script interrupted.")

if __name__ == '__main__':
    figure_eight = FigureEight()
    figure_eight.run()
