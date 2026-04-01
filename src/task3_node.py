#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class WallFollow:
    def __init__(self):
        rospy.init_node('wall_follow_node')
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
        self.laser_data = None
        self.rate_hz = 25
        self.rate = rospy.Rate(self.rate_hz)
        self.initial_pose_received = False
        self.initial_x = 0
        self.initial_y = 0
        self.initial_yaw = 0
        
        # distance = distance when turn type occurs
        # rate = angular turn rate for turn
        # speed = linear speed during turn
        
        # emergency = if wall too close to the front
        self.emergency_distance = 0.48
        self.emergency_turn_rate = 1.6
        self.emergency_turn_speed = 0.2
        
        # turn = if not emergency but turn is needed
        self.turn_distance = 0.7
        self.turn_rate = 1.1
        self.turn_speed = 0.22
        self.turn_time = self.rate_hz * 1.9 # amount of ticks a turn should be held for
        self.turning_right = self.turn_time # current amount of ticks turning
        
        # slight_turn = keep distance to wall
        self.ideal_wall_distance = 0.27 # distance from robot to right wall
        self.slight_turn_distance = 0.05 # deviation from wall distance before slight turn
        self.slight_turn_rate = 0.25
        self.slight_turn_speed = 0.24

        # if robot needs to move straight, helps it bear towards centerline
        self.straight_speed = 0.26
        self.bearing_degrees = 1 # amount of deviation from straight before robot bears towards centerline
        self.bearing_rate = 0.05
        
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)
        
    def laser_callback(self, msg):
        self.laser_data = msg
    
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
            rospy.loginfo_throttle(4, formatted_odom)  # Use loginfo_throttle to maintain 1 Hz logging rate
    
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
        # publish an empty twist message to stop the robot
        # (by default all velocities will be zero):
        self.vel_pub.publish(Twist())
        rospy.loginfo('Shutting down. Robot stopped.')
        self.ctrl_c = True
        
    def follow_wall(self):
        if self.laser_data is None:
            return
        
        vel_msg = Twist()
        front_ranges = self.laser_data.ranges[0:10] + self.laser_data.ranges[-10:]
        min_front = min(front_ranges)
        right_ranges = self.laser_data.ranges[255:285]
        min_right = min(right_ranges)
        front_right_ranges = self.laser_data.ranges[285:340]
        min_front_right = min(front_right_ranges)
        avg_front_right = sum(front_right_ranges[:-30]) / len(front_right_ranges[:-30])
        # left_ranges = self.laser_data.ranges[85:95]
        # min_left = min(left_ranges)
        # front_right_ranges = self.laser_data.ranges[280:310]
        
        wall_detected = [0,0,0,0]
        # wall_detected = 0
        if max(self.laser_data.ranges[-15:] + self.laser_data.ranges[:15]) <= 0.68:
            wall_detected[0] = 1
        else: 
            wall_detected[0] = 0
        if max(self.laser_data.ranges[75:105]) <= 0.45:
            wall_detected[1] = 1
        else:
            wall_detected[1] = 0
        if max(self.laser_data.ranges[165:195]) <= 0.45:
            wall_detected[2] = 1
        else:
            wall_detected[2] = 0
        if max(self.laser_data.ranges[255:285]) <= 0.45:
            wall_detected[3] = 1
        else:
            wall_detected[3] = 0

        # formatted_ranges = "front={:.2f} [m], frontright={:.2f} [m], right={:.2f} [m], length={:.1f}, timeinc={:.3f}, angleinc{:.3f}".format(min_front,min_front_right,min_right,len(self.laser_data.ranges),self.laser_data.time_increment,self.laser_data.angle_increment)
        # rospy.loginfo_throttle(2, formatted_ranges)  # Use loginfo_throttle to maintain 1 Hz logging rate
        
        # carry on right turn
        if self.turning_right < self.turn_time :
            if min_front_right < 0.3:
                vel_msg.linear.x = self.turn_speed / 1.5
                vel_msg.angular.z = self.turn_rate / 4
                self.turning_right += 0.25
                # rospy.loginfo("back left during right turn")
            # elif min_front_right < 0.33:
            #     vel_msg.linear.x = self.turn_speed / 1.5
            #     vel_msg.angular.z = -self.turn_rate / 1.5
            #     self.turning_right += 0.5
                # rospy.loginfo("back left during right turn")
            else:
                vel_msg.linear.x = self.turn_speed
                vel_msg.angular.z = -self.turn_rate
                self.turning_right += 1
                # rospy.loginfo_throttle(2,"turning right")

        # emergency turn
        elif min_front < self.emergency_distance:
            vel_msg.linear.x = self.emergency_turn_speed * (min_front ** 2) / (self.emergency_distance ** 2)
            # if min_left < self.emergency_distance and min_left < min_right:
            #     vel_msg.angular.z = -self.emergency_turn_rate
            #     rospy.loginfo_throttle(2,"turn right") 
            # else:
            vel_msg.angular.z = self.emergency_turn_rate
            # rospy.loginfo_throttle(0.25,"turn left" + str(self.emergency_turn_speed * (min_front ** 2) / (self.emergency_distance ** 2)))
            # rospy.loginfo_throttle(2,"turn left") 
        
        # can make right turn
        elif avg_front_right > self.turn_distance:
            vel_msg.linear.x = self.turn_speed
            vel_msg.angular.z = -self.turn_rate
            self.turning_right = 0
            rospy.loginfo("turn right")  

        # dead-end detection
        elif sum(wall_detected) >= 3 and wall_detected[0] == 1:
        # elif sum(self.laser_data.ranges) /360 < 0.6:
            rospy.loginfo("dead end")  
            rospy.loginfo(wall_detected)
            # if wall_detected[0] != 1:
                # vel_msg.linear.x = self.straight_speed
                # if right_ranges.index(min_right) < len(right_ranges) / 2 - self.bearing_degrees:
                #     vel_msg.angular.z = self.bearing_rate * (right_ranges.index(min_right) - len(right_ranges) / 2)
                # elif right_ranges.index(min_right) > len(right_ranges) / 2 + self.bearing_degrees: 
                #     vel_msg.angular.z = self.bearing_rate * (right_ranges.index(min_right) - len(right_ranges) / 2)
                # else:
                #     vel_msg.angular.z = 0
            # else:
            vel_msg.angular.z = self.emergency_turn_rate
            vel_msg.linear.x = -0.1

        # # prepare for emergency turn
        # elif min_front < self.emergency_distance + 0.05:
        #     vel_msg.linear.x = self.turn_speed / 3
        #     if min_left < self.emergency_distance and min_left < min_right:
        #         vel_msg.angular.z = -self.emergency_turn_rate / 2
        #         rospy.loginfo_throttle(2,"prepare to turn right") 
        #     else:
        #         vel_msg.angular.z = self.emergency_turn_rate / 2
        #         rospy.loginfo_throttle(2,"prepare to turn left") 
        
        # too close to right wall 
        elif min_right < self.ideal_wall_distance - self.slight_turn_distance:
            vel_msg.linear.x = self.slight_turn_speed
            vel_msg.angular.z = self.slight_turn_rate
            # rospy.loginfo("turn slight left")
        
        # too far from right wall
        elif min_right > self.ideal_wall_distance + self.slight_turn_distance:
            vel_msg.linear.x = self.slight_turn_speed
            vel_msg.angular.z = -self.slight_turn_rate
            # rospy.loginfo("turn slight right")
        
        # go straight
        else:
            if right_ranges.index(min_right) < len(right_ranges) / 2 - self.bearing_degrees or right_ranges.index(min_right) > len(right_ranges) / 2 + self.bearing_degrees:
                vel_msg.angular.z = self.bearing_rate * (right_ranges.index(min_right) - len(right_ranges) / 2)
            else:
                vel_msg.angular.z = 0
            vel_msg.linear.x = self.straight_speed
            # rospy.loginfo_throttle(0.5, "go straight")  

        self.vel_pub.publish(vel_msg)
    
    def run(self):
        start_time = rospy.Time.now()  # set the start time

        while not rospy.is_shutdown():
            elapsed_time = rospy.Time.now() - start_time  # calculate elapsed time

            # if more than 150 seconds have passed, stop the program
            if elapsed_time.to_sec() > 150:
                rospy.signal_shutdown('Time limit reached')
                return

            self.follow_wall()
            self.rate.sleep()

            
if __name__ == '__main__':
    wall_follow = WallFollow()
    wall_follow.run()