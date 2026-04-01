#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import roslaunch


class WallFollow:
    def __init__(self):
        rospy.init_node('wall_follow_node')
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=6)
        self.laser_data = None

        self.initial_pose_received = False
        self.initial_x = 0
        self.initial_y = 0
        self.initial_yaw = 0
        
        # initialize the ROSLaunch API
        self.launch = roslaunch.scriptapi.ROSLaunch()
        self.launch.start()
        
        # Define the path where you want to save the map
        self.map_path = "/home/student/catkin_ws/src/com2009_tb3_explorer/maps/"
        self.map_saver_node = roslaunch.core.Node(package="map_server", node_type="map_saver", args="-f " + self.map_path + "task5_map")
        
        self.rate_hz = 10
        self.rate = rospy.Rate(self.rate_hz)
        
        # distance = distance when turn type occurs
        # rate = angular turn rate for turn
        # speed = linear speed during turn
        
        # emergency = if wall too close to the front
        self.emergency_distance = 0.42
        self.emergency_turn_rate = 1
        self.emergency_turn_speed = 0.05
        self.emergency_turn_multiplier = 0.001
        
        # turn = if not emergency but turn is needed
        self.turn_distance = 0.8
        self.long_turn_distance = 0.95
        self.long_turn_rate = 0.65
        self.turn_rate = 0.9
        self.turn_speed = 0.2
        self.turn_time = self.rate_hz * 0.75 # amount of ticks a turn should be held for
        self.turning_right = self.turn_time # current amount of ticks turning
        
        # slight_turn = keep distance to wall
        self.ideal_wall_distance = 0.3 # distance from robot to right wall
        self.slight_turn_distance = 0.08 # deviation from wall distance before slight turn
        self.slight_turn_rate = 0.2
        self.slight_turn_speed = 0.2

        # if robot needs to move straight, helps it bear towards centerline
        self.straight_speed = 0.2
        self.bearing_degrees = 1 # amount of deviation from straight before robot bears towards centerline
        self.bearing_rate = 0.02
        
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

    def shutdownhook(self):
        rospy.loginfo('Shutting down. Robot stopped.')
        self.vel_pub.publish(Twist())
        self.launch.launch(self.map_saver_node)
        rospy.loginfo(f"Saving map at time: {rospy.get_time()}...")
        self.ctrl_c = True
        
    def follow_wall(self):
        if self.laser_data is None:
            return
        
        vel_msg = Twist()
        front_ranges = self.laser_data.ranges[0:15] + self.laser_data.ranges[-15:]
        mid_front = self.laser_data.ranges[15:30] + self.laser_data.ranges[-30:-15]
        wide_front = self.laser_data.ranges[-55:-30] + self.laser_data.ranges[30:55]
        min_front = min(front_ranges + mid_front)
        # min_front_tight = min(front_ranges[40:-40])
        right_ranges = self.laser_data.ranges[255:285]
        min_right = min(right_ranges)
        left_ranges = self.laser_data.ranges[75:115]
        min_left = min(left_ranges)
        front_right_ranges = self.laser_data.ranges[275:350]
        max_front_right = max(front_right_ranges[15:-55])
        min_front_right = min(front_right_ranges)
        avg_front_right = sum(front_right_ranges[15:-20]) / len(front_right_ranges[15:-20])
       
        if (min(wide_front) < 0.234) or (min(mid_front) < 0.21) or (min(front_ranges) < 0.2):
            if sum(self.laser_data.ranges[-45:-15]) > sum(self.laser_data.ranges[15:45]):
                vel_msg.angular.z = 0.3
            else:
                vel_msg.angular.z = -0.3
            if min(self.laser_data.ranges[125:235]) < 0.25:
                if (min(self.laser_data.ranges[150:210]) > min_front):
                    vel_msg.linear.x = -self.emergency_turn_speed / 2
                else:
                    vel_msg.linear.x = self.emergency_turn_speed / 2
            else:
                vel_msg.linear.x = -self.turn_speed
            self.turning_right += self.turn_time

        # can make right turn
        elif (avg_front_right > self.turn_distance):
            rospy.loginfo("turn right")
            if min(wide_front) < 0.28:
                vel_msg.linear.x = self.turn_speed / 2
                vel_msg.angular.z = -self.turn_rate / 2
            elif min(wide_front) < 0.3:
                vel_msg.linear.x = self.turn_speed / 2
                vel_msg.angular.z = -self.turn_rate
            else:
                vel_msg.linear.x = self.turn_speed
                vel_msg.angular.z = -self.turn_rate 
        
        # long turn
        elif max_front_right > self.long_turn_distance:
            rospy.loginfo("long turn right") 
            if min(wide_front) < 0.3:
                vel_msg.linear.x = self.turn_speed
                vel_msg.angular.z = -self.turn_rate / 2
                # self.turning_right += 0.25
                # rospy.loginfo("back left during right turn")
            else:
                vel_msg.linear.x = self.turn_speed
                vel_msg.angular.z = -self.long_turn_rate
            # self.turning_right = 0

             # emergency turn
        elif min_front < self.emergency_distance:
            vel_msg.angular.z = 1.5 * self.emergency_turn_rate * ((self.emergency_distance ** 2) / ((min_front - self.emergency_distance) ** 2))
            if sum(self.laser_data.ranges[-45:-30]) > sum(self.laser_data.ranges[30:45]):
                vel_msg.angular.z = - vel_msg.angular.z
            vel_msg.linear.x = self.emergency_turn_speed
            if vel_msg.angular.z > 1:
                vel_msg.angular.z = 1
            if vel_msg.angular.z < -1:
                vel_msg.angular.z = -1
            rospy.loginfo("emergency turn")
        
        # too close to left wall
        elif min_right > self.ideal_wall_distance and min_left < self.ideal_wall_distance - self.slight_turn_distance:
            vel_msg.linear.x = self.slight_turn_speed
            vel_msg.angular.z = -self.slight_turn_rate
            rospy.loginfo("turn slight right")
        
        # too close to right wall 
        elif min_right < self.ideal_wall_distance - self.slight_turn_distance and min_left > self.ideal_wall_distance:
            rospy.loginfo("turn slight left")
            vel_msg.linear.x = self.slight_turn_speed
            vel_msg.angular.z = self.slight_turn_rate
        
        # go straight
        else:
            if right_ranges.index(min_right) < len(right_ranges) / 2 - self.bearing_degrees or right_ranges.index(min_right) > len(right_ranges) / 2 + self.bearing_degrees:
                vel_msg.angular.z = self.bearing_rate * (right_ranges.index(min_right) - len(right_ranges) / 2)
                if vel_msg.angular.z > 0.5:
                    vel_msg.angular.z = 0.5
            else:
                vel_msg.angular.z = 0
            vel_msg.linear.x = self.straight_speed
            rospy.loginfo_throttle(0.5, "go straight")  
        rospy.loginfo_throttle(0.2,str(vel_msg.angular.z) + " " + str(vel_msg.linear.x))
        self.vel_pub.publish(vel_msg)
    
    def run(self):
        start_time = rospy.Time.now()  # set the start time
        map_save_interval = rospy.Duration(5)  # interval to save the map (e.g., every 60 seconds)
        last_map_save_time = rospy.Time.now()  # last time the map was saved

        while not rospy.is_shutdown():
            elapsed_time = rospy.Time.now() - start_time  # calculate elapsed time

            # if more than 180 seconds have passed, stop the program
            if elapsed_time.to_sec() > 180:
                rospy.signal_shutdown('Time limit reached (180 Seconds)')
                return

            if rospy.is_shutdown():
                return  # Check if a shutdown request has been received

            # if it's time to save the map
            if rospy.Time.now() - last_map_save_time >= map_save_interval:
                self.launch.launch(self.map_saver_node)
                print(f"Saving map at time: {rospy.get_time()}...")
                last_map_save_time = rospy.Time.now()

            self.follow_wall()
            self.rate.sleep()
            
if __name__ == '__main__':
    wall_follow = WallFollow()
    wall_follow.run()
