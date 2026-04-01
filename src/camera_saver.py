#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import os
from sensor_msgs.msg import Image

class ColourSearch:
    M00_MINIMUM = 10000  # Constant minimum value for 'm00'

    def __init__(self):
        rospy.init_node('color_search_node')
        
        self.target_colour = rospy.get_param('/target_colour', None)
        
        # Convert target colour to lowercase if it's not None
        if self.target_colour:
            self.target_colour = self.target_colour.lower()
        
        rospy.loginfo("TASK 5 BEACON: The target is {}.".format(self.target_colour))

        # If target colour is not set or empty, stop the node
        if not self.target_colour:
            rospy.logerr("No target colour provided. Shutting down.")
            rospy.signal_shutdown("No target colour provided.")
        
        self.mode = rospy.get_param('/mode', 0)

        # Image topic depends on the mode
        if self.mode == 0:
            self.image_topic = '/camera/color/image_raw'
            rospy.loginfo("Running in REAL mode.")
        elif self.mode == 1:
            self.image_topic = '/camera/rgb/image_raw'
            rospy.loginfo("Running in SIMULATION mode.")
        else:
            rospy.logerr("Invalid mode! Use 0 for real and 1 for simulation.")
            rospy.signal_shutdown("Invalid mode.")
            return

        # Subscriber for image data
        self.camera_subscriber = rospy.Subscriber(self.image_topic,
                                                Image, self.camera_callback)
        
        self.cvbridge_interface = CvBridge()

        self.pic_taken = False
        self.previous = 0
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

        # Rate of the node
        self.rate = rospy.Rate(10)

        self.m00 = 0
        self.cy = 0
        self.cv_img = 0


    def shutdown_ops(self):
        # Shutdown operations
        cv2.destroyAllWindows()
        self.ctrl_c = True

    def camera_callback(self, img_data):
        try:
            self.cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logwarn_throttle(1, f"Error while converting image: {e}")
            return

        height, width, _ = self.cv_img.shape

        # Crop dimensions
        crop_width = width - 800
        crop_height = 400
        crop_x = int((width/2) - (crop_width/2))
        crop_y = int((height/2) - (crop_height/2))

        # Cropped and HSV converted image
        crop_img = self.cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        # Simulation colors
        colors_simulation = {
            "red" : ((0, 195, 100),  (5, 255, 255)),
            "blue": ((115, 224, 100),(130, 255, 255)),
            "green": ((55, 100, 100), (65, 255, 255)),
            "yellow": ((28, 195, 100), (32.5, 255, 255))
        }

        # Real world colors
        colors_real = {
            "red" : ((0, 72, 100),  (17, 207, 255)),
            "blue": ((97, 113, 100),(104, 255, 255)),
            "green": ((81, 125, 100), (94, 255, 255)),
            "yellow": ((12, 27, 100), (51, 255, 255))
        }

        # Choose colors based on mode
        colors = colors_simulation if self.mode == 1 else colors_real

        # Ensure valid colour choice
        if self.target_colour not in colors:
            raise ValueError(f"Invalid target colour choice: '{self.target_colour}'")
        
        # Log the color range
        # rospy.loginfo("Searching for colour in range: {} - {}".format(self.target_colour[0], self.target_colour[1]))

        # Colour mask
        lower, upper = colors[self.target_colour]
        mask = cv2.inRange(hsv_img, lower, upper)
        res = cv2.bitwise_and(crop_img, crop_img, mask=mask)

        # Image moments to find centroid
        m = cv2.moments(mask)
        self.m00 = m['m00']
        self.cy = m['m10'] / (m['m00'] + 1e-5)

        if self.m00 > ColourSearch.M00_MINIMUM:
            cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)

        cv2.imshow('cropped image', crop_img)
        cv2.waitKey(1)

    def main(self):
        current_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        dir = os.path.join(current_dir, "snaps")

        # Loop until shutdown
        while not self.ctrl_c:
            # Check if the colour object is detected
            # rospy.loginfo_throttle(1, f"m00: {self.m00}")
            # rospy.loginfo_throttle(1, f"m00 previous: {self.previous}")
            if self.m00 > ColourSearch.M00_MINIMUM and self.m00 > self.previous:
                # if 0 <= self.cy <= 800:
                self.pic_taken = True
                rospy.loginfo_throttle(1, "Beacon Detected!")

                # Save the image
                image_path = os.path.join(dir, "the_beacon.jpg")
                cv2.imwrite(image_path, self.cv_img)

                rospy.loginfo_throttle(1, f"Saved image: {image_path}")
                rospy.loginfo_throttle(1, f"m00: {self.m00}")

                self.previous = self.m00
                self.rate.sleep()


if __name__ == '__main__':
    search_instance = ColourSearch()
    search_instance.main()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as ex:
        rospy.logerr_throttle(1, f"An error occurred: {ex}")

