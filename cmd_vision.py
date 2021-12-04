#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class Vision():
    def __init__(self):

        rospy.init_node('move', anonymous=True)

        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_callback)
        self.image_pub = rospy.Publisher("/camera/rgb/image_raw/image_with_contours", Image, queue_size=1)

        # The cylinder colors are red, green, and blue. Here we are defining upper and lower bounds for each color.
        # Once the ranges are made, the  program wil be able to decide which color is which.
        self.lower_red  = [0, 70, 50]
        self.upper_red  = [10, 255, 255]
        self.lower_green = [55, 100, 20]
        self.upper_green = [65, 255, 255]
        self.lower_blue  = [110, 100, 20]
        self.upper_blue  = [130, 255, 255]

        self.myColors = np.array([[self.lower_red, self.upper_red],
                         [self.lower_green, self.upper_green],
                         [self.lower_blue, self.upper_blue]])

        # Camera parameters
        # ALPHA_F = scale_factor * focal_length
        # CAMERA_WIDTH, CAMERA_HEIGHT are image dimensions
        self.ALPHA_F       = 1206.89
        self.CAMERA_WIDTH  = 960
        self.CAMERA_HEIGHT = 540

        # Store the center and areas of all 3 pillars
        self.center = [None, None, None]
        self.areas  = None

    def camera_callback(self, image):
        """Function identifies the red, green or blue pillars and publishes the
           cartesian coordinate of its center location within the image to
           /camera/rgb/image_raw/image_with_contour"""
        # Convert the ROS image message into a workable OpenCV image
        img = self.bridge_object.imgmsg_to_cv2(image, desired_encoding="bgr8")
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        img_contours = img.copy()

        # loc_center = np.array([])

        for i, (lower_bound, upper_bound) in enumerate(self.myColors):
            # test if the color of each pixel is within the specified bounds
            mask = cv2.inRange(img_hsv, np.array(lower_bound), np.array(upper_bound))
            # calculate the contours of the masked image
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            for c in contours:
                # find the minimum area of the contours
                rect = cv2.minAreaRect(c)
                # get the corner coordinates
                box = cv2.boxPoints(rect)
                box = np.int0(box)

                cv2.drawContours(img_contours, [box], 0, (0, 255, 255),3)

                # extract center and dimensions from the contour boundary box
                r_center, (r_width, r_height), _ = rect

                # Set the center relative to the middle of the camera frame
                # if self.center[i] is not None:
                self.center[i] = r_center[0]-self.CAMERA_WIDTH

                # Sets the area of the found contour. Larger area on screen
                # means that more of the contour is in view.
                if self.areas is not None:
                    if (r_width * r_height) > self.areas[i]:
                        self.areas[i] = r_width * r_height

                else:
                    self.areas = np.empty(3)
                # np.append(loc_center, center)

        # converting crop_img from cv to ros msg
        msg = self.bridge_object.cv2_to_imgmsg(img_contours, encoding="rgb8")
        # Publishing the Image message
        self.image_pub.publish(msg)

    def get_center(self):
        return self.center

    def get_pillar_areas(self):
        if self.areas is not None:
            return self.areas
        else:
            return np.empty(3)

    def camera_properties(self):
        """Returns the camera properties
            alpha_f: the scaling value multiplied with the focal length
            camera_width: the width of the image frame
            camera_height: the height of the image frame"""

        return self.ALPHA_F, self.CAMERA_WIDTH, self.CAMERA_HEIGHT


def main():
    color_detection = Vision()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

# This is the main function; the program starts here.
if __name__ == '__main__':
    main()
