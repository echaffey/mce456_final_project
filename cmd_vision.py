#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class Color_Detection(object):
    def __init__(self):
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_callback)
        self.bridge_object = CvBridge()
        self.image_pub = rospy.Publisher("/camera/rgb/image_raw/image_with_contours", Image, queue_size=1)

        # The cylinder colors are red, green, and blue. Here we are defining upper and lower bounds for each color.
        # Once the ranges are made, the  program wil be able to decide which color is which.
        self.lower_red = [175, 100, 20]
        self.upper_red = [5, 255, 255]
        self.lower_green = [55, 100, 20]
        self.upper_green = [65, 255, 255]
        self.lower_blue = [110, 100, 20]
        self.upper_blue = [130, 255, 255]

        self.myColors = [[self.lower_red, self.upper_red],  # these are the color ranges for red
                         [self.lower_green, self.upper_green],  # these are the color ranges for green
                         [self.lower_blue, self.upper_blue]]  # these are the color ranges for blue

    def camera_callback(self, image):
        # Convert the ROS image message into a workable OpenCV image
        img = self.bridge_object.imgmsg_to_cv2(image, desired_encoding="bgr8")
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # mask = cv2.inRange(img_hsv,self.lower_red, self.upper_red)
        imgDrawnWithContours = img.copy()
        # imgWithBoundingRectangles = img.copy()
        centerPointsArray = []  # Initialize list of center points

        for color in self.myColors:
            lower_bound, upper_bound = color

            # test if the color of each pixel is within the specified bounds
            mask = cv2.inRange(img_hsv, np.array(lower_bound), np.array(upper_bound))
            # calculate the contours of the masked image
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            for c in contours:

                rect = cv2.minAreaRect(c)

                box = cv2.boxPoints(rect)
                box = np.int0(box)
                cv2.drawContours(imgDrawnWithContours, [box], 0, (0, 255, 255))

                center,_,_ = rect
                centerPointsArray.append(center)

        print("The center points of each detected shape: ", centerPointsArray)
        # cv2.imshow("Raw Image", img)
        cv2.imshow("Image with Contours", imgDrawnWithContours)
        cv2.waitKey(3)

        # converting crop_img from cv to ros msg
        msg = self.bridge_object.cv2_to_imgmsg(imgDrawnWithContours, encoding="rgb8")
        # Publishing the Image message
        self.image_pub.publish(msg)


def main():
    color_detection = Color_Detection()
    rospy.init_node('color_detection_node', anonymous=False)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")


# This is the main function; the program starts here.
if __name__ == '__main__':
    main()
