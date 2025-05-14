#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage

class Lane_Detector:
    def __init__(self):
        self.cv_bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            '/robobae/camera_node/image/compressed',
            CompressedImage,
            self.image_callback,
            queue_size=1
        )
        rospy.init_node("my_lane_detector")

    def image_callback(self, msg):
        img = self.cv_bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

        # Step 1: Crop the bottom half of the image (adjust if needed)
        cropped = img[300:480, :]  # Keep lower portion

        # Step 2: Convert to HSV
        hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)

        # Step 3: Filter White Pixels
        white_lower = np.array([0, 0, 200])
        white_upper = np.array([180, 50, 255])
        white_mask = cv2.inRange(hsv, white_lower, white_upper)
        white_result = cv2.bitwise_and(cropped, cropped, mask=white_mask)

        # Step 4: Filter Yellow Pixels
        yellow_lower = np.array([20, 100, 100])
        yellow_upper = np.array([30, 255, 255])
        yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
        yellow_result = cv2.bitwise_and(cropped, cropped, mask=yellow_mask)

        # Step 5: Edge Detection on cropped image
        edges = cv2.Canny(cropped, 100, 200)

        # Step 6: Hough Transform on white mask
        lines_white = cv2.HoughLinesP(white_mask, 1, np.pi/180, 30, minLineLength=40, maxLineGap=10)

        # Step 7: Hough Transform on yellow mask
        lines_yellow = cv2.HoughLinesP(yellow_mask, 1, np.pi/180, 30, minLineLength=40, maxLineGap=10)

        # Step 8: Draw lines on a copy of cropped image
        output = np.copy(cropped)
        if lines_white is not None:
            for l in lines_white:
                x1, y1, x2, y2 = l[0]
                cv2.line(output, (x1, y1), (x2, y2), (255, 255, 255), 2)

        if lines_yellow is not None:
            for l in lines_yellow:
                x1, y1, x2, y2 = l[0]
                cv2.line(output, (x1, y1), (x2, y2), (0, 255, 255), 2)

        # Show results in OpenCV windows
        cv2.imshow("White Filtered", white_result)
        cv2.imshow("Yellow Filtered", yellow_result)
        cv2.imshow("Hough Lines", output)
        cv2.waitKey(1)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = Lane_Detector()
        node.run()
    except rospy.ROSInterruptException:
        pass
