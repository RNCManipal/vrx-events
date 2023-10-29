#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometric_msgs.msg import PoseStamped
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge

class ImageRecog(Node):

    def __init__(self):
        super().__init__('opencv')

        self.img_Sub = self.create_subscription(Image, '/wamv/sensors/cameras/front_right_camera/image_raw', self.imgcallback, 10)

        self.opencv_Pub = self.create_publisher(PoseStamped, '/vrx/perception/landmark', 1)

    def imgcallback(self, data):
        bridge = CvBridge()

        try:
            cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridge.CvBridgeError as e:
            self.get_logger().error(str(e))

        count = np.array([1, 1, 1, 1, 1, 1])
        while True:
            img = cv_image

            imgBlur = cv.GaussianBlur(img, (5, 5), cv.BORDER_DEFAULT)
            imgCanny = cv.Canny(imgBlur, 125, 175)

            contours, hierarchies = cv.findContours(imgCanny, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
            imgCanny = cv.drawContours(imgCanny, contours, -1, (255, 255, 255), 3)
            contours, hierarchies = cv.findContours(imgCanny, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

            existing_objects = []
            obj_min_size = 5

            for i in contours:
                area = cv.contourArea(i)

                if area < 50:
                    continue
                m = cv.moments(i)
                x, y = (m['m10'] / (m['m00'] + 1e-5), m['m01'] / (m['m00'] + 1e-5))

                if y > 650:
                    continue
                if y < 250:
                    continue

                skip = False
                for existing_pos in existing_objects:
                    if existing_pos[0] >= x - obj_min_size and existing_pos[0] <= x + obj_min_size:
                        if existing_pos[1] >= y - obj_min_size and existing_pos[1] <= y + obj_min_size:
                            skip = True
                            break
                if skip:
                    continue

                existing_objects.append((x, y))

                x = int(x)
                y = int(y)
                o = 2
                color = img[y - o:y + o, x - o:x + o, :]
                color = np.mean(color, axis=(0, 1))

                msg1 = PoseStamped()
                msg2 = PoseStamped()
                msg3 = PoseStamped()
                msg4 = PoseStamped()
                msg5 = PoseStamped()
                msg6 = PoseStamped()

                if (1 < color[0] < 4) and (-1 < color[1] < 3) and (72 < color[2] < 75) and count[0] == 1:  # count  has aproblem
                    count += 1
                    self.get_logger().info('mb_marker_buoy_red')
                    self.opencv_Pub.publish(msg1)

                elif (23 < color[0] < 33) and (44 < color[1] < 54) and (5 < color[2] < 10) and count[1] == 1: #count error
                    count += 1
                    self.get_logger().info('mb_marker_buoy_green')
                    self.opencv_Pub.publish(msg2)

                elif (-1 < color[0] < 1) and (-1 < color[1] < 1) and (-1 < color[2] < 1) and count[2] == 1:
                    count += 1                                                                 #ccount was not there
                    self.get_logger().info('mb_marker_buoy_black')
                    self.opencv_Pub.publish(msg3)

                elif (116 < color[0] < 126) and (116 < color[1] < 126) and (116 < color[2] < 126) and count[3] == 1:
                    count += 1
                    self.get_logger().info('mb_marker_buoy_white')
                    self.opencv_Pub.publish(msg4)

                elif (-1 < color[0] < 1) and (-1 < color[1] < 1) and (-1 < color[2] < 1) and count[4] == 1:
                    count += 1                                #count was not 
                    self.get_logger().info('round_black')
                    self.opencv_Pub.publish(msg5)

                elif (-1 < color[0] < 5) and (5 < color[1] < 30) and (90 < color[2] < 150) and count[5] == 1:
                    count += 1
                    self.get_logger().info('round_orange')
                    self.opencv_Pub.publish(msg6)

            key = cv.waitKey(1)
            if key == 27:
                break
            break

        cv.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    uav = ImageRecog()
    rclpy.spin(uav)
    uav.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
