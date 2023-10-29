#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge

class ImageRecog(Node):

    def __init__(self):
        super().__init__('opencv')

        self.img_Sub = self.create_subscription(Image, '/wamv/sensors/cameras/front_right_camera_sensor/image_raw', self.imgcallback, 10)

        self.opencv_Pub = self.create_publisher(PoseStamped, '/vrx/perception/landmark', 1)

    '''def imgcallback(self, data):
        bridge = CvBridge()

        try:
            cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridge.CvBridgeError as e:
            self.get_logger().error(str(e))

        count = np.array([1, 1, 1, 1, 1, 1])
        value = list()
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
                    self.get_logger().info(msg3.data)
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
            break'''
    

    def imgcallback(self,data):
        bridge = CvBridge()

        try:
            cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridge.CvBridgeError as e:
            self.get_logger().error(str(e))

        count = np.array([1,1,1,1,1,1])
        while True:
            img = cv_image

            img = cv.cvtColor(img, cv.COLOR_RGB2BGR) 
            imgBlur = cv.GaussianBlur(img, (5,5), cv.BORDER_DEFAULT)
            imgCanny = cv.Canny(imgBlur, 125, 175)

            # Number of contours
            contours, hierarchies = cv.findContours(imgCanny, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)


            # Drawing contours again on canny image (increase thickness) ; Increase clarity of contours
            imgCanny = cv.drawContours(imgCanny, contours, -1, (255, 255, 255), 3)
            contours, hierarchies = cv.findContours(imgCanny, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)


            existing_objects = []
            obj_min_size = 5

                # print(f"{len(contours)} contours found")


            # Loop through the contours
            for i in contours:

                # Area of contour
                area = cv.contourArea(i)
                # self.get_logger().info(f" area {area}")


                # Ignore the contour area < 50
                if area<50:
                    continue
                m = cv.moments(i)
                # self.get_logger().info(f"shape {m}")
                x, y = (m['m10'] / (m['m00'] + 1e-5), m['m01'] / (m['m00'] + 1e-5))
                # self.get_logger().info(f"x  {x} , y {y}")

                if y>650:
                    continue
                if y<250:
                    continue

                # self.get_logger().info(f"area ratio {(y*area)}")


                # Prevent duplication of values for objects
                skip = False
                for existing_pos in existing_objects:
                    if existing_pos[0]>=x-obj_min_size and existing_pos[0]<=x+obj_min_size:
                        if existing_pos[1]>=y-obj_min_size and existing_pos[1]<=y+obj_min_size:
                            skip = True
                            break
                if skip:
                    continue

                # print(area)
                # print(x, y)

                existing_objects.append((x, y))


                # Draw contours on objects
                img = cv.drawContours(img, [i], -1, (0,0,255), 3)
                imgCanny = cv.drawContours(imgCanny, [i], -1, (255), 3)


                # Find out average BGR values of objects
                x = int(x)
                y = int(y)
                o = 2
                color = img[y-o:y+o, x-o:x+o,:]
                color = np.mean(color, axis=(0, 1)) 
                # self.get_logger().info(f"colour {color}")
        # Color in BGR format

                #print(color)

                #cv.imshow("result", img)
                #cv.imshow("canny image", imgCanny)

                msg1 = PoseStamped()
                msg2 = PoseStamped()
                msg3 = PoseStamped()
                msg4 = PoseStamped()
                msg5 = PoseStamped()
                msg6 = PoseStamped()

                x, y, w, h = cv.boundingRect(i)
                aspect_ratio = float(w) / h

                # Calculate the number of vertices
                vertices = cv.approxPolyDP(i, 0.04 * cv.arcLength(i, True), True)

                
                
                if 2 <len(vertices) <= 6 :
                # Likely a cone
                # Perform actions for cones
                    # Colour matching
                    if((98<color[0]<102) and (0<color[1]<3) and (13<color[2]<15)  and count[0] == 1):
                    # msg1.header.frame_id = "red_marker"
                        #self.red.publish(msg1.header.frame_id)
                        count[0] += 1
                        self.get_logger().info(f"vertices {len(vertices)} mb_marker_buoy_red")
                        self.opencv_Pub.publish(msg1)
                        #self.opencv_Pub.publish('{header: {stamp: now, frame_id: "mb_marker_buoy_red"}')

                    if ((20<color[0]<30) and (70<color[1]<80) and (50<color[2]<62)  and (count[1] == 1)):# :
                        #msg2.header.frame_id = "green_marker"
                        #self.green.publish(msg2)
                        count[1] += 1
                        self.get_logger().info(f"vertices {len(vertices)} mb_marker_buoy_green")

                        self.opencv_Pub.publish(msg2)
                        #self.opencv_Pub.publish('{header: {stamp: now, frame_id: "mb_marker_buoy_green"}')

                    if ((-1<color[0]<1) and (-1<color[1]<1) and (-1<color[2]<1) and (count[2] == 1)):#:
                        count[2] += 1
                    # msg3.header.frame_id = "black_marker"
                        #self.black.publish(msg3)
                        self.get_logger().info(f"vertices {len(vertices)} mb_marker_buoy_black")
                        self.opencv_Pub.publish(msg3)

                    if ((250<color[0]) and (250<color[1]) and (250<color[2]) and (count[3] == 1)):# :
                        #msg4.header.frame_id = "white_marker"
                    # self.white.publish(msg4)
                        count[3] += 1
                        self.get_logger().info(f"vertices {len(vertices)} mb_marker_buoy_white")
                        self.opencv_Pub.publish(msg4)
                
                if  6 <len(vertices) <= 2 :
                    # Likely a sphere
                    # Perform actions for spheres

                    self.get_logger().info(f"vertices {len(vertices)}   entered round {color}  ")
                    if ((-1<color[0]<1) and (-1<color[1]<1) and (-1<color[2]<1)and (count[4] == 1)):# :
                        #msg5.header.frame_id = "black_marker_sphere"
                        #self.black_sphere.publish(msg5)
                        count[4] += 1
                        self.get_logger().info('mb_round_buoy_black')
                        self.opencv_Pub.publish(msg5)

                    if ((124<color[0]<130) and (65<color[1]<70) and (125<color[2]<130)  and (count[5] == 1)) :#:
                    # msg6.header.frame_id = "orange_marker_sphere"
                        #self.orange_sphere.publish(msg6)
                        count[5] +=  1 
                        self.get_logger().info('mb_round_buoy_orange')
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
