#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError

ROS_NODE_NAME="image_processing_node"
bridge = CvBridge()
image_pub = None

def img_process(ros_image_msg):
    global image_pub

    try:
        cv2_img = bridge.imgmsg_to_cv2(ros_image_msg,"bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: %s" % e)
        return

    hsv = cv2.cvtColor(cv2_img,cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv,np.array([40,40,40]),np.array([80,255,255]))

    contours, _ = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)

    if not contours:
        rospy.logerr("No contours found in the image")
        cv2.imshow("Object detection Frame",cv2_img)
        cv2.waitKey(1)
        return

    largest_contour = max(contours, key=cv2.contourArea)
    contour_frame = cv2_img.copy()

    if cv2.contourArea(largest_contour) > 10:
        (x,y), radius = cv2.minEnclosingCircle(largest_contour)
        center = (int(x),int(y))
        circle = cv2.circle(contour_frame, center, int(radius), (235,90,210), 4)
    else:
        circle = contour_frame

    cv2.imshow("Object detection Frame",circle)
    cv2.waitKey(1)

    try:
        ros_out_msg = bridge.cv2_to_imgmsg(circle,"bgr8")
        image_pub.publish(ros_out_msg)
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: %s" % e)

def cleanup():
    rospy.loginfo("Shutting down...")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node(ROS_NODE_NAME,log_level=rospy.INFO)
    rospy.on_shutdown(cleanup)

    image_pub = rospy.Publisher("/image_processing/processed_image",Image,queue_size=1)
    rospy.Subscriber("/usb_cam/image_raw",Image,img_process)
    rospy.loginfo("Image Processing Node is running. Publishing on /image_processing/processed_image")

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

