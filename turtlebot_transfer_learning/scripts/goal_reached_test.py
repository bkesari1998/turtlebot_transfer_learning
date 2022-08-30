import numpy as np
import rospy

from sensor_msgs.msg import Image

# Import OpenCV libraries and tools
import cv2
from cv_bridge import CvBridge, CvBridgeError

def done(cv_image_rgb, cv_image_depth):

    # Convert to hsv colorspace
    hsv = cv2.cvtColor(cv_image_rgb, cv2.COLOR_BGR2HSV)
    rgbd = cv2.cvtColor(cv_image_depth, cv2.COLOR_RGB)

    # Lower bound and upper bound for gray color
    lower_bound = np.array([0, 0, 71])   
    upper_bound = np.array([0, 0, 31])

    # find the colors within ghe boundaries
    mask = cv2.inRange(hsv, lower_bound, upper_bound)

    # remove noise 
    kernel = np.ones((7,7),np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    m = cv2.moments(mask)

    # object detected
    if m['m00'] != 0:
        cx = int(m['m10']/m['00'])
        cy = int(m['m01']/m['m00'])

        print(cv_image_depth[cx][cy])

    print("Object not found")

if __name__ == '__main__':

    rospy.init_node("goal_reached_test")

    bridge = CvBridge()

    rgb_img = rospy.wait_for_message("/camera/rgb/image_raw", Image, rospy.Duration(1))
    depth_img = rospy.wait_for_message("/camera/depth_registered/image_raw", Image, rospy.Duration(1))

    cv_image_rgb = bridge.imgmsg_to_cv2(rgb_img, "passthrough")
    cv_image_depth = bridge.imgmsg_to_cv2(depth_img, "passthrough")

    done(cv_image_rgb, cv_image_depth)

    rospy.spin()
