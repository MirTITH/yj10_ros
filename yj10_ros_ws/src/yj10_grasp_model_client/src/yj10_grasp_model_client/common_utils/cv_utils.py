import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def ShowImage(img_msg: Image, window_name: str):
    cv_image = CvBridge().imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
    cv2.imshow(window_name, cv_image)
    cv2.waitKey(1)


def ShowDepth(img_msg: Image, window_name: str):
    # Convert ROS Image message to OpenCV image
    depth_image = CvBridge().imgmsg_to_cv2(img_msg, desired_encoding="16UC1")

    # Normalize depth image for display
    depth_image_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)

    # Apply colormap to depth image for better visualization
    depth_image_colored = cv2.applyColorMap(depth_image_normalized, cv2.COLORMAP_JET)

    # Display the image
    cv2.imshow("Depth Image", depth_image_colored)
    cv2.waitKey(1)
