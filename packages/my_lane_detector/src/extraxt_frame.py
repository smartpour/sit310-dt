import rosbag
import cv2
from cv_bridge import CvBridge

bag = rosbag.Bag('/data/my_custom_video.bag')  # or your .bag file
bridge = CvBridge()

for topic, msg, t in bag.read_messages():
    if topic == '/robobae/camera_node/image/compressed':
        img = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        cv2.imwrite("/data/Raw Camera Image_screenshot_07.05.2025.png", img)
        print("Saved frame as /data/Raw Camera Image_screenshot_07.05.2025.png")
        break
