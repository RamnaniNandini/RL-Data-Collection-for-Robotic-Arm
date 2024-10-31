import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class USBImageCollector:
    def __init__(self):
        self.rate = rospy.Rate(30)
        self.cap = cv2.VideoCapture(8)
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("usb_cam_image", Image, queue_size=10)

    def run(self):
        try:
            while not rospy.is_shutdown():
                ret, frame = self.cap.read()
                if ret:
                    try:
                        ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                        self.image_pub.publish(ros_image)
                    except CvBridgeError as e:
                        rospy.logerr(e)
                self.rate.sleep()
        finally:
            rospy.loginfo("Shutting Down")
            self.cap.release()

def main():
    rospy.init_node('usb_cam_node', anonymous=True)
    usb_img_collector = USBImageCollector()
    usb_img_collector.run()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
