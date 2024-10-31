import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import pyrealsense2 as rs
import numpy as np

class REALSENSEImageCollector:
    def __init__(self):
        self.rate = rospy.Rate(30)
        
        self.rs_pipeline = rs.pipeline()
        self.rs_config = rs.config()
        self.rs_config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)  # Configure color stream
        

        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("realsense_cam_image", Image, queue_size=10)

    def run(self):
        try:
            self.rs_pipeline.start(self.rs_config)
            while not rospy.is_shutdown():
                # Capture frames
                frames = self.rs_pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                if color_frame:
                    image = np.asanyarray(color_frame.get_data())
                    try:
                        ros_image = self.bridge.cv2_to_imgmsg(image, "bgr8")
                        self.image_pub.publish(ros_image)
                    except CvBridgeError as e:
                        rospy.logerr(e)
        finally:
            rospy.loginfo("Shutting Down")
            self.rs_pipeline.stop()

            

def main():
    rospy.init_node('realsense_cam_node', anonymous=True)
    rs_img_collector = REALSENSEImageCollector()
    rs_img_collector.run()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
