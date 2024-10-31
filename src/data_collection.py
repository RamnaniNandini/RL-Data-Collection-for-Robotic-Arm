import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import sys
import termios
import tty
import select

from sensor_msgs.msg import JointState

class DataCollector:
    def __init__(self):
        self.rate = rospy.Rate(5)

        self.img_shape = (48, 48)

        self.usb_cam_img = None
        self.realsense_img = None
        self.joint_positions = None

        self.data_collection_path = '/home/nanz/catkin_ws/src/rl_data/collected_data/'
        self.bridge = CvBridge()

        self.observations = []
        self.episode_count = 0
        self.step_count = 0
        self.collecting_data = False

        # ROS subscribers for camera images
        rospy.Subscriber("usb_cam_image", Image, self.usb_cam_callback)
        rospy.Subscriber("realsense_cam_image", Image, self.realsense_callback)

        # ROS subscriber for JOINT_STATES
        rospy.Subscriber("/joint_states", JointState, self.joint_states_callback)

        # Keyboard input
        self.key = None
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    def usb_cam_callback(self, msg):
        self.usb_cam_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def realsense_callback(self, msg):
        self.realsense_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def joint_states_callback(self, data):
        self.joint_positions = data.position

    def collect_data(self):
        if self.usb_cam_img is not None:
            # resize the image to 48 x 48
            usb_resized_img = cv2.resize(self.usb_cam_img, self.img_shape)
            wrist_img = usb_resized_img.astype(np.uint8).flatten()
        else:
            wrist_img = np.zeros(48 * 48 * 3)
            
        if self.realsense_img is not None:
            rs_resized_img = cv2.resize(self.realsense_img, self.img_shape)
            main_img = rs_resized_img.astype(np.uint8).flatten()
        else:
            main_img = np.zeros(48 * 48 * 3)

        obs = {
            'main_image': main_img,
            'wrist_image': wrist_img,
            'joint_positions': self.joint_positions,
            'time_stamp': rospy.get_time()
            }

        self.observations.append(obs)
    
    def save_data(self):
        data = np.array(self.observations)
        file_name = os.path.join(self.data_collection_path, f'episode{self.episode_count}.npy')
        np.save(file_name, data)
        rospy.loginfo(f"Data saved to {file_name}")
        self.observations = []

    def run(self):
        try:
            while not rospy.is_shutdown():
                if self.collecting_data:
                    self.step_count += 1
                    self.collect_data()
                    rospy.loginfo("Observation Length: {}".format(len(self.observations)))
                    if self.step_count >= 100:
                        self.save_data()
                        self.episode_count += 1
                        self.step_count = 0
                        self.collecting_data = False
                        rospy.loginfo("Data Collection Completed for Episode {}".format(self.episode_count))
                    
                    # Check for keypress
                    if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                        self.key = sys.stdin.read(1)
                        if self.key == 'x':
                            self.save_data()
                            rospy.loginfo("Data collection stopped by user.")
                            rospy.loginfo("Press 's' to start data collection for the next episode.")
                            self.episode_count += 1  # Start the next episode
                            self.collecting_data = False
                            self.step_count = 0
                            self.observations = []  # Clear previous episode's data
                else:
                    rospy.loginfo("Press 's' to start data collection.")
                    self.key = sys.stdin.read(1)
                    if self.key == 's':
                        rospy.loginfo("Data collection started for Episode {}".format(self.episode_count + 1))
                        self.collecting_data = True

                self.rate.sleep()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            rospy.loginfo("Shutting Down")

def main():
    rospy.init_node('data_collector_node', anonymous=True)
    data_collector = DataCollector()
    data_collector.run()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
