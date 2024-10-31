# RL-Data-Collection-for-Robotic-Arm

Learning Robotic Manipulation Tasks Using Reinforcement Learning

AIM: To create a detailed data collection about the actions of a robotic arm as it lifts different objects with the movements of all joints and effectors involved. Consequently, we are planning to leverage this data set to enhance our robotic arm learning experience, helping it to achieve accuracy in task execution similar to that achieved by human hands.

I started this project by reading two papers about using transformers with reinforcement learning.
Insights from the paper “A Survey on Transformers in Reinforcement Learning”
This paper briefly reviews advances in Transformers for RL. It provided a taxonomy of these advances: 
a) Transformers can serve as a powerful module of RL, e.g., acting as a representation module or a world model.
b) Transformers can serve as a sequential decision-maker.
c) Transformers can benefit generalization across tasks and domains.

The taxonomy of Transformer-based RL 

Insights from the paper “Decision Transformer: Reinforcement Learning via Sequence Modeling”
This paper introduces a novel approach called Decision Transformer, which bridges the gap between language modeling and reinforcement learning (RL). Here are the key takeaways:
Unifying Language Modeling and RL: Decision Transformer treats RL as a sequence modeling problem. This allows it to leverage the powerful capabilities of Transformers, typically used for language tasks, for decision-making in RL settings.
Strong Performance: The proposed method achieves competitive or superior performance on standard offline RL benchmarks compared to specialized offline RL algorithms. This suggests that Decision Transformers are a promising direction for RL.
Potential for Improvement: The paper acknowledges opportunities for further development:
Self-supervised Pre Training: Utilizing self-supervised pre training techniques could enhance performance, especially with larger datasets.
Advanced Embeddings: Encoding returns, states, and actions with more sophisticated embeddings (e.g., return distributions for stochastic environments) could improve model capabilities.
State Evolution Modeling: Transformers might be applicable to modeling the state evolution within a trajectory, potentially offering an alternative to model-based RL approaches.

Insights from the video I watched regarding Forward and Inverse Kinematics of two-link robotic arm :
Reachable Workspace: This refers to the entire space that the robotic arm's end effector (gripper or tool) can access and manipulate objects within. Imagine it as the arm's "working area."
Acnestis: This term describes the unreachable regions at the edges of the workspace. Shortening or lengthening the links are the reason of these unreachable zones. 
Forward vs. Inverse Kinematics: The video explained the difference between these two crucial concepts:
Forward Kinematics: Given the joint angles (positions of the arm's "elbows"), this approach calculates the position and orientation of the end effector. It's like knowing the arm angles and wanting to predict where the hand will be.
Inverse Kinematics: This is the trickier problem. Given a desired position and orientation of the end effector, it determines the necessary joint angles to achieve that position. It's like knowing where you want the hand to be and needing to figure out how to bend the arm to get it there.
Multiple Solutions for Two Links: An interesting fact highlighted in the video is that for a two-link arm, there can often be two different sets of joint angles that will reach the same end effector location. This is because the arm can be configured in a "bent elbow up" or "bent elbow down" position to reach the same point. The video mentioned that there's only one solution (no "elbow up" or "down" options) at the edges of the reachable workspace (singular positions).

Additional Kinematic Concepts (potential video topics):
Velocity Kinematics: This area deals with calculating the linear and angular velocities of the end effector based on the current joint angles and speeds. It's important for smooth and controlled arm movements.
Manipulability Ellipse: This concept visualizes the achievable velocities of the end effector based on the current joint configuration. It helps assess the arm's dexterity at a particular point in its workspace.
Inverse Velocity Kinematics: This builds on velocity kinematics. Given a desired end effector velocity (both linear and angular), it calculates the required joint speeds to achieve that movement.

Further I have been learning Robot Operating System(ROS) and to work with gazebo simulation.

Through OpenManipulator-X, I am successful in the motion of a robotic arm and have also implemented a master and slave concept in it. 

Master and Slave Robot

Before sending commands for master and slave operation through OpenManipulator-X, it's crucial to ensure that the Dynamixel IDs are properly set for each component. For the slave robot, the Dynamixel IDs should be configured as follows: Joint Dynamixel ID: 11, Joint Dynamixel ID: 12, Joint Dynamixel ID: 13, Joint Dynamixel ID: 14, and Gripper Dynamixel ID: 15. Similarly, for the master robot, the Dynamixel IDs should be set as follows: Joint Dynamixel ID: 21, Joint Dynamixel ID: 22, Joint Dynamixel ID: 23, Joint Dynamixel ID: 24, and Gripper Dynamixel ID: 25. By ensuring correct configuration, the master-slave concept can be effectively implemented, facilitating coordinated motion between the two robotic arms.

Commands to be executed:
cd ~/catkin_ws/src/
git clone https://github.com/ROBOTIS-GIT/open_manipulator_applications.git
cd ~/catkin_ws && catkin_make
roslaunch open_manipulator_controller open_manipulator_controller.launch dynamixel_usb_port:=/dev/ttyUSB0
(It should print id 11,12,13,14,15 Slave)
roslaunch open_manipulator_master_slave open_manipulator_master.launch usb_port:=/dev/ttyUSB1
(It should print 21,22,23,24,25 Master)

Please select and enter the corresponding number for the desired control mode in the terminal:

1: Master-Slave Mode - In this mode, the master robot and slave robot move synchronously.
2: Start Recording Trajectory - Initiates synchronous motion of the master and slave robots while recording the moving trajectory.
3: Stop Recording Trajectory - Ends the recording process.
4: Play Recorded Trajectory - Reproduces the recorded trajectory only by the slave robot.

Data Collection

For data collection we have installed two cameras, one for getting the view of the complete robotic arm, its joints and gripper and another for the object and movement of the gripper.

In code we have called three nodes for collecting data(data_collecting.py), one for realsense camera(realsense_node.py), another for logitech camera(usb_cam_node.py) and one for joint and end effector position.
By running the launch file (roslaunch rl_data run.launch) that calls all three nodes simultaneously, we have executed the code.

Considerations in the code are as follows:
Execution stops after 100 observations.
Set a key to start ‘s’ and if you wish to stop before 100 intervals key ‘x’ should be used to stop the execution.
After every 100 steps it should store values in file named episode 1(for first 100), then in episode 2 (for next 100) and so on.

The final data(episode1.npy, episode2.npy and so on) is stored in collected data folder.

As the data is collected in array format to check whether the data stored is correct or not, I have added the check_data_collection.py node, after executing it, it shows the frames obtained by 100 observations in 1 episode by both the cameras.

Link for accessing the code, obtained results and video as an output have been saved in this link.




           

