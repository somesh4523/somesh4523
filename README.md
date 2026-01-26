# Hi there, I'm [Your Name]! 👋

I am a [Your Job Title/Role] specializing in [Your Main Field].

## 🛠️ Tech Stack
* **Languages:** Python, C++, MATLAB
* **Tools:** Simulink, ROS2, Git, Linux
* **Interests:** Control Theory, Robotics, Automation

---

## 🔐 Private & Commercial Projects
*Since many of my projects are proprietary or research-based, the source code is private. Below is a summary of my contributions to these projects.*

### 🚀 Krishi Cobot
**eYantra_25-26**
**ROS2, Gazebo, OpenCV**
**Description:**

* **Key Challenge:** This project simulates a "Smart Farmhouse" where a mobile robot and a robotic arm cooperate to manage crops.
* 1. Autonomous Navigation & Feedback Control
This task relies on ROS 2 Humble nodes to manage state estimation and velocity command generation. The mobile robot (eBot) utilizes Odometry data (nav_msgs/Odometry) for real-time position feedback and a 2D LIDAR (sensor_msgs/LaserScan) for obstacle sensing. The core navigation logic involves implementing a Proportional (P) Controller to minimize the Euclidean distance and heading error between the robot's current pose and the target waypoints. Velocity commands are published to the /cmd_vel topic using geometry_msgs/Twist messages to drive the differential drive kinematics.
* 2. Computer Vision & Feature Extraction
Perception is handled using the OpenCV library within a ROS 2 package. The pipeline begins with converting raw RGB images to the HSV Color Space to perform robust color thresholding (masking) for identifying "bad" fruits (greyish-white). For shape detection (triangles/squares), the system employs Canny Edge Detection and Contour Approximation (cv2.approxPolyDP) to analyze geometric vertices. Additionally, ArUco Marker detection is used for high-precision localization of fertilizer cans, requiring the calibration of camera intrinsics to solve the Perspective-n-Point (PnP) problem for pose estimation.
* 3. Robotic Manipulation & Kinematics
The manipulation task involves controlling a UR5 6-DOF Robotic Arm (or similar serial manipulator) within the Gazebo simulation environment. The technical core requires solving the Inverse Kinematics (IK) to map the desired 3D end-effector coordinates (derived from the vision system) to specific joint angles. This process relies heavily on the TF2 (Transform Library) to broadcast and listen to coordinate frame transformations, ensuring the target object's pose in the Camera Frame is correctly transformed into the Robot Base Frame for accurate pick-and-place execution.

### 🤖 Optimal Control of Multi-Robot Systems
**Context:** [e.g., Industrial Automation Project]
**Technologies:** [e.g., C++, ROS2, MoveIt]
**Description:**
* Designed the control architecture for a [type of robot].
* Implemented trajectory optimization for real-time path planning.
* Integrated sensor fusion from LiDAR and IMU data.

---

## 📫 Connect with Me
* [LinkedIn](https://linkedin.com/in/yourprofile)
* [Portfolio Website](https://yourwebsite.com)
* Email: your.email@example.com
