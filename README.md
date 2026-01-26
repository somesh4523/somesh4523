# Hi there, I'm Somesh! 👋

I am a PhD Scholar specializing in Control and Automation.

## 🛠️ Tech Stack
* **Languages:** Python, C++, MATLAB
* **Tools:** Simulink, ROS2, Git, Linux
* **Interests:** Control Theory, Robotics, Automation

---

## 🔐 Research Projects
*Since many of my projects are proprietary or research-based, the source code is private. Below is a summary of my contributions to these projects.*

### 🚀 Krishi Cobot
**eYantra_25-26**
**ROS2, Gazebo, OpenCV**
**Description:**

This project simulates a "Smart Farmhouse" where a mobile robot and a robotic arm cooperate to manage crops.
* 1. Autonomous Navigation & Feedback Control
This task relies on ROS 2 Humble nodes to manage state estimation and velocity command generation. The mobile robot (eBot) utilizes Odometry data (nav_msgs/Odometry) for real-time position feedback and a 2D LIDAR (sensor_msgs/LaserScan) for obstacle sensing. The core navigation logic involves implementing a Proportional (P) Controller to minimize the Euclidean distance and heading error between the robot's current pose and the target waypoints. Velocity commands are published to the /cmd_vel topic using geometry_msgs/Twist messages to drive the differential drive kinematics.
* 2. Computer Vision & Feature Extraction
Perception is handled using the OpenCV library within a ROS 2 package. The pipeline begins with converting raw RGB images to the HSV Color Space to perform robust color thresholding (masking) for identifying "bad" fruits (greyish-white). For shape detection (triangles/squares), the system employs Canny Edge Detection and Contour Approximation (cv2.approxPolyDP) to analyze geometric vertices. Additionally, ArUco Marker detection is used for high-precision localization of fertilizer cans, requiring the calibration of camera intrinsics to solve the Perspective-n-Point (PnP) problem for pose estimation.
* 3. Robotic Manipulation & Kinematics
The manipulation task involves controlling a UR5 6-DOF Robotic Arm (or similar serial manipulator) within the Gazebo simulation environment. The technical core requires solving the Inverse Kinematics (IK) to map the desired 3D end-effector coordinates (derived from the vision system) to specific joint angles. This process relies heavily on the TF2 (Transform Library) to broadcast and listen to coordinate frame transformations, ensuring the target object's pose in the Camera Frame is correctly transformed into the Robot Base Frame for accurate pick-and-place execution.

### 🤖 Optimal Control of Multi-Robot Systems
**PhD Research**
**Control Theory, MATLAB, SIMULINK**
**Advantages:**
* Nonlinear Model Predictive Control design ensures optimal control of each individual agent.
* High-level formation trajectory planner ensures desired formation.
* Integrated planning and tracking control architecture allows optimal control of Multi-Robot Systems in a formation.

### 🚀 Hologlyph Bots
**eYantra_25-26**
**ROS2, Gazebo, OpenCV, EasyEDA**
**Description:**

Using multiple nonholonomic robots to create large-scale designs or "geoglyphs," involving coordinated swarm robotics for efficiency in tasks like complex pattern formation.
* 1. Holonomic Navigation & Kinematics Control
This task relies on ROS 2 Humble nodes to manage the complex movement of a 3-Wheel Omnidirectional (Holonomic) Drive. Unlike differential drive robots, these bots utilize Inverse Kinematics (IK) matrices to resolve the desired chassis velocity vector $(v_x, v_y, \omega_z)$ into individual wheel angular velocities. The core navigation logic involves implementing a PID Controller to minimize the error between the robot's current pose and the target trajectory points. Velocity commands are published to the /cmd_vel topic using geometry_msgs/Twist messages, enabling simultaneous translation and rotation for smooth, drift-free motion.
* 2. Global Perception & State Estimation Perception is handled using a global "Eye-to-Hand" vision system with OpenCV within a ROS 2 package. The pipeline relies on an Overhead Camera to track the robots' positions in real-time using ArUco Markers. The system employs Perspective Transformation (cv2.getPerspectiveTransform and cv2.warpPerspective) to map the skewed camera feed onto a corrected 2D coordinate system (Homography) of the arena. This provides precise (x, y, \theta) state estimation for each robot, which is published as odometry data to close the feedback loop, compensating for wheel slippage and drift.
* 3. Trajectory Generation & Swarm Coordination The "manipulation" task involves the precise execution of mathematical path planning to "draw" shapes (Glyphs) such as Lissajous figures or geometric polygons. The technical core requires a Trajectory Planner that generates high-resolution waypoints based on parametric equations. This process relies on Multi-Robot Coordination algorithms to synchronize the movement of multiple bots (Swarm) within the shared Gazebo or physical arena, ensuring they maintain formation and avoid collisions while tracing their assigned segments of the composite image.

---

## 📫 Connect with Me
* [LinkedIn](https://linkedin.com/in/yourprofile)
* [Portfolio Website](https://yourwebsite.com)
* Email: your.email@example.com
