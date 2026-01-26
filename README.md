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
This project simulates a "Smart Farmhouse" where a mobile robot and a robotic arm cooperate to manage crops.
* 1. Autonomous Navigation & Feedback Control
This task relies on ROS 2 Humble nodes to manage state estimation and velocity command generation. The mobile robot (eBot) utilizes Odometry data (nav_msgs/Odometry) for real-time position feedback and a 2D LIDAR (sensor_msgs/LaserScan) for obstacle sensing. The core navigation logic involves implementing a Proportional (P) Controller to minimize the Euclidean distance and heading error between the robot's current pose and the target waypoints. Velocity commands are published to the /cmd_vel topic using geometry_msgs/Twist messages to drive the differential drive kinematics.
* 2. Computer Vision & Feature Extraction
Perception is handled using the OpenCV library within a ROS 2 package. The pipeline begins with converting raw RGB images to the HSV Color Space to perform robust color thresholding (masking) for identifying "bad" fruits (greyish-white). For shape detection (triangles/squares), the system employs Canny Edge Detection and Contour Approximation (cv2.approxPolyDP) to analyze geometric vertices. Additionally, ArUco Marker detection is used for high-precision localization of fertilizer cans, requiring the calibration of camera intrinsics to solve the Perspective-n-Point (PnP) problem for pose estimation.
* 3. Robotic Manipulation & Kinematics
The manipulation task involves controlling a UR5 6-DOF Robotic Arm (or similar serial manipulator) within the Gazebo simulation environment. The technical core requires solving the Inverse Kinematics (IK) to map the desired 3D end-effector coordinates (derived from the vision system) to specific joint angles. This process relies heavily on the TF2 (Transform Library) to broadcast and listen to coordinate frame transformations, ensuring the target object's pose in the Camera Frame is correctly transformed into the Robot Base Frame for accurate pick-and-place execution.

**Images**

<img width="1920" height="1080" alt="KC#2238_color_image" src="https://github.com/user-attachments/assets/30eee78b-b0d8-4831-b4ca-ed7096ec6d15" />

**Video Links**

[![task 1A](https://img.youtube.com/vi/LrYRBRy2o2j9KPXJ/0.jpg)](https://youtu.be/c06t9Fp7DUM?si=LrYRBRy2o2j9KPXJ)

*task 1a: https://youtu.be/c06t9Fp7DUM?si=LrYRBRy2o2j9KPXJ

*task 1c: https://youtu.be/6spwOHrpxCE?si=wuqR7UYc63uv4LN5


### 🤖 Optimal Control of Multi-Robot Systems
**PhD Research**
**Control Theory, MATLAB**
* Nonlinear Model Predictive Control design ensures optimal control of each individual agent.
* High-level formation trajectory planner ensures desired formation.
* Integrated planning and tracking control architecture allows optimal control of Multi-Robot Systems in a formation.

### 🤖 Anti-Swing Prescribed Time Adaptive Control of overhead gantry cranes
**PhD Research**
**Control Theory, MATLAB, SIMULINK**
* 1. Multi-Body Lagrangian Dynamics Modeling
The foundation of the control system is a high-fidelity dynamic model derived using the Euler-Lagrange formulation. The system is modeled as a 7-DOF underactuated mechanism where the state vector $q(t) = [x, y, l_1, \theta_1, \theta_2, \theta_3, \theta_4]^T$ represents the trolley/bridge displacement, hoisting cable length, and the spherical swing angles of the dual payload. The equations of motion are expressed as $\underline{M}(q)\ddot{q} + \underline{C}(q,\dot{q})\dot{q} + \underline{G}(q) = \underline{F}$, capturing the highly coupled nonlinearities between the actuated translation states and the unactuated sway dynamics.
* 2. Partial Feedback Linearization & Decoupling
To manage the underactuated nature of the system (where control inputs are fewer than outputs), the dynamics are transformed using Partial Feedback Linearization. The system matrices are decomposed into actuated ($q_a$) and unactuated ($q_b$) sub-blocks. Through algebraic manipulation, the original Lagrangian model is converted into a Block-Strict Feedback Form, effectively cascading the dynamics into lower-order subsystems. This transformation allows for the design of virtual controllers ($\alpha_1, \alpha_2$) that stabilize the intermediate states before computing the final control torque.
* 3. Adaptive Actuator Friction Compensation
A critical real-world challenge addressed is the non-linear friction in the trolley, bridge, and hoisting mechanisms. The controller employs an Adaptive Estimation Law to approximate unknown friction coefficients in real-time. The friction model incorporates both Coulomb and viscous friction components (modeled using hyperbolic tangent functions $\tanh(\dot{x}/\epsilon)$). The adaptive law updates the estimated friction vector $\hat{\Omega}$ online, minimizing the estimation error $\tilde{\Omega}$ and ensuring that steady-state positioning errors are eliminated without requiring offline parameter identification.
* 4. Prescribed-Time Control Synthesis
Unlike asymptotic controllers that converge over infinite time, this project implements a Prescribed-Time Control strategy using time-varying scaling functions. A scaling term $\Delta(t) = \mu_i / (T_s - t)$ is introduced into the error dynamics, which forces the tracking errors to zero exactly at the pre-set termination time $T_s$. The control law is derived using a Lyapunov Candidate Function, ensuring that the system states remain bounded during the transient phase and converge precisely as $t \to T_s$.
* 5. Zero-Overshoot Stability Analysis
To prevent dangerous payload collisions or cable snapping, the controller is mathematically constrained to ensure Zero Overshoot. Through rigorous stability analysis, specific gain conditions are derived (e.g., relating $\hat{c}_1$ and $\hat{c}_2$) that enforce strict monotonicity of the Lyapunov function. This guarantees that the trajectory tracking error does not cross zero (oscillate) before convergence, a critical improvement over traditional PID or sliding mode controllers which often exhibit transient oscillations.




### 🚀 Hologlyph Bots
**eYantra_25-26**
**ROS2, Gazebo, OpenCV, EasyEDA**
Using multiple nonholonomic robots to create large-scale designs or "geoglyphs," involving coordinated swarm robotics for efficiency in tasks like complex pattern formation.
* 1. Holonomic Navigation & Kinematics Control
This task relies on ROS 2 Humble nodes to manage the complex movement of a 3-Wheel Omnidirectional (Holonomic) Drive. Unlike differential drive robots, these bots utilize Inverse Kinematics (IK) matrices to resolve the desired chassis velocity vector $(v_x, v_y, \omega_z)$ into individual wheel angular velocities. The core navigation logic involves implementing a PID Controller to minimize the error between the robot's current pose and the target trajectory points. Velocity commands are published to the /cmd_vel topic using geometry_msgs/Twist messages, enabling simultaneous translation and rotation for smooth, drift-free motion.
* 2. Global Perception & State Estimation Perception is handled using a global "Eye-to-Hand" vision system with OpenCV within a ROS 2 package. The pipeline relies on an Overhead Camera to track the robots' positions in real-time using ArUco Markers. The system employs Perspective Transformation (cv2.getPerspectiveTransform and cv2.warpPerspective) to map the skewed camera feed onto a corrected 2D coordinate system (Homography) of the arena. This provides precise (x, y, \theta) state estimation for each robot, which is published as odometry data to close the feedback loop, compensating for wheel slippage and drift.
* 3. Trajectory Generation & Swarm Coordination The "manipulation" task involves the precise execution of mathematical path planning to "draw" shapes (Glyphs) such as Lissajous figures or geometric polygons. The technical core requires a Trajectory Planner that generates high-resolution waypoints based on parametric equations. This process relies on Multi-Robot Coordination algorithms to synchronize the movement of multiple bots (Swarm) within the shared Gazebo or physical arena, ensuring they maintain formation and avoid collisions while tracing their assigned segments of the composite image.

---

## 📫 Connect with Me
* [LinkedIn](www.linkedin.com/in/somesh-swami-8b7684219)
* Email: somesh.swami398@gmail.com
