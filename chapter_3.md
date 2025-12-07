# Chapter 3: Actuation & Control Systems

## Chapter Introduction
After understanding how humanoid robots perceive their environment and internal state, the next crucial step is to enable them to act upon it. This chapter delves into the twin pillars of **actuation** and **control systems**, which are responsible for generating and regulating robot movement. We will begin by exploring the diverse types of actuators that serve as a robot's "muscles," examining their operating principles and characteristics, with a particular focus on those suitable for humanoids. Subsequently, we will transition into the fundamentals of control theory, starting from basic motor control and progressing to sophisticated joint-level and whole-body control architectures. Emphasis will be placed on feedback control, PID controllers, and critical aspects of stability analysis, such as the Zero Moment Point (ZMP). By the end of this chapter, students will have a comprehensive understanding of how physical motion is orchestrated and precisely managed in complex robotic systems.

## Learning Objectives
Upon completing this chapter, students should be able to:
- Explain the fundamental role of actuation in robotic systems, especially for humanoids.
- Identify and describe the characteristics of different types of actuators, including electric, hydraulic, and series elastic actuators.
- Understand the basic principles of motor control, including PWM and closed-loop feedback.
- Describe the components and operation of a Proportional-Integral-Derivative (PID) controller.
- Differentiate between various joint-level control strategies (position, velocity, torque, impedance).
- Explain high-level whole-body control architectures and their relevance to humanoid balance and manipulation.
- Understand the concept of Zero Moment Point (ZMP) and its application in humanoid locomotion stability.
- Recognize the key challenges in actuation and control for highly dynamic humanoid robots.

## Key Concepts
-   **Actuation:** The process of converting energy (electrical, hydraulic, pneumatic) into mechanical motion or force to drive robot components.
-   **Actuator:** A device that converts an input signal (usually electrical) into physical motion or force.
-   **Control System:** A system that manages, commands, directs, or regulates the behavior of other devices or systems using control loops.
-   **Feedback Control:** A control strategy where the output of a system is measured and fed back to adjust the input, aiming to reduce the difference between the desired and actual output.
-   **PID Controller:** A ubiquitous feedback control loop mechanism that continuously calculates an error value as the difference between a desired setpoint and a measured process variable, and applies a correction based on proportional, integral, and derivative terms.
-   **Degrees of Freedom (DoF):** The number of independent parameters that define the configuration of a mechanical system.
-   **Zero Moment Point (ZMP):** A concept used in legged locomotion, defined as the point on the ground where the total moment of all active forces (gravity, inertial, contact) equals zero, critical for maintaining dynamic balance.
-   **Compliance:** The ability of a robot system or actuator to yield or deform in response to external forces, often desirable for safe human-robot interaction and robustness to impacts.

## Step-by-step teaching explanation

### 3.1 Introduction to Robot Actuation
Actuators are the muscles of a robot, responsible for generating all movement. For humanoids, actuators face stringent requirements: they must be powerful yet lightweight, precise, capable of high torque-to-weight ratios, and often compliant for safe interaction. The choice of actuator significantly impacts a robot's dynamic capabilities, energy consumption, and overall performance. The design challenge lies in finding actuators that can mimic human muscle characteristics—high power, compliance, and energy efficiency—within strict size and weight constraints.

### 3.2 Types of Actuators
-   **Electric Motors:** The most common type due to their clean operation, ease of control, and high efficiency.
    -   **DC Motors:** Simple, affordable, but require brushes, leading to wear.
    -   **Brushless DC (BLDC) Motors:** More efficient, longer lifespan, higher power density, but require more complex electronic commutation. Widely used in advanced robotics.
    -   **Servo Motors:** Often integrated packages with a DC/BLDC motor, gearbox, and control electronics, providing precise position control.
-   **Hydraulic and Pneumatic Actuators:** Offer very high power-to-weight ratios and stiffness, making them suitable for heavy-duty robots (like Boston Dynamics' Atlas). However, they are complex, noisy, require external power units (pumps/compressors), and can leak.
-   **Series Elastic Actuators (SEAs):** Introduce an elastic element (spring) in series with a motor and load. This provides compliance, intrinsic force control, shock absorption, and energy storage, making them excellent for safe human-robot interaction and dynamic tasks. The spring allows for accurate force sensing by measuring its deflection. E.g., a humanoid hip joint might use an SEA for compliant walking.
-   **Compliant Actuation Principles:** Beyond SEAs, other designs aim for compliance, either actively (through control algorithms) or passively (through mechanical design), to enhance safety, robustness, and efficiency.

### 3.3 Motor Control Fundamentals
Precise control of motors is foundational to robot motion.
-   **Pulse Width Modulation (PWM):** A technique used to control the average power delivered to an electrical device by varying the width of the pulses in a train of pulses. For motors, PWM effectively controls the average voltage, thus regulating speed and torque.
-   **H-bridge Circuits:** Electronic circuits that allow a voltage to be applied across a load (like a DC motor) in either direction. Essential for bidirectional motor control (forward/reverse).
-   **Closed-loop Control for Motors:** Unlike open-loop control (which just sends a command without checking the output), closed-loop motor control uses feedback (e.g., from an encoder) to continuously compare the actual motor speed/position to the desired one, and adjusts the input to minimize the error.

### 3.4 Control System Basics
A control system aims to regulate the behavior of a dynamic system (e.g., a robot joint) to achieve a desired output.
-   **Open-loop Control:** The control action is independent of the output. Simple but susceptible to disturbances and model inaccuracies. E.g., setting a motor to 50% power without checking its actual speed.
-   **Closed-loop Control (Feedback Control):** The control action is dependent on the output. It measures the output, compares it to a desired setpoint, and uses the error to adjust the input. This makes the system more robust and accurate. E.g., a thermostat. The fundamental components are a sensor, a controller, and an actuator.
-   **PID Controllers:** The most widely used feedback control algorithm. It computes a control output based on three terms:
    -   **Proportional (P) Term:** Proportional to the current error. A larger error leads to a larger corrective action. It reduces error but can result in oscillation or steady-state error.
    -   **Integral (I) Term:** Proportional to the accumulation of past errors. It eliminates steady-state error but can increase overshoot and settling time.
    -   **Derivative (D) Term:** Proportional to the rate of change of the error. It dampens oscillations and improves stability, responding to changes quickly, but can amplify noise.
    -   Tuning the P, I, and D gains is critical for optimal system performance.

### 3.5 Joint-Level Control
These strategies control individual robot joints:
-   **Position Control:** The controller tries to make a joint reach and hold a specific angle. Most common for basic tasks.
-   **Velocity Control:** The controller maintains a desired angular velocity for a joint. Useful for continuous movements.
-   **Torque/Force Control:** The controller regulates the torque or force exerted by a joint or end-effector. Critical for compliant interaction, grasping delicate objects, or walking on uneven terrain.
-   **Impedance Control:** A sophisticated method that controls the relationship between force and position at the robot's interaction point (e.g., end-effector). It makes the robot 'feel' stiffer or softer to external forces, enabling flexible interaction with the environment.

### 3.6 Whole-Body Control Architectures
For humanoids, controlling individual joints in isolation is insufficient. Whole-body control coordinates all joints and links to achieve complex tasks while maintaining balance and avoiding self-collision.
-   **Task-Space Control:** Focuses on controlling the robot's end-effectors (e.g., hands, feet) in Cartesian space (x, y, z, roll, pitch, yaw), rather than individual joint angles. Inverse kinematics is heavily used here.
-   **Operational-Space Control:** A framework that prioritizes tasks, allowing lower-priority tasks (e.g., maintaining joint limits) to be performed in the null space of higher-priority tasks (e.g., reaching for an object). This ensures that critical objectives are met while secondary constraints are respected.
-   **Hierarchical Control Frameworks:** Break down complex behaviors into a hierarchy of controllers, from low-level motor control to high-level trajectory planning and task execution. This allows for modularity and managing complexity.

### 3.7 Stability Analysis
Ensuring a humanoid robot remains stable, especially during locomotion, is paramount.
-   **Lyapunov Stability:** A mathematical method used to analyze the stability of dynamic systems. If a Lyapunov function can be found whose derivative is negative definite, the system is stable.
-   **Passivity-Based Control:** Utilizes the concept of passivity (systems that do not generate energy) to design robust controllers for physical interaction, often applied in compliant robotics.
-   **Zero Moment Point (ZMP):** A fundamental concept for bipedal locomotion. The ZMP is the point on the ground where the combined moment of gravity and inertial forces is zero. For stable walking, the ZMP must remain within the support polygon (the convex hull of the points of contact with the ground). Controllers actively shift the robot's center of mass to keep the ZMP within this region.

### 3.8 Challenges in Actuation and Control
Controlling high-DoF humanoid robots in dynamic environments presents significant challenges:
-   **High Degrees of Freedom:** Managing and coordinating dozens of joints simultaneously is computationally intensive and complex.
-   **Compliance and Safety:** Balancing the need for powerful actuation with the requirement for compliant, safe interaction with humans.
-   **Energy Management:** Efficiently distributing power to numerous actuators to maximize battery life and minimize heat generation.
-   **Real-time Constraints and Latency:** Control loops must operate at very high frequencies (e.g., 1 kHz or more) to ensure stable and responsive behavior, requiring low-latency sensor feedback and fast computation.
-   **Model Uncertainty and Disturbances:** Real-world dynamics are hard to model perfectly. Controllers must be robust to unmodeled dynamics, external forces, and sensor noise.

## Math + equations (if relevant)

### PID Controller Equation
The output $u(t)$ of a PID controller, which is fed to the actuator, is given by:

$$ u(t) = K_p e(t) + K_i \int_{0}^{t} e(\tau) d\tau + K_d \frac{de(t)}{dt} $$

Where:
-   $e(t) = r(t) - y(t)$ is the error, the difference between the desired setpoint $r(t)$ and the measured process variable $y(t)$.
-   $K_p$ is the proportional gain.
-   $K_i$ is the integral gain.
-   $K_d$ is the derivative gain.

Each term contributes to the control action:
-   **Proportional term ($K_p e(t)$):** Responds to the current error. A larger $K_p$ makes the system respond more aggressively, but too large can cause oscillations.
-   **Integral term ($K_i \int_{0}^{t} e(\tau) d\tau$):** Eliminates steady-state error by accumulating past errors. Prevents the system from settling at an offset from the setpoint.
-   **Derivative term ($K_d \frac{de(t)}{dt}$):** Responds to the rate of change of the error. Predicts future error and dampens oscillations, improving stability and reducing overshoot.

### Zero Moment Point (ZMP) (Conceptual Formula)
For stable bipedal locomotion, the ZMP must stay within the support polygon. The ZMP can be conceptually understood as:

$$ x_{ZMP} = \frac{\sum_i m_i (x_i \ddot{z}_i - z_i \ddot{x}_i) + \sum_j F_{jx} (y_{j} - y_{ZMP}) - F_{jy} (x_{j} - x_{ZMP}) + \sum_k \tau_{jy}}{\sum_i m_i (g + \ddot{z}_i)} $$

This is a simplified representation. The full derivation involves summing moments of gravitational, inertial, and contact forces. The key takeaway is that the ZMP is a calculated point on the ground plane, and its position is directly influenced by the robot's center of mass acceleration and contact forces. By controlling the robot's motion to keep the ZMP within the stable region, a humanoid can maintain balance during walking, running, or standing.

## Examples

### Example 3.1: PID Control of a Single Humanoid Joint
Consider a single hip joint of a humanoid robot that needs to move to a specific angle (e.g., 30 degrees). A PID controller is typically used for this joint-level position control.

-   **Setpoint ($r(t)$):** 30 degrees.
-   **Process Variable ($y(t)$):** The current measured angle of the hip joint, obtained from an encoder.
-   **Error ($e(t)$):** `30 - current_angle`.
-   **Controller Output ($u(t)$):** The voltage (or PWM signal) sent to the motor driving the hip joint.

When the joint is far from 30 degrees, the **P term** generates a strong corrective voltage. As it approaches 30, the P term decreases. If the joint overshoots, the P term reverses direction. If there's a small persistent error (e.g., due to gravity), the **I term** gradually builds up to eliminate this steady-state error. The **D term** monitors how fast the error is changing. If the joint is moving rapidly towards the setpoint, the D term provides a braking action to prevent overshoot, or if a sudden disturbance pushes the joint, it immediately provides a counter-force to restore stability.

### Example 3.2: Humanoid Balancing using ZMP Control
Boston Dynamics' Atlas robot needs to maintain balance while traversing uneven terrain or even performing complex acrobatic maneuvers. This requires sophisticated whole-body control, often employing ZMP-based strategies.

-   **Sensors:** Atlas uses IMUs, force/torque sensors in its feet, and joint encoders to precisely measure its body orientation, contact forces, and joint positions.
-   **Control Task:** Maintain dynamic balance while walking.
-   **Strategy:** The control system continuously calculates the current ZMP based on the robot's motion and ground contact forces. It then compares this to a desired ZMP trajectory, which is planned to keep the robot stable. If the actual ZMP deviates from the desired path, the controller generates corrective joint torques (e.g., by adjusting ankle or hip joint angles, or shifting the upper body) to bring the ZMP back within the support polygon (the area defined by the robot's feet on the ground). This active control of the ZMP is what allows Atlas to walk, run, and jump without falling over.

## Diagrams

### Diagram 3.1: PID Control Loop (Block Diagram)

```
+-----------+
| Setpoint  |--------+
|   r(t)    |        |  +----------+
+-----------+        |  |  Error   |
                     |  |  e(t)    |
                     V  +----------+
            +--------|----------+
            |        |          |
            |   +----V-----+   |
            |   | PID      |<--|
            |   | Controller |   |
            |   +----+-----+   |
            |        | u(t)    |
            |        V         |
            |  +-----------+   |
            |  | Actuator  |   |
            |  | (Motor)   |   |
            |  +-----+-----+   |
            |        |         |
            |        V y(t)    |
            |  +-----------+   |
            |  |   Plant   |<--+ Feedback
            |  | (Robot Joint) |
            |  +-----------+   |
            +------------------+
```

**Caption:** This block diagram illustrates the closed-loop feedback structure of a PID controller. The error between the desired setpoint and the measured output is fed into the PID controller, which then generates a control signal `u(t)` to drive the actuator (e.g., a motor) in the plant (e.g., a robot joint), with the output `y(t)` being measured by a sensor and fed back to the comparison point.

**Teaching Notes:** Walk through each block, explaining its role. Emphasize the feedback loop and how it enables the system to correct itself. Discuss the meaning of `r(t)`, `y(t)`, `e(t)`, and `u(t)`. This is a fundamental diagram for understanding control systems.

### Diagram 3.2: Comparison of Actuator Types (Simplified Table)

```
| Actuator Type  | Power Density  | Precision | Compliance | Complexity | Typical Applications (Humanoid Focus) |
|----------------|----------------|-----------|------------|------------|---------------------------------------|
| Electric (BLDC)| High           | Very High | Low (can add)| Medium     | Joint control, fine manipulation      |
| Hydraulic      | Very High      | High      | Low        | High       | High-power legs, dynamic movements    |
| Pneumatic      | Medium         | Medium    | High       | Medium     | Soft robotics, compliant grippers     |
| Series Elastic | High           | High      | Very High  | High       | Leg joints, safe human interaction    |
```

**Caption:** A simplified comparison of different actuator types based on key performance metrics relevant to humanoid robotics, including power density, precision, compliance, and complexity. This table helps in understanding the trade-offs involved in actuator selection.

**Teaching Notes:** Use this table to discuss the engineering challenges of humanoid design. Highlight that different parts of a humanoid robot (e.g., a hand vs. a leg) might require different actuator types due to varying demands. Discuss how SEAs offer a unique balance for humanoids.

### Diagram 3.3: Zero Moment Point (ZMP) Concept (Illustration)

```
        +-------------------+
        |      Robot Torso  |
        |  (Center of Mass) |
        +---------+---------+
                  |
                  |
        +---------V---------+
        |      Leg          |
        |                   |
        +---------+---------+
                  |
                  |
        +---------+---------+
        |      Foot         |
        | (Support Polygon) |
        +---------+---------+
                  |
        +---------O---------+ <--- ZMP (point on ground)
        |   Ground Surface  |
        +-------------------+
```

**Caption:** A conceptual illustration of the Zero Moment Point (ZMP) during single-support phase of a humanoid robot. The ZMP is the point on the ground where the effective ground reaction force acts, and it must remain within the support polygon (formed by the contact area of the foot) for the robot to maintain balance.

**Teaching Notes:** Explain that the ZMP is not a physical point on the robot, but a calculated point on the ground. Emphasize its dynamic nature and how controlling the robot's motion (especially the center of mass) is key to keeping the ZMP within the support polygon. Discuss single vs. double support phases and how ZMP applies.

## Lab Module 3.1: PID Controller Tuning for a Simulated Robot Joint

### Objective
This lab provides practical experience with implementing and tuning a Proportional-Integral-Derivative (PID) controller for a single simulated robot joint in a ROS/Gazebo environment. Students will learn how to adjust PID gains to achieve desired joint position control performance, observing the effects of each gain on stability, overshoot, and steady-state error.

### Theory Section
PID controllers are the workhorse of industrial control and are extensively used in robotics for regulating joint positions, velocities, or torques. Proper tuning of the proportional ($K_p$), integral ($K_i$), and derivative ($K_d$) gains is critical to achieve stable, fast, and accurate control without excessive overshoot or oscillation. An improperly tuned PID controller can lead to unstable behavior, poor tracking performance, or excessive wear on actuators. Common tuning methods include trial-and-error, Ziegler-Nichols, or more advanced optimization techniques. This lab will focus on a systematic trial-and-error approach to develop intuition for each gain's effect.

### Required Background
-   Familiarity with ROS (nodes, topics, services) and Gazebo.
-   Basic Python programming.
-   Understanding of closed-loop control and the conceptual roles of P, I, D terms.

### Materials/Software Needed
-   **Operating System:** Ubuntu 20.04 LTS (with ROS Noetic installed).
-   **Gazebo Simulator:** With a simulated robot model equipped with at least one controllable joint (e.g., the simple arm model from Lab 1.1).
-   **ROS Noetic:** Including `ros_control` and `rqt_reconfigure` (for dynamic PID gain tuning).
-   **Python 3.**
-   **Code Editor.**

### Step-by-step Instructions

#### Part 1: Setting up the Simulated Joint and Controller
1.  **Launch your simulated robot in Gazebo:** Ensure it has a `ros_control` `JointPositionController` configured for at least one joint. You can reuse the launch file from Lab 1.1 if it sets up a joint controller (e.g., `/arm_controller/joint_trajectory_controller` also handles position control).
    ```bash
    roslaunch my_humanoid_description display_and_control_arm.launch
    ```
2.  **Verify Controller is Running:** In a new terminal:
    ```bash
    rostopic list | grep command
    rostopic list | grep state
    ```
    You should see command and state topics for your joint controller (e.g., `/arm_controller/command`, `/arm_controller/state`).
3.  **Launch `rqt_reconfigure`:** This tool allows dynamic modification of ROS parameters, including PID gains if exposed by the controller.
    ```bash
    rosrun rqt_reconfigure rqt_reconfigure
    ```
    In the `rqt_reconfigure` GUI, navigate to your joint controller (e.g., `/arm_controller/pid_gains` or similar) to find parameters like `p`, `i`, `d`.

#### Part 2: Sending Joint Commands
1.  **Open a new Terminal.**
2.  **Source ROS Environment.**
3.  **Send a target position to a single joint:** We will use `rostopic pub` for simplicity. Replace `JOINT_NAME` with one of your robot's joint names (e.g., `shoulder_pan_joint`) and adjust the `POSITION` (e.g., `1.0` radians).
    ```bash
    rostopic pub -1 /arm_controller/joint_commands std_msgs/Float64 -- 1.0
    # Note: The topic and message type might vary based on your controller setup.
    # For JointTrajectoryController, you'd use the JointTrajectory message as in Lab 1.1,
    # but we need a simple way to set a *single* target position to see PID effect.
    # A simple JointPositionController might expose a topic like `/joint_name_controller/command` of type `std_msgs/Float64`.
    # Assume for this lab, a single joint position controller exists on `/joint_name_controller/command`
    # If using JointTrajectoryController, send a single-point trajectory as in Lab 1.1.
    ```
    Alternatively, if your controller exposes a simple `std_msgs/Float64` topic for a single joint:
    ```bash
    rostopic pub /joint_name_controller/command std_msgs/Float64 "data: 1.0" -r 10 # -r 10 for 10 Hz publish rate
    ```
    Observe the robot's joint moving to the target position in Gazebo/Rviz.

#### Part 3: PID Gain Tuning
1.  **Initial Gains:** Start with very low or zero `i` and `d` gains, and a small `p` gain (e.g., `p=10, i=0, d=0`). Send a target position command.
2.  **Tune P Gain:** Gradually increase the `p` gain in `rqt_reconfigure`. Observe:
    -   How quickly the joint reaches the target.
    -   Overshoot (does it go past the target and come back?).
    -   Oscillations (does it wobble around the target?).
    -   Steady-state error (does it settle exactly at the target, or slightly off?).
    Aim for a `p` gain that makes the system responsive but without excessive oscillation.
3.  **Tune D Gain:** Once a reasonable `p` gain is found, gradually increase the `d` gain. Observe:
    -   How `d` dampens oscillations and reduces overshoot.
    -   Too high `d` can make the system sluggish or noisy.
    Aim for a `d` gain that provides good damping without making the system too slow.
4.  **Tune I Gain:** Finally, gradually increase the `i` gain. Observe:
    -   How `i` eliminates any persistent steady-state error.
    -   Too high `i` can cause slow oscillations or windup.
    Aim for an `i` gain that removes steady-state error without reintroducing instability.
5.  **Document Tuning Process:** Keep a log of gain values and observed robot behavior for different tuning steps.

### Code Template (Python)
For sending target commands, students can adapt the `simple_arm_controller.py` from Lab 1.1 to send single-point trajectories or directly publish to a `std_msgs/Float64` topic if available. The core of this lab is hands-on tuning using `rqt_reconfigure` rather than extensive coding.

```python
# Example of how to publish a single joint command using std_msgs/Float64
# (Assumes a controller is set up to receive this type of command on this topic)

import rospy
from std_msgs.msg import Float64
import sys

def publish_joint_command(joint_topic, position):
    rospy.init_node('single_joint_publisher', anonymous=True)
    pub = rospy.Publisher(joint_topic, Float64, queue_size=10)
    rate = rospy.Rate(10) # 10 Hz

    rospy.loginfo(f"Publishing {position} to {joint_topic}")

    while not rospy.is_shutdown():
        msg = Float64()
        msg.data = float(position)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: python3 single_joint_publisher.py <joint_command_topic> <target_position>")
        sys.exit(1)

    joint_topic = sys.argv[1]
    target_position = sys.argv[2]

    try:
        publish_joint_command(joint_topic, target_position)
    except rospy.ROSInterruptException:
        pass
```

### Expected Results
-   **Part 1:** Simulated robot launches with a controllable joint. `rqt_reconfigure` connects to the joint controller and displays its PID gain parameters.
-   **Part 2:** Students can successfully send target position commands to the joint.
-   **Part 3:** Students systematically tune PID gains, observing the effects on joint movement: initially oscillatory or slow, eventually achieving stable and accurate position tracking with minimal overshoot and steady-state error. Documentation of tuning steps and observed behavior.

### Grading Rubric
-   **Setup and Launch (20%):** Successful setup of the simulated environment and `rqt_reconfigure`.
-   **Commanding Joint (20%):** Demonstrated ability to send target position commands to the simulated joint.
-   **PID Tuning Process (40%):** Systematic approach to tuning P, I, and D gains, clearly documenting the steps and rationale for adjustments. Achieves a well-tuned controller for position tracking.
-   **Observations and Analysis (20%):** Clear discussion of how each PID gain affects system response (overshoot, oscillation, steady-state error, responsiveness). Identification of challenges and insights gained during the tuning process.

## Summary
Chapter 3 provided a foundational understanding of actuation and control systems in humanoid robotics. We explored various actuator types, from electric motors to compliant series elastic actuators, highlighting their respective advantages and trade-offs. The core principles of control systems were introduced, with a detailed examination of feedback control and the ubiquitous PID controller. We then discussed different joint-level control strategies, such as position, velocity, torque, and impedance control, and scaled up to whole-body control architectures. Crucial for humanoid stability, the Zero Moment Point (ZMP) concept was explained. The chapter concluded by outlining the significant challenges in achieving robust and dynamic control for high-DoF humanoids. The accompanying lab module offered hands-on experience in PID controller tuning, allowing students to directly observe the impact of gain adjustments on robot joint performance.

## 8-12 Problem-set questions
1.  What is the primary function of an actuator in a robotic system? Compare and contrast electric motors with hydraulic actuators in terms of power density and complexity for humanoid applications.
2.  Explain the concept of "compliance" in robotic actuation. Why is compliance particularly desirable in humanoid robots, and how do Series Elastic Actuators (SEAs) achieve it?
3.  Describe the difference between open-loop and closed-loop control. Provide a simple everyday example of each.
4.  Briefly explain the role of each of the three terms (Proportional, Integral, Derivative) in a PID controller. What happens if the proportional gain ($K_p$) is set too high?
5.  A humanoid robot needs to grasp a delicate egg. Which type of joint-level control (position, velocity, torque, or impedance) would be most appropriate for the gripper fingers, and why?
6.  Define the Zero Moment Point (ZMP). How is the ZMP used to maintain dynamic balance during bipedal locomotion, and what does it mean if the ZMP falls outside the support polygon?
7.  What are "whole-body control architectures," and why are they necessary for humanoid robots instead of controlling each joint independently? Name one such architecture.
8.  Discuss two major challenges in controlling high-DoF humanoid robots in real-time, and explain why these challenges are significant.
9.  In Lab 3.1, you tuned a PID controller. Describe the typical behavior of a system when only the P-gain is active, and how adding the D-gain changes this behavior.
10. Explain how Pulse Width Modulation (PWM) is used to control the speed of a DC motor. Why is this more effective than simply varying the DC voltage directly?
11. Research and briefly describe one alternative to PID control (e.g., LQR, Model Predictive Control) that is often used in advanced robotics, and highlight its main advantage over PID.
12. A humanoid robot needs to interact physically with a human in a collaborative task. What control strategy would be crucial for ensuring safety during this interaction, and why?

## Instructor Notes
-   **Pacing:** This chapter builds on foundational control concepts. Ensure students have a good grasp of PID principles before moving to more advanced whole-body control. Practical examples and analogies are very helpful.
-   **Discussion Points:** Encourage discussion on the trade-offs between different actuator types and control strategies. Delve into the safety aspects of compliant actuation and human-robot interaction. A debate on "What are the limits of ZMP control?" can be fruitful.
-   **Lab Prep:** For Lab 3.1, thoroughly guide students on how to identify the correct ROS topics and parameters for their specific simulated robot's joint controller. The `rqt_reconfigure` tool can sometimes be tricky to navigate initially. Emphasize starting with conservative gain values and gradually increasing them. Warn about potential instability if gains are too high.
-   **Optional Extension:** For advanced students, challenge them to implement a custom PID controller in a Python script (publishing joint commands based on subscribed state feedback) rather than relying solely on `rqt_reconfigure` and existing ROS controllers. They could also explore auto-tuning methods.
-   **Real-World Context:** Show videos of humanoid robots demonstrating dynamic balance, compliant interaction, or fine manipulation to illustrate the concepts of ZMP, impedance control, and the capabilities of advanced actuators.
-   **Problem Set:** Problems 4, 6, and 9 are crucial for testing understanding of core control concepts and lab experience. Problem 11 encourages research into advanced control methods.
