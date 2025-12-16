---
title: "Understanding URDF"
---

# Lesson 4: Describing the Body: The Unified Robot Description Format (URDF)

Before we can control a robot, ROS needs to have a complete understanding of the robot's physical structure. What are its parts? How are they connected? How do they move? Where are the sensors?

The **Unified Robot Description Format (URDF)** is an XML file that serves as the robot's "digital blueprint" or "anatomy chart." It provides a standardized way to describe the physical properties of your robot so that all the different tools in the ROS ecosystem can understand it.

## Why is URDF so Important?

A well-defined URDF file is the foundation for many core robotics tasks:
*   **Visualization:** Tools like RViz2 use the URDF to display a 3D model of your robot, allowing you to see its state, sensor data, and planned movements in real-time.
*   **Simulation:** Physics simulators like Gazebo use the URDF (or a more advanced format like SDF that is often generated from a URDF) to understand the robot's mass, inertia, and collision properties to create a realistic digital twin.
*   **Kinematics and Dynamics:** Motion planning libraries use the URDF to calculate how to move the robot's joints to get its hand (or "end-effector") to a desired position. This is known as solving the robot's kinematics.

## The Core Components of URDF: Links and Joints

A URDF file describes a robot as a tree of **links** connected by **joints**.

### 1. `<link>`
A link represents a rigid, physical part of the robot's body. Each link can have properties describing its visual appearance, collision geometry, and inertial properties (mass and moments of inertia).

*   **`<visual>`:** Defines how the link should look in visualization tools. This is often a 3D mesh file (like a `.stl` or `.dae`) or a simple geometric shape (box, cylinder, sphere).
*   **`<collision>`:** Defines the physical boundary of the link for collision detection in simulators. This is often a simpler shape than the visual mesh to speed up physics calculations.
*   **`<inertial>`:** Defines the link's mass and inertia, which are critical for realistic physics simulation.

### 2. `<joint>`
A joint connects two links together and defines how they are allowed to move relative to each other. Every joint has a `parent` link and a `child` link, forming the tree structure.

*   **`type`:** This is the most important attribute of a joint.
    *   `revolute`: A rotating joint with defined limits, like an elbow or a knee.
    *   `continuous`: A rotating joint with no limits, like a wheel.
    *   `prismatic`: A sliding joint that moves along an axis, like a piston.
    *   `fixed`: A joint that doesn't move. This is used to rigidly connect two links together (e.g., mounting a camera onto the head).
*   **`<parent>` and `<child>`:** These tags define the two links that the joint connects.
*   **`<origin>`:** Defines the position and orientation of the joint relative to the parent link's origin.
*   **`<axis>`:** For non-fixed joints, this defines the axis of rotation or sliding.

## Simple URDF Example: A Torso and a Right Arm

Here is a heavily simplified URDF to show the relationship between a torso and an upper arm.

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">

  <!-- First, define the links (the body parts) -->

  <!-- The Torso Link -->
  <link name="torso_link">
    <visual>
      <geometry>
        <box size="0.3 0.5 0.1"/> <!-- A simple box shape for the torso -->
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.5 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/> <!-- mass in kilograms -->
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0" />
    </inertial>
  </link>

  <!-- The Right Upper Arm Link -->
  <link name="right_upper_arm_link">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/> <!-- A cylinder for the arm -->
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0" />
    </inertial>
  </link>
  
  <!-- Now, define the joint that connects them -->

  <!-- The Shoulder Joint (Revolute) -->
  <joint name="right_shoulder_joint" type="revolute">
    <parent link="torso_link"/>
    <child link="right_upper_arm_link"/>
    <origin xyz="0 -0.3 0.1" rpy="0 0 0"/> <!-- Position the arm on the side of the torso -->
    <axis xyz="0 1 0"/> <!-- The axis of rotation (swinging forward/backward around the Y-axis) -->
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/> <!-- Joint limits in radians -->
  </joint>

</robot>
```

This URDF tells ROS that we have a robot with two parts, a `torso_link` and a `right_upper_arm_link`. They are connected by a `right_shoulder_joint` that can rotate around the Y-axis. This simple description is all that's needed for RViz2 to show the arm moving and for a simulator to understand its basic physics. A complete humanoid URDF is simply a much larger version of this, with dozens of links and joints.

:::note URDF and Xacro
Real-world URDF files can become very long and repetitive. To make them easier to manage, ROS developers often use **Xacro (XML Macros)**. Xacro is a scripting language that allows you to create variables, macros, and mathematical expressions, which are then processed to generate a final URDF file. This is the standard way to write clean, professional-grade robot descriptions.
:::