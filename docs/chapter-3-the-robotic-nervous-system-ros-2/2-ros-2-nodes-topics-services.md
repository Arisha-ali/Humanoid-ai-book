---
title: "ROS 2 Nodes, Topics, & Services"
---

# Lesson 2: Core Concepts: Nodes, Topics, and Services

To understand ROS 2, we need to learn its language. The entire framework is built on three fundamental concepts that govern how different parts of the robot communicate. We'll use the example of our humanoid wanting to see and wave at a person to illustrate them.

## 1. Nodes: The Building Blocks

A **Node** is the smallest unit of computation in ROS. Think of it as a single, specialized program responsible for one specific task. In a complex system like a humanoid, you will have many nodes running simultaneously.

*   **Humanoid Example:**
    *   A `camera_driver_node` that manages the camera in the humanoid's head and captures raw images.
    *   An `object_detector_node` that is a Python AI program responsible for processing images to find objects or people.
    *   A `right_arm_controller_node` that controls the motors and reads the sensors for the right arm.
    *   A `locomotion_node` that manages the complex task of walking and maintaining balance.

Each node is independent. If the `object_detector_node` crashes, the `locomotion_node` can still keep the robot standing. This modularity makes the system robust and easier to debug.

## 2. Topics: The Broadcast System (Publish/Subscribe)

**Topics** are the primary way nodes communicate. They are named buses for sending and receiving messages. A node can "publish" messages to a topic, and any number of other nodes can "subscribe" to that topic to receive those messages. This is an asynchronous, many-to-many communication model.

It's like a radio station. The DJ (a publisher node) broadcasts a signal on a specific frequency (the topic name). Anyone with a radio (subscriber nodes) can tune in to listen. The DJ doesn't need to know who is listening; they just send the signal out.

*   **Humanoid Waving Scenario using Topics:**
    1.  The `camera_driver_node` **publishes** raw image data to the `/head_camera/image_raw` topic.
    2.  The `object_detector_node` **subscribes** to `/head_camera/image_raw`. When it receives an image, it processes it. If it finds a person, it **publishes** a new message with the person's location to the `/found_person/location` topic.
    3.  The `right_arm_controller_node` could **subscribe** to a `/joint_commands` topic. A separate `motion_planning_node`, after seeing the person, would publish the joint angles needed to wave to this topic.

This decoupling is powerful. You can add new nodes that listen to these topics (like a data-logging node or a UI display) without changing any of the existing nodes.

## 3. Services: The Direct Request (Request/Response)

While topics are great for continuous data streams, sometimes you need a direct, synchronous transaction. You need to ask a specific question and wait for a specific answer. This is what **Services** are for.

A "client" node makes a request to a service, and a "server" node performs a task and sends back a response. Unlike topics, this is a one-to-one communication model, and the client will pause its execution until it receives the response.

*   **Humanoid Example using a Service:**
    Imagine our AI agent needs to know the exact angle of the elbow joint *right now* to plan a precise movement.
    1.  The `motion_planning_node` acts as a **client** and sends a request to the `/get_joint_state` service. The request might specify which joint it's interested in (e.g., `joint_name: "right_elbow_joint"`).
    2.  The `right_arm_controller_node`, which is running the **server** for that service, receives the request. It reads the current angle directly from the elbow motor's encoder.
    3.  The server sends the angle (e.g., `angle: 1.25` radians) back to the `motion_planning_node` as a **response**.

The motion planner now has the exact information it needs to proceed. This request-response pattern is perfect for tasks that need a guaranteed answer before continuing.

---

By combining Nodes, Topics, and Services, we can build sophisticated robotic applications from simple, reusable components.