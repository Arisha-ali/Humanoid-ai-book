---
title: "Processing Sensor Data with ROS 2"
---

# Lesson 3: Processing Sensor Data with ROS 2

We now understand the key sensors and the importance of fusing their data. But how does this data actually flow through the robot's "nervous system"? How does our AI "brain" access the information coming from the hardware? The answer lies in the Robot Operating System (ROS 2).

ROS 2 provides a standardized way for sensor hardware drivers to "publish" their data onto the network, and for any other part of the system to "subscribe" to that data. This lesson will show you the practical side of this process.

## Standard Sensor Messages

To ensure that all components in the ROS 2 ecosystem can understand each other, ROS defines a set of standard message types for common sensors. This means that a camera driver from one manufacturer will publish images using the same message format as a camera from another manufacturer, making our software highly modular and reusable.

Here are some of the most common sensor message types, which are typically found in the `sensor_msgs` package:

*   `sensor_msgs/msg/Imu`: Used for data from an Inertial Measurement Unit. It contains fields for `orientation` (as a quaternion), `angular_velocity`, and `linear_acceleration`.
*   `sensor_msgs/msg/Image`: Used for data from a camera. It contains the image data itself, as well as metadata like `height`, `width`, and `encoding` (e.g., "rgb8").
*   `sensor_msgs/msg/PointCloud2`: The standard message for 3D point cloud data, typically from a LiDAR or a depth camera. This is a complex but powerful message type that can store millions of 3D points along with other data like color.
*   `sensor_msgs/msg/LaserScan`: A simpler message for 2D LiDAR data, representing distances in a single plane around the robot.

By using these standard messages, we can write code that processes sensor data without needing to know the specific details of the hardware that produced it.

## Subscribing to Sensor Data in Python

Let's write a simple ROS 2 node in Python that subscribes to an IMU topic and prints out the robot's orientation data. This is a fundamental skill for any roboticist.

This node represents a small part of a "balance controller" that needs to know if the robot is tilting.

```python
# Import the necessary libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu  # Import the standard Imu message type

class BalanceMonitorNode(Node):
    """
    A simple ROS 2 Node that subscribes to IMU data and prints it.
    """
    def __init__(self):
        # Initialize the Node with a name
        super().__init__('balance_monitor')
        
        # Create a subscriber.
        # It will receive Imu messages from the '/imu/data' topic.
        # When a message is received, the 'imu_callback' method is called.
        # The '10' is the queue size.
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10)
        
        self.get_logger().info('Balance Monitor node has been started.')
        self.get_logger().info('Listening for data on /imu/data...')

    def imu_callback(self, msg):
        """
        This method is called every time a new message is received on the topic.
        """
        # The 'msg' object has attributes that match the fields in the Imu message.
        orientation = msg.orientation
        
        # We can now access the orientation data. A quaternion has x, y, z, and w components.
        # A real balance controller would use this data in a control loop.
        # For this example, we will just log it to the console.
        self.get_logger().info(
            f'Received IMU Data: Orientation (w, x, y, z) = '
            f'({orientation.w:.2f}, {orientation.x:.2f}, {orientation.y:.2f}, {orientation.z:.2f})'
        )

def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)
    
    # Create an instance of our node
    balance_monitor = BalanceMonitorNode()
    
    # "spin" the node, which keeps the script alive and allows it to process callbacks.
    # Any messages received by the subscriber will be processed by the imu_callback method.
    rclpy.spin(balance_monitor)
    
    # Clean up and destroy the node when done (e.g., on Ctrl+C)
    balance_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### How It Works

1.  **Import `Imu`:** We import the standard message type so our code knows the structure of the data it will receive.
2.  **`create_subscription`:** In our node's `__init__` method, we declare that we want to subscribe to the `/imu/data` topic. We specify the message type (`Imu`) and provide a callback function (`self.imu_callback`).
3.  **The Callback Function:** The `imu_callback` method is the heart of our subscriber. ROS 2 automatically calls this function whenever a new message arrives on the `/imu/data` topic, passing the message object as an argument (`msg`).
4.  **Accessing Data:** Inside the callback, we can access the fields of the message using dot notation (e.g., `msg.orientation.w`).
5.  **`rclpy.spin`:** This crucial function enters a loop, waiting for messages and executing the appropriate callbacks. Without `spin`, the script would just exit.

## Conclusion

The ability to subscribe to and process sensor data is a fundamental building block of robotics programming. By leveraging ROS 2's standardized messages and communication framework, we can easily create Python nodes that listen to the robot's senses and form the basis of intelligent, reactive behaviors. This simple pattern of creating a subscriber with a callback function is one you will use time and time again as you build your humanoid's brain.
