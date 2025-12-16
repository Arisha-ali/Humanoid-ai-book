---
title: "Bridging Python AI Agents to ROS"
---

# Lesson 3: Bridging Python AI Agents to ROS 2

This is where the "AI" in Physical AI truly connects to the "Physical." Our high-level artificial intelligence, such as machine learning models for perception or large language models for cognitive planning, will almost certainly be written in Python. ROS 2 provides an official Python client library called `rclpy` that allows our Python code to become a first-class citizen in the ROS ecosystem.

This lesson demonstrates how a Python-based AI agent can communicate with the rest of the robot's systems by creating ROS 2 nodes, publishers, and subscribers.

## Why `rclpy` is Essential

The `rclpy` library provides the necessary tools to:
*   Initialize the ROS 2 communication system within a Python script.
*   Create nodes, the fundamental building blocks of a ROS application.
*   Create publishers to send messages (i.e., commands) to other nodes.
*   Create subscribers to receive messages (i.e., sensor data) from other nodes.
*   Create services and clients for request/response interactions.

This allows our Python "brain" to seamlessly integrate with the robot's "nervous system," commanding actuators and interpreting sensor data.

## Example: An AI "Gait Commander" Node

Let's create a practical example: a Python node that acts as a high-level AI agent. Its job is to decide when the robot should start or stop walking. It will do this by publishing a simple text command to a topic, which a lower-level locomotion controller would listen to.

This example demonstrates the "publisher" pattern, where our AI sends out a command.

```python
# Import the necessary libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # A standard message type for sending simple text

class GaitCommanderNode(Node):
    """
    A simple AI Node that publishes commands to control walking.
    In a real system, the decision to walk would come from a complex
    AI planning model. Here, we simulate it with a simple timer.
    """
    def __init__(self):
        # Initialize the Node with a unique name
        super().__init__('gait_commander_node')
        
        # Create a publisher. 
        # It will publish 'String' messages to the '/gait_command' topic.
        # The '10' is the queue size - a quality of service setting that
        # tells ROS how many messages to buffer if they are sent faster
        # than they can be processed.
        self.publisher_ = self.create_publisher(String, '/gait_command', 10)
        
        # We'll use a timer to call our decision-making method every 5 seconds.
        self.timer_period = 5.0  # seconds
        self.timer = self.create_timer(self.timer_period, self.make_decision_and_publish)
        
        self.get_logger().info('Gait Commander AI node has been started.')
        self.get_logger().info('Will publish a command every 5 seconds...')

    def make_decision_and_publish(self):
        # This method simulates the AI making a decision.
        # For this example, we'll just alternate between starting and stopping.
        
        # NOTE: In a real AI agent, this function would contain logic like:
        # if goal_is_far_away and path_is_clear:
        #     command = "start_walking"
        # else:
        #     command = "stop_walking"

        # Let's invent a simple state
        if not hasattr(self, 'is_walking'):
            self.is_walking = False
        
        command_str = "start_walking" if not self.is_walking else "stop_walking"
        self.is_walking = not self.is_walking # Toggle state

        # Create a String message object
        msg = String()
        msg.data = command_str
        
        # Publish the message to the topic
        self.publisher_.publish(msg)
        
        # Log the action to the console for debugging
        self.get_logger().info(f'AI Decision: Publishing gait command -> "{msg.data}"')

def main(args=None):
    # Initialize the rclpy library for this process
    rclpy.init(args=args)
    
    # Create an instance of our GaitCommanderNode
    gait_commander = GaitCommanderNode()
    
    # "spin" the node. This is a crucial step. rclpy.spin() enters a loop,
    # keeping the Python script from exiting and allowing the node to process
    # any incoming messages or timer callbacks.
    rclpy.spin(gait_commander)
    
    # This part is only reached when the node is shut down (e.g., with Ctrl+C).
    # It's good practice to explicitly destroy the node and shut down rclpy.
    gait_commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Running This Code

If you were to run this Python script on a robot with ROS 2, it would create a node that immediately starts publishing "start_walking" and "stop_walking" commands every five seconds to the `/gait_command` topic. A separate C++ or Python node responsible for controlling the leg motors would subscribe to this topic. When it receives the `"start_walking"` message, it would execute its complex walking algorithm. When it receives `"stop_walking"`, it would bring the robot to a stable halt.

This demonstrates the power of decoupling: our AI agent doesn't need to know *how* to walk; it only needs to know that it *can* command the robot to walk. This separation of concerns is fundamental to building complex, intelligent robots.