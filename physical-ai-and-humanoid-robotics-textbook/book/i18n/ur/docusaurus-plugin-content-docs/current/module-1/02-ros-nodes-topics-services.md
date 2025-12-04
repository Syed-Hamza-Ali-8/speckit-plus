---
sidebar_position: 2
---

# ROS 2 Nodes, Topics, and Services: The Building Blocks of a Robot's Mind

In the previous chapter, we introduced ROS 2 as the nervous system of a robot. Now, let's explore the fundamental building blocks of this system: nodes, topics, and services. These are the components you'll use to create the intricate web of communication that brings a robot to life.

## Nodes: The Workers of the ROS World

A **node** is the most basic unit of computation in ROS 2. You can think of a node as a small, specialized worker that is responsible for a single task. For example, in a mobile robot, you might have:

*   A `camera_driver` node that captures images from a camera.
*   A `path_planner` node that calculates a route from point A to point B.
*   A `motor_controller` node that sends commands to the wheels.

Each of these nodes is an independent program. This modular approach is one of the key strengths of ROS. It allows you to develop, test, and run each part of your robotics system in isolation. If the `camera_driver` crashes, the `motor_controller` can continue to function (though it might not have much to do!).

Nodes are typically written in Python or C++. Here is a conceptual example of what a simple "hello world" node might look like in Python:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        self.get_logger().info('Hello from my ROS 2 node!')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Topics: The Public Announcement System

Now that we have our workers (nodes), we need a way for them to communicate. This is where **topics** come in. A topic is a named bus that acts like a public announcement system. Nodes can publish messages to a topic, and any other node that is subscribed to that topic will receive the message.

This **publish-subscribe** model is a powerful paradigm for a few reasons:

*   **Decoupling:** The publishing node doesn't know or care who is listening. It just sends the message. The subscribing nodes don't know or care who is sending the message. They just listen for it. This decoupling makes the system highly modular.
*   **One-to-Many Communication:** A single message can be received by many subscribers at once. For example, the `camera_driver` node could publish images to an `/image_raw` topic, and both a `person_detector` node and a `video_recorder` node could subscribe to it.

A common example is a sensor data stream. An IMU (Inertial Measurement Unit) sensor might publish its orientation data to a `/imu_data` topic at a rate of 100 times per second. Any node that needs to know the robot's orientation can simply subscribe to this topic.

![ROS 2 Publish/Subscribe Model](/img/ros2-publish-subscribe.svg)

## Services: The Question-and-Answer Session

Topics are great for continuous data streams, but sometimes you need a different kind of communication: a direct request and response. This is what **services** are for.

A service is a two-way communication mechanism. It consists of a **request** and a **response**. One node acts as the **service server**, which advertises a service and waits for requests. Another node acts as the **service client**, which sends a request to the server and waits for a response.

Unlike topics, services are a one-to-one communication method. They are perfect for tasks that have a clear start and end, and where you need a confirmation that the task was completed. For example:

*   A `camera_control` node could offer a `/take_picture` service. A client could call this service, and the server would respond with the image data once the picture has been taken.
*   A `navigation` node could offer a `/move_to` service. A client could send a goal location in the request, and the server would respond with a `success` or `failure` message after attempting to move the robot.

![ROS 2 Request/Response Model](/img/ros2-request-response.svg)

## Putting It All Together: A Simple Example

Imagine a simple robot whose only job is to move forward until it detects an obstacle. Here's how you might design this system using ROS 2:

1.  **A `laser_scanner` node:** This node publishes data from a laser scanner to a `/scan` topic. The data contains information about the distance to objects in front of the robot.
2.  **A `collision_avoidance` node:** This node subscribes to the `/scan` topic. If it detects an object that is too close, it sends a command to a `/cmd_vel` topic to stop the robot. Otherwise, it sends a command to move forward.
3.  **A `robot_driver` node:** This node subscribes to the `/cmd_vel` topic and translates the received velocity commands into motor signals for the robot's wheels.
4.  **A `status_reporter` node:** This node offers a `/get_status` service. When another node calls this service, it responds with the robot's current status (e.g., "moving," "stopped," "obstacle detected").

This simple example illustrates the power of the ROS 2 communication patterns. By combining nodes, topics, and services, you can build complex robotic behaviors from simple, reusable components. In the next chapter, we'll get our hands dirty and learn how to use `rclpy`, the Python client library for ROS 2, to create our own nodes.
