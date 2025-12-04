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

Nodes are typically written in Python or C++. Here is a conceptual example of what a simple "hello world" node might look like in Python using the `rclpy` library:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        self.get_logger().info('Hello from my ROS 2 node!')

def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)
    # Create an instance of our node
    node = MyNode()
    # "Spin" the node, which means it will continue to run and process callbacks
    rclpy.spin(node)
    # Clean up and shutdown the node
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

## Messages: The Language of ROS

So, what exactly is being sent over these topics? The data is packaged in the form of **messages**. A message is a simple data structure, similar to a `struct` in C++ or a class in Python. It defines the fields and types of data that are being transmitted.

ROS 2 comes with a rich set of standard message types for common robotics data, such as:

*   `std_msgs/String`: For sending simple text messages.
*   `sensor_msgs/Image`: For sending camera images.
*   `sensor_msgs/LaserScan`: For sending data from a LIDAR sensor.
*   `geometry_msgs/Twist`: For sending velocity commands to a robot (linear and angular speed).

You can also define your own custom messages to suit the specific needs of your application. A message definition looks like this:

```
# A custom message for representing a person's age and name
string name
int32 age
```

This `.msg` file would be compiled into Python and C++ code that you can then use in your nodes.

## Services: The Question-and-Answer Session

Topics are great for continuous data streams, but sometimes you need a different kind of communication: a direct request and response. This is what **services** are for.

A service is a two-way communication mechanism. It consists of a **request** and a **response**. One node acts as the **service server**, which advertises a service and waits for requests. Another node acts as the **service client**, which sends a request to the server and waits for a response.

Unlike topics, services are a one-to-one communication method. They are perfect for tasks that have a clear start and end, and where you need a confirmation that the task was completed. For example:

*   A `camera_control` node could offer a `/take_picture` service. A client could call this service, and the server would respond with the image data once the picture has been taken.
*   A `navigation` node could offer a `/move_to` service. A client could send a goal location in the request, and the server would respond with a `success` or `failure` message after attempting to move the robot.

![ROS 2 Request/Response Model](/img/ros2-request-response.svg)

## Code Example: A Simple Publisher and Subscriber

Let's make this more concrete with a full code example of a publisher and a subscriber.

### The Publisher Node

This node will publish a "hello world" message with a counter to the `/chatter` topic every second.

```python
# a_simple_publisher.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### The Subscriber Node

This node will subscribe to the `/chatter` topic and print any messages it receives.

```python
# a_simple_subscriber.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    simple_subscriber = SimpleSubscriber()
    rclpy.spin(simple_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

To run this example, you would first need to build your ROS 2 package, then run each node in a separate terminal:

```bash
# Terminal 1
ros2 run your_package_name a_simple_publisher

# Terminal 2
ros2 run your_package_name a_simple_subscriber
```

You would then see the publisher's messages being printed in the subscriber's terminal. This simple example demonstrates the core power of ROS 2's communication system. By combining these building blocks, you can create sophisticated robotic applications. In the next chapter, we'll dive deeper into `rclpy` and explore more advanced features of the ROS 2 Python client library.