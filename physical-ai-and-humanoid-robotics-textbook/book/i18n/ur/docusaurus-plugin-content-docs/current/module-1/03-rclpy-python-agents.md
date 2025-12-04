---
sidebar_position: 3
---

# Bridging Python Agents to ROS 2 with `rclpy`

So far, we've learned about the high-level concepts of ROS 2. Now it's time to get practical. How do we actually write ROS 2 nodes in Python? The answer is `rclpy`—the official Python client library for ROS 2.

`rclpy` allows you to access all the core ROS 2 functionalities directly from your Python code. This is incredibly powerful because it means you can leverage the vast ecosystem of Python libraries for AI, machine learning, and data science, and seamlessly integrate them into a robust robotics system.

## Setting Up Your First `rclpy` Node

Let's start with the "Hello, World!" of `rclpy`. The following code creates a simple node that prints a message.

```python
import rclpy
from rclpy.node import Node

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello, World! %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Let's break down what's happening here:

1.  **`rclpy.init()`**: This initializes the ROS 2 client library.
2.  **`Node` class**: We create our own class that inherits from `rclpy.node.Node`. This is the standard way to create a node.
3.  **`create_publisher()`**: In the node's constructor, we create a publisher that will send messages of type `String` to the `chatter` topic.
4.  **`create_timer()`**: We create a timer that will call the `timer_callback` function every 0.5 seconds.
5.  **`timer_callback()`**: This function creates a `String` message, publishes it to the `chatter` topic, and logs a message to the console.
6.  **`rclpy.spin()`**: This is the main event loop for the node. It keeps the node running and responsive to events like timer callbacks and incoming messages.

## Connecting a Python AI Agent to ROS 2

Now, let's imagine you have a sophisticated AI agent written in Python. Maybe it's a deep learning model that detects objects in images, or a natural language processing model that understands voice commands. How do you connect this agent to your robot?

The key is to wrap your AI agent in a ROS 2 node. This node will act as a bridge between the world of ROS and your Python code. Here's a conceptual example of how you might integrate an image classification model:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
# Assume you have a pre-trained image classification model
from my_ai_library import classify_image

class ImageClassifierNode(Node):
    def __init__(self):
        super().__init__('image_classifier')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.publisher = self.create_publisher(String, '/detected_object', 10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # Convert the ROS Image message to an OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Use your AI model to classify the image
        object_name = classify_image(cv_image)

        # Publish the result to a topic
        result_msg = String()
        result_msg.data = object_name
        self.publisher.publish(result_msg)
        self.get_logger().info('Detected object: "%s"' % object_name)

def main(args=None):
    rclpy.init(args=args)
    image_classifier = ImageClassifierNode()
    rclpy.spin(image_classifier)
    image_classifier.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

In this example:

1.  The `ImageClassifierNode` subscribes to the `/camera/image_raw` topic, which is where the robot's camera is publishing images.
2.  The `image_callback` function is called every time a new image is received.
3.  We use the `cv_bridge` library to convert the ROS `Image` message into an OpenCV image, which is a common format for computer vision libraries in Python.
4.  We then pass the image to our `classify_image` function, which represents our AI model.
5.  Finally, we publish the result of the classification to a `/detected_object` topic. Other nodes in the system can then subscribe to this topic to know what the robot is seeing.

This pattern of subscribing to sensor data, processing it with a Python library, and publishing the result is a fundamental workflow in Physical AI. `rclpy` makes this integration straightforward, allowing you to combine the power of ROS 2 with the rich ecosystem of Python AI libraries.

In the next chapter, we'll discuss the `URDF` format, which is how we describe the physical structure of a robot to ROS.
