---
sidebar_position: 3
---

# Bridging Python Agents to ROS 2 with `rclpy`

So far, we've learned about the high-level concepts of ROS 2. Now it's time to get practical. How do we actually write ROS 2 nodes in Python? The answer is `rclpy`—the official Python client library for ROS 2.

`rclpy` allows you to access all the core ROS 2 functionalities directly from your Python code. This is incredibly powerful because it means you can leverage the vast ecosystem of Python libraries for AI, machine learning, and data science, and seamlessly integrate them into a robust robotics system.

## Anatomy of a ROS 2 Python Package

Before we dive into the code, let's understand how a ROS 2 Python package is structured. A typical package might look like this:

```
my_python_package/
├── package.xml
├── setup.py
├── setup.cfg
└── my_python_package/
    ├── __init__.py
    ├── my_node.py
    └── another_node.py
```

*   `package.xml`: This file contains meta-information about the package, such as its name, version, and dependencies.
*   `setup.py`: This is a standard Python setup script that tells ROS 2 how to build and install your package, and where to find your executable nodes.
*   `my_python_package/`: This is the directory that contains your Python source code.

## Setting Up Your First `rclpy` Node

Let's expand on the "Hello, World!" example from the previous chapter. The following code creates a simple node that publishes a message every second.

```python
# my_publisher_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyPublisherNode(Node):
    def __init__(self):
        super().__init__('my_publisher_node')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.i = 0
        self.get_logger().info('Publisher node has been started.')

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello from rclpy: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    my_publisher_node = MyPublisherNode()
    
    # "Spinning" the node means keeping it running and processing callbacks.
    # Callbacks can be from timers, subscriptions, service requests, etc.
    rclpy.spin(my_publisher_node)
    
    # This part of the code will only be reached when rclpy.spin() is interrupted
    # (e.g., by pressing Ctrl+C in the terminal).
    my_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Let's break down the key `rclpy` components:

1.  **`rclpy.init()`**: This must be called before any other `rclpy` functions. It initializes the ROS 2 communication system.
2.  **`rclpy.node.Node`**: We create our own class that inherits from `Node`. This is the standard way to create a node. The `super().__init__('node_name')` call is essential to initialize the node with a unique name in the ROS graph.
3.  **`create_publisher()`**: This is a method of the `Node` class that creates a publisher. You specify the message type (`String`), the topic name (`chatter`), and the "queue size" (10), which is a quality-of-service setting that we'll discuss later.
4.  **`create_timer()`**: This creates a timer that will call the `timer_callback` function every 1.0 second. This is a common way to create a node that performs a periodic task.
5.  **`get_logger()`**: This is how you log messages in ROS 2. Using the node's logger will automatically include the node name and timestamp in the log output, which is very useful for debugging.
6.  **`rclpy.spin()`**: This is the main event loop for the node. It's a blocking call that waits for events (like a timer tick or an incoming message) and executes the appropriate callback function. It will continue to run until the node is shut down.

## Connecting a Python AI Agent to ROS 2

Now, let's imagine you have a sophisticated AI agent written in Python. Maybe it's a deep learning model that detects objects in images, or a natural language processing model that understands voice commands. How do you connect this agent to your robot?

The key is to wrap your AI agent in a ROS 2 node. This node will act as a bridge between the world of ROS and your Python code. Here's a conceptual example of how you might integrate an image classification model:

```python
# image_classifier_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge, CvBridgeError
# Assume you have a pre-trained image classification model in a separate file
from my_ai_library import classify_image

class ImageClassifierNode(Node):
    def __init__(self):
        super().__init__('image_classifier_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.publisher_ = self.create_publisher(String, '/detected_object', 10)
        self.bridge = CvBridge()
        self.get_logger().info('Image classifier node has been started.')

    def image_callback(self, msg):
        self.get_logger().info('Received an image.')
        try:
            # Convert the ROS Image message to an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'Could not convert image: {e}')
            return

        # Use your AI model to classify the image
        object_name = classify_image(cv_image)

        # Publish the result to a topic
        if object_name:
            result_msg = String()
            result_msg.data = object_name
            self.publisher_.publish(result_msg)
            self.get_logger().info(f'Detected object: "{object_name}"')

def main(args=None):
    rclpy.init(args=args)
    image_classifier_node = ImageClassifierNode()
    rclpy.spin(image_classifier_node)
    image_classifier_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

In this example:

1.  The `ImageClassifierNode` subscribes to the `/camera/image_raw` topic, which is where the robot's camera is publishing images.
2.  The `image_callback` function is called every time a new image is received.
3.  We use the `cv_bridge` library to convert the ROS `Image` message into an OpenCV image, which is a common format for computer vision libraries in Python. We also include error handling in case the conversion fails.
4.  We then pass the image to our `classify_image` function, which represents our AI model.
5.  Finally, we publish the result of the classification to a `/detected_object` topic. Other nodes in the system can then subscribe to this topic to know what the robot is seeing.

This pattern of subscribing to sensor data, processing it with a Python library, and publishing the result is a fundamental workflow in Physical AI. `rclpy` makes this integration straightforward, allowing you to combine the power of ROS 2 with the rich ecosystem of Python AI libraries.

In the next chapter, we'll discuss the `URDF` format, which is how we describe the physical structure of a robot to ROS.