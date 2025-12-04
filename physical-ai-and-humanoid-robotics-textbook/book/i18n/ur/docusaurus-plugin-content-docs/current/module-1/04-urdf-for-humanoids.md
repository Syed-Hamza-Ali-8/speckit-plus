---
sidebar_position: 4
---

# Understanding URDF for Humanoids: The Blueprint of a Robot

We've learned how to create the mind of a robot with ROS 2 nodes, but how do we describe its body? This is where the **Unified Robot Description Format (URDF)** comes in. URDF is an XML format used in ROS to describe all the physical aspects of a robot. Think of it as the robot's blueprint.

A URDF file defines the robot's **links** (its body parts), its **joints** (which connect the links and define how they can move), and its visual appearance. This information is crucial for a wide range of robotics tasks, from simulation and visualization to motion planning and collision detection.

## The Core Components of a URDF File

A URDF file is structured around two main components: links and joints.

### Links: The Bones of the Robot

A **link** represents a rigid part of the robot's body. For a humanoid robot, you would have links for the torso, the head, the upper arms, the forearms, the hands, and so on. Each link has a name and can have several properties, including:

*   **`<visual>`**: This describes the visual appearance of the link, such as its shape (e.g., a box, a cylinder, or a 3D mesh), its material, and its color. This is what you see in a visualization tool like RViz.
*   **`<collision>`**: This defines the collision geometry of the link, which is used by physics engines for collision detection. The collision geometry is often a simplified version of the visual geometry to save computation time.
*   **`<inertial>`**: This specifies the inertial properties of the link, such as its mass and moment of inertia. This is essential for accurate physics simulation.

Here's an example of a simple link definition for a robot's torso:

```xml
<link name="torso">
  <visual>
    <geometry>
      <box size="0.5 0.3 1.0" />
    </geometry>
    <material name="blue">
      <color rgba="0.0 0.0 1.0 1.0" />
    </material>
  </visual>
  <collision>
    <geometry>
      <box size="0.5 0.3 1.0" />
    </geometry>
  </collision>
  <inertial>
    <mass value="10" />
    <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0" />
  </inertial>
</link>
```

### Joints: The Muscles of the Robot

A **joint** connects two links and defines how they can move relative to each other. For a humanoid robot, you would have joints for the neck, the shoulders, the elbows, the hips, the knees, etc.

There are several types of joints, but the most common are:

*   **`revolute`**: A hinge joint that rotates around a single axis, like an elbow.
*   **`continuous`**: Similar to a revolute joint, but with no rotation limits. This is often used for wheels.
*   **`prismatic`**: A sliding joint that moves along a single axis.
*   **`fixed`**: A joint that doesn't allow any motion between the two links. This is useful for rigidly connecting parts of the robot.

Each joint definition includes:

*   **`parent` and `child`**: The names of the two links that the joint connects.
*   **`origin`**: The pose of the joint relative to the parent link.
*   **`axis`**: The axis of rotation (for revolute joints) or translation (for prismatic joints).
*   **`limits`**: The upper and lower limits of the joint's motion, as well as the maximum velocity and effort.

Here's an example of a revolute joint that connects a `torso` link to an `upper_arm` link:

```xml
<joint name="shoulder" type="revolute">
  <parent link="torso" />
  <child link="upper_arm" />
  <origin xyz="0.0 0.2 0.4" rpy="0.0 0.0 0.0" />
  <axis xyz="0.0 1.0 0.0" />
  <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0" />
</joint>
```

## Why URDF is Essential for Humanoid Robots

For a complex robot like a humanoid, a URDF file is absolutely essential. It provides a standardized way to represent the robot's kinematic chain—the series of links and joints that make up its body. This information is used by many core ROS packages:

*   **`robot_state_publisher`**: This node reads the URDF file and publishes the robot's state (the position and orientation of each link) to the `/tf` topic.
*   **`rviz`**: The 3D visualization tool for ROS, uses the URDF and the `/tf` data to display a 3D model of the robot.
*   **`gazebo`**: The most popular simulator for ROS, uses the URDF to create a simulated version of the robot that can interact with a virtual world.
*   **`moveit`**: The motion planning framework for ROS, uses the URDF to plan collision-free paths for the robot's arms and legs.

By creating a detailed URDF file for your humanoid robot, you unlock the power of the entire ROS ecosystem. This concludes our first module on the fundamentals of ROS 2. In the next module, we'll dive into the world of simulation and learn how to create a digital twin of our robot.
