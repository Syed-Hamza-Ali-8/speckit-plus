---
sidebar_position: 4
---

# Understanding URDF for Humanoids: The Blueprint of a Robot

We've learned how to create the mind of a robot with ROS 2 nodes, but how do we describe its body? This is where the **Unified Robot Description Format (URDF)** comes in. URDF is an XML format used in ROS to describe all the physical aspects of a robot. Think of it as the robot's blueprint.

A URDF file defines the robot's **links** (its body parts), its **joints** (which connect the links and define how they can move), its visual appearance, and its collision properties. This information is crucial for a wide range of robotics tasks, from simulation and visualization to motion planning and collision detection.

## The Core Components of a URDF File

A URDF file is structured around two main components: **links** and **joints**. Together, they form a tree-like structure called a **kinematic chain**, with a single "root" link (usually the torso or base of the robot) from which all other links branch out.

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
*   **`origin`**: The pose of the joint relative to the parent link, defining its position and orientation.
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

## Example: A Simple Humanoid URDF

Let's look at a more complete (though still very simple) URDF for a humanoid robot with a torso, a head, and one arm.

```xml
<robot name="simple_humanoid">

  <!-- Torso Link -->
  <link name="torso">
    <visual>
      <geometry><box size="0.5 0.3 1.0" /></geometry>
      <material name="blue"><color rgba="0.0 0.0 1.0 1.0" /></material>
    </visual>
  </link>

  <!-- Head Link -->
  <link name="head">
    <visual>
      <geometry><sphere radius="0.2" /></geometry>
      <material name="white"><color rgba="1.0 1.0 1.0 1.0" /></material>
    </visual>
  </link>

  <!-- Neck Joint -->
  <joint name="neck" type="revolute">
    <parent link="torso" />
    <child link="head" />
    <origin xyz="0.0 0.0 0.6" rpy="0.0 0.0 0.0" />
    <axis xyz="0.0 0.0 1.0" />
    <limit lower="-0.785" upper="0.785" />
  </joint>

  <!-- Upper Arm Link -->
  <link name="upper_arm">
    <visual>
      <geometry><cylinder length="0.4" radius="0.05" /></geometry>
      <material name="red"><color rgba="1.0 0.0 0.0 1.0" /></material>
    </visual>
  </link>

  <!-- Shoulder Joint -->
  <joint name="shoulder" type="revolute">
    <parent link="torso" />
    <child link="upper_arm" />
    <origin xyz="0.0 0.2 0.4" rpy="0.0 0.0 0.0" />
    <axis xyz="0.0 1.0 0.0" />
    <limit lower="-1.57" upper="1.57" />
  </joint>

</robot>
```

## Simplifying URDFs with XACRO

For a complex humanoid robot with dozens of links and joints, writing a URDF file by hand can become tedious and error-prone. This is where **XACRO (XML Macros)** comes in. XACRO is a macro language that allows you to create more readable and reusable URDF files.

With XACRO, you can:

*   Define constants for commonly used values (e.g., the length of an arm).
*   Create macros for repeating structures (e.g., a macro to generate a complete arm with all its links and joints).
*   Include other XACRO files, allowing you to build a complex robot description from smaller, modular files.

Here's a taste of how you might use XACRO to define a property and use it in your URDF:

```xml
<robot name="simple_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:property name="torso_height" value="1.0" />

  <link name="torso">
    <visual>
      <geometry><box size="0.5 0.3 ${torso_height}" /></geometry>
      ...
    </visual>
  </link>

</robot>
```

We will use XACRO extensively when we start building more complex robot models.

## Visualizing Your Robot with RViz

Once you have a URDF file, you can easily visualize your robot in 3D using **RViz**. You'll need to run a few ROS 2 nodes to do this:

1.  **`robot_state_publisher`**: This node reads your URDF and publishes the state of the robot (the transforms between the different links) to the `/tf2` topic.
2.  **`joint_state_publisher_gui`**: This node provides a simple GUI with sliders that allow you to manually move the joints of your robot.
3.  **`rviz2`**: The main RViz application.

With these nodes running, you can add a "RobotModel" display in RViz, and you will see a 3D model of your robot. You can then use the sliders in the `joint_state_publisher_gui` to move the robot's joints and see the model update in real-time. This is an invaluable tool for debugging your URDF and for getting a feel for your robot's kinematics.

This concludes our first module on the fundamentals of ROS 2. In the next module, we'll dive into the world of simulation and learn how to create a digital twin of our robot using Gazebo.