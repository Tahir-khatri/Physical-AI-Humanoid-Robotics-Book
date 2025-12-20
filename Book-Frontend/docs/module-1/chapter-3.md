---
id: chapter-3
title: 'Chapter 3: Humanoid Anatomy (URDF)'
sidebar_label: 'Humanoid Anatomy (URDF)'
---

# Chapter 3: Humanoid Anatomy (URDF)

We have established our robot's nervous system with ROS 2 and built the Python bridge to send commands. But what, exactly, are we commanding? A humanoid robot is a complex mechanical assembly of rigid bodies and articulating joints. For any part of the ROS 2 ecosystem to work with the robot's physical structure, it needs a standardized description of that structure. This is the role of the **Unified Robot Description Format (URDF)**.

URDF is an XML-based file format used in ROS to describe all the physical elements of a robot. It is not just for visualization; it's a true digital twin of the robot's kinematics and dynamics. It tells ROS tools:

-   What are the parts of the robot? (**Links**)
-   How are these parts connected? (**Joints**)
-   How do the parts move relative to each other? (**Kinematics**)
-   What does the robot look like? (**Visual Meshes**)
-   How does it interact with physics simulators? (**Collision and Inertial Properties**)

By creating a URDF, you provide a single source of truth about the robot's physical form that can be used by visualizers (like RViz2), physics simulators (like Gazebo), and motion planning libraries (like MoveIt).

## The Core Components of URDF

A URDF file describes a robot as a tree of links and joints. There is always one special link that acts as the root of the tree, from which all other parts branch out.

### 1. Links: The Rigid Bodies

A **`<link>`** element represents a single rigid body of the robot. Think of it as a bone. A link has physical properties, and its own coordinate frame. For a humanoid, links would include the torso, the upper arm, the forearm, the hand, the thigh, the shin, and the foot.

A minimal `<link>` element just needs a name:

```xml
<link name="torso" />
```

However, a useful link has several sub-elements:

-   **`<visual>`**: This describes what the link *looks like*. You can use simple geometric shapes (box, cylinder, sphere) or, more commonly, point to an external 3D mesh file (like `.stl` or `.dae`). You can also specify its material and color.
-   **`<collision>`**: This defines the bounding shape of the link used for collision detection in a physics simulator. It is often a simplified version of the visual mesh to improve performance.
-   **`<inertial>`**: This describes the link's dynamic properties: its mass, center of mass, and moment of inertia tensor. These are critical for realistic physics simulation.

Here's a more complete example for a robot's torso link:

```xml
<link name="torso">
  <visual>
    <geometry>
      <!-- A simple box shape for the torso -->
      <box size="0.3 0.5 0.2" />
    </geometry>
    <origin xyz="0 0 0.25" rpy="0 0 0" />
    <material name="blue">
      <color rgba="0.2 0.4 0.8 1.0" />
    </material>
  </visual>
  <collision>
    <geometry>
      <box size="0.3 0.5 0.2" />
    </geometry>
    <origin xyz="0 0 0.25" rpy="0 0 0" />
  </collision>
  <inertial>
    <mass value="10.0" /> <!-- Mass in kilograms -->
    <inertia ixx="1.0" ixy="0.0" ixz="0.0"
             iyy="1.0" iyz="0.0"
             izz="1.0" />
    <origin xyz="0 0 0.25" rpy="0 0 0" />
  </inertial>
</link>
```
**Note on `<origin>`**: The origin tag is crucial. It defines a new coordinate frame relative to the link's own frame. This is used to position the visual, collision, and inertial properties, which often do not share the same center.

### 2. Joints: The Connections

A **`<joint>`** element describes the kinematic and dynamic properties of the connection between two links. It defines how one link (the `child`) moves relative to another link (the `parent`). This parent-child relationship is what forms the tree structure of the robot.

A joint requires a name and a type.

```xml
<joint name="right_shoulder_pan_joint" type="revolute">
  <!-- ... joint properties ... -->
</joint>
```

Key elements of a joint:

-   **`<parent link="..."/>`**: The name of the existing link that this joint connects from.
-   **`<child link="..."/>`**: The name of the link that this joint connects to. This `child` link is defined "hanging off" the `parent` link via this joint.
-   **`<origin xyz="..." rpy="..." />`**: This is one of the most important tags. It specifies the transform from the parent link's coordinate frame to the child link's coordinate frame **when the joint is at its zero position**. It defines the static position and orientation of the joint.
-   **`<axis xyz="..." />`**: For any joint that moves, this defines the axis of rotation or translation. The vector is specified in the joint's coordinate frame. For example, `xyz="0 0 1"` means the joint rotates around its Z-axis.
-   **`<limit lower="..." upper="..." effort="..." velocity="..." />`**: This defines the joint's range of motion. For a `revolute` joint, `lower` and `upper` are specified in radians. `effort` is the maximum force or torque the joint can apply, and `velocity` is its maximum speed.

#### Common Joint Types

-   **`revolute`**: A hinge joint that rotates around a single axis (e.g., an elbow). It has a limited range of motion.
-   **`continuous`**: A joint that rotates continuously around an axis without limits (e.g., a wheel).
-   **`prismatic`**: A sliding joint that moves along a single axis (e.g., a piston). It has a limited range.
-   **`fixed`**: A rigid connection that does not allow any motion. This is very useful for connecting multiple static parts together, like welding a sensor bracket onto a link.
-   **`floating`**: Allows motion in all 6 degrees of freedom. Typically used to connect the root link of a mobile robot to the world.
-   **`planar`**: Allows motion in a 2D plane.

## Building a Simple Humanoid Arm

Let's put this together to model a simple robotic arm with a torso, an upper arm, and a forearm.

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid_arm">

  <!-- The Base Link of the Robot (e.g., the torso) -->
  <link name="torso">
    <visual>
      <geometry><box size="0.1 0.5 0.5" /></geometry>
    </visual>
  </link>

  <!-- ******************** Right Arm ******************** -->

  <!-- Upper Arm Link -->
  <link name="right_upper_arm">
    <visual>
      <geometry><cylinder length="0.4" radius="0.05" /></geometry>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <material name="gray"><color rgba="0.7 0.7 0.7 1.0" /></material>
    </visual>
  </link>

  <!-- Shoulder Joint -->
  <joint name="right_shoulder_joint" type="revolute">
    <parent link="torso" />
    <child link="right_upper_arm" />
    <!-- Position the shoulder joint relative to the torso's center -->
    <origin xyz="0 -0.3 0.2" rpy="1.5707 0 0" />
    <!-- The arm rotates around the Y-axis of the joint's frame -->
    <axis xyz="0 1 0" />
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0" />
  </joint>

  <!-- Forearm Link -->
  <link name="right_forearm">
    <visual>
      <geometry><cylinder length="0.35" radius="0.04" /></geometry>
      <origin xyz="0 0 -0.175" rpy="0 0 0"/>
      <material name="silver"><color rgba="0.9 0.9 0.9 1.0" /></material>
    </visual>
  </link>

  <!-- Elbow Joint -->
  <joint name="right_elbow_joint" type="revolute">
    <parent link="right_upper_arm" />
    <child link="right_forearm" />
    <!-- Position the elbow joint at the end of the upper arm -->
    <origin xyz="0 0 -0.4" rpy="0 0 0" />
    <axis xyz="0 1 0" />
    <limit lower="0" upper="2.35" effort="100" velocity="1.0" />
  </joint>

</robot>
```

In this example, we build the kinematic chain: `torso` -> `right_shoulder_joint` -> `right_upper_arm` -> `right_elbow_joint` -> `right_forearm`. Each `joint`'s origin tag carefully places the child link relative to its parent.

## Visualizing the URDF

The true power of URDF becomes apparent when you visualize it. ROS 2 provides a powerful 3D visualization tool called **RViz2**. To view a URDF, you typically need to do two things:

1.  **Publish the Robot Description**: A special node needs to read your URDF file and publish its content to a topic, usually named `/robot_description`.
2.  **Publish Joint States**: Another node, the `robot_state_publisher`, subscribes to `/robot_description` and also to a topic of joint positions (usually `/joint_states`). It uses this information to calculate the 3D position of every link in the robot (a process called forward kinematics) and publishes these transforms for RViz2 to use.

A simple launch file can automate this process, allowing you to load your URDF and see it in RViz2 with a single command. You can even get a GUI with sliders to move the joints and see your robot model articulate in real-time.

## Conclusion: The Body Blueprint

You have now completed the final foundational piece of our robotics module. You've created the nervous system (ROS 2), built the bridge for the mind to command the body (`rclpy`), and now you have the blueprint for the body itself (URDF).

With these three components, the entire ROS 2 ecosystem opens up to you. You can now take this URDF model and use it in a physics simulator, you can use it with motion planning libraries to generate complex trajectories, and you can use it to build sophisticated control and AI applications.

You have completed Module 1. You have the fundamental knowledge to begin building truly intelligent, physically embodied AI. The subsequent modules in this book will build upon this foundation, adding perception, navigation, and more advanced AI behaviors.
