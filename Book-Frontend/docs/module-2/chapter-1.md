---
id: chapter-1
title: 'Chapter 1: Advanced Gazebo Physics'
sidebar_label: 'Advanced Gazebo Physics'
---

# Chapter 1: Advanced Gazebo Physics

Welcome to the deep end of simulation. In Module 1, we defined our robot's body; now, we must define the world it lives in and the very laws of physics that govern it. A digital twin is only as valuable as its fidelity to the real world. For a humanoid robot, whose entire existence is a constant battle against gravity, a finely-tuned physics simulation is not a luxury—it is the absolute prerequisite for any meaningful testing of walking, balancing, or manipulation algorithms.

This chapter provides an exhaustive guide to configuring the physics engines within Gazebo, the de facto simulation standard in the ROS ecosystem. We will move beyond default settings to explore the critical parameters that govern stability, contact dynamics, and friction. Our focus will be on the **Open Dynamics Engine (ODE)**, Gazebo's long-standing default, known for its stability with complex, articulated systems like humanoids.

## The Physics Engine: A Tale of Two Solvers

Gazebo is a simulator, but the core calculations of how objects move and interact are handled by a separate physics engine. Gazebo primarily supports two major open-source engines:

1.  **ODE (Open Dynamics Engine)**: The traditional default. It is a mature, robust engine particularly well-suited for robotics. Its "world" solver is an iterative, constraint-based method that is effective at preventing the joint "explosions" and instabilities that can plague complex models.
2.  **Bullet**: A more modern engine known for its high performance and advanced features, including GPU acceleration and soft-body simulation. It's widely used in gaming and is an excellent choice, though its configuration can sometimes be less intuitive for robotics newcomers compared to ODE.

While both are capable, we will focus on ODE because its parameters are explicitly designed to solve the kinds of problems encountered in legged robotics. The principles, however, are transferable.

Physics properties in Gazebo are defined within the `<physics>` tag in your `.world` file.

```xml
<sdf version='1.7'>
  <world name='default'>
    <!-- ... lights, models, etc. ... -->
    <physics type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <ode>
        <solver>
          <type>world</type>
          <iters>50</iters>
          <sor>1.3</sor>
          <use_dynamic_moi_rescaling>false</use_dynamic_moi_rescaling>
        </solver>
        <constraints>
          <cfm>0.0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>
  </world>
</sdf>
```

-   **`max_step_size`**: The duration of a single physics update step in simulation seconds. A smaller value (`0.001` corresponding to 1ms) increases accuracy but requires more computation. For humanoid balancing, a 1ms step size is a standard starting point.
-   **`real_time_update_rate`**: How many simulation steps to attempt per second. `max_step_size * real_time_update_rate` should equal `real_time_factor`.
-   **`iters`**: The number of iterations the solver runs for each step. More iterations lead to more accurate constraint resolution (i.e., less joint slop or object penetration) at the cost of performance. For humanoids, 50-100 iterations is common.
-   **`sor`**: The Successive Over-Relaxation parameter. It can help the solver converge faster. A value between 1.0 and 1.4 is typical.
-   **`cfm` (Constraint Force Mixing)**: A crucial parameter for stability. It introduces a small amount of "softness" into every constraint. A value of `0.0` means constraints are hard. If your robot model "explodes" or jitters, introducing a tiny `cfm` value (e.g., `1e-5`) can dramatically improve stability by allowing for minuscule, unnoticeable constraint violations. It's like adding a tiny bit of springiness to every joint.
-   **`erp` (Error Reduction Parameter)**: The proportion of joint error that is corrected in each time step. A value of `0.2` (20%) is a good default. If your joints seem too "mushy", you can increase this to `0.8`, but be aware that higher values can introduce instability.

## The Ground You Walk On: Contact and Friction

Arguably the most important physics interaction for a humanoid is the contact between its feet and the ground. This is governed by friction and contact parameters, defined in the `<surface>` tag of a link's `<collision>` element.

You must define these properties for **both** the ground plane and the robot's feet. Gazebo combines the parameters of the two colliding surfaces to determine the final behavior.

```xml
<!-- In the ground plane model's SDF file -->
<collision name='ground_collision'>
  <geometry>
    <plane><size>100 100</size></plane>
  </geometry>
  <surface>
    <friction>
      <ode>
        <mu>1.0</mu>
        <mu2>1.0</mu2>
        <fdir1>0 0 0</fdir1>
        <slip1>0.0</slip1>
        <slip2>0.0</slip2>
      </ode>
    </friction>
    <contact>
      <ode>
        <soft_cfm>0</soft_cfm>
        <soft_erp>0.2</soft_erp>
        <kp>1e+13</kp>
        <kd>1</kd>
        <max_vel>0.01</max_vel>
        <min_depth>0.0</min_depth>
      </ode>
    </contact>
  </surface>
</collision>

<!-- In the robot's URDF, for the foot link -->
<collision>
  <geometry>
    <box size="0.2 0.1 0.02" />
  </geometry>
  <surface>
    <friction>
      <ode>
        <mu>1.0</mu>
        <mu2>1.0</mu2>
      </ode>
    </friction>
    <!-- ... contact properties ... -->
  </surface>
</collision>
```

### Deconstructing Friction

Friction in ODE is modeled using a friction pyramid, which approximates the true friction cone.

-   **`<mu>` (mu)**: The primary coefficient of friction. This is the Coulomb friction coefficient for the "first" friction direction. A value of `1.0` is a good starting point for a high-traction surface like a robot foot on a textured floor.
-   **`<mu2>` (mu2)**: The coefficient of friction for the "second" friction direction, which is orthogonal to the first. For an isotropic material (friction is the same in all directions), `mu` and `mu2` should be equal.
-   **`<fdir1>` (fdir1)**: The primary friction direction vector. If you leave this as `0 0 0`, Gazebo will choose a direction automatically, which is usually sufficient. You would only set this for anisotropic materials, like wanting a ski to slide forward but not sideways.
-   **`<slip1>` and `<slip2>`**: These parameters introduce "slip" or "slop" into the friction model. They define how much the solver is allowed to cheat. A value of `0.0` means the friction constraint is strictly enforced (no slip). If you observe your robot's feet sticking unrealistically to the ground, introducing a very small amount of slip (e.g., `0.001`) can help.

### Deconstructing Contact Stiffness

The `<contact>` block defines how two surfaces behave when they come into contact—specifically, their hardness and bounciness. For a humanoid that needs to stand firmly, you want hard, non-bouncy contacts.

-   **`<kp>` and `<kd>`**: These are the stiffness and damping coefficients for the contact spring. `kp` defines how "hard" the surface is, while `kd` defines how much it dampens energy upon impact. For a robot foot on the ground, you want a very high stiffness (`kp=1e+13`) and a damping value (`kd=1`) that dissipates energy to prevent bouncing.
-   **`<max_vel>`**: The maximum velocity correction applied during a contact. Setting this to a small value (e.g., `0.01`) can prevent contacts from causing jittery, high-velocity corrections, improving stability.
-   **`<min_depth>`**: The minimum penetration depth allowed before the contact forces are applied. A small value (`0.0`) is standard.

## Joint Dynamics and Damping

The final piece of the stability puzzle lies in the joints themselves. In a real robot, joints are not perfectly free-spinning; they have internal friction and resistance from the motor and gearbox. Simulating this is critical. This is done by adding a `<dynamics>` tag to your joint definition in the URDF.

```xml
<joint name="right_knee_joint" type="revolute">
  <parent link="right_thigh" />
  <child link="right_shin" />
  <origin xyz="0 0 -0.4" rpy="0 0 0" />
  <axis xyz="0 1 0" />
  <limit lower="0" upper="2.5" effort="100" velocity="1.0" />
  <dynamics damping="0.7" friction="0.1" />
</joint>
```

-   **`damping`**: This simulates viscous friction—resistance that is proportional to the joint's velocity. It's like moving the joint through thick oil. This is arguably the **single most important parameter** for stabilizing a humanoid. Without damping, the robot's limbs will oscillate and "buzz" uncontrollably from the tiny errors in the physics solver. A value between `0.1` and `2.0` is typical. You will need to tune this for each joint based on the mass of the connected link.
-   **`friction`**: This simulates static friction (stiction). It's the force that must be overcome to start moving the joint from a standstill. This can help joints hold their position more firmly but can also make movements look less smooth if set too high.

## A Practical Tuning Workflow

Achieving a stable humanoid simulation is an iterative process. Here is a recommended workflow:

1.  **Start with Defaults**: Load your robot model into Gazebo with default physics. It will likely fall over or explode.
2.  **Increase Solver Iterations**: In your world file, increase `<iters>` to 50 or 100. This is a quick way to gain some initial stability.
3.  **Tune Joint Damping**: Add `<dynamics damping="0.5" />` to **all** of your `revolute` and `continuous` joints. This should be your highest priority. Your robot may now collapse "softly" instead of exploding. Adjust the damping values up or down for different joints until they seem to move with a realistic amount of resistance.
4.  **Configure Ground Contact**: Set high friction (`mu=1.0`, `mu2=1.0`) and hard contact parameters (`kp`, `kd`) on both the ground plane and the robot's feet surfaces.
5.  **Introduce `cfm` and `erp`**: If you still see jittering or instability, try adding a very small global `cfm` (e.g., `1e-5`) in the world physics settings. Ensure `erp` is around `0.2`.
6.  **Iterate**: Make small, one-at-a-time changes and observe the effect. Tuning is a process of balancing realism and stability. The goal is not a perfect physical replica, but a simulation that is stable and predictable enough to serve as a reliable testbed for your AI.

By mastering these advanced physics parameters, you transform Gazebo from a simple visualizer into a powerful engineering tool, capable of creating a digital twin that behaves and responds with a high degree of fidelity to its real-world counterpart.
