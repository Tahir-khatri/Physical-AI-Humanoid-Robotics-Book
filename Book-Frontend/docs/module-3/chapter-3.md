---
id: chapter-3
title: 'Chapter 3: Nav2 & Bipedal Path Planning'
sidebar_label: 'Nav2 & Bipedal Path Planning'
---

# Chapter 3: Nav2 & Bipedal Path Planning

We have equipped our robot with a photorealistic world and a hardware-accelerated perception system. It can see and understand its environment. The final step in creating a truly autonomous agent is to give it the ability to **navigate**—to move from point A to point B intelligently and without collisions. For this, we turn to **Navigation 2 (Nav2)**, the standard navigation stack in ROS 2.

However, Nav2 was primarily designed for wheeled robots that are holonomic or differential-drive. A humanoid is a fundamentally different kind of machine. It's a bipedal, dynamically-balanced system with complex kinematic constraints. It cannot turn in place, it has a minimum turning radius, and its movements must be carefully controlled to maintain balance. Simply launching Nav2 with its default configuration would result in jerky, unstable, and likely falling behavior.

This chapter provides a comprehensive breakdown of how to configure and tune the Nav2 stack specifically for the unique challenges of bipedal humanoid movement. We will explore the key plugins and parameters that must be adapted to achieve stable and effective humanoid navigation.

## A Deeper Look at the Nav2 Architecture

Nav2 is not a single program but a collection of specialized, lifecycle-managed servers that work together. Understanding this architecture is key to effective configuration.

-   **Lifecycle Manager**: The top-level node responsible for starting, stopping, and managing the state of all other Nav2 servers.
-   **Map Server**: Responsible for loading the static map from the SLAM system and serving it to the rest of Nav2.
-   **BT Navigator Server**: The high-level executive. It receives a goal, loads a specific Behavior Tree (BT), and ticks through the BT to orchestrate the entire navigation process.
-   **Planner Server**: Hosts global path planning plugins. Its job is to compute a long-range, kinematically-feasible path from the robot's current position to the goal, avoiding obstacles in the global costmap. A common default is `SmacPlannerHybrid`.
-   **Controller Server**: Hosts local trajectory planning plugins. Its job is to compute valid velocity commands (`cmd_vel`) that follow the global plan while avoiding immediate obstacles in the local costmap. This is the most critical component to tune for a humanoid. Common controllers include `DWB` (Dynamic Window Approach) and `TEB` (Timed Elastic Band).
-   **Recovery Server**: Hosts recovery behavior plugins, which are triggered by the BT Navigator when the robot is stuck.

## The Core Challenge: Adapting the Controller Server

The default Nav2 controllers are designed for robots that can spin in place and change velocity instantaneously. A humanoid can do neither. Our primary task is to configure the **Controller Server** to respect our robot's physical limitations. We will focus on the `dwb_core` (the basis for DWB), as its plugin-based critic system is highly configurable.

This configuration is done entirely through a YAML file.

**File: `nav2_biped_params.yaml`**
```yaml
controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 10.0
    min_x_velocity: 0.0
    max_x_velocity: 0.5
    min_y_velocity: 0.0
    max_y_velocity: 0.0
    min_theta_velocity: -0.2
    max_theta_velocity: 0.2
    
    acc_lim_x: 0.25
    acc_lim_y: 0.0
    acc_lim_theta: 0.1
    
    decel_lim_x: -0.5
    decel_lim_y: 0.0
    decel_lim_theta: -0.2

    holonomic_robot: false
    sim_time: 2.0

    critics:
      - "RotateToGoal"
      - "Oscillation"
      - "BaseObstacle"
      - "GoalAlign"
      - "PathAlign"
      - "PathDist"
      - "GoalDist"

    # --- Parameterization of Critics ---
    PathAlign:
      scale: 32.0
      forward_point_distance: 0.1
    GoalAlign:
      scale: 24.0
    RotateToGoal:
      scale: 32.0
      slow_down_angle: 0.5
    GoalDist:
      scale: 24.0
    PathDist:
      scale: 32.0
    Oscillation:
      oscillation_reset_dist: 0.1
```

### Dissecting the Bipedal Parameters
-   **`max_x_velocity: 0.5`**: We are explicitly telling Nav2 that our robot cannot move faster than a slow walking pace (0.5 m/s).
-   **`max_y_velocity: 0.0`**: This is critical. We are forbidding any sideways (strafe) movement.
-   **`acc_lim_x: 0.25`**: We severely limit the robot's forward acceleration. A humanoid must carefully shift its center of mass to start walking.
-   **`acc_lim_theta: 0.1`**: Similarly, we limit the rotational acceleration. A biped turns by taking steps.

#### DWB Critics: The Cost Functions
The DWB controller works by generating hundreds of possible short-term trajectories and scoring them using a set of "critic" plugins. The trajectory with the best overall score is chosen. For a biped, the *weighting* (the `scale` parameter) of these critics is what guides the behavior.
-   **`PathAlign` & `GoalAlign`**: We give these critics a high scale. We want the robot to turn to face the direction of travel *before* it starts walking, rather than trying to walk and turn at the same time, which is dynamically unstable. The high scale encourages the planner to favor "turn-then-move" maneuvers.
-   **`RotateToGoal`**: This is also given a high scale to ensure the robot performs a final, careful turn to face the correct goal orientation upon arrival.

## Advanced Costmap Configuration for Bipeds

The costmaps must also be adapted. A humanoid's "swept volume" (the space it occupies while moving) is much larger than its static footprint due to arm and leg swing.
The `inflation_layer` is the most important part of this.

```yaml
# In a costmap_common_params.yaml file
inflation_layer:
  plugin: "nav2_costmap_2d::InflationLayer"
  cost_scaling_factor: 3.0
  inflation_radius: 0.8 # meters
```
Setting an `inflation_radius` of 0.8m or more ensures the global planner gives the humanoid a wide berth around obstacles.

Beyond inflation, consider the other layers:
- **`static_layer`**: This layer is built from the map provided by the SLAM system. It should be configured to subscribe to the map topic from your VSLAM node.
- **`obstacle_layer`**: This layer processes live sensor data (e.g., from a LiDAR) to add dynamic obstacles to the costmap. For a biped, it's crucial that the `observation_sources` are configured correctly to register obstacles near the robot's feet and torso.
- **`voxel_layer`**: This is a 3D version of the obstacle layer, useful if your robot needs to navigate under or over things (e.g., ducking under a table). For simple planar navigation, it can be disabled for performance.

## Behavior Trees: Customizing High-Level Logic

The default Nav2 behavior tree (BT) includes recovery behaviors like `spin`. Asking a bipedal robot to spin 360 degrees in place is a recipe for falling. We must provide a custom BT. A BT is an XML file that defines a tree of actions and conditions.

**Conceptual Custom Bipedal BT (`biped_navigate_w_backup.xml`):**
```xml
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="3" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithInitialGoal">
        <RateController hz="1.0">
          <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
        </RateController>
        <FollowPath path="{path}" controller_id="DWBLocalPlanner"/>
      </PipelineSequence>
      <Fallback name="RecoveryFallback">
        <GoalUpdated/>
        <Sequence name="RecoveryActions">
          <ClearCostmapLayer name="ClearLocalCostmap-Subtree" costmap_layer_name="obstacle_layer" />
          <BackUp backup_dist="0.3" backup_speed="0.05" />
          <Wait wait_duration="2" />
        </Sequence>
      </Fallback>
    </RecoveryNode>
  </BehaviorTree>
</root>
```
In this custom BT:
1.  We define a `PipelineSequence` that computes a global plan and then attempts to follow it.
2.  If the `FollowPath` action fails, the `RecoveryNode` takes over.
3.  The `RecoveryFallback` first checks if the goal has been updated. If not, it proceeds to our custom recovery sequence.
4.  Our custom sequence clears the local costmap of dynamic obstacles, carefully backs up 0.3 meters at a very slow speed, and then waits for two seconds before retrying the main navigation pipeline. The dangerous `Spin` action has been completely removed.

## Tuning Philosophy: Safety, Stability, and Efficiency

Tuning Nav2 for a biped is an iterative process that balances three competing goals:
1.  **Safety**: The robot must not collide with obstacles or people. This is achieved with a large `inflation_radius` and a high weight on the `BaseObstacle` critic. The trade-off is that the robot may be unable to find paths through narrow spaces.
2.  **Stability**: The robot must not fall over. This is achieved by setting very conservative velocity and acceleration limits. The trade-off is slow navigation speed.
3.  **Efficiency**: The robot should reach its goal in a reasonable amount of time. This is achieved by carefully tuning the `PathAlign` and `GoalDist` critics to reward progress. The trade-off is that a high incentive for progress can sometimes lead to less stable maneuvers if not balanced by the acceleration limits.

Your tuning process should be methodical. Start with very slow, stable parameters. Verify the robot can move in a straight line. Then, test turning. Gradually increase speeds and accelerations, always monitoring the robot's stability in simulation. Only by respecting the unique physical nature of the bipedal form can you bridge the gap between high-level path planning and low-level dynamic control, enabling the AI brain to guide the physical body intelligently and safely through a complex world.