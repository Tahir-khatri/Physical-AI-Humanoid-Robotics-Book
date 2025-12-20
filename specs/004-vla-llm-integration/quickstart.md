# Quickstart: Module 4 - VLA Capstone Project

**Date**: 2025-12-19
**Feature**: [Module 4: Vision-Language-Action (VLA)](spec.md)

This guide provides the necessary steps to launch and test the final capstone project, which integrates all four modules of the book into a single, autonomous, voice-controlled humanoid system.

## Prerequisites

-   A fully configured simulation environment from Modules 1, 2, and 3. This includes a simulated humanoid robot in Isaac Sim, a functional Nav2 stack, and the hardware-accelerated perception pipelines.
-   Python environment with libraries for OpenAI Whisper and the chosen LLM API (e.g., `openai`, `google-generativeai`).
-   API keys for Whisper and the LLM, configured as environment variables.
-   A working microphone configured on the host machine.
-   All ROS 2 workspaces (for all modules) have been built and sourced.

## Launching the Full System

Launching the capstone project involves bringing up multiple components in the correct order. This is typically managed by a top-level ROS 2 launch file.

1.  **Launch the Simulation**:
    -   Start NVIDIA Isaac Sim and load the environment containing the humanoid robot and the objects for manipulation (e.g., the red block and green table).
    -   Ensure the ROS 2 Bridge is active.

2.  **Launch Core Robotics Modules**:
    -   In a sourced terminal, launch the perception and navigation stacks from Module 3.
        ```bash
        ros2 launch my_robot_bringup module_3_main.launch.py
        ```
        This should start the Isaac ROS VSLAM pipeline and the bipedal-tuned Nav2 stack.

3.  **Launch the VLA Cognitive Layer**:
    -   In a new, sourced terminal, launch the VLA components from Module 4.
        ```bash
        # This launch file would start both the Whisper bridge and the LLM orchestrator
        ros2 launch my_robot_vla vla_main.launch.py
        ```

## Testing the Capstone Project

Once all systems are running, you can test the full voice-to-action pipeline.

**Checklist:**

-   [ ] **1. System Sanity Check**:
    -   [ ] Verify that the `whisper_ros_bridge` node is running and subscribed to the microphone.
    -   [ ] Verify that the `llm_orchestrator` node is running and subscribed to the `/voice_transcription` topic.
    -   [ ] In RViz2, confirm that the robot's pose is correctly localized in the map and that the costmaps are active.

-   [ ] **2. Simple Voice Command Test**:
    -   [ ] Speak a simple navigation command: "Robot, go to the green table."
    -   [ ] **Expected Behavior**:
        -   The `/voice_transcription` topic should publish the transcribed text.
        -   The `llm_orchestrator` should log that it is creating a plan.
        -   A navigation goal should be sent to the Nav2 action server.
        -   The robot should navigate to the green table.

-   [ ] **3. Full Fetch-and-Place Test (SC-003)**:
    -   [ ] Ensure the target object (e.g., a red block) is present in the simulated world.
    -   [ ] Speak the full command: "Robot, please find the red block and place it on the green table."
    -   [ ] **Expected Behavior**:
        1.  Whisper transcribes the command.
        2.  The LLM orchestrator receives the text, queries the LLM, and receives a structured plan (e.g., `[navigate_to, find_object, pick_up, navigate_to, place_at]`).
        3.  The orchestrator begins executing the plan, first sending a navigation goal for the table where the block is located.
        4.  The robot navigates using Nav2.
        5.  The robot uses its vision system to find the "red block".
        6.  The robot executes a manipulation routine to pick up the block.
        7.  The robot navigates to the "green table".
        8.  The robot executes a routine to place the block.
    -   [ ] Repeat this test 10 times and record the number of successful end-to-end completions to verify the >75% success rate.
