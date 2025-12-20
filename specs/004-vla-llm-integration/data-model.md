# Data Model: Module 4

**Date**: 2025-12-19
**Feature**: [Module 4: Vision-Language-Action (VLA)](spec.md)

This module focuses on high-level AI and orchestration. The "data" in this case refers not to a persistent database but to the key data structures and architectural components that enable the VLA pipeline.

## Key Architectural Components & Data Structures

### Component: Voice-to-Text Bridge (`whisper_ros_bridge.py`)

-   **Description**: A Python ROS 2 node that captures audio from a microphone, sends it to the Whisper API/library for transcription, and publishes the resulting text to a ROS 2 topic.
-   **Input**: Audio stream from a system microphone.
-   **Output**:
    -   **Topic**: `/voice_transcription`
    -   **Type**: `std_msgs/msg/String`
    -   **Content**: The transcribed text from the user's speech.

### Component: Cognitive Bridge (`llm_orchestrator.py`)

-   **Description**: The central "brain" of the VLA system. This Python ROS 2 node listens for high-level commands, queries an LLM to create a plan, and then executes that plan.
-   **Input**:
    -   **Topic**: `/voice_transcription` (or another text command topic).
    -   **Type**: `std_msgs/msg/String`
-   **Internal Data Structures**:
    -   **Skill Library**: A dictionary or similar structure defining the set of "primitive" actions the robot can perform. Example: `{'navigate_to': {'params': ['location_name']}, 'find_object': ...}`. This is used both for prompting the LLM and for validating its output.
    -   **System Prompt**: A carefully engineered prompt template that provides the LLM with its goal, constraints, and the available Skill Library.
-   **Output**:
    -   Calls to various ROS 2 Action Servers (e.g., `/navigate_to_pose`, `/follow_path`) to execute the planned steps.
    -   Publishes the current plan and status to a topic for monitoring.

### Data Structure: LLM Plan JSON

-   **Description**: The structured format that the Cognitive Bridge asks the LLM to return. Using a structured format like JSON is far more reliable than parsing free-form text. The prompt given to the LLM will explicitly request the output in this format.
-   **Example**:
    ```json
    {
      "command": "bring me the water bottle from the table",
      "plan": [
        {
          "skill": "navigate_to",
          "parameters": {
            "location_name": "table"
          },
          "reasoning": "First, I need to go to the table where the bottle is."
        },
        {
          "skill": "find_object",
          "parameters": {
            "object_name": "water_bottle",
            "search_area": "on_the_table"
          },
          "reasoning": "Next, I need to locate the water bottle on the table."
        },
        {
          "skill": "pick_up",
          "parameters": {
            "object_id": "water_bottle_1" 
          },
          "reasoning": "Once found, I must pick up the bottle."
        },
        {
          "skill": "navigate_to",
          "parameters": {
            "location_name": "user"
          },
          "reasoning": "Finally, I need to bring the bottle back to the user."
        }
      ]
    }
    ```
-   **Validation**: The Cognitive Bridge node is responsible for parsing this JSON and validating that each `skill` in the plan exists in its Skill Library and that the `parameters` are valid.
