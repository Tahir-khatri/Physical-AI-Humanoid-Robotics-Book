---
id: chapter-2
title: 'Chapter 2: Cognitive Planning & LLM-to-ROS Orchestration'
sidebar_label: 'Cognitive Planning with LLMs'
---

# Chapter 2: Cognitive Planning & LLM-to-ROS Orchestration

In the previous chapter, we built a bridge from human speech to text. Now, we will build a bridge from text to *intent*. A simple command parser with keyword spotting is effective for basic, predefined commands, but it is brittle. It fails as soon as a user phrases a command in an unexpected way. To create a truly intelligent and flexible robot, we need a system that can *understand* the user's goal, not just recognize keywords.

This is the domain of Large Language Models (LLMs) like GPT-4 and Gemini. These models possess a remarkable ability to reason, parse complex sentences, and decompose high-level goals into logical sub-tasks. In this chapter, we will build a "Cognitive Bridge"—a sophisticated ROS 2 node that acts as an orchestrator. It will take a natural language command, use an LLM to form a structured plan, and then execute that plan by calling the appropriate ROS 2 action servers.

## The LLM as a High-Level Planner

It is crucial to understand the role of the LLM in a robotics stack. We are **not** asking the LLM to write code or control hardware directly. This would be unsafe and unpredictable. Instead, we are using the LLM as a high-level **semantic planner**.

Our strategy is to define a "Skill Library"—a finite set of high-level capabilities that our robot possesses. We then present this library to the LLM and ask it to use these skills as building blocks to create a plan.

**Example Skill Library:**
- `navigate_to(location)`: Moves the robot to a named location.
- `find_object(object_name, search_area)`: Uses the vision system to find an object.
- `pick_up(object_id)`: Executes the manipulation routine to grasp a found object.
- `place_at(location_name)`: Navigates to a location and places the held object.
- `say(text)`: Uses a text-to-speech engine to speak.

The LLM's task is not to implement these skills, but to **sequence** them correctly based on the user's command.

## The Cognitive Bridge Architecture

Our `llm_orchestrator.py` node will have the following architecture:

1.  **Subscription**: Subscribes to the `/voice_transcription` topic to receive commands.
2.  **Prompt Engineering**: When a command is received, the node wraps it in a carefully engineered **system prompt**. This prompt is the key to getting reliable, structured output from the LLM.
3.  **API Call**: The node sends the full prompt to the LLM API (e.g., OpenAI's or Google's).
4.  **JSON Parsing and Validation**: The node receives the LLM's response, which we will instruct to be in JSON format. It parses the JSON and validates it against its internal Skill Library to ensure the LLM hasn't hallucinated a non-existent skill.
5.  **Execution Engine**: The node iterates through the validated plan, calling the corresponding ROS 2 action server for each step and waiting for the result before proceeding to the next.

## The Art of the System Prompt

Getting reliable, structured data from an LLM requires a well-designed prompt. Your prompt should be a template that includes several key pieces of information:

-   **The Role**: Tell the LLM what it is. "You are the central planner for a humanoid robot."
-   **The Goal**: Tell it what its objective is. "Your task is to decompose a user's command into a sequence of executable skills."
-   **The Skill Library**: Provide the complete list of available skills and their parameters, formatted clearly.
-   **The Output Format**: Explicitly define the JSON structure you want it to return. This is the most critical part for reliable parsing.
-   **Constraints & Rules**: Give it rules to follow. "Only use skills from the provided library. If a command is ambiguous or unsafe, respond with an error."
-   **Examples (Few-Shot Prompting)**: Provide one or two examples of a user command and the corresponding correct JSON output. This significantly improves the model's accuracy.

**Conceptual System Prompt Template:**
```text
You are the high-level planner for a sophisticated humanoid robot. Your task is to decompose a user's command into a structured plan in JSON format.

Available Skills:
1. navigate_to(location: str): Moves the robot to a predefined location. Known locations: [kitchen, living_room, table, user].
2. find_object(object_name: str, search_area: str): Searches a general area for a specific object.
3. pick_up(object_id: str): Grasps a specific object that has been found.
4. place_at(location_name: str): Moves to a location and places the currently held object.
5. say(text: str): Speaks a given text string.

Rules:
- You must respond in valid JSON format.
- The plan should be a list of objects, where each object has a "skill" and a "parameters" dictionary.
- If a command is impossible, unsafe, or ambiguous, respond with an error skill.

Example:
User Command: "Robot, can you get me the red soda from the kitchen?"
Your JSON Response:
{
  "plan": [
    { "skill": "navigate_to", "parameters": { "location": "kitchen" } },
    { "skill": "find_object", "parameters": { "object_name": "red soda", "search_area": "kitchen" } },
    { "skill": "pick_up", "parameters": { "object_id": "red_soda_1" } },
    { "skill": "navigate_to", "parameters": { "location": "user" } },
    { "skill": "say", "parameters": { "text": "Here is your soda." } }
  ]
}

Now, generate the plan for this new command.
User Command: "{user_command}"
Your JSON Response:
```

## Building the `llm_orchestrator.py` Node

This node ties everything together. It manages the state of the plan execution.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import openai # Or google.generativeai

# Assume action clients for navigate_to, pick_up, etc. have been written
from my_robot_actions.robot_action_client import RobotActionClient

class LLMOrchestrator(Node):
    def __init__(self):
        super().__init__('llm_orchestrator')
        self.subscription = self.create_subscription(
            String,
            '/voice_transcription',
            self.command_callback,
            10)
            
        # Initialize your LLM client (e.g., set API key)
        # openai.api_key = "..."
        
        self.skill_library = ["navigate_to", "find_object", "pick_up", "place_at", "say"]
        self.robot_actions = RobotActionClient() # A helper class that manages all ROS 2 action clients

        self.get_logger().info("LLM Orchestrator is ready.")

    def command_callback(self, msg):
        user_command = msg.data
        self.get_logger().info(f"Received command: '{user_command}'")
        
        system_prompt = self.build_system_prompt(user_command)
        
        try:
            # --- LLM API Call ---
            # response = openai.ChatCompletion.create(...)
            # For this example, we'll use a hardcoded response.
            llm_response_str = """
            {
              "plan": [
                { "skill": "navigate_to", "parameters": { "location": "table" } },
                { "skill": "find_object", "parameters": { "object_name": "water_bottle" } },
                { "skill": "pick_up", "parameters": { "object_id": "water_bottle_1" } }
              ]
            }
            """
            
            # --- Parsing and Validation ---
            plan_data = json.loads(llm_response_str)
            plan = plan_data.get("plan", [])

            if not self.is_plan_valid(plan):
                self.get_logger().error("LLM generated an invalid plan.")
                return

            # --- Plan Execution ---
            self.execute_plan(plan)

        except Exception as e:
            self.get_logger().error(f"Failed to process command: {e}")

    def build_system_prompt(self, command):
        # In a real system, you would load this from a file and format it
        return f"You are a robot planner... User Command: '{command}'"

    def is_plan_valid(self, plan):
        for step in plan:
            if step.get("skill") not in self.skill_library:
                return False
        return True

    def execute_plan(self, plan):
        self.get_logger().info(f"Executing plan with {len(plan)} steps...")
        for i, step in enumerate(plan):
            skill = step["skill"]
            params = step["parameters"]
            self.get_logger().info(f"Step {i+1}: Executing skill '{skill}' with params {params}")
            
            # The future/result handling of the action client is simplified here
            # A real implementation needs to block until the action is complete
            if skill == "navigate_to":
                future = self.robot_actions.send_navigation_goal(params["location"])
                # rclpy.spin_until_future_complete(self, future) # Block until done
            elif skill == "pick_up":
                future = self.robot_actions.send_pickup_goal(params["object_id"])
                # rclpy.spin_until_future_complete(self, future)
            # ... and so on for other skills
            
            # Check if the action was successful before proceeding
            # if not future.result().success:
            #    self.get_logger().error(f"Skill '{skill}' failed! Aborting plan.")
            #    return

        self.get_logger().info("Plan executed successfully!")

# ... main function ...
```

This node represents a significant leap in capability. The robot is no longer limited to a hardcoded set of command phrases. It can now understand a vast range of natural language requests and, by consulting an LLM, formulate a logical sequence of actions to achieve the user's goal. This cognitive bridge between language and action is the core of modern Embodied AI and the primary focus of the final capstone project in the next chapter.
