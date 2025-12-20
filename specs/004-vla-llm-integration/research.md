# Research: Module 4 Plan - VLA and LLM Integration

**Date**: 2025-12-19
**Feature**: [Module 4: Vision-Language-Action (VLA)](spec.md)

This document records the key architectural and strategic decisions for the final module, which integrates Large Language Models (LLMs) into the robotics stack.

## Key Decisions

### Decision: Speech Integration - OpenAI Whisper
- **Rationale**: The specification requires a robust voice-to-text pipeline. While browser-based WebSpeech APIs exist, they are often inconsistent and less accurate, especially in environments with background noise (like a running robot). **OpenAI's Whisper** model is a state-of-the-art, open-source speech recognition system known for its high accuracy and robustness to noise. Using Whisper (either via its API or a locally run instance) provides a much higher quality and more reliable transcription, which is critical for the entire downstream VLA pipeline. A misinterpreted command leads to incorrect action.
- **Alternatives considered**:
  - **WebSpeech API**: Simpler to integrate for web-based demos, but lacks the accuracy and robustness of Whisper. Not suitable for a production-grade robotics application.
  - **Other Cloud STT Services (e.g., Google Speech-to-Text)**: These are viable alternatives, but Whisper has the significant advantage of being open-source, allowing for local execution which can reduce latency and improve privacy.

### Decision: Orchestration Layer - "Cognitive Bridge" Node
- **Rationale**: There must be a clear separation between the LLM's abstract planning and the robot's concrete execution. A dedicated Python node, the "Cognitive Bridge," will be implemented to serve this purpose. This node will be responsible for:
    1.  Receiving a high-level command (from voice or text).
    2.  Formatting this command into a detailed prompt for the LLM.
    3.  Calling the LLM API and receiving the structured response (e.g., a JSON array of sub-tasks).
    4.  Parsing this response and validating it against a known "Skill Library".
    5.  Translating the validated sub-tasks into a sequence of ROS 2 action goals or a dynamically generated Behavior Tree.
- **Alternatives considered**:
  - **Direct LLM-to-ROS Calls**: Having the LLM directly generate ROS 2 code or commands is brittle and unsafe. The LLM does not have real-time knowledge of the robot's state and could generate dangerous or nonsensical commands. The Cognitive Bridge acts as a necessary safety and abstraction layer.
  - **Monolithic Script**: Combining voice recognition, LLM calls, and ROS 2 actions in a single script. This is poor architecture. Separating the logic into a dedicated orchestration node makes the system more modular, testable, and maintainable.

### Decision: Skill Library - A Primitive Action Set
- **Rationale**: LLMs are powerful planners, but they need to be constrained to a set of actions the robot can actually perform. We will define a "Skill Library" of primitive actions that the LLM can use as its building blocks. This library will be provided to the LLM as part of its system prompt. The Cognitive Bridge will be responsible for executing these primitives.
- **Example Primitives**:
    -   `navigate_to(location_name)`
    -   `find_object(object_name, search_area)`
    -   `pick_up(object_id)`
    -   `place_at(location_name)`
    -   `wait(seconds)`
    -   `say(text)`
- **Rationale for this approach**: This "API-like" interaction turns the LLM into a high-level semantic planner. It doesn't need to know *how* to navigate; it just needs to know that it *can* navigate by calling the `navigate_to` skill. This makes the prompts more reliable and the system's behavior more predictable. It also makes the system extensible: adding a new capability to the robot simply involves creating a new skill and adding it to the LLM's prompt.
- **Alternatives considered**:
  - **Unconstrained Output**: Allowing the LLM to generate free-form text descriptions of what to do. This would be incredibly difficult to parse reliably and would lead to unpredictable behavior. The skill library provides the necessary structure.
