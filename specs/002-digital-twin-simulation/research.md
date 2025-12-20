# Research: Module 2 Plan - Simulation and Prompt Engineering

**Date**: 2025-12-19
**Feature**: [Module 2: The Digital Twin (Gazebo & Unity)](spec.md)

This document records the key technical and strategic decisions for generating the highly detailed content of Module 2.

## Key Decisions

### Decision: Physics Engine - ODE (Open Dynamics Engine)
- **Rationale**: For bipedal humanoid stability, ODE is a well-established and robust choice within the Gazebo ecosystem. It provides a good balance of performance and accuracy, with extensive documentation and community support. Its parameters for contact dynamics (`cfm`, `erp`), friction, and damping are well-understood and critical for achieving the stability required by the spec. While Bullet is also powerful, ODE's long history with Gazebo makes it a more reliable starting point for the exhaustive guide required.
- **Alternatives considered**:
  - **Bullet**: A powerful and more modern physics engine. It's an excellent alternative, but for the specific goal of documenting a stable baseline, ODE's behavior is often considered more predictable and less prone to "explosions" with complex models. The guide will focus on ODE but may mention Bullet as an alternative.
  - **Simbody**: A more specialized engine focused on biomechanical simulation. It's too complex for the scope of this module and less common in general robotics.

### Decision: Rendering Pipeline - Unity HDRP
- **Rationale**: The feature specification requires photorealistic rendering for immersive human-robot interaction (HRI). Unity's High Definition Render Pipeline (HDRP) is specifically designed for this purpose. It provides advanced features for lighting, materials, and post-processing that are essential for creating visually stunning and believable simulation environments. This choice directly addresses the "Immersive Rendering & HRI" success criteria.
- **Alternatives considered**:
  - **Unity's Universal Render Pipeline (URP)**: URP is more performance-focused and suitable for a wider range of platforms. However, it lacks the high-end graphical fidelity of HDRP, which is the primary requirement for this chapter.
  - **Unreal Engine**: While powerful, integrating ROS 2 with Unreal is generally less straightforward than with Unity, which has more mature and officially supported tools like the Unity Robotics Hub. Sticking with Unity provides a smoother path for robotics engineers.

### Decision: Prompt Engineering for Gemini CLI
- **Rationale**: The constraint of generating 1,500+ words of "very detailed" and "exhaustive" technical content requires a specific strategy. A simple, high-level prompt is insufficient and risks generating superficial or incorrect information. The chosen strategy is to use highly-structured, multi-shot prompts. This involves providing the Gemini CLI with:
    1.  **A Clear Role:** "You are an expert robotics professor writing a chapter for a graduate-level textbook."
    2.  **Detailed Structure:** A precise outline for the chapter, including all required sub-sections (e.g., "Introduction to Friction Models," "Configuring Surface Friction in SDF," "Practical Example: Tuning Foot Friction").
    3.  **Key Terminology:** A list of mandatory keywords, parameters, and concepts to include (e.g., `mu`, `mu2`, `fdir1`, `slip1`, `slip2`).
    4.  **Constraints:** Explicitly stating the minimum word count and the requirement for code snippets, parameter tables, and mathematical formulas.
- **Alternatives considered**:
  - **Single, Large Prompt:** A single large prompt is more likely to result in repetitive content or hallucinations as the context window is stretched.
  - **Iterative Generation:** Generating section by section and then combining them is a valid approach, but the multi-shot strategy within a single, well-engineered prompt is more efficient for the agent-based workflow. It ensures better coherence across the entire chapter.
