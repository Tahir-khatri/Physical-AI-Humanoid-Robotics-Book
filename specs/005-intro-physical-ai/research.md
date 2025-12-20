# Research: Introduction Plan

**Date**: 2025-12-19
**Feature**: [Introduction: Physical AI & Humanoid Robotics](spec.md)

This document records the key pedagogical and structural decisions for the book's introductory chapters.

## Key Decisions

### Decision: Curriculum Flow & Chapter Sequence
- **Rationale**: The introduction will mirror the structure of the four main modules, providing a logical and progressive roadmap for the reader. The chosen sequence—**1. Middleware (ROS 2), 2. Simulation (Gazebo/Unity), 3. Perception (Isaac), 4. Cognition (VLA/LLMs)**—is pedagogically sound. It builds from the ground up, starting with the fundamental communication layer, then establishing a safe place to experiment (simulation), then building the ability to see, and finally adding the intelligence to act on what is seen. This "full-stack" approach ensures the reader has all necessary context before moving to the next level of complexity.
- **Alternatives considered**:
  - **AI-First Approach**: Starting with the LLM and VLA concepts. This was rejected because without a firm grasp of the underlying robotics architecture (ROS 2, simulation), the VLA concepts would be too abstract and their implementation details would lack context.
  - **Hardware-First Approach**: Starting with URDFs and physics. This was rejected as it's less engaging and doesn't immediately connect to the "AI" aspect of the book's title. Starting with ROS 2 as the "nervous system" provides a better central metaphor.

### Decision: Tooling Choice Justification
- **Rationale**: The introduction must briefly justify the choice of the ROS 2 / Gazebo / Unity / NVIDIA Isaac stack. This stack was chosen because it represents the current, de-facto industry standard for high-performance, AI-driven robotics development.
    - **ROS 2**: The undisputed middleware standard for robotics R&D.
    - **Gazebo/Unity**: This pair represents the best-of-both-worlds in simulation: Gazebo for robust, widely-supported physics and ROS integration; Unity for cutting-edge, photorealistic rendering essential for vision-based AI.
    - **NVIDIA Isaac**: For hardware-accelerated perception and simulation, the NVIDIA ecosystem is unparalleled. Highlighting this positions the book as a guide to production-grade robotics, not just academic theory.
- **Alternatives considered**:
  - **Focusing on a single simulator (e.g., only Gazebo)**: This would simplify the curriculum but would fail to teach the critical skill of using high-fidelity renderers like Unity/Isaac Sim for training perception systems.
  - **Using alternative, less common tools**: This would make the book less practical for readers looking to gain skills for the robotics job market. Sticking to the industry standard provides the most value.

### Decision: Landing Experience
- **Rationale**: The `sidebars.ts` file will be configured to make the "Introduction" section the very first item, appearing above "Module 1". This ensures that any new reader landing on the Docusaurus site is immediately presented with the high-level overview, providing context and a clear roadmap before they are asked to dive into the technical deep-dive modules. The first page of the introduction will serve as the homepage for the entire book.
- **Alternatives considered**:
  - **Placing it in a "Module 0"**: While logical, naming it "Introduction" is more user-friendly and immediately understandable.
  - **Scattering the overviews**: Placing each overview chapter at the beginning of its respective module. This was rejected as it fails to provide a single, cohesive up-front summary of the entire curriculum.
