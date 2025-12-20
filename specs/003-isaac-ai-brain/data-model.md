# Data Model: Module 3

**Date**: 2025-12-19
**Feature**: [Module 3: The AI-Robot Brain (NVIDIA Isaac™)](spec.md)

This feature focuses on generating highly technical documentation and configurations. The data model describes the structure of this content.

## Content as Data

The primary data entities are the three chapters, along with the sample configuration files they reference.

### Entity: Chapter

-   **Description**: A single, exhaustive technical chapter on a topic within the NVIDIA Isaac ecosystem.
-   **Storage**: Stored as a `.md` file within the `Book-Frontend/docs/module-3/` directory.
-   **Attributes**:
    -   **`title`** (string): The H1 title of the chapter.
    -   **`content`** (string): The body of the chapter in Markdown format, exceeding 1,500 words. Must include code snippets for Python/Replicator, YAML for Nav2/Isaac ROS, and LaTeX for mathematical formulas.
    -   **`docusaurus_metadata`** (object): Front-matter including `id`, `title`, and `sidebar_label`.
-   **Validation Rules**:
    -   Word count must be >= 1,500.
    -   All code/config snippets must be syntactically valid.
    -   The file must be renderable by Docusaurus.

### Entity: Configuration File

-   **Description**: A sample configuration file used in the chapter content. These are critical artifacts for the reader to reproduce the examples.
-   **Storage**: Stored as `.yaml` or `.json` files within the `specs/003-isaac-ai-brain/sample_configs/` directory.
-   **Examples**:
    -   `nav2_biped_params.yaml`: Contains the tuned parameters for the Nav2 controller and planner.
    -   `isaac_ros_vslam.json`: A launch configuration file for an Isaac ROS GEM.
    -   `replicator_script.py`: A Python script for Omniverse Replicator to generate synthetic data.

### Relationships

-   The three **Chapter** entities are ordered sequentially in `Book-Frontend/sidebars.ts` to form the Module 3 learning path.
-   Each **Chapter**'s content will reference one or more **Configuration File** entities, instructing the reader on how to use them.
