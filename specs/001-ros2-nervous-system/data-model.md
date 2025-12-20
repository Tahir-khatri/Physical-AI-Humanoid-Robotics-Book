# Data Model: Module 1

**Date**: 2025-12-19
**Feature**: [Module 1: The Robotic Nervous System (ROS 2)](spec.md)

This feature is focused on content generation and does not introduce a traditional data model with entities and databases. The "data" for this feature is the set of Markdown files that constitute the book's content.

## Content as Data

The primary data entities are the chapters themselves, which will be stored as Markdown files.

### Entity: Chapter

- **Description**: A single chapter in the book module.
- **Storage**: Stored as a `.md` file in the `docs/module-1/` directory.
- **Attributes (as file content and metadata)**:
  - **`title`** (string): The title of the chapter, typically the H1 heading.
  - **`content`** (string): The body of the chapter in Markdown format.
  - **`docusaurus_metadata`** (object): Front-matter in the Markdown file used by Docusaurus (e.g., `id`, `title`, `sidebar_label`).
- **Relationships**:
  - Chapters are ordered sequentially within the `sidebars.js` configuration file to form the module's narrative flow.
