# Data Model: Introduction

**Date**: 2025-12-19
**Feature**: [Introduction: Physical AI & Humanoid Robotics](spec.md)

This feature is purely for content generation and does not have a traditional data model. The data entities are the four markdown chapters of the introduction.

## Content as Data

### Entity: Introductory Chapter

-   **Description**: A single, high-level overview chapter that summarizes one of the four main modules of the book.
-   **Storage**: Stored as a `.md` file within the `Book-Frontend/docs/introduction/` directory.
-   **Attributes**:
    -   **`title`** (string): The H1 title of the chapter.
    -   **`content`** (string): The body of the chapter in Markdown format, between 1,300 and 1,500 words. It will contain high-level explanations and architectural diagrams, but no detailed code snippets.
    -   **`docusaurus_metadata`** (object): Front-matter including `id`, `title`, `sidebar_label`, and a tag for "Overview".
-   **Validation Rules**:
    -   Word count must be between 1,300 and 1,500.
    -   The file must be a valid Markdown file that can be parsed and rendered by Docusaurus.
    -   Each chapter must contain at least one link to the corresponding deep-dive module.

### Relationships

The four **Introductory Chapter** entities will be ordered sequentially in `Book-Frontend/sidebars.ts`. This "Introduction" category will be placed *before* the "Module 1" category to ensure it serves as the primary landing experience for new readers.
