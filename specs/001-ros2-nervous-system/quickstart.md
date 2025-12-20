# Quickstart: Module 1 Docusaurus Site

**Date**: 2025-12-19
**Feature**: [Module 1: The Robotic Nervous System (ROS 2)](spec.md)

This guide provides instructions on how to set up the Docusaurus development environment and build the static site.

## Prerequisites

- [Node.js](https://nodejs.org/) (LTS version recommended)
- [npm](https://www.npmjs.com/) (comes with Node.js)

## Setup and Installation

1.  **Initialize Docusaurus Project**:
    The first step is to create a new Docusaurus site.

    ```bash
    npm init docusaurus@latest . -- --template classic
    ```

2.  **Install Dependencies**:
    If not already installed by the init command, install the necessary dependencies.

    ```bash
    npm install
    ```

3.  **Directory Structure**:
    Ensure the following directory exists for the book content:

    ```
    /docs/module-1/
    ```

## Development

1.  **Start the Development Server**:
    To start a live-reloading development server, run the following command from the project root.

    ```bash
    npm run start
    ```

    This will open a browser window with the documentation site, usually at `http://localhost:3000`. Changes to Markdown files will be reflected automatically.

2.  **Content Generation**:
    The Markdown files for the chapters (`chapter-1.md`, `chapter-2.md`, `chapter-3.md`) will be generated into the `/docs/module-1/` directory by the Spec-Kit Plus and Gemini CLI tooling, as defined in the project's tasks.

## Build and Validation

1.  **Build the Static Site**:
    To create a production-ready build of the static site, run:

    ```bash
    npm run build
    ```

    This command will check for broken links and other common issues. The output will be placed in the `/build` directory. This serves as the primary validation test for the content's integrity.

2.  **Serve the Build Locally**:
    To preview the production build locally, you can use the `serve` command.

    ```bash
    npm run serve
    ```
