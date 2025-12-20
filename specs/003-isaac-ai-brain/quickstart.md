# Quickstart: Module 3 Validation

**Date**: 2025-12-19
**Feature**: [Module 3: The AI-Robot Brain (NVIDIA Isaac™)](spec.md)

This guide provides the steps to validate the generated content for Module 3, focusing on word count, build integrity, and benchmark verification.

## Prerequisites

-   [Node.js](https://nodejs.org/) (LTS)
-   Project Node.js dependencies installed (`npm install` in `Book-Frontend`).
-   A local environment capable of running NVIDIA Isaac Sim (including compatible NVIDIA driver and Docker).

## Validation Steps

### 1. Word Count Validation

This check ensures each chapter meets the 1,500-word minimum requirement.

**Create a script `check-module-3-words.mjs`:**

```javascript
import fs from 'fs';
import path from 'path';

const docsDir = path.join('Book-Frontend', 'docs', 'module-3');
// ... (rest of the word count script from Module 2, adapted for module-3)
```

**To run this validation:**

```bash
node check-module-3-words.mjs
```
*(Note: The full script would be created as part of the implementation task).*

### 2. Build Integrity Check

This ensures the Docusaurus site builds correctly with the new content.

**To run, navigate to the `Book-Frontend` directory and execute:**

```bash
npm run build
```

### 3. Acceptance Criteria & Benchmark Validation

This is a manual or semi-automated step to verify the technical claims made in the chapters against the success criteria from the `spec.md`.

**Checklist:**

-   [ ] **SC-001 (Synthetic Data)**:
    -   [ ] Run the provided Omniverse Replicator script (`replicator_script.py`).
    -   [ ] Verify that it generates at least 1,000 images and corresponding label files.
    -   [ ] Manually inspect 5-10 random samples to confirm bounding boxes are accurate.

-   [ ] **SC-002 (VSLAM Performance)**:
    -   [ ] Set up the Isaac ROS VSLAM container with the provided configuration.
    -   [ ] Run the simulated stereo camera feed into the GEM.
    -   [ ] Use a tool like `nvtop` or `nvidia-smi` to monitor GPU usage.
    -   [ ] Measure the processing frame rate to ensure it meets or exceeds 30 FPS.

-   [ ] **SC-003 (Nav2 Navigation)**:
    -   [ ] Launch the full Nav2 stack with the provided bipedal-specific `nav2_biped_params.yaml`.
    -   [ ] In a test environment with obstacles, send 10 navigation goals to a point 10 meters away.
    -   [ ] Record the number of successful attempts (robot reaches the goal without collision or falling).
    -   [ ] Verify the success rate is >= 90%.

-   [ ] **SC-004 (Nav2 Knowledge)**:
    -   [ ] Manually review Chapter 3 to confirm it clearly explains the roles of the controller, planner, and recovery behavior plugins in Nav2.
