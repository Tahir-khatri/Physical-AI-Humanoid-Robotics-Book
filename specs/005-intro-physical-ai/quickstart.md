# Quickstart: Introduction Validation

**Date**: 2025-12-19
**Feature**: [Introduction: Physical AI & Humanoid Robotics](spec.md)

This guide provides the necessary steps to validate the generated content for the Introduction section.

## Prerequisites

-   [Node.js](https://nodejs.org/) (LTS)
-   The project's Node.js dependencies must be installed (`npm install` from the `Book-Frontend` directory).

## Validation Steps

### 1. Word Count Validation

This check ensures each introductory chapter meets the 1,300-1,500 word count requirement.

**Create a script `check-intro-words.mjs`:**

```javascript
import fs from 'fs';
import path from 'path';

const docsDir = path.join('Book-Frontend', 'docs', 'introduction');
const files = [
    '01-ros-2-overview.md',
    '02-digital-twin-overview.md',
    '03-isaac-perception-overview.md',
    '04-vla-capstone-overview.md'
];
const minWordCount = 1300;
const maxWordCount = 1500;
let all_passed = true;

console.log('--- Running Word Count Validation for Introduction ---');

for (const file of files) {
  const filePath = path.join(docsDir, file);
  if (!fs.existsSync(filePath)) {
    console.error(`✗ ERROR: ${file} not found!`);
    all_passed = false;
    continue;
  }
  const content = fs.readFileSync(filePath, 'utf-8');
  const wordCount = content.split(/\s+/).filter(Boolean).length;

  if (wordCount >= minWordCount && wordCount <= maxWordCount) {
    console.log(`✓ PASS: ${file} has ${wordCount} words.`);
  } else {
    console.error(`✗ FAIL: ${file} has ${wordCount} words.`);
    all_passed = false;
  }
}

console.log('--- Validation Complete ---');
process.exit(all_passed ? 0 : 1);
```

**To run this validation:**

```bash
node check-intro-words.mjs
```

### 2. Connectivity Check (Manual)

This is a manual check to ensure the introductory chapters correctly link to their corresponding modules.

-   [ ] Open `01-ros-2-overview.md` and verify it contains a link to `/docs/module-1/chapter-1`.
-   [ ] Open `02-digital-twin-overview.md` and verify it contains a link to `/docs/module-2/chapter-1`.
-   [ ] Open `03-isaac-perception-overview.md` and verify it contains a link to `/docs/module-3/chapter-1`.
-   [ ] Open `04-vla-capstone-overview.md` and verify it contains a link to `/docs/module-4/chapter-1`.

### 3. Build Integrity Check

This ensures the Docusaurus site builds correctly with the new introductory content.

**To run, navigate to the `Book-Frontend` directory and execute:**

```bash
npm run build
```
A successful build confirms that all files are formatted correctly and all links are valid.
