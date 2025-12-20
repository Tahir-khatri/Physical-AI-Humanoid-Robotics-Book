# Quickstart: Module 2 Validation

**Date**: 2025-12-19
**Feature**: [Module 2: The Digital Twin (Gazebo & Unity)](spec.md)

This guide provides the necessary steps to validate the generated content for Module 2, specifically checking the word count requirement and site integrity.

## Prerequisites

-   [Node.js](https://nodejs.org/) (LTS)
-   The project's Node.js dependencies must be installed (`npm install` from the `Book-Frontend` directory).

## Validation Steps

### 1. Word Count Validation

A custom script will be needed to verify that each chapter meets the 1,500-word minimum.

**Example (conceptual) script `check-word-counts.mjs`:**

```javascript
import fs from 'fs';
import path from 'path';

const docsDir = path.join('Book-Frontend', 'docs', 'module-2');
const files = ['chapter-1.md', 'chapter-2.md', 'chapter-3.md'];
const minWordCount = 1500;
let all_passed = true;

console.log('--- Running Word Count Validation for Module 2 ---');

for (const file of files) {
  const filePath = path.join(docsDir, file);
  if (!fs.existsSync(filePath)) {
    console.error(`✗ ERROR: ${file} not found!`);
    all_passed = false;
    continue;
  }
  const content = fs.readFileSync(filePath, 'utf-8');
  // Simple word count: split by whitespace and filter empty strings
  const wordCount = content.split(/\s+/).filter(Boolean).length;

  if (wordCount >= minWordCount) {
    console.log(`✓ PASS: ${file} has ${wordCount} words (>= ${minWordCount}).`);
  } else {
    console.error(`✗ FAIL: ${file} has only ${wordCount} words (< ${minWordCount}).`);
    all_passed = false;
  }
}

console.log('--- Validation Complete ---');
process.exit(all_passed ? 0 : 1);
```

**To run this validation:**

```bash
node check-word-counts.mjs
```

### 2. Build Integrity Check

This step ensures that all Markdown files, including any LaTeX equations and cross-links, are correctly formatted and that the Docusaurus site can be built without errors.

**To run this validation, navigate to the `Book-Frontend` directory and run:**

```bash
npm run build
```

A successful build indicates that the content is syntactically correct and all internal links are valid. This is a critical acceptance test.
