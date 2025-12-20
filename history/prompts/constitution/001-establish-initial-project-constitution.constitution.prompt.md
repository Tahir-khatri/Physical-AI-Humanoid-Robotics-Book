---
id: 001
title: Establish Initial Project Constitution
stage: constitution
date: 2025-12-19
surface: agent
model: gemini-pro
feature: none
branch: main
user: n/a
command: /sp.constitution
labels: [constitution, setup]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
  - .specify/templates/plan-template.md
tests:
  - n/a
---

## Prompt

/sp.constitution Project: AI-Driven Book & RAG Chatbot Integration

Core Principles:

Spec-Driven: All content/code must be generated via spec-kit-plus and Gemini CLI.

Grounding: RAG responses must be strictly tethered to the book’s specific content.

Modernity: Leverage serverless infra (Neon/Qdrant) for high-performance retrieval.

Key Standards:

Structure: Docusaurus-compliant Markdown (.md/.mdx) with metadata.

Backend: FastAPI with OpenAI ChatKit SDK for RAG logic.

Data: Qdrant for vector embeddings; Neon Postgres for relational data.

RAG Feature: Support for "selected-text-only" context queries.

Constraints:

Tooling: No manual content authoring; automated CLI pipeline only.

Hosting: Frontend on GitHub Pages; Backend on serverless compute.

Tiers: Must operate within Qdrant Cloud Free Tier and Neon Free Tier limits.

Success Criteria:

Full Pipeline: Successful build from spec file to GitHub Pages deployment.

Functional RAG: Chatbot accurately retrieves and answers based on book chapters.

Context Selection: Implementation of the user-highlighted text retrieval feature.

## Response snapshot

Updated the project constitution at `.specify/memory/constitution.md` with the provided principles and standards. The new version is 1.0.0. I also updated the `.specify/templates/plan-template.md` to reflect the new constitutional checks.

## Outcome

- ✅ Impact: High - Establishes foundational principles for the project.
- 🧪 Tests: n/a
- 📁 Files: 2 modified
- 🔁 Next prompts: /sp.specify
- 🧠 Reflection: The constitution provides a strong foundation for future development.

## Evaluation notes (flywheel)

- Failure modes observed: n/a
- Graders run and results (PASS/FAIL): n/a
- Prompt variant (if applicable): n/a
- Next experiment: n/a
