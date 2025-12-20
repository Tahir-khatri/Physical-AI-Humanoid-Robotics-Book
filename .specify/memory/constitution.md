<!--
Sync Impact Report:
- Version change: none -> 1.0.0
- Added Principles:
  - Spec-Driven
  - Grounding
  - Modernity
- Added Sections:
  - Key Standards
  - Constraints
  - Success Criteria
- Templates requiring updates:
  - ✅ .specify/templates/plan-template.md
  - ✅ .specify/templates/spec-template.md
  - ✅ .specify/templates/tasks-template.md
- Follow-up TODOs: none
-->
# AI-Driven Book & RAG Chatbot Integration Constitution

## Core Principles

### I. Spec-Driven
All content/code must be generated via spec-kit-plus and Gemini CLI.

### II. Grounding
RAG responses must be strictly tethered to the book’s specific content.

### III. Modernity
Leverage serverless infra (Neon/Qdrant) for high-performance retrieval.

## Key Standards

- **Structure**: Docusaurus-compliant Markdown (.md/.mdx) with metadata.
- **Backend**: FastAPI with OpenAI ChatKit SDK for RAG logic.
- **Data**: Qdrant for vector embeddings; Neon Postgres for relational data.
- **RAG Feature**: Support for "selected-text-only" context queries.

## Constraints

- **Tooling**: No manual content authoring; automated CLI pipeline only.
- **Hosting**: Frontend on GitHub Pages; Backend on serverless compute.
- **Tiers**: Must operate within Qdrant Cloud Free Tier and Neon Free Tier limits.

## Success Criteria

- **Full Pipeline**: Successful build from spec file to GitHub Pages deployment.
- **Functional RAG**: Chatbot accurately retrieves and answers based on book chapters.
- **Context Selection**: Implementation of the user-highlighted text retrieval feature.

## Governance

This Constitution supersedes all other practices. Amendments require documentation, formal approval, and a clear migration plan. All pull requests and reviews must verify compliance with these principles.

**Version**: 1.0.0 | **Ratified**: 2025-12-19 | **Last Amended**: 2025-12-19