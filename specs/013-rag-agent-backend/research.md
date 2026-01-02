# Research: RAG Agent Backend

**Date**: 2026-01-02
**Feature**: `013-rag-agent-backend`

This document summarizes the technical research and decisions for building the RAG agent backend.

## 1. Core Technology Stack

### Decision
The backend will be built using the following stack:
- **Web Framework**: FastAPI, served by Uvicorn.
- **AI Agent**: OpenAI Assistants API (v2) using the `openai` Python library.
- **Vector Database**: Qdrant Cloud (for retrieval).
- **Relational Database**: Neon Serverless Postgres (for chat history).
- **Database Connector**: `psycopg2-binary` for connecting to Postgres.

### Rationale
- **FastAPI**: A modern, high-performance Python web framework that is well-suited for building APIs and has excellent support for asynchronous operations.
- **OpenAI Assistants API**: Provides a robust framework for building stateful, tool-using AI agents, handling much of the complexity of conversation management.
- **Qdrant & Neon**: These are specified in the project constitution and are excellent modern, serverless choices for their respective tasks.
- **`psycopg2-binary`**: A straightforward and widely-used adapter for connecting Python applications to PostgreSQL databases. It's simpler than a full ORM like SQLAlchemy for this use case.

## 2. Agent and Tool Design

### Decision
The core logic will involve creating a custom "tool" for the OpenAI Assistant.
1.  A `QdrantRetriever` tool will be defined as a Python function.
2.  When the agent decides to use this tool, it will call the function with a search query.
3.  The function will then:
    a. Generate an embedding for the search query using Cohere.
    b. Query the Qdrant collection using this embedding to find the top N most similar document chunks.
    c. Return the formatted text of these chunks as a string to the agent.
4.  The agent will use this string as context to formulate its final answer.

### Rationale
- **Tool-Based Approach**: Using the OpenAI Assistants' tool-use feature is the standard and most powerful way to connect an agent to external data sources. It allows the agent to decide when and what to search for.
- **Symmetric Embeddings**: The search query must be embedded with the same model (Cohere) that was used to embed the documents to ensure meaningful similarity search results.

## 3. Chat History Management

### Decision
Chat history will be managed externally in the Neon Postgres database. A custom Python module (`database.py`) will handle this.
1.  Each conversation will have a unique `session_id`.
2.  Before processing a user's request, the backend will fetch the recent message history for that `session_id` from Postgres.
3.  This history will be passed to the OpenAI Assistant to provide conversational context.
4.  After the agent generates a response, the new user message and the agent's response will be saved to the database.

### Rationale
- **Persistence**: Storing history in Postgres ensures that conversations are durable and can be continued across different user sessions or server restarts.
- **Scalability**: Decoupling chat history from the agent's in-memory state allows the backend to be stateless and scale more easily.
- **Control**: It gives us full control over the data, including how long to retain it and the ability to analyze it later.

## 4. Application Structure

### Decision
The backend will be organized in a new `backend/agent/` directory.
- `main.py`: Contains the FastAPI application, API route definitions (`/chat`), and server startup logic.
- `agent.py`: Contains the core logic for the AI Agent, including the definition of the `QdrantRetriever` tool.
- `database.py`: Contains all functions for interacting with the Neon Postgres database (e.g., creating tables, fetching history, saving messages).

### Rationale
- **Separation of Concerns**: This structure cleanly separates the web layer (FastAPI), the AI logic (Agent), and the data persistence layer (Database), making the application easier to develop, test, and maintain.
