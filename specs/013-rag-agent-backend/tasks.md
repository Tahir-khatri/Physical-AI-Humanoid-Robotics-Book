# Tasks: RAG Agent Backend

**Branch**: `013-rag-agent-backend` | **Date**: 2026-01-02 | **Plan**: [plan.md](./plan.md)

This document breaks down the implementation plan for the RAG Agent Backend into actionable tasks.

## Phase 1: Project Setup

- [ ] T001 Create the `backend/agent/` directory.
- [ ] T002 Create the initial empty files: `backend/agent/main.py`, `backend/agent/agent.py`, and `backend/agent/database.py`.
- [ ] T003 [P] Add the new dependencies (`fastapi`, `uvicorn[standard]`, `openai`, `psycopg2-binary`, `SQLAlchemy`) to `backend/pyproject.toml` by running `uv pip install`.
- [ ] T004 [P] In `backend/.env.example`, add placeholders for `OPENAI_API_KEY` and `NEON_DATABASE_URL`.

## Phase 2: Foundational Code (Database & Core Types)

- [ ] T005 In `backend/agent/database.py`, define the SQLAlchemy table models for `chat_sessions` and `chat_messages` as specified in `data-model.md`.
- [ ] T006 In `backend/agent/database.py`, implement a function `init_db()` to connect to the Postgres database and create the tables if they don't exist.
- [ ] T007 In `backend/agent/main.py`, define the Pydantic models for API requests and responses: `ChatRequest`, `ChatResponse`, and `ContextualChatRequest`.
- [ ] T008 [P] In `backend/agent/main.py`, create the basic FastAPI app instance.

## Phase 3: User Story 1 - Ask a Question (MVP)

**Goal**: A user can send a question and get a response without conversation history.

- [ ] T009 [US1] In `backend/agent/agent.py`, implement the `QdrantRetriever` tool function. This function will take a query string, embed it using Cohere, search Qdrant, and return the formatted context.
- [ ] T010 [US1] In `backend/agent/agent.py`, implement the agent execution logic `run_agent_turn()`. This initial version will initialize the OpenAI Assistant with the `QdrantRetriever` tool and run it with the user's question. It will not handle chat history yet.
- [ ] T011 [US1] In `backend/agent/main.py`, implement the `POST /chat` endpoint. This version will receive a `ChatRequest`, call `run_agent_turn()`, and return the response. It will ignore the `session_id` for now.

## Phase 4: User Story 2 - Continue a Conversation

**Goal**: The agent can remember previous turns in the conversation.

- [ ] T012 [US2] In `backend/agent/database.py`, implement the `get_or_create_session(session_id: str)` function to find an existing session or create a new one.
- [ ] T013 [US2] In `backend/agent/database.py`, implement `get_chat_history(session_id: str)` to retrieve all messages for a given session.
- [ ] T014 [US2] In `backend/agent/database.py`, implement `add_message_to_history(session_id: str, role: str, content: str)`.
- [ ] T015 [US2] In `backend/agent/agent.py`, modify `run_agent_turn()` to accept a list of previous messages (chat history) and pass it to the OpenAI Assistant.
- [ ] T016 [US2] In `backend/agent/main.py`, enhance the `/chat` endpoint to handle the `session_id`. It should now call the database functions to get history before running the agent and save the new messages after.

## Phase 5: User Story 3 - Query with Selected Text

**Goal**: The agent can answer questions about a specific piece of text.

- [ ] T017 [US3] In `backend/agent/main.py`, implement the `POST /chat/contextual` endpoint.
- [ ] T018 [US3] In `backend/agent/agent.py`, modify `run_agent_turn()` to accept an optional `context` string. If provided, this context should be prepended to the system prompt or user message to guide the agent's response.

## Phase 6: Polish & Orchestration

- [ ] T019 [P] Add comprehensive logging for queries, responses, and errors to all modules.
- [ ] T020 [P] Add detailed docstrings and complete type hints to all functions.
- [ ] T021 In `backend/agent/main.py`, add a root `/` GET endpoint for health checks.
- [ ] T022 In `backend/agent/main.py`, add the `uvicorn.run()` call within an `if __name__ == "__main__":` block to make the server runnable directly.

## Dependencies

```mermaid
graph TD
    subgraph Phase 1 & 2
        direction LR
        A[Setup] --> B[Foundation];
    end
    subgraph Phase 3 (MVP)
        direction LR
        C[US1: Ask Question];
    end
    subgraph Phase 4 & 5
        direction LR
        D[US2: Conversation];
        E[US3: Contextual Query];
    end
    subgraph Phase 6
        direction TB
        F[Polish];
    end

    B --> C;
    C --> D;
    C --> E;
    E --> F;
    D --> F;
```

## Implementation Strategy

The implementation will proceed phase by phase. The MVP (Phase 3) will deliver a functional, stateless chatbot. Subsequent phases will add conversation memory (Phase 4) and contextual query capabilities (Phase 5), building upon the core functionality.
