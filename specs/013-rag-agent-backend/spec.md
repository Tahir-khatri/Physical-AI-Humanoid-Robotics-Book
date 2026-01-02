# Feature Specification: RAG Agent Backend

**Feature Branch**: `013-rag-agent-backend`  
**Created**: 2026-01-02
**Status**: Draft  
**Input**: User description: "Build RAG Agent backend using OpenAI Agents SDK + FastAPI Target audience: Developers implementing the chatbot backend for the project book Focus: Create an API backend that exposes an AI Agent with retrieval capabilities over the embedded book content stored in Qdrant Cloud Success criteria: - AI Agent built using OpenAI Agents / ChatKit SDK - Backend served via FastAPI with clear API routes - Retrieval queries vector data from Qdrant Cloud - Agent answers questions using retrieved book content only - Supports user-selected text retrieval - Environment-driven config for keys + endpoints - End-to-end tested locally Constraints: - Stack: FastAPI, OpenAI Agents/ChatKit SDK, Qdrant Cloud - Database: Neon Serverless Postgres for chat/session storage - Backend only (no UI logic) - Secure handling of API keys + configs - Script / service-based execution Not building: - Frontend integration - UI widgets or styling - Analytics or evaluation pipelines - Non-book external knowledge retrieval"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Ask a Question (Priority: P1)

A user sends a question to the chatbot backend via an API call and receives an answer based on the book's content.

**Why this priority**: This is the primary function of the chatbot. Without it, the feature has no value.

**Independent Test**: Can be tested by sending a POST request with a question to the `/chat` endpoint and verifying that the response contains a relevant, content-grounded answer.

**Acceptance Scenarios**:

1. **Given** a running FastAPI server with a configured AI Agent,
**When** a user sends a question like "What is ROS 2?" to the `/chat` endpoint,
**Then** the system returns a successful JSON response containing an answer generated from the content in the Qdrant vector store.

---

### User Story 2 - Continue a Conversation (Priority: P2)

A user asks a follow-up question, and the agent uses the previous conversation history to provide a context-aware response.

**Why this priority**: Contextual conversations make the chatbot more useful and natural to interact with.

**Independent Test**: Can be tested by sending a series of requests to the `/chat` endpoint with a persistent `session_id`.

**Acceptance Scenarios**:

1. **Given** a user has already asked "What is ROS 2?" in a specific chat session,
**When** the user sends a follow-up question like "Tell me more about the middleware" within the same session,
**Then** the system returns an answer that correctly uses the context of "ROS 2" from the previous turn.

---

### User Story 3 - Query with Selected Text (Priority: P2)

A user highlights a specific piece of text in the book's frontend and asks a question about it.

**Why this priority**: This provides a powerful way for users to get targeted explanations for specific passages.

**Independent Test**: Can be tested by sending a POST request to a dedicated endpoint (e.g., `/chat/contextual`) that includes both a question and the selected text.

**Acceptance Scenarios**:

1. **Given** a running FastAPI server,
**When** a user sends a question "What does this mean?" along with a payload containing the selected text "DDS is a middleware standard...",
**Then** the system returns an answer that specifically explains the provided text snippet.

### Edge Cases

- What happens if the user's question has no relevant content in the vector store? The agent should respond gracefully, stating it could not find an answer in the book.
- How does the system handle an invalid API key for OpenAI? The system should return a 500-level error with a clear log message.
- What if the Qdrant or Neon database is unavailable? The API should return a service unavailable (503) error.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: The system MUST provide a FastAPI backend server.
- **FR-002**: The system MUST expose a `/chat` endpoint that accepts a user's question and optionally a session ID.
- **FR-003**: The system MUST use the OpenAI Agents/ChatKit SDK to build the core AI agent logic.
- **FR-004**: The agent MUST query the project's Qdrant Cloud collection to find context relevant to the user's question.
- **FR-005**: The agent MUST use the retrieved context to generate a text-based answer.
- **FR-006**: The system MUST connect to a Neon Serverless Postgres database.
- **FR-007**: The system MUST persist chat messages (user questions and agent answers) and associate them with a session in the Postgres database.
- **FR-008**: The agent MUST use chat history from the current session to inform its answers to follow-up questions.
- **FR-009**: The system MUST expose an endpoint (e.g., `/chat/contextual`) that allows a user to submit selected text along with their question for a more targeted answer.
- **FR-010**: All external service credentials (OpenAI, Qdrant, Neon) MUST be loaded from environment variables.

### Key Entities *(include if feature involves data)*

- **ChatSession**: Represents a single, continuous conversation. (Attributes: `session_id`, `created_at`).
- **ChatMessage**: Represents a single turn within a ChatSession. (Attributes: `message_id`, `session_id`, `role` (user or assistant), `content`, `timestamp`).
- **RetrievedContext**: Represents a chunk of text retrieved from Qdrant to be used for generating an answer.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: The `/chat` endpoint successfully returns a non-empty, relevant answer for 95% of test queries known to have context in the book.
- **SC-002**: End-to-end response time for a standard query (question -> retrieval -> answer) is under 5 seconds on average.
- **SC-003**: All chat interactions are successfully logged to the Neon Postgres database with the correct session ID.
- **SC-004**: When provided with selected text, the agent's answer directly addresses the context of that text.