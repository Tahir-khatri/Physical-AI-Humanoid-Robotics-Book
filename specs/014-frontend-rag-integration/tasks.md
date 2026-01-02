# Tasks: Frontend RAG Integration

**Branch**: `014-frontend-rag-integration` | **Date**: 2026-01-02 | **Plan**: [plan.md](./plan.md)

This document breaks down the implementation plan for integrating the RAG backend with a frontend UI.

## Phase 1: Backend CORS Configuration

- [ ] T001 In `backend/agent/main.py`, import `CORSMiddleware` from `fastapi.middleware.cors`.
- [ ] T002 In `backend/agent/main.py`, add the `CORSMiddleware` to the FastAPI app instance to allow requests from local development server origins.

## Phase 2: Frontend UI Structure

- [ ] T003 Create the `frontend/` directory in the project root.
- [ ] T004 Create `frontend/index.html` with a basic structure for the chat interface (a chat container, a form with a text input, and a send button).
- [ ] T005 [P] Create `frontend/style.css` and add minimal CSS for a functional chat layout, including distinct styles for user and assistant messages.

## Phase 3: User Story 1 - Ask a Question from Frontend (MVP)

**Goal**: A user can send a question from the UI and see a response.

- [ ] T006 [US1] Create `frontend/app.js` to contain the frontend logic.
- [ ] T007 [US1] In `frontend/app.js`, add an event listener to the chat form to prevent default submission and capture the user's input text.
- [ ] T008 [US1] In `frontend/app.js`, implement a function `displayMessage(sender, text)` that creates and appends a new message div to the chat container.
- [ ] T009 [US1] In `frontend/app.js`, implement a function `sendMessageToBackend(question)` that uses the `fetch` API to send a POST request to `http://127.0.0.1:8000/chat` with the user's question.
- [ ] T010 [US1] In `frontend/app.js`, connect the form's event listener to call `displayMessage` for the user's query and then call `sendMessageToBackend`, displaying the assistant's response upon success.

## Phase 4: User Story 2 - Continue a Conversation from Frontend

**Goal**: The frontend can maintain a conversation across multiple turns.

- [ ] T011 [US2] In `frontend/app.js`, declare a global variable `currentSessionId` initialized to `null`.
- [ ] T012 [US2] In `frontend/app.js`, modify `sendMessageToBackend` to accept the `sessionId` and include it in the request payload if it exists.
- [ ] T013 [US2] In `frontend/app.js`, update the response handling logic to extract the `session_id` from the backend's response and store it in the `currentSessionId` variable.

## Phase 5: User Story 3 - Contextual Query from Frontend

**Goal**: The frontend can send a question with additional context.

- [ ] T014 [US3] In `frontend/index.html`, add a `<textarea>` element for the user to paste contextual text.
- [ ] T015 [US3] In `frontend/app.js`, modify the `sendMessageToBackend` function to check if the context textarea has content. If it does, the function should send the request to the `/chat/contextual` endpoint and include the context in the payload.

## Phase 6: Polish

- [ ] T016 [P] In `frontend/app.js`, add `try...catch` blocks to the `fetch` call to handle network errors and display a user-friendly error message in the chat UI.
- [ ] T017 [P] In `frontend/app.js`, add comments explaining the core functions.

## Dependencies

```mermaid
graph TD
    subgraph Phase 1 & 2
        direction LR
        A[Backend CORS] --> B[Frontend UI];
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
    D --> E;
    E --> F;
```

## Implementation Strategy

The implementation will begin by configuring the backend for CORS, then building the static frontend structure. The core MVP will be achieved by implementing the basic chat functionality (US1). Subsequent phases will layer on conversation history and contextual queries.
