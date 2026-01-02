# Quickstart: Frontend RAG Integration

**Date**: 2026-01-02
**Feature**: `014-frontend-rag-integration`

This guide provides instructions on how to run the full end-to-end RAG chatbot application locally, including both the backend and the new frontend UI.

## 1. Prerequisites

- All prerequisites from the `013-rag-agent-backend` quickstart guide must be met.
- You must have a correctly configured `.env` file in the `backend/` directory with all API keys and URLs.
- A recommended tool for serving the frontend is the "Live Server" extension for Visual Studio Code, or a similar tool.

## 2. Running the Application End-to-End

To run the full application, you will need to start two separate processes in two different terminals.

### Terminal 1: Start the Backend Server

1.  Navigate to the project root directory in your terminal.
2.  Run the FastAPI backend server using the `uvicorn` command:
    ```bash
    uvicorn backend.agent.main:app --reload --port 8000
    ```
3.  The backend server should start and be listening on `http://127.0.0.1:8000`. Keep this terminal window open.

### Terminal 2: Start the Frontend Server

1.  Open the project in your code editor (e.g., VS Code with the "Live Server" extension installed).
2.  Navigate to the `frontend/` directory.
3.  Right-click on the `index.html` file and select "Open with Live Server".
4.  This will open a new browser tab, typically at an address like `http://127.0.0.1:5500`.

## 3. Using the Chatbot

- With both the backend and frontend servers running, you can now use the chatbot.
- Open the browser tab with the frontend UI (`http://127.0.0.1:5500`).
- Type a question into the input box and press Enter or click the "Send" button.
- The frontend will send the request to the backend, and the agent's response should appear in the chat window.
