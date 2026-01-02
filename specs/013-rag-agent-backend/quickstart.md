# Quickstart: RAG Agent Backend

**Date**: 2026-01-02
**Feature**: `013-rag-agent-backend`

This guide provides instructions on how to set up and run the RAG Agent Backend.

## 1. Prerequisites

- Python 3.10+
- `uv` Python package manager installed.
- A running Qdrant Cloud instance, populated with data from the ingestion pipeline (`011-rag-content-pipeline`).
- A provisioned Neon Serverless Postgres database.
- An OpenAI API key.

## 2. Setup

### a. Create the Environment File

The backend requires several API keys and configuration settings, which are loaded from a `.env` file in the `backend/` directory.

1.  Navigate to the `backend/` directory.
2.  Create a file named `.env`.
3.  Copy the contents of the example below and paste them into your `.env` file, replacing the placeholder values with your actual credentials.

**.env file contents:**
```env
# OpenAI API Key
OPENAI_API_KEY="your_openai_api_key_here"

# Qdrant Configuration
QDRANT_URL="your_qdrant_cloud_url_here"
QDRANT_API_KEY="your_qdrant_api_key_here"
QDRANT_COLLECTION_NAME="physical-ai-book-v1"

# Neon Serverless Postgres Connection URL
NEON_DATABASE_URL="postgres://user:password@host:port/dbname"

# Cohere API Key (for embedding search queries)
COHERE_API_KEY="your_cohere_api_key_here"
```

### b. Install Dependencies

The project uses `uv` to manage dependencies.

1.  Open your terminal in the `backend/` directory.
2.  Activate the `uv` environment and install the required packages.
    ```bash
    uv pip install fastapi uvicorn[standard] openai "psycopg2-binary"
    ```

## 3. Running the Backend Server

Once the setup is complete, you can run the FastAPI server.

1.  Make sure you are in the project root directory.
2.  Execute the server using `uvicorn`:
    ```bash
    uvicorn backend.agent.main:app --reload
    ```
3.  The server will start, typically on `http://127.0.0.1:8000`. You will see log output in your terminal confirming the server is running.

## 4. Testing the API

You can interact with the API using tools like `curl` or the automatic interactive documentation provided by FastAPI.

### a. Via Interactive Docs

Open your web browser and navigate to `http://127.0.0.1:8000/docs`. You will find an interactive OpenAPI interface where you can test the `/chat` and `/chat/contextual` endpoints directly.

### b. Via `curl`

**Ask a standard question:**
```bash
curl -X POST "http://127.0.0.1:8000/chat" \
-H "Content-Type: application/json" \
-d 
{
  "question": "What is the purpose of the ROS 2 middleware layer?"
}
```

**Ask a follow-up question (using the `session_id` from the first response):**
```bash
curl -X POST "http://127.0.0.1:8000/chat" \
-H "Content-Type: application/json" \
-d 
{
  "question": "Tell me more about it",
  "session_id": "the_session_id_from_the_previous_response"
}
```
