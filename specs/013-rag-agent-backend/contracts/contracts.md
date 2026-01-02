# API Contracts: RAG Agent Backend

**Date**: 2026-01-02
**Feature**: `013-rag-agent-backend`

This document defines the API data contracts for the FastAPI backend. These will be implemented as Pydantic models to ensure type safety and generate OpenAPI documentation automatically.

---

## 1. `/chat` Endpoint

This is the primary endpoint for interacting with the chatbot.

**Method**: `POST`
**URL**: `/chat`

### Request Body

The request will be a JSON object with the following structure.

**Pydantic Model: `ChatRequest`**
```python
from pydantic import BaseModel
from typing import Optional

class ChatRequest(BaseModel):
    question: str
    session_id: Optional[str] = None
```

- **`question`** (string, required): The user's query.
- **`session_id`** (string, optional): A unique identifier for the conversation. If not provided, a new session may be initiated.

### Response Body

The response will be a JSON object containing the agent's answer.

**Pydantic Model: `ChatResponse`**
```python
from pydantic import BaseModel

class ChatResponse(BaseModel):
    answer: str
    session_id: str
```

- **`answer`** (string): The agent's generated response.
- **`session_id`** (string): The unique identifier for the conversation, returned so the client can continue the conversation.

---

## 2. `/chat/contextual` Endpoint

This endpoint handles questions that are based on user-selected text.

**Method**: `POST`
**URL**: `/chat/contextual`

### Request Body

The request will be a JSON object containing the question and the selected text context.

**Pydantic Model: `ContextualChatRequest`**
```python
from pydantic import BaseModel
from typing import Optional

class ContextualChatRequest(BaseModel):
    question: str
    context: str
    session_id: Optional[str] = None
```

- **`question`** (string, required): The user's query about the context.
- **`context`** (string, required): The piece of text the user has selected.
- **`session_id`** (string, optional): The conversation session ID.

### Response Body

The response format is identical to the standard `/chat` endpoint.

**Pydantic Model: `ChatResponse`**
```python
class ChatResponse(BaseModel):
    answer: str
    session_id: str
```

---
