# Data Model: RAG Agent Backend

**Date**: 2026-01-02
**Feature**: `013-rag-agent-backend`

This document defines the database schema for storing chat sessions and messages in the Neon Serverless Postgres database.

## Schema Overview

Two tables will be created to manage conversational history: `chat_sessions` and `chat_messages`.

---

## Table: `chat_sessions`

This table stores a record for each unique conversation session.

### Columns

| Column Name  | Data Type | Constraints      | Description                               |
|--------------|-----------|------------------|-------------------------------------------|
| `id`         | `SERIAL`  | `PRIMARY KEY`    | Auto-incrementing unique identifier.      |
| `session_id` | `VARCHAR(255)` | `UNIQUE NOT NULL` | A unique string (e.g., UUID) identifying the session, provided by the client. |
| `created_at` | `TIMESTAMP WITH TIME ZONE` | `DEFAULT NOW()` | The timestamp when the session was created. |

### Example

| id | session_id                             | created_at                  |
|----|----------------------------------------|-----------------------------|
| 1  | "abc-123-def-456"                      | "2026-01-02 10:00:00+00"    |

---

## Table: `chat_messages`

This table stores every individual message from both the user and the assistant for each session.

### Columns

| Column Name  | Data Type | Constraints      | Description                               |
|--------------|-----------|------------------|-------------------------------------------|
| `id`         | `SERIAL`  | `PRIMARY KEY`    | Auto-incrementing unique identifier.      |
| `session_id` | `INTEGER` | `REFERENCES chat_sessions(id)` | Foreign key linking to the `chat_sessions` table. |
| `role`       | `VARCHAR(20)` | `NOT NULL`       | The role of the message sender, either 'user' or 'assistant'. |
| `content`    | `TEXT`    | `NOT NULL`       | The text content of the message.        |
| `created_at` | `TIMESTAMP WITH TIME ZONE` | `DEFAULT NOW()` | The timestamp when the message was saved. |

### Example

| id | session_id | role      | content                    | created_at                  |
|----|------------|-----------|----------------------------|-----------------------------|
| 1  | 1          | "user"    | "What is ROS 2?"           | "2026-01-02 10:00:05+00"    |
| 2  | 1          | "assistant" | "ROS 2 is a set of..."     | "2026-01-02 10:00:08+00"    |
| 3  | 1          | "user"    | "Tell me more about DDS."  | "2026-01-02 10:01:15+00"    |

---

## Relationships

- A `chat_session` can have many `chat_messages`.
- Each `chat_message` belongs to exactly one `chat_session`.
- The relationship is enforced by the `session_id` foreign key in the `chat_messages` table.
