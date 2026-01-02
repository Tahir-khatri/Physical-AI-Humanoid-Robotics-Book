from fastapi import FastAPI
from pydantic import BaseModel
from typing import Optional
from uuid import uuid4 # Added import
from .agent import run_agent_turn # Added import
from .database import init_db, get_or_create_session, get_chat_history, add_message_to_history, ChatMessage # Added imports and ChatMessage

# Pydantic Models for API Requests and Responses (from contracts/contracts.md)
class ChatRequest(BaseModel):
    question: str
    session_id: Optional[str] = None

class ChatResponse(BaseModel):
    answer: str
    session_id: str

class ContextualChatRequest(BaseModel):
    question: str
    context: str
    session_id: Optional[str] = None

# FastAPI app instance
app = FastAPI()

# Initialize the database on startup
@app.on_event("startup")
def startup_event():
    init_db()

@app.get("/")
async def health_check():
    """
    Health check endpoint.
    """
    return {"status": "ok"}

@app.post("/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest) -> ChatResponse:
    """
    Handles chat requests, processes user questions with the AI agent,
    and manages conversation history.
    """
    logging.info(f"Received /chat request: question='{request.question[:50]}...', session_id='{request.session_id}'")
    session_id_str = request.session_id if request.session_id else str(uuid4())
    session_obj = get_or_create_session(session_id_str)
    logging.info(f"Processing chat for session: {session_id_str}")

    # Get chat history
    chat_messages = get_chat_history(session_obj.session_id)
    formatted_history = [{"role": msg.role, "content": msg.content} for msg in chat_messages]

    # Add user's current message to history for agent
    formatted_history.append({"role": "user", "content": request.question})

    # Run agent turn with history
    agent_answer = run_agent_turn(request.question, formatted_history)

    # Save user message and agent response
    add_message_to_history(session_obj.session_id, "user", request.question)
    add_message_to_history(session_obj.session_id, "assistant", agent_answer)

    return ChatResponse(answer=agent_answer, session_id=session_id_str)

@app.post("/chat/contextual", response_model=ChatResponse)
async def contextual_chat_endpoint(request: ContextualChatRequest) -> ChatResponse:
    """
    Handles contextual chat requests, processing user questions with provided
    text context and managing conversation history.
    """
    logging.info(f"Received /chat/contextual request: question='{request.question[:50]}...', context='{request.context[:50]}...', session_id='{request.session_id}'")
    session_id_str = request.session_id if request.session_id else str(uuid4())
    session_obj = get_or_create_session(session_id_str)
    logging.info(f"Processing contextual chat for session: {session_id_str}")

    # Get chat history
    chat_messages = get_chat_history(session_obj.session_id)
    formatted_history = [{"role": msg.role, "content": msg.content} for msg in chat_messages]

    # Prepend context to the user's question
    contextual_question = f"Based on this text: '{request.context}', answer the question: '{request.question}'"
    formatted_history.append({"role": "user", "content": contextual_question})

    # Run agent turn with history
    agent_answer = run_agent_turn(contextual_question, formatted_history)

    # Save user message and agent response
    add_message_to_history(session_obj.session_id, "user", contextual_question)
    add_message_to_history(session_obj.session_id, "assistant", agent_answer)

    return ChatResponse(answer=agent_answer, session_id=session_id_str)

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)

