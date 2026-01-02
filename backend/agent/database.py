import os
import logging
from datetime import datetime

from sqlalchemy import create_engine, Column, Integer, String, Text, DateTime, ForeignKey
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker, relationship
from sqlalchemy.sql import func

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# Database connection URL from environment variable
DATABASE_URL = os.getenv("NEON_DATABASE_URL")

# Ensure DATABASE_URL is set
if DATABASE_URL is None:
    raise ValueError("NEON_DATABASE_URL environment variable is not set.")

# SQLAlchemy setup
Base = declarative_base()
engine = create_engine(DATABASE_URL)
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

class ChatSession(Base):
    __tablename__ = "chat_sessions"

    id = Column(Integer, primary_key=True, index=True)
    session_id = Column(String, unique=True, nullable=False)
    created_at = Column(DateTime(timezone=True), server_default=func.now())

    messages = relationship("ChatMessage", back_populates="session")

class ChatMessage(Base):
    __tablename__ = "chat_messages"

    id = Column(Integer, primary_key=True, index=True)
    session_id = Column(Integer, ForeignKey("chat_sessions.id"))
    role = Column(String, nullable=False) # 'user' or 'assistant'
    content = Column(Text, nullable=False)
    created_at = Column(DateTime(timezone=True), server_default=func.now())

    session = relationship("ChatSession", back_populates="messages")

def init_db():
    """Initializes the database and creates tables if they don't exist."""
    logging.info("Initializing database and creating tables...")
    Base.metadata.create_all(bind=engine)
    logging.info("Database initialization complete.")

def get_or_create_session(session_id: str) -> ChatSession:
    """
    Retrieves an existing chat session or creates a new one if it doesn't exist.

    Args:
        session_id (str): The unique identifier for the chat session.

    Returns:
        ChatSession: The retrieved or newly created ChatSession object.
    """
    db = SessionLocal()
    session = db.query(ChatSession).filter(ChatSession.session_id == session_id).first()
    if not session:
        session = ChatSession(session_id=session_id)
        db.add(session)
        db.commit()
        db.refresh(session)
        logging.info(f"Created new chat session: {session_id}")
    db.close()
    return session

def get_chat_history(session_id: str) -> List[ChatMessage]:
    """
    Retrieves all chat messages for a given session, ordered by creation time.

    Args:
        session_id (str): The unique identifier for the chat session.

    Returns:
        List[ChatMessage]: A list of ChatMessage objects for the session.
    """
    db = SessionLocal()
    session = db.query(ChatSession).filter(ChatSession.session_id == session_id).first()
    if session:
        messages = db.query(ChatMessage).filter(ChatMessage.session_id == session.id).order_by(ChatMessage.created_at).all()
        db.close()
        return messages
    db.close()
    return []

def add_message_to_history(session_id: str, role: str, content: str):
    """
    Adds a new message to the chat history for a given session.

    Args:
        session_id (str): The unique identifier for the chat session.
        role (str): The role of the message sender ('user' or 'assistant').
        content (str): The text content of the message.
    """
    db = SessionLocal()
    session = db.query(ChatSession).filter(ChatSession.session_id == session_id).first()
    if session:
        message = ChatMessage(session_id=session.id, role=role, content=content)
        db.add(message)
        db.commit()
        db.refresh(message)
        logging.info(f"Added message to session {session_id}: {role} - {content[:50]}...")
    db.close()




