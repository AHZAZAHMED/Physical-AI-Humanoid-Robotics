"""
Chat service for the Physical AI Textbook Platform
Handles conversation logic and chat history management
"""
import logging
from typing import List, Dict, Any, Optional
from datetime import datetime
import uuid
from sqlalchemy.orm import Session

from src.auth.models_db import User
from src.rag.retriever import get_relevant_content
from src.db.connection import get_db

# Import the LLM service
from src.services.llm_service import llm_service

# Configure logging
logger = logging.getLogger(__name__)


class ChatMessage:
    """
    Represents a single chat message
    """
    def __init__(self, role: str, content: str, timestamp: datetime = None):
        self.id = str(uuid.uuid4())
        self.role = role  # 'user' or 'assistant'
        self.content = content
        self.timestamp = timestamp or datetime.utcnow()


class ChatSession:
    """
    Represents a chat session with history
    """
    def __init__(self, session_id: str = None, user_id: int = None):
        self.session_id = session_id or str(uuid.uuid4())
        self.user_id = user_id
        self.messages: List[ChatMessage] = []
        self.created_at = datetime.utcnow()
        self.updated_at = datetime.utcnow()

    def add_message(self, role: str, content: str):
        """
        Add a message to the session
        """
        message = ChatMessage(role, content)
        self.messages.append(message)
        self.updated_at = datetime.utcnow()

    def get_history(self, limit: int = 10) -> List[Dict[str, Any]]:
        """
        Get chat history with limited messages
        """
        return [
            {
                "id": msg.id,
                "role": msg.role,
                "content": msg.content,
                "timestamp": msg.timestamp.isoformat()
            }
            for msg in self.messages[-limit:]
        ]


class ChatService:
    """
    Service class for chat-related operations
    """

    def __init__(self):
        self.sessions: Dict[str, ChatSession] = {}

    def create_session(self, user_id: int = None) -> ChatSession:
        """
        Create a new chat session
        """
        session = ChatSession(user_id=user_id)
        self.sessions[session.session_id] = session
        return session

    def get_session(self, session_id: str) -> Optional[ChatSession]:
        """
        Get an existing chat session
        """
        return self.sessions.get(session_id)

    def add_user_message(self, session_id: str, content: str) -> ChatSession:
        """
        Add a user message to the session
        """
        session = self.get_session(session_id)
        if not session:
            raise ValueError(f"Session {session_id} not found")

        session.add_message("user", content)
        return session

    def generate_response(self, session_id: str, user_message: str, top_k: int = 5) -> str:
        """
        Generate a response based on the user message and conversation context
        """
        session = self.get_session(session_id)
        if not session:
            raise ValueError(f"Session {session_id} not found")

        # Add the user's message to the session
        session.add_message("user", user_message)

        # Retrieve relevant content from the RAG system
        try:
            relevant_chunks = get_relevant_content(user_message, top_k=top_k)
        except Exception as e:
            # If RAG retrieval fails, return an error message
            error_response = f"Sorry, I encountered an error retrieving information: {str(e)}"
            session.add_message("assistant", error_response)
            return error_response

        # Get conversation history for context
        conversation_history = self.get_conversation_context(session_id, num_messages=5)

        # If we have relevant content, use the LLM to generate a contextual response
        if relevant_chunks:
            context = "\n\n".join([chunk["text"] for chunk in relevant_chunks[:3]])  # Use top 3 results

            # Use the LLM service to generate a comprehensive response
            try:
                response = llm_service.generate_response(
                    prompt=user_message,
                    context=context,
                    conversation_history=conversation_history
                )
            except Exception as llm_error:
                # Fallback to the original response format if LLM service fails
                logger.error(f"Error calling LLM service: {str(llm_error)}")
                response = f"Based on the textbook content, here's what I found regarding '{user_message}':\n\n{context[:1000]}..."  # Limit response length
        else:
            response = f"I couldn't find specific information about '{user_message}' in the textbook. Please check if your question relates to Physical AI & Humanoid Robotics topics covered in the textbook."

        # Add the assistant's response to the session
        session.add_message("assistant", response)

        return response

    def get_conversation_context(self, session_id: str, num_messages: int = 5) -> List[Dict[str, str]]:
        """
        Get the recent conversation context for providing to the LLM
        """
        session = self.get_session(session_id)
        if not session:
            return []

        # Get the most recent messages
        recent_messages = session.messages[-num_messages:]

        return [
            {
                "role": msg.role,
                "content": msg.content
            }
            for msg in recent_messages
        ]

    def clear_session(self, session_id: str) -> bool:
        """
        Clear a chat session
        """
        if session_id in self.sessions:
            del self.sessions[session_id]
            return True
        return False

    def get_session_history(self, session_id: str, limit: int = 10) -> List[Dict[str, Any]]:
        """
        Get the full history of a session
        """
        session = self.get_session(session_id)
        if not session:
            return []

        return session.get_history(limit=limit)


# Global instance for use in other modules
chat_service = ChatService()


def process_chat_message(session_id: str, user_message: str, user_id: int = None) -> str:
    """
    Process a chat message and return the response
    """
    # Get or create session
    session = chat_service.get_session(session_id)
    if not session:
        session = chat_service.create_session(user_id)

    # Generate and return response
    return chat_service.generate_response(session_id, user_message)


def create_new_chat_session(user_id: int = None) -> str:
    """
    Create a new chat session and return its ID
    """
    session = chat_service.create_session(user_id)
    return session.session_id


def get_chat_history(session_id: str, limit: int = 10) -> List[Dict[str, Any]]:
    """
    Get chat history for a session
    """
    return chat_service.get_session_history(session_id, limit)