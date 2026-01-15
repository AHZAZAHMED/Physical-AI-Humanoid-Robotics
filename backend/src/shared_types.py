"""
Shared types and interfaces for request/response payloads
between the frontend chatbot UI and backend RAG service.
"""
from typing import List, Optional, Dict, Any
from pydantic import BaseModel
from datetime import datetime


class Source(BaseModel):
    """Represents a source citation from the RAG system."""
    content: str
    metadata: Dict[str, Any]
    similarityScore: Optional[float] = None
    bookTitle: Optional[str] = None


class ChatRequest(BaseModel):
    """Represents a request from the frontend to the backend."""
    query: str
    sessionId: Optional[str] = None
    userId: Optional[str] = None


class ChatResponse(BaseModel):
    """Represents a response from the backend to the frontend."""
    response: str
    sources: List[Source] = []
    sessionId: str
    error: Optional[Dict[str, Any]] = None
    timestamp: str = datetime.now().isoformat()


class ErrorResponse(BaseModel):
    """Represents an error response from the backend."""
    message: str
    type: Optional[str] = None
    details: Optional[Dict[str, Any]] = None


class HealthResponse(BaseModel):
    """Represents the health status of the backend service."""
    status: str  # "healthy" | "unhealthy"
    timestamp: str = datetime.now().isoformat()
    services: Optional[Dict[str, str]] = None  # Service status mapping