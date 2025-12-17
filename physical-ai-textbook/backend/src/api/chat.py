"""
Chat API endpoints for the Physical AI Textbook Platform
"""
import logging
from fastapi import APIRouter, HTTPException, Depends
from typing import List
from sqlalchemy.orm import Session
from pydantic import BaseModel
import os
from dotenv import load_dotenv
from sentence_transformers import SentenceTransformer
from qdrant_client import QdrantClient
from qdrant_client.http import models
from datetime import datetime
import uuid

from src.db.connection import get_db
from src.auth.session import get_current_user
from src.auth.models_db import User

# Import the LLM service
from src.services.llm_service import llm_service

# Configure logging
logger = logging.getLogger(__name__)

# Load environment variables
load_dotenv()

router = APIRouter()

# Initialize the embedding model and Qdrant client
model = SentenceTransformer('intfloat/e5-base-v2')

qdrant_url = os.getenv("QDRANT_URL")
qdrant_api_key = os.getenv("QDRANT_API_KEY")

if qdrant_url:
    # Use cloud instance
    client = QdrantClient(
        url=qdrant_url,
        api_key=qdrant_api_key,
    )
else:
    # Use local instance
    client = QdrantClient(host="localhost", port=6333)

COLLECTION_NAME = "textbook_content"

# Pydantic models for chat requests and responses
class ChatRequest(BaseModel):
    message: str
    session_id: str = None
    user_id: int = None

class ChatResponse(BaseModel):
    response: str
    message_id: str
    sources: List[str] = []
    timestamp: str

class HealthResponse(BaseModel):
    status: str
    timestamp: str
    version: str

@router.post("/", response_model=ChatResponse)
async def chat_with_bot(request: ChatRequest, current_user: User = Depends(get_current_user), db: Session = Depends(get_db)):
    """
    Chat with the RAG bot
    """
    try:
        # Prefix the query with "query: " for the e5-base-v2 model
        prefixed_query = f"query: {request.message}"

        # Generate embedding for the query
        query_embedding = model.encode([prefixed_query])[0].tolist()

        # Search in Qdrant for relevant content
        search_results = client.search(
            collection_name=COLLECTION_NAME,
            query_vector=query_embedding,
            limit=5  # Return top 5 results
        )

        # Format results
        sources = []
        context_parts = []
        for hit in search_results:
            text = hit.payload.get("text", "")
            metadata = hit.payload.get("metadata", {})
            source_file = metadata.get("source_file", "unknown")

            context_parts.append(text)
            sources.append(f"/docs/{source_file}" if not source_file.startswith('/docs/') else source_file)

        # Create a response based on the retrieved content using the LLM
        if context_parts:
            context = "\n\n".join(context_parts[:3])  # Use top 3 results

            # Use the LLM service to generate a comprehensive response
            try:
                response = llm_service.generate_tutor_response(
                    question=request.message,
                    context=context,
                    sources=sources
                )
            except Exception as llm_error:
                # Fallback to the original response format if LLM service fails
                logger.error(f"Error calling LLM service: {str(llm_error)}")
                response = f"Based on the textbook content, here's what I found regarding '{request.message}':\n\n{context[:500]}..."  # Limit response length
        else:
            response = f"I couldn't find specific information about '{request.message}' in the textbook. Please check if your question relates to Physical AI & Humanoid Robotics topics covered in the textbook."

        # Generate response
        chat_response = ChatResponse(
            response=response,
            message_id=str(uuid.uuid4()),
            sources=list(set(sources)),  # Remove duplicates
            timestamp=datetime.utcnow().isoformat()
        )

        return chat_response
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing chat request: {str(e)}")

@router.get("/health", response_model=HealthResponse)
async def chat_health():
    """
    Health check for chatbot service
    """
    try:
        # Try to access the Qdrant collection to verify connectivity
        client.get_collection(COLLECTION_NAME)

        return HealthResponse(
            status="healthy",
            timestamp=datetime.utcnow().isoformat(),
            version="1.0.0"
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Chat service health check failed: {str(e)}")

@router.get("/session/{session_id}")
async def get_chat_session(session_id: str, current_user: User = Depends(get_current_user)):
    """
    Get a specific chat session
    """
    # Placeholder implementation - in a full implementation, this would retrieve chat history
    return {"session_id": session_id, "messages": []}

@router.delete("/session/{session_id}")
async def delete_chat_session(session_id: str, current_user: User = Depends(get_current_user)):
    """
    Delete a specific chat session
    """
    # Placeholder implementation - in a full implementation, this would delete chat history
    return {"message": f"Session {session_id} deleted successfully"}