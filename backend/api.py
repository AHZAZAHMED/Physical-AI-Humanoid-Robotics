"""
FastAPI application for RAG Chatbot integration.

This module exposes endpoints for the frontend chatbot UI to communicate with
the RAG backend service.
"""
from fastapi import FastAPI, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware
from starlette.middleware.trustedhost import TrustedHostMiddleware
import uvicorn
import logging
import time
import os
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

# Import our custom modules
from src.shared_types import ChatRequest, ChatResponse, ErrorResponse
from src.agent_connector import AgentConnector
from src.request_validator import RequestValidator

# Create global instances
agent_connector = AgentConnector()
request_validator = RequestValidator()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Create FastAPI app instance
app = FastAPI(
    title="RAG Chatbot API",
    description="API for integrating RAG backend with frontend chatbot UI",
    version="1.0.0"
)

# Add security headers via middleware
@app.middleware("http")
async def add_security_headers(request, call_next):
    response = await call_next(request)
    response.headers["X-Content-Type-Options"] = "nosniff"
    response.headers["X-Frame-Options"] = "DENY"
    response.headers["X-XSS-Protection"] = "1; mode=block"
    response.headers["Strict-Transport-Security"] = "max-age=31536000; includeSubDomains"
    return response

# Add CORS middleware to allow frontend communication
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/")
async def root():
    """Root endpoint for basic health check."""
    return {"message": "RAG Chatbot API is running"}


@app.post("/api/chat", response_model=ChatResponse)
async def chat_endpoint(chat_request: ChatRequest):
    """
    Process a user query and return a RAG-enhanced response.

    Args:
        chat_request: The chat request containing the query and optional session info

    Returns:
        ChatResponse with the AI-generated response and sources
    """
    try:
        # Validate the request
        is_valid, error_response = request_validator.validate_chat_request(chat_request)
        if not is_valid:
            raise HTTPException(status_code=400, detail=error_response.message)

        # Sanitize the request
        sanitized_request = request_validator.sanitize_chat_request(chat_request)

        # Process the request through the agent connector
        response = agent_connector.process_chat_request(sanitized_request)

        return response

    except HTTPException:
        # Re-raise HTTP exceptions as they are
        raise
    except Exception as e:
        logger.error(f"Error in chat endpoint: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=500,
            detail=f"Internal server error: {str(e)}"
        )


@app.middleware("http")
async def add_process_time_header(request, call_next):
    """Middleware to add response time header for performance monitoring."""
    start_time = time.time()
    response = await call_next(request)
    process_time = time.time() - start_time
    response.headers["X-Process-Time"] = str(process_time)
    return response


@app.get("/api/health")
async def health_check():
    """Health check endpoint to verify backend service availability."""
    return {
        "status": "healthy",
        "timestamp": __import__('datetime').datetime.now().isoformat()
    }


if __name__ == "__main__":
    # Run the application with uvicorn for local development
    uvicorn.run(app, host="0.0.0.0", port=8000)