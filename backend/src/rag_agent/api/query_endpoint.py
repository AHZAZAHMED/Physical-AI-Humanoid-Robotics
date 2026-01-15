"""
API endpoint for processing user queries through the RAG pipeline.
"""
from fastapi import APIRouter, HTTPException, Depends, Request
from typing import Dict, Any, Optional
import time
import uuid
import html
import re
from ..agent import RAGAgent
from ..config import AgentConfiguration
from ..utils.logging import rag_logger


# Create API router
router = APIRouter()

# Global agent instance (in production, you'd want dependency injection)
agent: Optional[RAGAgent] = None


def get_agent():
    """
    Get the RAG agent instance, creating it if needed.
    """
    global agent
    if agent is None:
        config = AgentConfiguration()
        agent = RAGAgent(config)
    return agent


def sanitize_input(input_str: str) -> str:
    """
    Sanitize input string to prevent injection attacks.

    Args:
        input_str: Input string to sanitize

    Returns:
        Sanitized string
    """
    if not input_str:
        return input_str

    # Remove or escape potentially dangerous characters
    # HTML escape to prevent XSS
    sanitized = html.escape(input_str)

    # Remove control characters except common whitespace
    sanitized = re.sub(r'[\x00-\x08\x0B\x0C\x0E-\x1F\x7F]', '', sanitized)

    return sanitized


@router.post("/api/v1/query")
async def process_query(
    request: Request,
    query: str = None,
    parameters: Dict[str, Any] = None
):
    """
    Process a user query through the RAG pipeline and return a grounded response.
    """
    # Extract query and parameters from request body
    try:
        body = await request.json()

        # Validate request body structure
        if not isinstance(body, dict):
            raise HTTPException(status_code=400, detail="Request body must be a JSON object")

        query = body.get("query")
        parameters = body.get("parameters", {})

        # Additional validation to ensure parameters is a dict
        if not isinstance(parameters, dict):
            raise HTTPException(status_code=400, detail="Parameters must be a JSON object")

    except ValueError:  # catches JSON decode errors
        rag_logger.error("Error parsing request body: Invalid JSON")
        raise HTTPException(status_code=400, detail="Invalid JSON format in request body")
    except Exception as e:
        rag_logger.error(f"Error parsing request body: {str(e)}")
        raise HTTPException(status_code=400, detail="Invalid request body format")

    # Validate query exists
    if query is None:
        raise HTTPException(status_code=400, detail="Query parameter is required")

    # Validate query is a string
    if not isinstance(query, str):
        raise HTTPException(status_code=400, detail="Query must be a string")

    # Sanitize the query to prevent injection attacks
    query = sanitize_input(query)

    # Validate query is not empty after sanitization
    if not query or len(query.strip()) == 0:
        raise HTTPException(status_code=400, detail="Query parameter cannot be empty or whitespace only")

    # Validate query length against configuration
    config = AgentConfiguration()
    if len(query) > config.max_query_length:
        raise HTTPException(
            status_code=400,
            detail=f"Query must be less than {config.max_query_length} characters"
        )

    # Validate parameters if provided with more security
    if parameters:
        # Limit the number of parameters to prevent parameter flooding
        if len(parameters) > 10:
            raise HTTPException(status_code=400, detail="Too many parameters provided")

        # Validate each parameter individually
        for key, value in parameters.items():
            # Only allow specific parameter keys
            allowed_keys = {"top_k", "confidence_threshold", "include_metadata"}
            if key not in allowed_keys:
                raise HTTPException(
                    status_code=400,
                    detail=f"Parameter '{key}' is not allowed. Allowed parameters: {', '.join(allowed_keys)}"
                )

            # Validate each parameter value based on its key
            if key == "top_k":
                if not isinstance(value, int) or not (1 <= value <= 20):
                    raise HTTPException(
                        status_code=400,
                        detail="top_k must be an integer between 1 and 20"
                    )
            elif key == "confidence_threshold":
                if not isinstance(value, (int, float)) or not (0.0 <= value <= 1.0):
                    raise HTTPException(
                        status_code=400,
                        detail="confidence_threshold must be a number between 0.0 and 1.0"
                    )
            elif key == "include_metadata":
                if not isinstance(value, bool):
                    raise HTTPException(
                        status_code=400,
                        detail="include_metadata must be a boolean value"
                    )

    try:
        # Get agent instance
        rag_agent = get_agent()

        # Process the query
        start_time = time.time()
        result = rag_agent.process_query(query, parameters)
        response_time_ms = int((time.time() - start_time) * 1000)

        # Format response - sanitize output to prevent XSS in responses
        response = {
            "response_id": result.id,
            "query_id": result.query_id,
            "response": html.escape(result.text),  # Sanitize response text
            "sources": result.sources,  # Sources are already validated in models
            "grounding_confidence": result.grounding_confidence,
            "response_time_ms": response_time_ms,
            "timestamp": result.timestamp.isoformat() if hasattr(result, 'timestamp') and result.timestamp else time.time()
        }

        return response

    except HTTPException:
        # Re-raise HTTP exceptions as-is
        raise
    except Exception as e:
        # Handle other exceptions
        rag_logger.error(f"Error processing query: {str(e)}")

        # Check for specific error types and handle appropriately
        error_str = str(type(e).__name__)
        if "InsufficientContextError" in error_str or "InsufficientContextError" in str(e):
            raise HTTPException(
                status_code=422,
                detail="No relevant content found in the book for the given query"
            )
        elif "ValidationError" in error_str or "ValidationError" in str(e):
            raise HTTPException(
                status_code=400,
                detail=f"Validation error: {str(e)}"
            )
        elif "GroundingError" in error_str or "GroundingError" in str(e):
            raise HTTPException(
                status_code=422,
                detail="Response could not be adequately grounded in the provided context"
            )
        else:
            raise HTTPException(
                status_code=500,
                detail="An unexpected error occurred during query processing"
            )


@router.get("/api/v1/config")
async def get_config():
    """
    Get the current system configuration.
    """
    config = AgentConfiguration()

    return {
        "default_top_k": config.default_top_k,
        "default_confidence_threshold": config.default_confidence_threshold,
        "max_query_length": config.max_query_length,
        "max_context_length": config.max_context_length,
        "agent_model": config.agent_model,
        "embedding_model": config.embedding_model,
        "max_retries": config.max_retries,
        "timeout_seconds": config.timeout_seconds,
        "timestamp": time.time()
    }


@router.put("/api/v1/config")
async def update_config(config_data: Dict[str, Any]):
    """
    Update the system configuration.
    Note: In a real implementation, this would require more sophisticated configuration management
    since we can't easily change the configuration of a running agent instance.
    """
    # Validate input type
    if not isinstance(config_data, dict):
        raise HTTPException(status_code=400, detail="Configuration data must be a JSON object")

    # Limit the number of configuration parameters to prevent flooding
    if len(config_data) > 10:
        raise HTTPException(status_code=400, detail="Too many configuration parameters provided")

    # In a real implementation, you'd need to reload the agent with new config
    # For now, we'll just validate the provided values
    errors = []

    # Define allowed configuration keys
    allowed_keys = {
        "default_top_k", "default_confidence_threshold", "max_query_length",
        "max_context_length", "agent_model", "embedding_model", "max_retries",
        "timeout_seconds", "grounding_threshold", "max_cache_size", "cache_ttl_seconds"
    }

    # Validate each parameter individually
    for key, value in config_data.items():
        if key not in allowed_keys:
            errors.append(f"Configuration parameter '{key}' is not allowed. Allowed parameters: {', '.join(allowed_keys)}")
            continue

        # Validate each parameter value based on its key
        if key == "default_top_k":
            if not isinstance(value, int) or not (1 <= value <= 20):
                errors.append("default_top_k must be an integer between 1 and 20")
        elif key == "default_confidence_threshold":
            if not isinstance(value, (int, float)) or not (0.0 <= value <= 1.0):
                errors.append("default_confidence_threshold must be a number between 0.0 and 1.0")
        elif key == "max_query_length":
            if not isinstance(value, int) or value <= 0:
                errors.append("max_query_length must be a positive integer")
        elif key == "max_context_length":
            if not isinstance(value, int) or value <= 0:
                errors.append("max_context_length must be a positive integer")
        elif key == "agent_model":
            valid_models = ["gpt-4-turbo", "gpt-4", "gpt-3.5-turbo", "gpt-4o", "gpt-4o-mini"]
            if not isinstance(value, str) or value not in valid_models:
                errors.append(f"agent_model must be one of {valid_models}")
        elif key == "embedding_model":
            valid_embeddings = ["text-embedding-ada-002", "text-embedding-3-small", "text-embedding-3-large"]
            if not isinstance(value, str) or value not in valid_embeddings:
                errors.append(f"embedding_model must be one of {valid_embeddings}")
        elif key == "max_retries":
            if not isinstance(value, int) or value <= 0:
                errors.append("max_retries must be a positive integer")
        elif key == "timeout_seconds":
            if not isinstance(value, int) or value <= 0:
                errors.append("timeout_seconds must be a positive integer")
        elif key == "grounding_threshold":
            if not isinstance(value, (int, float)) or not (0.0 <= value <= 1.0):
                errors.append("grounding_threshold must be a number between 0.0 and 1.0")
        elif key == "max_cache_size":
            if not isinstance(value, int) or value <= 0:
                errors.append("max_cache_size must be a positive integer")
        elif key == "cache_ttl_seconds":
            if not isinstance(value, int) or value <= 0:
                errors.append("cache_ttl_seconds must be a positive integer")

    if errors:
        raise HTTPException(status_code=400, detail={"errors": errors})

    # In a real implementation, you would update the running configuration
    # For now, we'll just return the values as if they were updated
    return {
        "updated_config": config_data,
        "timestamp": time.time()
    }