"""
Error handling middleware for API endpoints.
"""
from fastapi import Request, HTTPException
from fastapi.responses import JSONResponse
from typing import Dict, Any
import traceback
import time
import sys
import os

# Handle both direct execution and module imports
try:
    # Try relative imports first (when used as a module)
    from ..utils.logging import rag_logger
except ImportError:
    # Fallback for direct execution - add project root to path
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(__file__))))
    from utils.logging import rag_logger


async def global_exception_handler(request: Request, exc: Exception):
    """
    Global exception handler for the application.
    """
    # Log the error with traceback
    error_id = f"error_{int(time.time())}_{hash(str(exc)) % 10000}"
    rag_logger.error(f"Error ID: {error_id} - {str(exc)}")
    rag_logger.error(f"Traceback: {traceback.format_exc()}")

    # Determine the appropriate error response based on the exception type
    if isinstance(exc, HTTPException):
        # If it's already an HTTPException, use its status code and detail
        status_code = exc.status_code
        error_detail = exc.detail
    else:
        # For other exceptions, return a 500 error
        status_code = 500
        error_detail = "An internal server error occurred"

    # Format the error response according to the API contracts
    error_response = {
        "error": "internal_server_error" if status_code == 500 else "unknown_error",
        "message": str(error_detail),
        "timestamp": time.time(),
        "request_id": error_id
    }

    return JSONResponse(
        status_code=status_code,
        content=error_response
    )


def add_error_handlers(app):
    """
    Add error handlers to the FastAPI application.
    """
    @app.exception_handler(HTTPException)
    async def http_exception_handler(request: Request, exc: HTTPException):
        return await global_exception_handler(request, exc)

    @app.exception_handler(Exception)
    async def general_exception_handler(request: Request, exc: Exception):
        return await global_exception_handler(request, exc)