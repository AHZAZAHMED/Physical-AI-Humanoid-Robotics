"""
Authentication middleware for API endpoints.
"""
from fastapi import HTTPException, Request
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from typing import Optional
import os


class APIKeyAuth:
    """
    Simple API key authentication middleware.
    """
    def __init__(self):
        self.required_api_key = os.getenv("API_KEY", "your_default_api_key_here")
        self.security = HTTPBearer()

    async def __call__(self, request: Request, credentials: HTTPAuthorizationCredentials = None):
        """
        Authenticate the request using API key.
        """
        # For this implementation, we'll check for a simple API key in the Authorization header
        # In a real system, you'd want more sophisticated authentication
        auth_header = request.headers.get("Authorization")

        if not auth_header:
            raise HTTPException(status_code=401, detail="Authorization header is required")

        # Check if it's a Bearer token
        if not auth_header.startswith("Bearer "):
            raise HTTPException(status_code=401, detail="Authorization header must be in format 'Bearer <api_key>'")

        token = auth_header.split(" ")[1]

        if token != self.required_api_key:
            raise HTTPException(status_code=401, detail="Invalid API key")

        # If we get here, the request is authenticated
        return True


# Create an instance of the auth middleware
auth_middleware = APIKeyAuth()