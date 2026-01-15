"""
Rate limiting middleware for API endpoints.
"""
from fastapi import HTTPException, Request
from typing import Dict
import time
from collections import defaultdict


class RateLimiter:
    """
    Simple in-memory rate limiter.
    Note: This is a basic implementation. In production, you'd want to use
    a distributed rate limiter like one based on Redis.
    """
    def __init__(self, requests: int = 100, window: int = 60):
        """
        Initialize rate limiter.

        Args:
            requests: Number of requests allowed per window
            window: Time window in seconds
        """
        self.requests = requests
        self.window = window
        self.requests_log: Dict[str, list] = defaultdict(list)

    def is_allowed(self, identifier: str) -> bool:
        """
        Check if a request from the given identifier is allowed.

        Args:
            identifier: Unique identifier for the requester (e.g., IP address or API key)

        Returns:
            True if request is allowed, False otherwise
        """
        now = time.time()
        # Remove requests that are outside the current window
        self.requests_log[identifier] = [
            req_time for req_time in self.requests_log[identifier]
            if now - req_time < self.window
        ]

        # Check if we're under the limit
        if len(self.requests_log[identifier]) < self.requests:
            # Add current request to the log
            self.requests_log[identifier].append(now)
            return True

        return False


# Create a global rate limiter instance
rate_limiter = RateLimiter(requests=100, window=60)  # 100 requests per minute


async def rate_limit_middleware(request: Request):
    """
    Rate limiting middleware function.
    """
    # For this implementation, we'll use the client IP as the identifier
    # In a real system, you might use the authenticated user ID or API key
    client_ip = request.client.host if request.client else "unknown"

    if not rate_limiter.is_allowed(client_ip):
        raise HTTPException(
            status_code=429,
            detail="Rate limit exceeded: 100 requests per minute"
        )

    return True