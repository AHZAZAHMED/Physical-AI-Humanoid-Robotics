"""
Request validator module for sanitizing and validating API requests.

This module provides validation and sanitization for incoming requests
to prevent injection attacks and ensure data integrity.
"""
import re
from typing import Union
from src.shared_types import ChatRequest, ErrorResponse
import html
import logging

logger = logging.getLogger(__name__)


class RequestValidator:
    """
    Validator class for sanitizing and validating API requests.
    Provides methods for input sanitization and validation.
    """

    @staticmethod
    def sanitize_input(input_text: str) -> str:
        """
        Sanitize input text to prevent XSS and other injection attacks.

        Args:
            input_text: The input text to sanitize

        Returns:
            Sanitized input text
        """
        if not input_text:
            return input_text

        # Remove HTML tags
        clean_text = re.sub(r'<[^>]+>', '', input_text)

        # Sanitize HTML entities
        clean_text = html.escape(clean_text, quote=True)

        # Remove potentially harmful characters/sequences
        # Remove control characters except whitespace
        clean_text = re.sub(r'[\x00-\x08\x0B\x0C\x0E-\x1F\x7F]', '', clean_text)

        # Limit length to prevent extremely long inputs
        clean_text = clean_text[:1000]

        return clean_text

    @staticmethod
    def validate_chat_request(request: ChatRequest) -> tuple[bool, Union[ErrorResponse, None]]:
        """
        Validate a chat request.

        Args:
            request: The chat request to validate

        Returns:
            Tuple of (is_valid, error_response_if_invalid)
        """
        # Validate query is not empty
        if not request.query or not request.query.strip():
            return False, ErrorResponse(
                message="Query cannot be empty",
                type="ValidationError"
            )

        # Validate query length
        if len(request.query.strip()) > 1000:
            return False, ErrorResponse(
                message="Query is too long (max 1000 characters)",
                type="ValidationError"
            )

        # Validate session ID format if provided (simple UUID format check)
        if request.sessionId:
            # Simple check for UUID-like format (not strict, just basic validation)
            if not re.match(r'^[a-zA-Z0-9_-]+$', request.sessionId):
                return False, ErrorResponse(
                    message="Invalid session ID format",
                    type="ValidationError"
                )

        # Validate user ID format if provided
        if request.userId:
            if not re.match(r'^[a-zA-Z0-9_-]+$', request.userId):
                return False, ErrorResponse(
                    message="Invalid user ID format",
                    type="ValidationError"
                )

        return True, None

    @staticmethod
    def sanitize_chat_request(request: ChatRequest) -> ChatRequest:
        """
        Sanitize a chat request by cleaning the input fields.

        Args:
            request: The chat request to sanitize

        Returns:
            Sanitized chat request
        """
        # Sanitize the query
        request.query = RequestValidator.sanitize_input(request.query)

        # Sanitize other fields if they exist
        if hasattr(request, 'sessionId') and request.sessionId:
            # For session ID, we just validate format but don't sanitize as it should be clean
            pass

        if hasattr(request, 'userId') and request.userId:
            # For user ID, we just validate format but don't sanitize as it should be clean
            pass

        return request


# Global instance of the request validator
request_validator = RequestValidator()