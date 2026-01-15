"""
Custom exception classes for the RAG agent system.
"""


class RAGException(Exception):
    """
    Base exception class for RAG agent system.
    """
    pass


class ConfigurationError(RAGException):
    """
    Raised when there's an issue with the configuration.
    """
    pass


class EmbeddingError(RAGException):
    """
    Raised when there's an issue with embedding generation.
    """
    pass


class SearchError(RAGException):
    """
    Raised when there's an issue with vector search.
    """
    pass


class GenerationError(RAGException):
    """
    Raised when there's an issue with response generation.
    """
    pass


class ValidationError(RAGException):
    """
    Raised when there's an issue with input validation.
    """
    pass


class GroundingError(RAGException):
    """
    Raised when there's an issue with response grounding validation.
    """
    pass


class InsufficientContextError(RAGException):
    """
    Raised when there's insufficient context to generate a proper response.
    """
    pass


class DependencyError(RAGException):
    """
    Raised when there's an issue with external dependencies.
    """
    pass


class TimeoutError(RAGException):
    """
    Raised when an operation times out.
    """
    pass