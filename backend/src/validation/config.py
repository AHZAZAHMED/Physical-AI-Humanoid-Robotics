"""
Configuration module for RAG retrieval validation system.

This module handles environment variable loading and configuration
for the validation system.
"""

import os
from typing import Optional


class ValidationConfig:
    """Configuration class for validation system."""

    def __init__(self):
        # Qdrant configuration
        self.qdrant_url: str = os.getenv("QDRANT_URL", "")
        self.qdrant_api_key: str = os.getenv("QDRANT_API_KEY", "")

        # Cohere configuration
        self.cohere_api_key: str = os.getenv("COHERE_API_KEY", "")

        # Validation-specific settings
        self.collection_name: str = os.getenv("VALIDATION_COLLECTION_NAME", "rag_embeddings")
        self.similarity_threshold: float = float(os.getenv("SIMILARITY_THRESHOLD", "0.5"))
        self.max_concurrent_requests: int = int(os.getenv("MAX_CONCURRENT_REQUESTS", "10"))
        self.timeout_seconds: int = int(os.getenv("REQUEST_TIMEOUT_SECONDS", "30"))

        # Validation parameters
        self.top_k_results: int = int(os.getenv("TOP_K_RESULTS", "5"))
        self.test_query_categories: list = [
            "keyword",
            "semantic",
            "section-specific"
        ]

    def validate(self) -> tuple[bool, str]:
        """
        Validate that all required configuration values are present.

        Returns:
            tuple: (is_valid, error_message)
        """
        if not self.qdrant_url:
            return False, "QDRANT_URL environment variable is required"

        if not self.qdrant_api_key:
            return False, "QDRANT_API_KEY environment variable is required"

        if not self.cohere_api_key:
            return False, "COHERE_API_KEY environment variable is required"

        return True, ""

    def get_qdrant_client_params(self) -> dict:
        """Get parameters for initializing Qdrant client."""
        return {
            "url": self.qdrant_url,
            "api_key": self.qdrant_api_key,
            "prefer_grpc": False  # Using HTTP for simplicity
        }


# Global configuration instance
config = ValidationConfig()