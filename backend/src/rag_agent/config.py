"""
Configuration class for the RAG agent system.
"""

import os
from typing import Optional


class AgentConfiguration:
    """
    Configuration class that holds all the settings for the RAG agent system.
    """

    def __init__(self):
        # -------------------------------
        # Qdrant configuration
        # -------------------------------
        self.qdrant_host = os.getenv("QDRANT_HOST", "localhost")
        self.qdrant_api_key = os.getenv("QDRANT_API_KEY")
        self.qdrant_collection_name = os.getenv(
            "QDRANT_COLLECTION_NAME",
            "rag_embeddings"
        )

        # -------------------------------
        # OpenAI / OpenRouter configuration
        # -------------------------------
        self.openai_api_key = (
            os.getenv("OPENAI_API_KEY")
            or os.getenv("OPENROUTER_API_KEY")
        )

        self.embedding_model = os.getenv(
            "EMBEDDING_MODEL",
            "text-embedding-ada-002"
        )

        self.agent_model = (
            os.getenv("AGENT_MODEL", "gemini-2.5-flash")
            or os.getenv("GEMINI_MODEL", "gemini-2.5-flash")
        )

        # -------------------------------
        # Agent-specific parameters
        # -------------------------------
        self.default_top_k: int = int(
            os.getenv("DEFAULT_TOP_K", "5")
        )

        self.default_confidence_threshold: float = float(
            os.getenv("DEFAULT_CONFIDENCE_THRESHOLD", "0.7")
        )

        self.max_query_length: int = int(
            os.getenv("MAX_QUERY_LENGTH", "1000")
        )

        self.max_context_length: int = int(
            os.getenv("MAX_CONTEXT_LENGTH", "2000")
        )

        self.max_retries: int = int(
            os.getenv("MAX_RETRIES", "3")
        )

        self.timeout_seconds: int = int(
            os.getenv("TIMEOUT_SECONDS", "30")
        )

        # -------------------------------
        # Validation parameters
        # -------------------------------
        self.grounding_threshold: float = float(
            os.getenv("GROUNDING_THRESHOLD", "0.2")
        )

        # -------------------------------
        # Caching parameters
        # -------------------------------
        self.max_cache_size: int = int(
            os.getenv("MAX_CACHE_SIZE", "100")
        )

        self.cache_ttl_seconds: int = int(
            os.getenv("CACHE_TTL_SECONDS", "3600")
        )  # 1 hour

    def validate(self) -> bool:
        """
        Validate that all required configuration values are present.
        """

        # Check for either OpenAI, OpenRouter, or Gemini API key
        if not self.openai_api_key and not os.getenv("OPENROUTER_API_KEY") and not os.getenv("GEMINI_API_KEY"):
            raise ValueError(
                "Either OPENAI_API_KEY, OPENROUTER_API_KEY, or GEMINI_API_KEY is required"
            )

        if not self.qdrant_host:
            raise ValueError("QDRANT_HOST is required")

        if not self.qdrant_collection_name:
            raise ValueError("QDRANT_COLLECTION_NAME is required")

        # Validate ranges
        if not (1 <= self.default_top_k <= 20):
            raise ValueError(
                "default_top_k must be between 1 and 20"
            )

        if not (0.0 <= self.default_confidence_threshold <= 1.0):
            raise ValueError(
                "default_confidence_threshold must be between 0.0 and 1.0"
            )

        if not (0.0 <= self.grounding_threshold <= 1.0):
            raise ValueError(
                "grounding_threshold must be between 0.0 and 1.0"
            )

        return True
