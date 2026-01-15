"""
AgentConfiguration model representing the configuration for the RAG agent system.
"""
from dataclasses import dataclass
from typing import Optional


@dataclass
class AgentConfigurationModel:
    """
    Represents the configuration for the RAG agent system.
    """
    id: str
    default_top_k: int = 5
    default_confidence_threshold: float = 0.7
    max_query_length: int = 1000
    max_context_length: int = 2000
    agent_model: str = "gpt-4-turbo"
    embedding_model: str = "text-embedding-ada-002"
    max_retries: int = 3
    timeout_seconds: int = 30

    def validate(self) -> bool:
        """
        Validate the agent configuration parameters.
        """
        if not (1 <= self.default_top_k <= 20):
            raise ValueError("default_top_k must be between 1 and 20")

        if not (0.0 <= self.default_confidence_threshold <= 1.0):
            raise ValueError("default_confidence_threshold must be between 0.0 and 1.0")

        if not (100 <= self.max_query_length <= 5000):
            raise ValueError("max_query_length must be between 100 and 5000")

        # Validate agent model is one of the expected values
        valid_models = ["gpt-4-turbo", "gpt-4", "gpt-3.5-turbo"]
        if self.agent_model not in valid_models:
            raise ValueError(f"agent_model must be one of {valid_models}")

        # Validate embedding model is one of the expected values
        valid_embedding_models = ["text-embedding-ada-002", "text-embedding-3-small", "text-embedding-3-large"]
        if self.embedding_model not in valid_embedding_models:
            raise ValueError(f"embedding_model must be one of {valid_embedding_models}")

        if not (1 <= self.max_retries <= 10):
            raise ValueError("max_retries must be between 1 and 10")

        return True