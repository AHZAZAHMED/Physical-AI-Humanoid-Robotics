"""
Unit tests for the configuration module.
"""
import os
import pytest
from src.config import AgentConfiguration
from src.exceptions import ValidationError


class TestAgentConfiguration:
    """Test class for AgentConfiguration functionality."""

    def setup_method(self):
        """Set up test fixtures before each test method."""
        # Save original environment variables
        self.original_env = {
            key: os.environ.get(key) for key in [
                "QDRANT_HOST", "QDRANT_API_KEY", "QDRANT_COLLECTION_NAME",
                "OPENAI_API_KEY", "EMBEDDING_MODEL", "AGENT_MODEL",
                "DEFAULT_TOP_K", "DEFAULT_CONFIDENCE_THRESHOLD",
                "MAX_QUERY_LENGTH", "MAX_CONTEXT_LENGTH", "MAX_RETRIES",
                "TIMEOUT_SECONDS", "GROUNDING_THRESHOLD",
                "MAX_CACHE_SIZE", "CACHE_TTL_SECONDS"
            ]
        }

    def teardown_method(self):
        """Restore original environment variables after each test method."""
        for key, value in self.original_env.items():
            if value is not None:
                os.environ[key] = value
            elif key in os.environ:
                del os.environ[key]

    def test_default_configuration(self):
        """Test that default configuration values are set correctly."""
        config = AgentConfiguration()

        assert config.qdrant_host == "localhost"
        assert config.qdrant_collection_name == "book_content_chunks"
        assert config.embedding_model == "text-embedding-ada-002"
        assert config.agent_model == "gpt-4-turbo"
        assert config.default_top_k == 5
        assert config.default_confidence_threshold == 0.7
        assert config.max_query_length == 1000
        assert config.max_context_length == 2000
        assert config.max_retries == 3
        assert config.timeout_seconds == 30
        assert config.grounding_threshold == 0.8
        assert config.max_cache_size == 100
        assert config.cache_ttl_seconds == 3600

    def test_configuration_from_environment(self):
        """Test that configuration values can be overridden by environment variables."""
        os.environ["QDRANT_HOST"] = "test-host"
        os.environ["QDRANT_API_KEY"] = "test-key"
        os.environ["QDRANT_COLLECTION_NAME"] = "test-collection"
        os.environ["OPENAI_API_KEY"] = "test-openai-key"
        os.environ["EMBEDDING_MODEL"] = "test-embedding-model"
        os.environ["AGENT_MODEL"] = "test-agent-model"
        os.environ["DEFAULT_TOP_K"] = "10"
        os.environ["DEFAULT_CONFIDENCE_THRESHOLD"] = "0.8"
        os.environ["MAX_QUERY_LENGTH"] = "2000"
        os.environ["MAX_CONTEXT_LENGTH"] = "3000"
        os.environ["MAX_RETRIES"] = "5"
        os.environ["TIMEOUT_SECONDS"] = "60"
        os.environ["GROUNDING_THRESHOLD"] = "0.9"
        os.environ["MAX_CACHE_SIZE"] = "200"
        os.environ["CACHE_TTL_SECONDS"] = "7200"

        config = AgentConfiguration()

        assert config.qdrant_host == "test-host"
        assert config.qdrant_api_key == "test-key"
        assert config.qdrant_collection_name == "test-collection"
        assert config.openai_api_key == "test-openai-key"
        assert config.embedding_model == "test-embedding-model"
        assert config.agent_model == "test-agent-model"
        assert config.default_top_k == 10
        assert config.default_confidence_threshold == 0.8
        assert config.max_query_length == 2000
        assert config.max_context_length == 3000
        assert config.max_retries == 5
        assert config.timeout_seconds == 60
        assert config.grounding_threshold == 0.9
        assert config.max_cache_size == 200
        assert config.cache_ttl_seconds == 7200

    def test_validate_missing_openai_api_key(self):
        """Test that validation fails when OPENAI_API_KEY is missing."""
        config = AgentConfiguration()
        config.openai_api_key = None

        with pytest.raises(ValueError, match="OPENAI_API_KEY is required"):
            config.validate()

    def test_validate_missing_qdrant_host(self):
        """Test that validation fails when QDRANT_HOST is missing."""
        config = AgentConfiguration()
        config.qdrant_host = ""

        with pytest.raises(ValueError, match="QDRANT_HOST is required"):
            config.validate()

    def test_validate_missing_qdrant_collection_name(self):
        """Test that validation fails when QDRANT_COLLECTION_NAME is missing."""
        config = AgentConfiguration()
        config.qdrant_collection_name = ""

        with pytest.raises(ValueError, match="QDRANT_COLLECTION_NAME is required"):
            config.validate()

    def test_validate_invalid_top_k(self):
        """Test that validation fails with invalid top_k value."""
        config = AgentConfiguration()
        config.default_top_k = 25  # Should be between 1 and 20

        with pytest.raises(ValueError, match="default_top_k must be between 1 and 20"):
            config.validate()

    def test_validate_negative_top_k(self):
        """Test that validation fails with negative top_k value."""
        config = AgentConfiguration()
        config.default_top_k = -5  # Should be between 1 and 20

        with pytest.raises(ValueError, match="default_top_k must be between 1 and 20"):
            config.validate()

    def test_validate_invalid_confidence_threshold_high(self):
        """Test that validation fails with confidence threshold > 1.0."""
        config = AgentConfiguration()
        config.default_confidence_threshold = 1.5  # Should be between 0.0 and 1.0

        with pytest.raises(ValueError, match="default_confidence_threshold must be between 0.0 and 1.0"):
            config.validate()

    def test_validate_invalid_confidence_threshold_low(self):
        """Test that validation fails with confidence threshold < 0.0."""
        config = AgentConfiguration()
        config.default_confidence_threshold = -0.5  # Should be between 0.0 and 1.0

        with pytest.raises(ValueError, match="default_confidence_threshold must be between 0.0 and 1.0"):
            config.validate()

    def test_validate_invalid_grounding_threshold_high(self):
        """Test that validation fails with grounding threshold > 1.0."""
        config = AgentConfiguration()
        config.grounding_threshold = 1.5  # Should be between 0.0 and 1.0

        with pytest.raises(ValueError, match="grounding_threshold must be between 0.0 and 1.0"):
            config.validate()

    def test_validate_invalid_grounding_threshold_low(self):
        """Test that validation fails with grounding threshold < 0.0."""
        config = AgentConfiguration()
        config.grounding_threshold = -0.5  # Should be between 0.0 and 1.0

        with pytest.raises(ValueError, match="grounding_threshold must be between 0.0 and 1.0"):
            config.validate()

    def test_validate_valid_configuration(self):
        """Test that validation passes with valid configuration."""
        config = AgentConfiguration()
        config.openai_api_key = "test-key"  # Required for validation
        config.qdrant_host = "test-host"  # Required for validation
        config.qdrant_collection_name = "test-collection"  # Required for validation

        # Should not raise any exceptions
        assert config.validate() is True