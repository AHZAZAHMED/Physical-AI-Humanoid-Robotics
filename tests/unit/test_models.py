"""
Unit tests for the data models.
"""
import pytest
from src.models.query import Query
from src.models.retrieved_chunk import RetrievedChunk
from src.models.generated_response import GeneratedResponse
from src.models.agent_configuration import AgentConfiguration
from src.exceptions import ValidationError


class TestQueryModel:
    """Test class for Query model."""

    def test_query_creation_valid(self):
        """Test creating a valid Query object."""
        query = Query(
            id="test-query-123",
            text="This is a test query",
            parameters={"top_k": 5, "confidence_threshold": 0.7}
        )

        assert query.id == "test-query-123"
        assert query.text == "This is a test query"
        assert query.parameters == {"top_k": 5, "confidence_threshold": 0.7}

    def test_query_validation_valid(self):
        """Test validation of a valid Query object."""
        query = Query(
            id="test-query-123",
            text="This is a test query",
            parameters={"top_k": 5, "confidence_threshold": 0.7}
        )

        # Should not raise any exceptions
        query.validate()

    def test_query_validation_empty_text(self):
        """Test validation fails with empty text."""
        query = Query(
            id="test-query-123",
            text="",
            parameters={"top_k": 5, "confidence_threshold": 0.7}
        )

        with pytest.raises(ValidationError, match="Query text must not be empty"):
            query.validate()

    def test_query_validation_none_text(self):
        """Test validation fails with None text."""
        query = Query(
            id="test-query-123",
            text=None,
            parameters={"top_k": 5, "confidence_threshold": 0.7}
        )

        with pytest.raises(ValidationError, match="Query text must not be empty"):
            query.validate()

    def test_query_validation_whitespace_text(self):
        """Test validation fails with whitespace-only text."""
        query = Query(
            id="test-query-123",
            text="   ",
            parameters={"top_k": 5, "confidence_threshold": 0.7}
        )

        with pytest.raises(ValidationError, match="Query text must not be empty"):
            query.validate()


class TestRetrievedChunkModel:
    """Test class for RetrievedChunk model."""

    def test_retrieved_chunk_creation_valid(self):
        """Test creating a valid RetrievedChunk object."""
        chunk = RetrievedChunk(
            id="chunk-123",
            content="This is some retrieved content",
            similarity_score=0.85,
            confidence_score=0.80,
            metadata={"book": "Test Book", "chapter": "1", "section": "1.1"}
        )

        assert chunk.id == "chunk-123"
        assert chunk.content == "This is some retrieved content"
        assert chunk.similarity_score == 0.85
        assert chunk.confidence_score == 0.80
        assert chunk.metadata == {"book": "Test Book", "chapter": "1", "section": "1.1"}

    def test_retrieved_chunk_validation_valid(self):
        """Test validation of a valid RetrievedChunk object."""
        chunk = RetrievedChunk(
            id="chunk-123",
            content="This is some retrieved content",
            similarity_score=0.85,
            confidence_score=0.80,
            metadata={"book": "Test Book", "chapter": "1", "section": "1.1"}
        )

        # Should not raise any exceptions
        chunk.validate()

    def test_retrieved_chunk_validation_missing_id(self):
        """Test validation fails with missing id."""
        chunk = RetrievedChunk(
            id=None,
            content="This is some retrieved content",
            similarity_score=0.85,
            confidence_score=0.80,
            metadata={"book": "Test Book", "chapter": "1", "section": "1.1"}
        )

        with pytest.raises(ValidationError, match="ID must be provided"):
            chunk.validate()

    def test_retrieved_chunk_validation_invalid_similarity_score(self):
        """Test validation fails with invalid similarity score."""
        chunk = RetrievedChunk(
            id="chunk-123",
            content="This is some retrieved content",
            similarity_score=1.5,  # Should be between 0 and 1
            confidence_score=0.80,
            metadata={"book": "Test Book", "chapter": "1", "section": "1.1"}
        )

        with pytest.raises(ValidationError, match="Similarity score must be between 0.0 and 1.0"):
            chunk.validate()

    def test_retrieved_chunk_validation_negative_similarity_score(self):
        """Test validation fails with negative similarity score."""
        chunk = RetrievedChunk(
            id="chunk-123",
            content="This is some retrieved content",
            similarity_score=-0.1,  # Should be between 0 and 1
            confidence_score=0.80,
            metadata={"book": "Test Book", "chapter": "1", "section": "1.1"}
        )

        with pytest.raises(ValidationError, match="Similarity score must be between 0.0 and 1.0"):
            chunk.validate()

    def test_retrieved_chunk_validation_invalid_confidence_score(self):
        """Test validation fails with invalid confidence score."""
        chunk = RetrievedChunk(
            id="chunk-123",
            content="This is some retrieved content",
            similarity_score=0.85,
            confidence_score=1.2,  # Should be between 0 and 1
            metadata={"book": "Test Book", "chapter": "1", "section": "1.1"}
        )

        with pytest.raises(ValidationError, match="Confidence score must be between 0.0 and 1.0"):
            chunk.validate()


class TestGeneratedResponseModel:
    """Test class for GeneratedResponse model."""

    def test_generated_response_creation_valid(self):
        """Test creating a valid GeneratedResponse object."""
        response = GeneratedResponse(
            id="response-123",
            query_id="query-123",
            text="This is the generated response text",
            sources=[
                {
                    "content": "Source content preview",
                    "source": {"book": "Test Book", "chapter": "1", "section": "1.1"},
                    "similarity_score": 0.9,
                    "confidence_score": 0.85
                }
            ],
            grounding_confidence=0.92,
            response_time_ms=1250
        )

        assert response.id == "response-123"
        assert response.query_id == "query-123"
        assert response.text == "This is the generated response text"
        assert len(response.sources) == 1
        assert response.grounding_confidence == 0.92
        assert response.response_time_ms == 1250

    def test_generated_response_validation_valid(self):
        """Test validation of a valid GeneratedResponse object."""
        response = GeneratedResponse(
            id="response-123",
            query_id="query-123",
            text="This is the generated response text",
            sources=[
                {
                    "content": "Source content preview",
                    "source": {"book": "Test Book", "chapter": "1", "section": "1.1"},
                    "similarity_score": 0.9,
                    "confidence_score": 0.85
                }
            ],
            grounding_confidence=0.92,
            response_time_ms=1250
        )

        # Should not raise any exceptions
        response.validate()

    def test_generated_response_validation_missing_id(self):
        """Test validation fails with missing id."""
        response = GeneratedResponse(
            id=None,
            query_id="query-123",
            text="This is the generated response text",
            sources=[],
            grounding_confidence=0.92,
            response_time_ms=1250
        )

        with pytest.raises(ValidationError, match="ID must be provided"):
            response.validate()

    def test_generated_response_validation_missing_query_id(self):
        """Test validation fails with missing query_id."""
        response = GeneratedResponse(
            id="response-123",
            query_id=None,
            text="This is the generated response text",
            sources=[],
            grounding_confidence=0.92,
            response_time_ms=1250
        )

        with pytest.raises(ValidationError, match="Query ID must be provided"):
            response.validate()

    def test_generated_response_validation_invalid_grounding_confidence(self):
        """Test validation fails with invalid grounding confidence."""
        response = GeneratedResponse(
            id="response-123",
            query_id="query-123",
            text="This is the generated response text",
            sources=[],
            grounding_confidence=1.5,  # Should be between 0 and 1
            response_time_ms=1250
        )

        with pytest.raises(ValidationError, match="Grounding confidence must be between 0.0 and 1.0"):
            response.validate()

    def test_generated_response_validation_negative_response_time(self):
        """Test validation fails with negative response time."""
        response = GeneratedResponse(
            id="response-123",
            query_id="query-123",
            text="This is the generated response text",
            sources=[],
            grounding_confidence=0.92,
            response_time_ms=-100  # Should be non-negative
        )

        with pytest.raises(ValidationError, match="Response time must be non-negative"):
            response.validate()


class TestAgentConfigurationModel:
    """Test class for AgentConfiguration model."""

    def test_agent_configuration_creation(self):
        """Test creating an AgentConfiguration object."""
        config = AgentConfiguration()

        # Verify default values are set
        assert config.default_top_k == 5
        assert config.default_confidence_threshold == 0.7
        assert config.grounding_threshold == 0.8
        assert config.max_cache_size == 100
        assert config.cache_ttl_seconds == 3600