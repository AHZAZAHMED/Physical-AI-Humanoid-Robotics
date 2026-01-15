"""
Unit tests for the RAG Agent core functionality.
"""
import pytest
import unittest.mock as mock
from unittest.mock import Mock, MagicMock
from src.agent import RAGAgent
from src.config import AgentConfiguration
from src.models.query import Query
from src.models.retrieved_chunk import RetrievedChunk
from src.models.generated_response import GeneratedResponse
from src.exceptions import ValidationError, InsufficientContextError, GroundingError
from src.services.stats_service import QueryMetrics


class TestRAGAgent:
    """Test class for RAGAgent functionality."""

    def setup_method(self):
        """Set up test fixtures before each test method."""
        self.config = AgentConfiguration()
        self.agent = RAGAgent(self.config)

    def test_agent_initialization(self):
        """Test that the agent initializes correctly with configuration."""
        assert self.agent.config == self.config
        assert self.agent.system_prompt is not None
        assert "educational AI assistant" in self.agent.system_prompt

    @mock.patch('src.clients.openai_client.OpenAIService')
    @mock.patch('src.clients.qdrant_client.QdrantService')
    def test_process_query_success(self, mock_qdrant_service, mock_openai_service):
        """Test successful query processing."""
        # Mock services
        mock_openai_service.create_embedding.return_value = [0.1, 0.2, 0.3]
        mock_qdrant_service.search_vectors.return_value = [
            {
                "id": "chunk1",
                "content": "Test content about robotics",
                "similarity_score": 0.9,
                "confidence_score": 0.85,
                "metadata": {"book_title": "Robotics 101", "chapter": "1", "section": "1.1", "page": 10}
            }
        ]
        mock_openai_service.generate_response.return_value = "Test response based on content"

        # Replace services in agent
        self.agent.openai_service = mock_openai_service
        self.agent.qdrant_service = mock_qdrant_service

        # Mock the grounding validation to return a high confidence
        with mock.patch.object(self.agent, '_validate_response_grounding', return_value=0.95):
            response = self.agent.process_query("Test query about robotics")

            # Verify response
            assert isinstance(response, GeneratedResponse)
            assert "Test response" in response.text
            assert len(response.sources) > 0

    def test_process_query_empty_input(self):
        """Test that empty query raises ValidationError."""
        with pytest.raises(ValidationError, match="Query text must not be empty"):
            self.agent.process_query("")

    def test_process_query_whitespace_only(self):
        """Test that whitespace-only query raises ValidationError."""
        with pytest.raises(ValidationError, match="Query text must not be empty"):
            self.agent.process_query("   ")

    def test_process_query_too_long(self):
        """Test that overly long query raises ValidationError."""
        long_query = "This is a very long query. " * 500  # Much longer than default max length
        with pytest.raises(ValidationError, match="Query text must be less than"):
            self.agent.process_query(long_query)

    @mock.patch('src.clients.openai_client.OpenAIService')
    @mock.patch('src.clients.qdrant_client.QdrantService')
    def test_process_query_insufficient_context(self, mock_qdrant_service, mock_openai_service):
        """Test that query with no relevant results raises InsufficientContextError."""
        # Mock services to return no results
        mock_openai_service.create_embedding.return_value = [0.1, 0.2, 0.3]
        mock_qdrant_service.search_vectors.return_value = []

        # Replace services in agent
        self.agent.openai_service = mock_openai_service
        self.agent.qdrant_service = mock_qdrant_service

        with pytest.raises(InsufficientContextError, match="No relevant content found"):
            self.agent.process_query("Query with no relevant content")

    @mock.patch('src.clients.openai_client.OpenAIService')
    @mock.patch('src.clients.qdrant_client.QdrantService')
    def test_process_query_low_grounding_confidence(self, mock_qdrant_service, mock_openai_service):
        """Test that response with low grounding confidence raises GroundingError."""
        # Mock services
        mock_openai_service.create_embedding.return_value = [0.1, 0.2, 0.3]
        mock_qdrant_service.search_vectors.return_value = [
            {
                "id": "chunk1",
                "content": "Test content about robotics",
                "similarity_score": 0.9,
                "confidence_score": 0.85,
                "metadata": {"book_title": "Robotics 101", "chapter": "1", "section": "1.1", "page": 10}
            }
        ]

        # Replace services in agent
        self.agent.openai_service = mock_openai_service
        self.agent.qdrant_service = mock_qdrant_service

        # Mock the grounding validation to return a low confidence
        with mock.patch.object(self.agent, '_validate_response_grounding', return_value=0.1):
            with pytest.raises(GroundingError, match="not sufficiently grounded"):
                self.agent.process_query("Test query")

    def test_format_context_for_agent(self):
        """Test formatting context for the agent."""
        chunks = [
            RetrievedChunk(
                id="chunk1",
                content="First piece of content",
                similarity_score=0.9,
                confidence_score=0.85,
                metadata={"book_title": "Test Book", "chapter": "1", "section": "1.1", "page": 10}
            ),
            RetrievedChunk(
                id="chunk2",
                content="Second piece of content",
                similarity_score=0.8,
                confidence_score=0.75,
                metadata={"book_title": "Test Book", "chapter": "2", "section": "2.1", "page": 20}
            )
        ]

        context = self.agent._format_context_for_agent(chunks)

        assert "First piece of content" in context
        assert "Second piece of content" in context
        assert "Test Book" in context
        assert "Chapter: 1" in context
        assert "Chapter: 2" in context

    def test_format_sources(self):
        """Test formatting sources for the response."""
        chunks = [
            RetrievedChunk(
                id="chunk1",
                content="This is a long piece of content that should be truncated in the source preview.",
                similarity_score=0.9,
                confidence_score=0.85,
                metadata={"book_title": "Test Book", "chapter": "1", "section": "1.1", "page": 10}
            )
        ]

        sources = self.agent._format_sources(chunks)

        assert len(sources) == 1
        source = sources[0]
        assert "content" in source
        assert "source" in source
        assert source["similarity_score"] == 0.9
        assert source["confidence_score"] == 0.85
        assert len(source["content"]) <= 203  # 200 + "..."


class TestQueryMetrics:
    """Test class for QueryMetrics functionality."""

    def test_query_metrics_creation(self):
        """Test creating QueryMetrics instance."""
        metrics = QueryMetrics(
            query_id="test-query-123",
            response_time_ms=150,
            grounding_confidence=0.85,
            top_k_used=5,
            confidence_threshold=0.7,
            retrieved_chunks_count=3,
            timestamp=1234567890.0,
            success=True,
            error_type="TestError"
        )

        assert metrics.query_id == "test-query-123"
        assert metrics.response_time_ms == 150
        assert metrics.grounding_confidence == 0.85
        assert metrics.success is True
        assert metrics.error_type == "TestError"