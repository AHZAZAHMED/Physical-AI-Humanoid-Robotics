"""
Basic functionality tests for the RAG agent system.
"""
import pytest
import os
from unittest.mock import Mock, patch
from src.agent import RAGAgent
from src.config import AgentConfiguration
from src.models.query import Query
from src.models.retrieved_chunk import RetrievedChunk
from src.models.generated_response import GeneratedResponse


def test_agent_initialization():
    """
    Test that the RAG agent initializes correctly with valid configuration.
    """
    config = AgentConfiguration()
    agent = RAGAgent(config)

    assert agent is not None
    assert agent.config == config


def test_query_model_validation():
    """
    Test that the Query model validates correctly.
    """
    query = Query(
        id="test_id",
        text="This is a test query",
        parameters={"top_k": 5, "confidence_threshold": 0.7}
    )

    # Should not raise an exception
    assert query.validate() == True


def test_query_model_validation_empty_text():
    """
    Test that the Query model raises an error for empty text.
    """
    query = Query(
        id="test_id",
        text="",
        parameters={"top_k": 5, "confidence_threshold": 0.7}
    )

    with pytest.raises(ValueError, match="Query text must not be empty"):
        query.validate()


def test_retrieved_chunk_model_validation():
    """
    Test that the RetrievedChunk model validates correctly.
    """
    chunk = RetrievedChunk(
        id="test_id",
        content="This is test content",
        similarity_score=0.8,
        confidence_score=0.8,
        metadata={
            "book_title": "Test Book",
            "chapter": "Chapter 1",
            "section": "Section 1.1"
        }
    )

    # Should not raise an exception
    assert chunk.validate() == True


def test_retrieved_chunk_model_validation_missing_metadata():
    """
    Test that the RetrievedChunk model raises an error for missing metadata.
    """
    chunk = RetrievedChunk(
        id="test_id",
        content="This is test content",
        similarity_score=0.8,
        confidence_score=0.8,
        metadata={
            "book_title": "Test Book",
            # Missing "chapter" and "section"
        }
    )

    with pytest.raises(ValueError, match="Metadata must contain 'chapter' field"):
        chunk.validate()


def test_generated_response_model_validation():
    """
    Test that the GeneratedResponse model validates correctly.
    """
    response = GeneratedResponse(
        id="test_id",
        query_id="query_id",
        text="This is a test response",
        sources=[],
        grounding_confidence=0.9,
        response_time_ms=100
    )

    # Should not raise an exception
    assert response.validate() == True


def test_generated_response_model_validation_negative_time():
    """
    Test that the GeneratedResponse model raises an error for negative response time.
    """
    response = GeneratedResponse(
        id="test_id",
        query_id="query_id",
        text="This is a test response",
        sources=[],
        grounding_confidence=0.9,
        response_time_ms=-100
    )

    with pytest.raises(ValueError, match="Response time must be positive"):
        response.validate()


@patch('src.clients.openai_client.OpenAIService')
@patch('src.clients.qdrant_client.QdrantService')
def test_process_query_success(mock_qdrant_service, mock_openai_service):
    """
    Test that process_query works with mocked services.
    """
    # Mock the services
    mock_openai = Mock()
    mock_openai.create_embedding.return_value = [0.1, 0.2, 0.3]  # Mock embedding
    mock_openai.generate_response.return_value = "This is a test response"
    mock_openai_service.return_value = mock_openai

    mock_qdrant = Mock()
    mock_qdrant.search_vectors.return_value = [
        {
            "id": "chunk1",
            "content": "Test content for the query",
            "similarity_score": 0.85,
            "confidence_score": 0.82,
            "metadata": {
                "book_title": "Test Book",
                "chapter": "Chapter 1",
                "section": "Section 1.1",
                "page": 10
            }
        }
    ]
    mock_qdrant_service.return_value = mock_qdrant

    # Create agent and process a query
    config = AgentConfiguration()
    agent = RAGAgent(config)

    # Process a simple query
    result = agent.process_query("Test query")

    # Verify the result is a GeneratedResponse
    assert isinstance(result, GeneratedResponse)
    assert result.text == "This is a test response"
    assert len(result.sources) > 0


if __name__ == "__main__":
    pytest.main([__file__])