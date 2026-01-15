"""
Unit tests for RAG retrieval quality validation.

This module tests the content relevance and quality of retrieved results.
"""
import pytest
import os
from unittest.mock import Mock, patch, MagicMock
from backend.src.validation.retrieval_validator import RetrievalValidator, ValidationResult
from backend.src.validation.query_generator import QueryGenerator


class TestRetrievalQuality:
    """Test class for retrieval quality validation."""

    def setup_method(self):
        """Set up test fixtures before each test method."""
        # Mock configuration to avoid needing real API keys
        with patch.dict(os.environ, {
            'QDRANT_URL': 'https://test.qdrant.com',
            'QDRANT_API_KEY': 'test-key',
            'COHERE_API_KEY': 'test-key'
        }):
            from backend.src.validation.config import config
            # Temporarily modify config to use test values
            self.validator = RetrievalValidator()
            # Mock the clients to avoid real API calls
            self.validator.qdrant_client = Mock()
            self.validator.cohere_client = Mock()

    def test_validate_single_query_success(self):
        """Test successful validation of a single query."""
        # Mock the embedding response
        mock_embedding_response = Mock()
        mock_embedding_response.embeddings = [[0.1, 0.2, 0.3]]
        self.validator.cohere_client.embed.return_value = mock_embedding_response

        # Mock the search results
        mock_search_result = Mock()
        mock_search_result.payload = {
            'content': 'Test content for validation',
            'metadata': {'source_url': 'https://example.com', 'section_title': 'Test Section'}
        }
        mock_search_result.score = 0.8
        self.validator.qdrant_client.search.return_value = [mock_search_result]

        # Test the validation
        result = self.validator.validate_single_query("test query", "keyword")

        # Assertions
        assert isinstance(result, ValidationResult)
        assert result.query == "test query"
        assert result.query_category == "keyword"
        assert result.status == "PASS"
        assert len(result.results) == 1
        assert result.results[0]['content'] == 'Test content for validation'
        assert result.results[0]['similarity_score'] == 0.8
        assert result.results[0]['relevance'] is True  # Score above threshold

    def test_validate_single_query_failure_low_similarity(self):
        """Test validation failure due to low similarity score."""
        # Mock the embedding response
        mock_embedding_response = Mock()
        mock_embedding_response.embeddings = [[0.1, 0.2, 0.3]]
        self.validator.cohere_client.embed.return_value = mock_embedding_response

        # Mock the search results with low score
        mock_search_result = Mock()
        mock_search_result.payload = {
            'content': 'Test content for validation',
            'metadata': {'source_url': 'https://example.com', 'section_title': 'Test Section'}
        }
        mock_search_result.score = 0.1  # Below threshold
        self.validator.qdrant_client.search.return_value = [mock_search_result]

        # Test the validation
        result = self.validator.validate_single_query("test query", "keyword")

        # Assertions
        assert isinstance(result, ValidationResult)
        assert result.status == "PASS"  # Still PASS because we have results, but avg similarity might be low
        assert len(result.results) == 1
        assert result.results[0]['relevance'] is False  # Score below threshold

    def test_validate_single_query_error_handling(self):
        """Test error handling in single query validation."""
        # Mock the embedding to raise an exception
        self.validator.cohere_client.embed.side_effect = Exception("API Error")

        # Test the validation
        result = self.validator.validate_single_query("test query", "keyword")

        # Assertions
        assert isinstance(result, ValidationResult)
        assert result.status == "ERROR"
        assert 'error' in result.metrics

    def test_validate_batch_queries(self):
        """Test validation of multiple queries in batch."""
        # Mock the embedding response
        mock_embedding_response = Mock()
        mock_embedding_response.embeddings = [[0.1, 0.2, 0.3]]
        self.validator.cohere_client.embed.return_value = mock_embedding_response

        # Mock the search results
        mock_search_result = Mock()
        mock_search_result.payload = {
            'content': 'Test content for validation',
            'metadata': {'source_url': 'https://example.com', 'section_title': 'Test Section'}
        }
        mock_search_result.score = 0.8
        self.validator.qdrant_client.search.return_value = [mock_search_result]

        # Test the validation
        queries = [("test query 1", "keyword"), ("test query 2", "semantic")]
        results = self.validator.validate_batch_queries(queries)

        # Assertions
        assert len(results) == 2
        for result in results:
            assert isinstance(result, ValidationResult)
            assert result.status == "PASS"

    def test_collection_exists(self):
        """Test checking if collection exists."""
        # Mock the get_collection method to return success
        self.validator.qdrant_client.get_collection.return_value = Mock()

        # Test the method
        exists = self.validator.check_collection_exists()

        # Assertions
        assert exists is True
        self.validator.qdrant_client.get_collection.assert_called_once_with(
            self.validator.collection_name
        )

    def test_collection_not_exists(self):
        """Test checking if collection exists when it doesn't."""
        # Mock the get_collection method to raise an exception
        self.validator.qdrant_client.get_collection.side_effect = Exception("Collection not found")

        # Test the method
        exists = self.validator.check_collection_exists()

        # Assertions
        assert exists is False