"""
End-to-end tests for the RAG retrieval validation system.

This module tests the complete validation workflow to ensure
all components work together correctly.
"""
import pytest
from unittest.mock import Mock, patch
import os
from backend.src.validation.validation_orchestrator import ValidationOrchestrator
from backend.src.validation.query_generator import QueryGenerator
from backend.src.validation.retrieval_validator import RetrievalValidator


class TestEndToEndValidation:
    """Test class for end-to-end validation workflow."""

    def test_validation_orchestrator_initialization(self):
        """Test that validation orchestrator initializes correctly."""
        # Mock configuration to avoid needing real API keys
        with patch.dict(os.environ, {
            'QDRANT_URL': 'https://test.qdrant.com',
            'QDRANT_API_KEY': 'test-key',
            'COHERE_API_KEY': 'test-key',
            'VALIDATION_COLLECTION_NAME': 'test_collection'
        }):
            # Mock the external dependencies to avoid real API calls
            with patch('backend.src.validation.retrieval_validator.QdrantClient') as mock_qdrant, \
                 patch('backend.src.validation.retrieval_validator.cohere.Client') as mock_cohere:

                # Setup mock returns
                mock_qdrant.return_value.get_collections.return_value = Mock()
                mock_qdrant.return_value.get_collections.return_value.collections = [Mock()]
                mock_qdrant.return_value.get_collections.return_value.collections[0].name = 'test_collection'

                mock_cohere.return_value.embed.return_value = Mock()
                mock_cohere.return_value.embed.return_value.embeddings = [[0.1, 0.2, 0.3]]

                # Initialize orchestrator
                orchestrator = ValidationOrchestrator()

                # Assertions
                assert orchestrator is not None
                assert hasattr(orchestrator, 'retrieval_validator')
                assert hasattr(orchestrator, 'query_generator')
                assert hasattr(orchestrator, 'result_analyzer')

    def test_query_generation_integration(self):
        """Test that query generator works with orchestrator."""
        # Mock configuration
        with patch.dict(os.environ, {
            'QDRANT_URL': 'https://test.qdrant.com',
            'QDRANT_API_KEY': 'test-key',
            'COHERE_API_KEY': 'test-key'
        }):
            with patch('backend.src.validation.retrieval_validator.QdrantClient') as mock_qdrant, \
                 patch('backend.src.validation.retrieval_validator.cohere.Client') as mock_cohere:

                # Setup mocks
                mock_qdrant.return_value.get_collections.return_value = Mock()
                mock_qdrant.return_value.get_collections.return_value.collections = [Mock()]
                mock_qdrant.return_value.get_collections.return_value.collections[0].name = 'test_collection'

                mock_cohere.return_value.embed.return_value = Mock()
                mock_cohere.return_value.embed.return_value.embeddings = [[0.1, 0.2, 0.3]]

                orchestrator = ValidationOrchestrator()

                # Generate queries using the orchestrator's query generator
                queries = orchestrator.query_generator.generate_test_queries({
                    'keyword': 2,
                    'semantic': 1,
                    'section-specific': 1
                })

                # Assertions
                assert len(queries) == 4  # 2 keyword + 1 semantic + 1 section-specific
                for query, category in queries:
                    assert isinstance(query, str)
                    assert category in ['keyword', 'semantic', 'section-specific']

    def test_retrieval_validation_with_mock_data(self):
        """Test retrieval validation with mocked data."""
        # Mock configuration
        with patch.dict(os.environ, {
            'QDRANT_URL': 'https://test.qdrant.com',
            'QDRANT_API_KEY': 'test-key',
            'COHERE_API_KEY': 'test-key'
        }):
            with patch('backend.src.validation.retrieval_validator.QdrantClient') as mock_qdrant, \
                 patch('backend.src.validation.retrieval_validator.cohere.Client') as mock_cohere:

                # Setup mocks
                mock_qdrant.return_value.get_collections.return_value = Mock()
                mock_qdrant.return_value.get_collections.return_value.collections = [Mock()]
                mock_qdrant.return_value.get_collections.return_value.collections[0].name = 'test_collection'

                mock_cohere.return_value.embed.return_value = Mock()
                mock_cohere.return_value.embed.return_value.embeddings = [[0.1, 0.2, 0.3]]

                # Create validator
                validator = RetrievalValidator()

                # Mock the search method to return test results
                mock_search_result = Mock()
                mock_search_result.payload = {
                    'content': 'Test content for validation',
                    'metadata': {'source_url': 'https://example.com', 'section_title': 'Test Section'}
                }
                mock_search_result.score = 0.8
                mock_qdrant.return_value.search.return_value = [mock_search_result]

                # Validate a single query
                result = validator.validate_single_query("test query", "keyword")

                # Assertions
                assert result is not None
                assert result.query == "test query"
                assert result.query_category == "keyword"
                assert len(result.results) == 1
                assert result.results[0]['content'] == 'Test content for validation'
                assert result.results[0]['similarity_score'] == 0.8

    def test_comprehensive_validation_workflow(self):
        """Test the complete validation workflow."""
        # Mock configuration
        with patch.dict(os.environ, {
            'QDRANT_URL': 'https://test.qdrant.com',
            'QDRANT_API_KEY': 'test-key',
            'COHERE_API_KEY': 'test-key'
        }):
            with patch('backend.src.validation.retrieval_validator.QdrantClient') as mock_qdrant, \
                 patch('backend.src.validation.retrieval_validator.cohere.Client') as mock_cohere:

                # Setup mocks
                mock_qdrant.return_value.get_collections.return_value = Mock()
                mock_qdrant.return_value.get_collections.return_value.collections = [Mock()]
                mock_qdrant.return_value.get_collections.return_value.collections[0].name = 'test_collection'

                mock_cohere.return_value.embed.return_value = Mock()
                mock_cohere.return_value.embed.return_value.embeddings = [[0.1, 0.2, 0.3]]

                # Mock search results
                mock_search_result = Mock()
                mock_search_result.payload = {
                    'content': 'Test content for validation',
                    'metadata': {'source_url': 'https://example.com', 'section_title': 'Test Section'}
                }
                mock_search_result.score = 0.8
                mock_qdrant.return_value.search.return_value = [mock_search_result]

                # Create orchestrator
                orchestrator = ValidationOrchestrator()

                # Run a simple validation
                results = orchestrator.run_content_relevance_validation([
                    ("test query", "keyword")
                ])

                # Assertions
                assert 'total_results' in results
                assert 'relevant_results' in results
                assert 'relevance_rate' in results
                assert 'validation_results' in results
                assert len(results['validation_results']) == 1

    def test_validation_report_generation(self):
        """Test validation report generation."""
        # Mock configuration
        with patch.dict(os.environ, {
            'QDRANT_URL': 'https://test.qdrant.com',
            'QDRANT_API_KEY': 'test-key',
            'COHERE_API_KEY': 'test-key'
        }):
            with patch('backend.src.validation.retrieval_validator.QdrantClient') as mock_qdrant, \
                 patch('backend.src.validation.retrieval_validator.cohere.Client') as mock_cohere:

                # Setup mocks
                mock_qdrant.return_value.get_collections.return_value = Mock()
                mock_qdrant.return_value.get_collections.return_value.collections = [Mock()]
                mock_qdrant.return_value.get_collections.return_value.collections[0].name = 'test_collection'

                mock_cohere.return_value.embed.return_value = Mock()
                mock_cohere.return_value.embed.return_value.embeddings = [[0.1, 0.2, 0.3]]

                # Mock search results
                mock_search_result = Mock()
                mock_search_result.payload = {
                    'content': 'Test content for validation',
                    'metadata': {'source_url': 'https://example.com', 'section_title': 'Test Section'}
                }
                mock_search_result.score = 0.8
                mock_qdrant.return_value.search.return_value = [mock_search_result]

                orchestrator = ValidationOrchestrator()

                # Run a simple validation to get results
                content_results = orchestrator.run_content_relevance_validation([
                    ("test query", "keyword")
                ])

                # Create comprehensive results structure for report generation
                comprehensive_results = {
                    'summary': {
                        'total_tests': 1,
                        'passed_tests': 1,
                        'failed_tests': 0,
                        'pass_rate': 1.0,
                        'avg_latency_ms': 100.0
                    },
                    'retrieval_results': content_results['validation_results'],
                    'pass_fail_determination': {
                        'overall_status': 'PASS'
                    },
                    'timestamp': '2023-01-01T00:00:00',
                    'execution_time_seconds': 1.0
                }

                # Generate different types of reports
                json_report = orchestrator.generate_final_validation_report(comprehensive_results, 'json')
                text_report = orchestrator.generate_final_validation_report(comprehensive_results, 'text')
                detailed_report = orchestrator.generate_final_validation_report(comprehensive_results, 'detailed')

                # Assertions
                assert isinstance(json_report, str)
                assert isinstance(text_report, str)
                assert isinstance(detailed_report, str)
                assert "RAG Retrieval Validation Report" in text_report
                assert "RAG Retrieval Validation Report" in detailed_report
                assert '"total_tests": 1' in json_report