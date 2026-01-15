"""
Unit tests for RAG validation orchestrator.

This module tests the main validation orchestrator that coordinates
all validation components.
"""
import pytest
from unittest.mock import Mock, patch
import os
from backend.src.validation.validation_orchestrator import ValidationOrchestrator


class TestValidationOrchestrator:
    """Test class for validation orchestrator."""

    def setup_method(self):
        """Set up test fixtures before each test method."""
        # Mock configuration to avoid needing real API keys
        with patch.dict(os.environ, {
            'QDRANT_URL': 'https://test.qdrant.com',
            'QDRANT_API_KEY': 'test-key',
            'COHERE_API_KEY': 'test-key'
        }):
            self.orchestrator = ValidationOrchestrator()
            # Mock the validators to avoid real API calls
            self.orchestrator.retrieval_validator = Mock()
            self.orchestrator.result_analyzer = Mock()
            self.orchestrator.metadata_validator = Mock()
            self.orchestrator.performance_utils = Mock()

    def test_run_content_relevance_validation(self):
        """Test content relevance validation."""
        # Mock the retrieval validator
        mock_result = Mock()
        mock_result.query = "test query"
        mock_result.query_category = "keyword"
        mock_result.results = [
            {
                'content': 'Test content',
                'similarity_score': 0.8,
                'metadata': {},
                'relevance': True
            }
        ]
        mock_result.metrics = {'latency_ms': 100}
        mock_result.status = "PASS"
        self.orchestrator.retrieval_validator.validate_batch_queries.return_value = [mock_result]

        # Mock the result analyzer
        self.orchestrator.result_analyzer.add_validation_results.return_value = None
        self.orchestrator.result_analyzer.generate_summary.return_value = {
            'total_tests': 1,
            'passed_tests': 1,
            'failed_tests': 0,
            'pass_rate': 1.0
        }

        # Run the validation
        results = self.orchestrator.run_content_relevance_validation([
            ("test query", "keyword")
        ])

        # Assertions
        assert 'total_results' in results
        assert 'relevant_results' in results
        assert 'relevance_rate' in results
        assert 'is_acceptable' in results
        assert results['relevance_rate'] >= 0  # Should have some relevance

    def test_run_metadata_integrity_validation(self):
        """Test metadata integrity validation."""
        # Mock the retrieval validator
        mock_result = Mock()
        mock_result.results = [
            {
                'content': 'Test content',
                'similarity_score': 0.8,
                'metadata': {
                    'source_url': 'https://example.com',
                    'section_title': 'Test Section',
                    'chunk_id': 'chunk_1',
                    'created_at': '2023-01-01'
                },
                'relevance': True
            }
        ]
        self.orchestrator.retrieval_validator.validate_batch_queries.return_value = [mock_result]

        # Mock metadata validation results
        mock_metadata_results = {
            'total_chunks': 1,
            'valid_chunks': 1,
            'invalid_chunks': 0,
            'integrity_score': 1.0
        }

        # Since we're mocking, we'll patch the function directly
        with patch('backend.src.validation.validation_orchestrator.validate_metadata_for_retrieval_results',
                  return_value=mock_metadata_results):
            results = self.orchestrator.run_metadata_integrity_validation([
                ("test query", "keyword")
            ])

        # Assertions
        assert 'metadata_validation' in results
        assert 'is_integrity_acceptable' in results
        assert results['is_integrity_acceptable'] is True

    def test_run_performance_validation(self):
        """Test performance validation."""
        # Mock the retrieval validator
        mock_result = Mock()
        mock_result.query = "test query"
        mock_result.results = []
        mock_result.metrics = {'latency_ms': 100}
        mock_result.status = "PASS"

        def mock_validate_single_query(query):
            return mock_result

        self.orchestrator.retrieval_validator.validate_single_query = mock_validate_single_query

        # Mock performance results
        mock_performance_metrics = Mock()
        mock_performance_metrics.avg_latency_ms = 100.0
        mock_performance_metrics.p95_latency_ms = 150.0
        mock_performance_metrics.throughput_per_second = 10.0
        mock_performance_metrics.error_rate = 0.0
        mock_performance_metrics.total_requests = 10
        mock_performance_metrics.successful_requests = 10

        mock_performance_results = {
            'metrics': mock_performance_metrics,
            'latencies': [100, 110, 90, 105, 95]
        }

        mock_validation_results = {
            'overall_status': 'PASS',
            'passed': ['all_thresholds'],
            'failed': []
        }

        # Mock the performance utilities
        self.orchestrator.performance_utils.validate_performance_thresholds.return_value = mock_validation_results

        # Since we're mocking test_retrieval_performance, patch it
        with patch('backend.src.validation.validation_orchestrator.test_retrieval_performance',
                  return_value=mock_performance_results):
            results = self.orchestrator.run_performance_validation("test query")

        # Assertions
        assert 'performance_metrics' in results
        assert 'validation_results' in results
        assert 'is_performance_acceptable' in results

    def test_generate_text_report(self):
        """Test text report generation."""
        sample_results = {
            'summary': {
                'total_tests': 10,
                'passed_tests': 8,
                'failed_tests': 2,
                'pass_rate': 0.8,
                'avg_latency_ms': 150.0
            },
            'pass_fail_determination': {
                'overall_status': 'PASS'
            },
            'execution_time_seconds': 5.5,
            'timestamp': '2023-01-01T00:00:00'
        }

        report = self.orchestrator._generate_text_report(sample_results)

        # Assertions
        assert "RAG Retrieval Validation Report" in report
        assert "Total Tests: 10" in report
        assert "Passed: 8" in report
        assert "Pass Rate: 80.00%" in report
        assert "Overall Status: PASS" in report

    def test_generate_detailed_report(self):
        """Test detailed report generation."""
        sample_results = {
            'summary': {
                'total_tests': 10,
                'passed_tests': 8,
                'failed_tests': 2,
                'pass_rate': 0.8,
                'avg_latency_ms': 150.0
            },
            'pass_fail_determination': {
                'overall_status': 'PASS'
            },
            'metadata_validation': {
                'total_chunks': 5,
                'valid_chunks': 5,
                'integrity_score': 1.0,
                'field_compliance': {
                    'source_url': {'compliance_rate': 1.0},
                    'section_title': {'compliance_rate': 1.0}
                }
            },
            'performance_validation': {
                'metrics': Mock(),
                'validation': {'overall_status': 'PASS'}
            },
            'execution_time_seconds': 5.5,
            'timestamp': '2023-01-01T00:00:00'
        }

        # Set up mock metrics
        sample_results['performance_validation']['metrics'].avg_latency_ms = 150.0
        sample_results['performance_validation']['metrics'].p95_latency_ms = 200.0
        sample_results['performance_validation']['metrics'].throughput_per_second = 8.0
        sample_results['performance_validation']['metrics'].error_rate = 0.1
        sample_results['performance_validation']['metrics'].total_requests = 10
        sample_results['performance_validation']['metrics'].successful_requests = 9

        report = self.orchestrator._generate_detailed_report(sample_results)

        # Assertions
        assert "RAG Retrieval Validation Report" in report
        assert "Metadata Validation Details:" in report
        assert "Performance Validation Details:" in report
        assert "Integrity Score: 100.00%" in report

    def test_result_to_dict_conversion(self):
        """Test conversion of ValidationResult to dictionary."""
        from backend.src.validation.retrieval_validator import ValidationResult

        validation_result = ValidationResult(
            query="test query",
            results=[{'content': 'test content', 'similarity_score': 0.8}],
            metrics={'latency_ms': 100},
            status="PASS",
            query_category="keyword"
        )

        result_dict = self.orchestrator._result_to_dict(validation_result)

        # Assertions
        assert 'query' in result_dict
        assert 'query_category' in result_dict
        assert 'results' in result_dict
        assert 'metrics' in result_dict
        assert 'status' in result_dict
        assert result_dict['query'] == "test query"
        assert result_dict['status'] == "PASS"

    def test_save_validation_results_json(self):
        """Test saving validation results in JSON format."""
        import tempfile
        import json

        sample_results = {
            'summary': {'total_tests': 1, 'passed_tests': 1},
            'timestamp': '2023-01-01T00:00:00'
        }

        # Create a temporary file
        with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
            temp_path = f.name

        try:
            # Save results
            self.orchestrator.save_validation_results(sample_results, temp_path, 'json')

            # Read and verify
            with open(temp_path, 'r') as f:
                saved_data = json.load(f)

            assert saved_data['summary']['total_tests'] == 1
            assert saved_data['summary']['passed_tests'] == 1
        finally:
            # Clean up
            import os
            os.unlink(temp_path)

    def test_save_validation_results_text(self):
        """Test saving validation results in text format."""
        import tempfile

        sample_results = {
            'summary': {'total_tests': 1, 'passed_tests': 1},
            'pass_fail_determination': {'overall_status': 'PASS'},
            'timestamp': '2023-01-01T00:00:00'
        }

        # Create a temporary file
        with tempfile.NamedTemporaryFile(mode='w', suffix='.txt', delete=False) as f:
            temp_path = f.name

        try:
            # Save results
            self.orchestrator.save_validation_results(sample_results, temp_path, 'text')

            # Read and verify
            with open(temp_path, 'r') as f:
                content = f.read()

            assert "RAG Retrieval Validation Report" in content
            assert "Total Tests: 1" in content
        finally:
            # Clean up
            import os
            os.unlink(temp_path)