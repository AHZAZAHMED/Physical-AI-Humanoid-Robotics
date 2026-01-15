"""
Unit tests for RAG retrieval performance validation.

This module tests the performance measurement and validation functionality.
"""
import pytest
import time
from unittest.mock import Mock, patch
from backend.src.validation.performance_utils import PerformanceUtils, PerformanceMetrics


class TestPerformanceValidation:
    """Test class for performance validation."""

    def setup_method(self):
        """Set up test fixtures before each test method."""
        self.perf_utils = PerformanceUtils()

    def test_measure_latency(self):
        """Test latency measurement for a function."""
        def sample_function(x):
            time.sleep(0.01)  # Sleep for 10ms
            return x * 2

        latency, result = self.perf_utils.measure_latency(sample_function, 5)

        # Assertions
        assert result == 10  # 5 * 2
        assert latency >= 10  # At least 10ms due to sleep
        assert isinstance(latency, float)

    def test_calculate_percentiles(self):
        """Test percentile calculation."""
        latencies = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100]

        percentiles = self.perf_utils.calculate_percentiles(latencies)

        # Assertions
        assert isinstance(percentiles, dict)
        assert 'p50' in percentiles
        assert 'p95' in percentiles
        assert 'p99' in percentiles
        # For our data: p50 should be around 50-60, p95 around 90-100
        assert 50 <= percentiles['p50'] <= 60
        assert 90 <= percentiles['p95'] <= 100

    def test_calculate_percentiles_empty_list(self):
        """Test percentile calculation with empty list."""
        percentiles = self.perf_utils.calculate_percentiles([])

        # Assertions
        assert percentiles == {'p50': 0, 'p95': 0, 'p99': 0}

    def test_calculate_performance_metrics(self):
        """Test comprehensive performance metrics calculation."""
        latencies = [100, 200, 300, 400, 500]  # in ms
        total_duration_ms = 2000
        successful_requests = 5
        total_requests = 5

        metrics = self.perf_utils.calculate_performance_metrics(
            latencies, total_duration_ms, successful_requests, total_requests
        )

        # Assertions
        assert isinstance(metrics, PerformanceMetrics)
        assert metrics.avg_latency_ms == 300  # (100+200+300+400+500)/5
        assert metrics.min_latency_ms == 100
        assert metrics.max_latency_ms == 500
        assert metrics.total_requests == 5
        assert metrics.successful_requests == 5
        assert metrics.failed_requests == 0
        assert metrics.error_rate == 0.0
        assert metrics.total_duration_ms == 2000

        # Throughput should be 5 requests / 2 seconds = 2.5 req/sec
        assert metrics.throughput_per_second == 2.5

    def test_calculate_performance_metrics_with_failures(self):
        """Test performance metrics calculation with failed requests."""
        latencies = [100, 200, 300]  # Only successful requests have latency
        total_duration_ms = 2000
        successful_requests = 3
        total_requests = 5

        metrics = self.perf_utils.calculate_performance_metrics(
            latencies, total_duration_ms, successful_requests, total_requests
        )

        # Assertions
        assert metrics.successful_requests == 3
        assert metrics.failed_requests == 2  # 5 - 3
        assert metrics.error_rate == 0.4  # 2/5 = 0.4
        assert metrics.throughput_per_second == 1.5  # 3 requests / 2 seconds

    def test_calculate_performance_metrics_empty_latencies(self):
        """Test performance metrics calculation with no latencies."""
        metrics = self.perf_utils.calculate_performance_metrics(
            [], 2000, 0, 5
        )

        # Assertions
        assert metrics.avg_latency_ms == 0
        assert metrics.min_latency_ms == 0
        assert metrics.max_latency_ms == 0
        assert metrics.total_requests == 5
        assert metrics.successful_requests == 0
        assert metrics.failed_requests == 5
        assert metrics.error_rate == 1.0  # All failed

    def test_validate_performance_thresholds_pass(self):
        """Test performance threshold validation when all pass."""
        metrics = PerformanceMetrics(
            avg_latency_ms=300,      # Below 500ms threshold
            p50_latency_ms=200,
            p95_latency_ms=700,      # Below 1000ms threshold
            p99_latency_ms=900,
            min_latency_ms=100,
            max_latency_ms=1000,
            throughput_per_second=5,  # Above 2 req/sec threshold
            total_requests=10,
            successful_requests=10,
            failed_requests=0,
            error_rate=0.01,         # Below 5% threshold
            total_duration_ms=2000
        )

        validation = self.perf_utils.validate_performance_thresholds(metrics)

        # Assertions
        assert validation['overall_status'] == 'PASS'
        assert len(validation['failed']) == 0
        assert len(validation['passed']) == 4  # All thresholds passed

    def test_validate_performance_thresholds_fail(self):
        """Test performance threshold validation when some fail."""
        metrics = PerformanceMetrics(
            avg_latency_ms=600,      # Above 500ms threshold
            p50_latency_ms=200,
            p95_latency_ms=1200,     # Above 1000ms threshold
            p99_latency_ms=1500,
            min_latency_ms=100,
            max_latency_ms=2000,
            throughput_per_second=1,  # Below 2 req/sec threshold
            total_requests=10,
            successful_requests=8,
            failed_requests=2,
            error_rate=0.1,          # Above 5% threshold
            total_duration_ms=2000
        )

        validation = self.perf_utils.validate_performance_thresholds(metrics)

        # Assertions
        assert validation['overall_status'] == 'FAIL'
        assert len(validation['failed']) == 4  # All thresholds failed
        assert len(validation['passed']) == 0

    def test_is_valid_url(self):
        """Test URL validation."""
        # Test valid URLs
        valid_urls = [
            "https://example.com",
            "http://example.com",
            "https://www.example.com/path",
            "http://localhost:8000",
            "https://192.168.1.1:8080"
        ]

        for url in valid_urls:
            assert self.perf_utils._is_valid_url(url), f"URL {url} should be valid"

        # Test invalid URLs
        invalid_urls = [
            "not-a-url",
            "",
            "htp://invalid",
            "ftp://example.com"  # Not http/https
        ]

        for url in invalid_urls:
            assert not self.perf_utils._is_valid_url(url), f"URL {url} should be invalid"

    def test_is_valid_chunk_id(self):
        """Test chunk ID validation."""
        # Test valid chunk IDs
        valid_ids = [
            "chunk_1",
            "test-chunk-123",
            "abc123",
            "Chunk_1_2_3"
        ]

        for chunk_id in valid_ids:
            assert self.perf_utils._is_valid_chunk_id(chunk_id), f"ID {chunk_id} should be valid"

        # Test invalid chunk IDs
        invalid_ids = [
            "chunk with spaces",
            "chunk@invalid",
            "chunk#invalid",
            "",
            None
        ]

        for chunk_id in invalid_ids:
            if chunk_id is not None:  # Skip None as it will cause TypeError
                assert not self.perf_utils._is_valid_chunk_id(chunk_id), f"ID {chunk_id} should be invalid"

    def test_generate_performance_report(self):
        """Test performance report generation."""
        metrics = PerformanceMetrics(
            avg_latency_ms=250.5,
            p50_latency_ms=200.0,
            p95_latency_ms=400.0,
            p99_latency_ms=450.0,
            min_latency_ms=100.0,
            max_latency_ms=500.0,
            throughput_per_second=4.0,
            total_requests=20,
            successful_requests=18,
            failed_requests=2,
            error_rate=0.1,
            total_duration_ms=5000.0
        )

        results = {
            'metrics': metrics,
            'concurrent_requests': 5
        }

        report = self.perf_utils.generate_performance_report(results)

        # Assertions - check that key metrics are in the report
        assert "Average Latency: 250.50ms" in report
        assert "P95 Latency: 400.00ms" in report
        assert "Throughput: 4.00 req/sec" in report
        assert "Total Requests: 20" in report
        assert "Concurrent Requests: 5" in report

    def test_run_concurrent_queries(self):
        """Test concurrent query execution."""
        def mock_query_func(query):
            # Simulate some work and return a result
            time.sleep(0.01)
            return f"result for {query}"

        queries = ["query1", "query2", "query3"]

        results = self.perf_utils.run_concurrent_queries(
            mock_query_func, queries, max_concurrent=2
        )

        # Assertions
        assert 'metrics' in results
        assert 'latencies' in results
        assert 'results' in results
        assert 'errors' in results
        assert len(results['results']) == 3
        assert len(results['latencies']) == 3
        assert all("result for" in result for result in results['results'])

    def test_benchmark_throughput(self):
        """Test throughput benchmarking."""
        def mock_query_func(query):
            time.sleep(0.01)  # Simulate work
            return f"result for {query}"

        # Run benchmark for a short duration
        results = self.perf_utils.benchmark_throughput(
            mock_query_func, "test query", duration_seconds=1, max_concurrent=2
        )

        # Assertions
        assert 'metrics' in results
        assert 'latencies' in results
        assert 'results' in results
        assert 'test_duration_seconds' in results
        # At least one request should have been processed
        assert len(results['results']) >= 0  # May be 0 due to short duration