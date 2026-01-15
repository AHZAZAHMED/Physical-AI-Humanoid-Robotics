"""
Unit tests for the stats service.
"""
import pytest
import time
from src.services.stats_service import StatsService, QueryMetrics


class TestStatsService:
    """Test class for StatsService functionality."""

    def setup_method(self):
        """Set up test fixtures before each test method."""
        self.stats_service = StatsService(max_metrics_history=100)

    def test_initialization(self):
        """Test that StatsService initializes correctly."""
        assert self.stats_service._query_count == 0
        assert self.stats_service._successful_responses == 0
        assert self.stats_service._insufficient_context_count == 0
        assert self.stats_service._total_response_time == 0.0
        assert len(self.stats_service._response_times) == 0
        assert len(self.stats_service._grounding_scores) == 0

    def test_record_query_success(self):
        """Test recording a successful query."""
        metrics = QueryMetrics(
            query_id="test-query-123",
            response_time_ms=150,
            grounding_confidence=0.85,
            top_k_used=5,
            confidence_threshold=0.7,
            retrieved_chunks_count=3,
            timestamp=time.time(),
            success=True
        )

        self.stats_service.record_query(metrics)

        assert self.stats_service._query_count == 1
        assert self.stats_service._successful_responses == 1
        assert self.stats_service._total_response_time == 150.0
        assert len(self.stats_service._response_times) == 1
        assert len(self.stats_service._grounding_scores) == 1

    def test_record_query_failure(self):
        """Test recording a failed query."""
        metrics = QueryMetrics(
            query_id="test-query-124",
            response_time_ms=200,
            grounding_confidence=0.0,
            top_k_used=5,
            confidence_threshold=0.7,
            retrieved_chunks_count=0,
            timestamp=time.time(),
            success=False,
            error_type="ValidationError"
        )

        self.stats_service.record_query(metrics)

        assert self.stats_service._query_count == 1
        assert self.stats_service._successful_responses == 0
        assert self.stats_service._total_response_time == 0.0  # Failed queries don't count toward response time
        assert len(self.stats_service._response_times) == 0
        assert len(self.stats_service._grounding_scores) == 0

    def test_record_insufficient_context_error(self):
        """Test recording a query with insufficient context error."""
        metrics = QueryMetrics(
            query_id="test-query-125",
            response_time_ms=100,
            grounding_confidence=0.0,
            top_k_used=5,
            confidence_threshold=0.7,
            retrieved_chunks_count=0,
            timestamp=time.time(),
            success=False,
            error_type="InsufficientContextError"
        )

        self.stats_service.record_query(metrics)

        assert self.stats_service._query_count == 1
        assert self.stats_service._successful_responses == 0
        assert self.stats_service._insufficient_context_count == 1
        assert self.stats_service._total_response_time == 0.0
        assert len(self.stats_service._response_times) == 0
        assert len(self.stats_service._grounding_scores) == 0

    def test_get_statistics(self):
        """Test getting statistics."""
        # Add some metrics
        metrics1 = QueryMetrics(
            query_id="test-query-123",
            response_time_ms=150,
            grounding_confidence=0.85,
            top_k_used=5,
            confidence_threshold=0.7,
            retrieved_chunks_count=3,
            timestamp=time.time(),
            success=True
        )
        self.stats_service.record_query(metrics1)

        metrics2 = QueryMetrics(
            query_id="test-query-124",
            response_time_ms=200,
            grounding_confidence=0.0,
            top_k_used=5,
            confidence_threshold=0.7,
            retrieved_chunks_count=0,
            timestamp=time.time(),
            success=False,
            error_type="ValidationError"
        )
        self.stats_service.record_query(metrics2)

        stats = self.stats_service.get_statistics()

        assert stats["total_queries"] == 2
        assert stats["successful_responses"] == 1
        assert stats["insufficient_context"] == 0  # This was a ValidationError, not InsufficientContextError
        assert stats["avg_response_time_ms"] == 150.0  # Only successful queries contribute to avg response time
        assert stats["grounding_accuracy"] == 0.85  # Only successful queries contribute to grounding accuracy

    def test_get_detailed_stats(self):
        """Test getting detailed statistics."""
        # Add some metrics
        metrics = QueryMetrics(
            query_id="test-query-123",
            response_time_ms=150,
            grounding_confidence=0.85,
            top_k_used=5,
            confidence_threshold=0.7,
            retrieved_chunks_count=3,
            timestamp=time.time(),
            success=True
        )
        self.stats_service.record_query(metrics)

        detailed_stats = self.stats_service.get_detailed_stats()

        assert "recent_queries" in detailed_stats
        assert len(detailed_stats["recent_queries"]) >= 1
        assert detailed_stats["total_queries"] == 1

    def test_cache_size_limit(self):
        """Test that metrics history is limited by max_metrics_history."""
        # Create a service with a small history limit
        limited_service = StatsService(max_metrics_history=3)

        # Add more metrics than the limit
        for i in range(5):
            metrics = QueryMetrics(
                query_id=f"test-query-{i}",
                response_time_ms=100 + i * 10,
                grounding_confidence=0.8 + i * 0.01,
                top_k_used=5,
                confidence_threshold=0.7,
                retrieved_chunks_count=2,
                timestamp=time.time(),
                success=True
            )
            limited_service.record_query(metrics)

        # The service should only keep the last 3 metrics
        stats = limited_service.get_statistics()
        assert stats["active_metrics_count"] <= 3


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
            timestamp=time.time(),
            success=True,
            error_type="TestError"
        )

        assert metrics.query_id == "test-query-123"
        assert metrics.response_time_ms == 150
        assert metrics.grounding_confidence == 0.85
        assert metrics.top_k_used == 5
        assert metrics.confidence_threshold == 0.7
        assert metrics.retrieved_chunks_count == 3
        assert metrics.success is True
        assert metrics.error_type == "TestError"

    def test_query_metrics_creation_optional_error_type(self):
        """Test creating QueryMetrics instance without error_type."""
        metrics = QueryMetrics(
            query_id="test-query-123",
            response_time_ms=150,
            grounding_confidence=0.85,
            top_k_used=5,
            confidence_threshold=0.7,
            retrieved_chunks_count=3,
            timestamp=time.time(),
            success=True
        )

        assert metrics.error_type is None