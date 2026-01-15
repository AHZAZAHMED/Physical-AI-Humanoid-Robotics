"""
Statistics service for tracking query metrics and performance data.
"""
import time
import threading
from typing import Dict, Any, Optional
from dataclasses import dataclass, asdict
from datetime import datetime
from collections import defaultdict, deque
import json


@dataclass
class QueryMetrics:
    """Data class to represent query metrics."""
    query_id: str
    response_time_ms: int
    grounding_confidence: float
    top_k_used: int
    confidence_threshold: float
    retrieved_chunks_count: int
    timestamp: float
    success: bool
    error_type: Optional[str] = None


class StatsService:
    """
    Service for tracking and retrieving statistics about query performance.
    Uses in-memory storage for simplicity - in production, this would connect to a database.
    """

    def __init__(self, max_metrics_history: int = 10000):
        """
        Initialize the statistics service.

        Args:
            max_metrics_history: Maximum number of query metrics to keep in memory
        """
        self.max_metrics_history = max_metrics_history
        self._metrics: deque = deque(maxlen=max_metrics_history)
        self._lock = threading.Lock()

        # For aggregating real-time statistics
        self._query_count = 0
        self._successful_responses = 0
        self._insufficient_context_count = 0
        self._total_response_time = 0.0
        self._response_times = deque(maxlen=1000)  # Keep last 1000 response times for percentiles
        self._grounding_scores = deque(maxlen=1000)  # Keep last 1000 grounding scores

    def record_query(self, metrics: QueryMetrics) -> None:
        """
        Record metrics for a completed query.

        Args:
            metrics: QueryMetrics object containing the metrics to record
        """
        with self._lock:
            self._metrics.append(metrics)

            # Update aggregated stats
            self._query_count += 1
            if metrics.success:
                self._successful_responses += 1
                if metrics.response_time_ms is not None:
                    self._total_response_time += metrics.response_time_ms
                    self._response_times.append(metrics.response_time_ms)
                if metrics.grounding_confidence is not None:
                    self._grounding_scores.append(metrics.grounding_confidence)
            elif metrics.error_type == "InsufficientContextError":
                self._insufficient_context_count += 1

    def get_statistics(self) -> Dict[str, Any]:
        """
        Get current usage statistics and performance metrics.

        Returns:
            Dictionary containing various statistics
        """
        with self._lock:
            # Calculate response time statistics
            response_times_list = list(self._response_times)
            avg_response_time = (
                self._total_response_time / len(response_times_list)
                if response_times_list else 0
            )

            # Calculate p95 response time
            p95_response_time = 0
            if response_times_list:
                sorted_times = sorted(response_times_list)
                p95_idx = int(0.95 * len(sorted_times))
                if p95_idx < len(sorted_times):
                    p95_response_time = sorted_times[p95_idx]

            # Calculate grounding accuracy
            grounding_scores_list = list(self._grounding_scores)
            grounding_accuracy = (
                sum(grounding_scores_list) / len(grounding_scores_list)
                if grounding_scores_list else 0
            )

            return {
                "total_queries": self._query_count,
                "successful_responses": self._successful_responses,
                "insufficient_context": self._insufficient_context_count,
                "avg_response_time_ms": round(avg_response_time, 2),
                "p95_response_time_ms": p95_response_time,
                "grounding_accuracy": round(grounding_accuracy, 4),
                "timestamp": time.time(),
                "active_metrics_count": len(self._metrics)
            }

    def get_detailed_stats(self) -> Dict[str, Any]:
        """
        Get detailed statistics including recent query performance.

        Returns:
            Dictionary containing detailed statistics
        """
        with self._lock:
            # Get recent metrics
            recent_metrics = [asdict(m) for m in list(self._metrics)[-10:]]  # Last 10 queries

            return {
                **self.get_statistics(),
                "recent_queries": recent_metrics
            }


# Global instance of the stats service
# In a real application, this would be managed by a dependency injection framework
_stats_service = StatsService()


def get_stats_service() -> StatsService:
    """
    Get the global stats service instance.

    Returns:
        StatsService instance
    """
    return _stats_service