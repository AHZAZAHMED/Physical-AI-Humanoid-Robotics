"""
Performance measurement utilities for RAG retrieval validation.

This module provides functions for measuring and analyzing retrieval performance
including latency, throughput, and concurrent request handling.
"""

import time
import asyncio
import logging
from typing import List, Dict, Any, Callable, Awaitable
from concurrent.futures import ThreadPoolExecutor
import statistics
from dataclasses import dataclass
from .config import config

# Configure logging
logger = logging.getLogger(__name__)


@dataclass
class PerformanceMetrics:
    """Data class to hold performance metrics."""
    avg_latency_ms: float
    p50_latency_ms: float
    p95_latency_ms: float
    p99_latency_ms: float
    min_latency_ms: float
    max_latency_ms: float
    throughput_per_second: float
    total_requests: int
    successful_requests: int
    failed_requests: int
    error_rate: float
    total_duration_ms: float


class PerformanceUtils:
    """Utilities for measuring and analyzing retrieval performance."""

    def __init__(self):
        self.max_concurrent_requests = config.max_concurrent_requests

    def measure_latency(self, func: Callable, *args, **kwargs) -> tuple[float, Any]:
        """
        Measure the execution time of a function.

        Args:
            func: Function to measure
            *args: Arguments to pass to the function
            **kwargs: Keyword arguments to pass to the function

        Returns:
            tuple: (latency_in_ms, function_result)
        """
        start_time = time.time()
        result = func(*args, **kwargs)
        end_time = time.time()
        latency_ms = (end_time - start_time) * 1000
        return latency_ms, result

    async def measure_async_latency(self, func: Callable, *args, **kwargs) -> tuple[float, Any]:
        """
        Measure the execution time of an async function.

        Args:
            func: Async function to measure
            *args: Arguments to pass to the function
            **kwargs: Keyword arguments to pass to the function

        Returns:
            tuple: (latency_in_ms, function_result)
        """
        start_time = time.time()
        result = await func(*args, **kwargs)
        end_time = time.time()
        latency_ms = (end_time - start_time) * 1000
        return latency_ms, result

    def calculate_percentiles(self, latencies: List[float]) -> Dict[str, float]:
        """
        Calculate percentile latencies from a list of latency measurements.

        Args:
            latencies: List of latency measurements in milliseconds

        Returns:
            Dict: Percentile latencies
        """
        if not latencies:
            return {
                'p50': 0,
                'p95': 0,
                'p99': 0
            }

        sorted_latencies = sorted(latencies)
        n = len(sorted_latencies)

        p50_idx = int(0.50 * n)
        p95_idx = int(0.95 * n)
        p99_idx = int(0.99 * n)

        return {
            'p50': sorted_latencies[min(p50_idx, n-1)],
            'p95': sorted_latencies[min(p95_idx, n-1)],
            'p99': sorted_latencies[min(p99_idx, n-1)]
        }

    def calculate_performance_metrics(self, latencies: List[float],
                                   total_duration_ms: float,
                                   successful_requests: int,
                                   total_requests: int) -> PerformanceMetrics:
        """
        Calculate comprehensive performance metrics.

        Args:
            latencies: List of latency measurements in milliseconds
            total_duration_ms: Total duration of the test in milliseconds
            successful_requests: Number of successful requests
            total_requests: Total number of requests

        Returns:
            PerformanceMetrics: Calculated performance metrics
        """
        if not latencies:
            return PerformanceMetrics(
                avg_latency_ms=0,
                p50_latency_ms=0,
                p95_latency_ms=0,
                p99_latency_ms=0,
                min_latency_ms=0,
                max_latency_ms=0,
                throughput_per_second=0,
                total_requests=total_requests,
                successful_requests=successful_requests,
                failed_requests=total_requests - successful_requests,
                error_rate=1.0 if total_requests > 0 else 0,
                total_duration_ms=total_duration_ms
            )

        percentiles = self.calculate_percentiles(latencies)

        avg_latency = statistics.mean(latencies)
        min_latency = min(latencies)
        max_latency = max(latencies)

        # Calculate throughput (requests per second)
        total_duration_sec = total_duration_ms / 1000
        throughput = successful_requests / total_duration_sec if total_duration_sec > 0 else 0

        error_rate = (total_requests - successful_requests) / total_requests if total_requests > 0 else 0

        return PerformanceMetrics(
            avg_latency_ms=avg_latency,
            p50_latency_ms=percentiles['p50'],
            p95_latency_ms=percentiles['p95'],
            p99_latency_ms=percentiles['p99'],
            min_latency_ms=min_latency,
            max_latency_ms=max_latency,
            throughput_per_second=throughput,
            total_requests=total_requests,
            successful_requests=successful_requests,
            failed_requests=total_requests - successful_requests,
            error_rate=error_rate,
            total_duration_ms=total_duration_ms
        )

    def run_concurrent_queries(self, query_func: Callable, queries: List,
                              max_concurrent: int = None) -> Dict[str, Any]:
        """
        Run queries concurrently and measure performance.

        Args:
            query_func: Function to execute queries (should accept a single query as argument)
            queries: List of queries to execute
            max_concurrent: Maximum number of concurrent requests (defaults to config value)

        Returns:
            Dict: Performance results and metrics
        """
        if max_concurrent is None:
            max_concurrent = self.max_concurrent_requests

        latencies = []
        results = []
        errors = []
        start_time = time.time()

        def execute_query(query):
            try:
                latency, result = self.measure_latency(query_func, query)
                return latency, result, None
            except Exception as e:
                return 0, None, str(e)

        with ThreadPoolExecutor(max_workers=max_concurrent) as executor:
            futures = [executor.submit(execute_query, query) for query in queries]

            for future in futures:
                latency, result, error = future.result()
                if error:
                    errors.append(error)
                else:
                    latencies.append(latency)
                    results.append(result)

        total_duration_ms = (time.time() - start_time) * 1000
        successful_requests = len(latencies)
        total_requests = len(queries)

        metrics = self.calculate_performance_metrics(
            latencies, total_duration_ms, successful_requests, total_requests
        )

        return {
            'metrics': metrics,
            'latencies': latencies,
            'results': results,
            'errors': errors,
            'concurrent_requests': max_concurrent
        }

    async def run_async_concurrent_queries(self, query_func: Callable, queries: List,
                                         max_concurrent: int = None) -> Dict[str, Any]:
        """
        Run queries concurrently using async/await and measure performance.

        Args:
            query_func: Async function to execute queries
            queries: List of queries to execute
            max_concurrent: Maximum number of concurrent requests

        Returns:
            Dict: Performance results and metrics
        """
        if max_concurrent is None:
            max_concurrent = self.max_concurrent_requests

        latencies = []
        results = []
        errors = []
        semaphore = asyncio.Semaphore(max_concurrent)
        start_time = time.time()

        async def execute_query(query):
            async with semaphore:
                try:
                    latency, result = await self.measure_async_latency(query_func, query)
                    return latency, result, None
                except Exception as e:
                    return 0, None, str(e)

        tasks = [execute_query(query) for query in queries]
        completed_tasks = await asyncio.gather(*tasks, return_exceptions=True)

        for task_result in completed_tasks:
            if isinstance(task_result, Exception):
                errors.append(str(task_result))
            else:
                latency, result, error = task_result
                if error:
                    errors.append(error)
                else:
                    latencies.append(latency)
                    results.append(result)

        total_duration_ms = (time.time() - start_time) * 1000
        successful_requests = len(latencies)
        total_requests = len(queries)

        metrics = self.calculate_performance_metrics(
            latencies, total_duration_ms, successful_requests, total_requests
        )

        return {
            'metrics': metrics,
            'latencies': latencies,
            'results': results,
            'errors': errors,
            'concurrent_requests': max_concurrent
        }

    def benchmark_throughput(self, query_func: Callable, query: str,
                           duration_seconds: int = 30,
                           max_concurrent: int = None) -> Dict[str, Any]:
        """
        Benchmark throughput by running queries for a specified duration.

        Args:
            query_func: Function to execute queries
            query: Query to execute repeatedly
            duration_seconds: Duration to run the benchmark in seconds
            max_concurrent: Maximum number of concurrent requests

        Returns:
            Dict: Throughput benchmark results
        """
        if max_concurrent is None:
            max_concurrent = self.max_concurrent_requests

        latencies = []
        results = []
        errors = []
        start_time = time.time()

        def execute_query(query):
            try:
                latency, result = self.measure_latency(query_func, query)
                return latency, result, None
            except Exception as e:
                return 0, None, str(e)

        with ThreadPoolExecutor(max_workers=max_concurrent) as executor:
            futures = []
            # Submit queries for the duration
            while (time.time() - start_time) < duration_seconds:
                future = executor.submit(execute_query, query)
                futures.append(future)

            # Wait for all queries to complete
            for future in futures:
                latency, result, error = future.result()
                if (time.time() - start_time) > duration_seconds:
                    break
                if error:
                    errors.append(error)
                else:
                    latencies.append(latency)
                    results.append(result)

        total_duration_ms = (time.time() - start_time) * 1000
        successful_requests = len(latencies)
        total_requests = len(futures)

        metrics = self.calculate_performance_metrics(
            latencies, total_duration_ms, successful_requests, total_requests
        )

        return {
            'metrics': metrics,
            'latencies': latencies,
            'results': results,
            'errors': errors,
            'test_duration_seconds': duration_seconds,
            'max_concurrent': max_concurrent
        }

    def validate_performance_thresholds(self, metrics: PerformanceMetrics) -> Dict[str, Any]:
        """
        Validate performance against defined thresholds.

        Args:
            metrics: PerformanceMetrics object to validate

        Returns:
            Dict: Validation results with pass/fail status
        """
        # Define performance thresholds
        thresholds = {
            'max_avg_latency_ms': 500,  # 500ms average latency
            'max_p95_latency_ms': 1000,  # 1000ms 95th percentile latency
            'min_throughput_per_second': 2,  # 2 requests per second
            'max_error_rate': 0.05  # 5% error rate
        }

        validation_results = {
            'thresholds': thresholds,
            'actual_values': {
                'avg_latency_ms': metrics.avg_latency_ms,
                'p95_latency_ms': metrics.p95_latency_ms,
                'throughput_per_second': metrics.throughput_per_second,
                'error_rate': metrics.error_rate
            },
            'passed': [],
            'failed': [],
            'overall_status': 'PASS'
        }

        # Check average latency
        if metrics.avg_latency_ms <= thresholds['max_avg_latency_ms']:
            validation_results['passed'].append('avg_latency_threshold')
        else:
            validation_results['failed'].append('avg_latency_threshold')

        # Check 95th percentile latency
        if metrics.p95_latency_ms <= thresholds['max_p95_latency_ms']:
            validation_results['passed'].append('p95_latency_threshold')
        else:
            validation_results['failed'].append('p95_latency_threshold')

        # Check throughput
        if metrics.throughput_per_second >= thresholds['min_throughput_per_second']:
            validation_results['passed'].append('throughput_threshold')
        else:
            validation_results['failed'].append('throughput_threshold')

        # Check error rate
        if metrics.error_rate <= thresholds['max_error_rate']:
            validation_results['passed'].append('error_rate_threshold')
        else:
            validation_results['failed'].append('error_rate_threshold')

        # Determine overall status
        if validation_results['failed']:
            validation_results['overall_status'] = 'FAIL'

        return validation_results

    def generate_performance_report(self, performance_results: Dict[str, Any]) -> str:
        """
        Generate a human-readable performance report.

        Args:
            performance_results: Results from performance testing

        Returns:
            str: Formatted performance report
        """
        metrics = performance_results.get('metrics', {})
        if isinstance(metrics, PerformanceMetrics):
            report = f"""
RAG Retrieval Performance Report
================================

Performance Metrics:
- Average Latency: {metrics.avg_latency_ms:.2f}ms
- P50 Latency: {metrics.p50_latency_ms:.2f}ms
- P95 Latency: {metrics.p95_latency_ms:.2f}ms
- P99 Latency: {metrics.p99_latency_ms:.2f}ms
- Min Latency: {metrics.min_latency_ms:.2f}ms
- Max Latency: {metrics.max_latency_ms:.2f}ms
- Throughput: {metrics.throughput_per_second:.2f} req/sec
- Total Requests: {metrics.total_requests}
- Successful Requests: {metrics.successful_requests}
- Failed Requests: {metrics.failed_requests}
- Error Rate: {metrics.error_rate:.2%}
- Total Duration: {metrics.total_duration_ms:.2f}ms

Concurrent Requests: {performance_results.get('concurrent_requests', 'N/A')}
"""
        else:
            report = "Performance report: Metrics not available"

        # Add validation results if present
        validation = performance_results.get('validation', {})
        if validation:
            report += f"\nThreshold Validation:\n"
            report += f"- Overall Status: {validation.get('overall_status', 'N/A')}\n"
            report += f"- Passed: {', '.join(validation.get('passed', [])) or 'None'}\n"
            report += f"- Failed: {', '.join(validation.get('failed', [])) or 'None'}\n"

        return report


# Convenience functions for common performance testing scenarios
def test_retrieval_performance(query_func: Callable, test_queries: List[str]) -> Dict[str, Any]:
    """
    Convenience function to test retrieval performance with a set of queries.

    Args:
        query_func: Function that performs the retrieval (should accept a query string)
        test_queries: List of queries to test with

    Returns:
        Dict: Performance test results
    """
    perf_utils = PerformanceUtils()
    return perf_utils.run_concurrent_queries(query_func, test_queries)


def benchmark_retrieval_throughput(query_func: Callable, sample_query: str,
                                 duration: int = 30) -> Dict[str, Any]:
    """
    Convenience function to benchmark retrieval throughput.

    Args:
        query_func: Function that performs the retrieval
        sample_query: Sample query to test with
        duration: Duration of the benchmark in seconds

    Returns:
        Dict: Throughput benchmark results
    """
    perf_utils = PerformanceUtils()
    return perf_utils.benchmark_throughput(query_func, sample_query, duration)