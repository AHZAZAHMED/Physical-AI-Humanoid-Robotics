"""
Main validation orchestrator for RAG retrieval validation system.

This module coordinates all validation components to execute comprehensive
RAG retrieval validation workflows.
"""

import logging
import time
import json
from typing import List, Dict, Any, Tuple
from datetime import datetime

from .retrieval_validator import RetrievalValidator, ValidationResult
from .query_generator import QueryGenerator
from .result_analyzer import ResultAnalyzer
from .metadata_validator import MetadataValidator, validate_metadata_for_retrieval_results
from .chunk_validator import validate_retrieval_for_validation_result
from .performance_utils import PerformanceUtils, test_retrieval_performance
from .config import config

# Configure logging
logger = logging.getLogger(__name__)


class ValidationOrchestrator:
    """Main orchestrator for RAG retrieval validation."""

    def __init__(self):
        """Initialize the validation orchestrator with all required components."""
        self.retrieval_validator = RetrievalValidator()
        self.query_generator = QueryGenerator()
        self.result_analyzer = ResultAnalyzer()
        self.metadata_validator = MetadataValidator()
        self.performance_utils = PerformanceUtils()
        self.chunk_validator = None  # Lazy load to avoid dependency issues

        # Validation parameters
        self.similarity_threshold = config.similarity_threshold
        self.top_k_results = config.top_k_results

    def run_comprehensive_validation(self,
                                   custom_queries: List[Tuple[str, str]] = None,
                                   query_counts: Dict[str, int] = None,
                                   validate_metadata: bool = True,
                                   validate_performance: bool = True,
                                   validate_chunks: bool = True) -> Dict[str, Any]:
        """
        Run comprehensive RAG retrieval validation.

        Args:
            custom_queries: List of (query, category) tuples to use instead of generated ones
            query_counts: Dictionary with counts for each query category
            validate_metadata: Whether to run metadata validation
            validate_performance: Whether to run performance validation
            validate_chunks: Whether to run chunk-level validation

        Returns:
            Dict: Comprehensive validation results
        """
        start_time = time.time()
        logger.info("Starting comprehensive RAG retrieval validation")

        # Generate or use custom queries
        if custom_queries:
            queries = custom_queries
        else:
            queries = self.query_generator.generate_test_queries(query_counts)

        logger.info(f"Generated {len(queries)} test queries")

        # Execute retrieval validation
        retrieval_results = self.retrieval_validator.validate_batch_queries(queries)
        logger.info(f"Completed retrieval validation for {len(retrieval_results)} queries")

        # Add results to analyzer
        self.result_analyzer.add_validation_results(retrieval_results)

        # Initialize results dictionary
        comprehensive_results = {
            'summary': self.result_analyzer.generate_summary(),
            'retrieval_results': [self._result_to_dict(r) for r in retrieval_results],
            'metadata_validation': None,
            'performance_validation': None,
            'chunk_validation': None,
            'pass_fail_determination': self.result_analyzer.create_pass_fail_determination(),
            'timestamp': datetime.now().isoformat(),
            'execution_time_seconds': time.time() - start_time
        }

        # Run metadata validation if requested
        if validate_metadata:
            logger.info("Starting metadata validation")
            metadata_results = validate_metadata_for_retrieval_results(retrieval_results)
            comprehensive_results['metadata_validation'] = metadata_results

        # Run performance validation if requested
        if validate_performance:
            logger.info("Starting performance validation")
            # Use the first few queries to test performance
            test_queries = [q[0] for q in queries[:min(5, len(queries))]]

            def query_func(query):
                return self.retrieval_validator.validate_single_query(query)

            performance_results = test_retrieval_performance(query_func, test_queries)
            validation_results = self.performance_utils.validate_performance_thresholds(
                performance_results['metrics']
            )
            performance_results['validation'] = validation_results

            comprehensive_results['performance_validation'] = performance_results

        # Run chunk validation if requested
        if validate_chunks:
            logger.info("Starting chunk validation")
            try:
                # Import here to avoid circular dependencies
                from .chunk_validator import validate_retrieval_for_validation_result
                chunk_validations = []

                for result in retrieval_results:
                    chunk_validation = validate_retrieval_for_validation_result(result)
                    chunk_validations.append(chunk_validation)

                comprehensive_results['chunk_validation'] = chunk_validations
            except ImportError:
                logger.warning("Chunk validator not available, skipping chunk validation")
                comprehensive_results['chunk_validation'] = None

        logger.info("Completed comprehensive RAG retrieval validation")
        return comprehensive_results

    def run_content_relevance_validation(self, queries: List[Tuple[str, str]] = None) -> Dict[str, Any]:
        """
        Run focused validation on content relevance.

        Args:
            queries: List of (query, category) tuples to validate

        Returns:
            Dict: Content relevance validation results
        """
        if queries is None:
            queries = self.query_generator.generate_test_queries({
                'keyword': 3,
                'semantic': 3,
                'section-specific': 2
            })

        logger.info(f"Starting content relevance validation for {len(queries)} queries")

        results = self.retrieval_validator.validate_batch_queries(queries)
        self.result_analyzer.add_validation_results(results)

        # Calculate relevance metrics
        relevant_results_count = 0
        total_results = 0

        for result in results:
            for res in result.results:
                total_results += 1
                if res.get('relevance', False):
                    relevant_results_count += 1

        relevance_rate = relevant_results_count / total_results if total_results > 0 else 0

        content_relevance_results = {
            'total_results': total_results,
            'relevant_results': relevant_results_count,
            'relevance_rate': relevance_rate,
            'min_acceptable_rate': 0.7,  # 70% threshold
            'is_acceptable': relevance_rate >= 0.7,
            'validation_results': [self._result_to_dict(r) for r in results],
            'timestamp': datetime.now().isoformat()
        }

        logger.info(f"Content relevance validation completed: {relevance_rate:.2%} relevance rate")
        return content_relevance_results

    def run_metadata_integrity_validation(self, queries: List[Tuple[str, str]] = None) -> Dict[str, Any]:
        """
        Run focused validation on metadata integrity.

        Args:
            queries: List of (query, category) tuples to validate

        Returns:
            Dict: Metadata integrity validation results
        """
        if queries is None:
            queries = self.query_generator.generate_test_queries({
                'keyword': 2,
                'semantic': 2,
                'section-specific': 1
            })

        logger.info(f"Starting metadata integrity validation for {len(queries)} queries")

        results = self.retrieval_validator.validate_batch_queries(queries)

        # Validate metadata for all retrieved chunks
        metadata_results = validate_metadata_for_retrieval_results(results)

        metadata_integrity_results = {
            'metadata_validation': metadata_results,
            'is_integrity_acceptable': metadata_results['integrity_score'] >= 0.9,  # 90% threshold
            'timestamp': datetime.now().isoformat()
        }

        logger.info(f"Metadata integrity validation completed: {metadata_results['integrity_score']:.2%} integrity score")
        return metadata_integrity_results

    def run_performance_validation(self, sample_query: str = None) -> Dict[str, Any]:
        """
        Run focused validation on performance metrics.

        Args:
            sample_query: Query to use for performance testing

        Returns:
            Dict: Performance validation results
        """
        if sample_query is None:
            sample_query = "Physical AI concepts"  # Default sample query

        logger.info(f"Starting performance validation with query: '{sample_query}'")

        def query_func(query):
            return self.retrieval_validator.validate_single_query(query)

        # Run performance test
        performance_results = test_retrieval_performance(query_func, [sample_query] * 10)  # 10 iterations
        validation_results = self.performance_utils.validate_performance_thresholds(
            performance_results['metrics']
        )
        performance_results['validation'] = validation_results

        performance_validation_results = {
            'performance_metrics': performance_results['metrics'],
            'validation_results': validation_results,
            'latencies': performance_results['latencies'],
            'is_performance_acceptable': validation_results['overall_status'] == 'PASS',
            'timestamp': datetime.now().isoformat()
        }

        logger.info(f"Performance validation completed: {performance_results['metrics'].avg_latency_ms:.2f}ms avg latency")
        return performance_validation_results

    def generate_final_validation_report(self, comprehensive_results: Dict[str, Any],
                                      output_format: str = 'json') -> str:
        """
        Generate a final validation report.

        Args:
            comprehensive_results: Results from comprehensive validation
            output_format: Format for the report ('json', 'text', 'detailed')

        Returns:
            str: Generated report
        """
        if output_format.lower() == 'json':
            return json.dumps(comprehensive_results, indent=2, default=str)
        elif output_format.lower() == 'text':
            return self._generate_text_report(comprehensive_results)
        elif output_format.lower() == 'detailed':
            return self._generate_detailed_report(comprehensive_results)
        else:
            raise ValueError(f"Unsupported output format: {output_format}")

    def _generate_text_report(self, results: Dict[str, Any]) -> str:
        """Generate a simple text report."""
        summary = results.get('summary', {})
        pass_fail = results.get('pass_fail_determination', {})

        report = f"""
RAG Retrieval Validation Report
===============================

Execution Time: {results.get('execution_time_seconds', 0):.2f} seconds
Timestamp: {results.get('timestamp', 'N/A')}

Summary:
--------
Total Tests: {summary.get('total_tests', 0)}
Passed: {summary.get('passed_tests', 0)}
Failed: {summary.get('failed_tests', 0)}
Error: {summary.get('error_tests', 0)}
Pass Rate: {summary.get('pass_rate', 0):.2%}
Avg Latency: {summary.get('avg_latency_ms', 0):.2f}ms

Overall Status: {pass_fail.get('overall_status', 'N/A')}

Validation Components:
----------------------
Metadata Integrity: {results.get('metadata_validation', {}).get('integrity_score', 0):.2%}
Performance Status: {results.get('performance_validation', {}).get('validation', {}).get('overall_status', 'N/A')}
"""

        return report

    def _generate_detailed_report(self, results: Dict[str, Any]) -> str:
        """Generate a detailed validation report."""
        report = self._generate_text_report(results)

        # Add metadata validation details
        metadata_validation = results.get('metadata_validation')
        if metadata_validation:
            report += f"""
Metadata Validation Details:
--------------------------
Total Chunks: {metadata_validation.get('total_chunks', 0)}
Valid Chunks: {metadata_validation.get('valid_chunks', 0)}
Invalid Chunks: {metadata_validation.get('invalid_chunks', 0)}
Integrity Score: {metadata_validation.get('integrity_score', 0):.2%}

Field Compliance:
"""
            for field, compliance in metadata_validation.get('field_compliance', {}).items():
                report += f"  {field}: {compliance.get('compliance_rate', 0):.2%}\n"

        # Add performance validation details
        perf_validation = results.get('performance_validation')
        if perf_validation and 'metrics' in perf_validation:
            metrics = perf_validation['metrics']
            report += f"""
Performance Validation Details:
----------------------------
Average Latency: {metrics.avg_latency_ms:.2f}ms
P95 Latency: {metrics.p95_latency_ms:.2f}ms
Throughput: {metrics.throughput_per_second:.2f} req/sec
Error Rate: {metrics.error_rate:.2%}
Total Requests: {metrics.total_requests}
Successful Requests: {metrics.successful_requests}
"""

        return report

    def _result_to_dict(self, result: ValidationResult) -> Dict[str, Any]:
        """Convert ValidationResult to dictionary for serialization."""
        return {
            'query': result.query,
            'query_category': result.query_category,
            'results': result.results,
            'metrics': result.metrics,
            'status': result.status,
            'timestamp': result.timestamp
        }

    def save_validation_results(self, results: Dict[str, Any], filepath: str,
                              format: str = 'json'):
        """
        Save validation results to a file.

        Args:
            results: Validation results to save
            filepath: Path to save the results
            format: Format to save ('json' or 'text')
        """
        if format.lower() == 'json':
            with open(filepath, 'w', encoding='utf-8') as f:
                json.dump(results, f, indent=2, default=str)
        elif format.lower() == 'text':
            report = self._generate_text_report(results)
            with open(filepath, 'w', encoding='utf-8') as f:
                f.write(report)
        else:
            raise ValueError(f"Unsupported format: {format}")

        logger.info(f"Validation results saved to {filepath}")


def run_validation_pipeline(config_overrides: Dict[str, Any] = None) -> Dict[str, Any]:
    """
    Run the complete validation pipeline with default settings.

    Args:
        config_overrides: Optional configuration overrides

    Returns:
        Dict: Complete validation results
    """
    orchestrator = ValidationOrchestrator()

    # Apply config overrides if provided
    if config_overrides:
        for key, value in config_overrides.items():
            if hasattr(config, key):
                setattr(config, key, value)

    # Run comprehensive validation
    results = orchestrator.run_comprehensive_validation(
        validate_metadata=True,
        validate_performance=True,
        validate_chunks=True
    )

    return results


def run_quick_validation(sample_query: str = "Physical AI concepts") -> Dict[str, Any]:
    """
    Run a quick validation with a single sample query.

    Args:
        sample_query: Query to use for quick validation

    Returns:
        Dict: Quick validation results
    """
    orchestrator = ValidationOrchestrator()

    # Run quick content relevance validation
    queries = [(sample_query, "keyword")]
    results = orchestrator.run_content_relevance_validation(queries)

    return results