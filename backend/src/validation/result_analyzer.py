"""
Result analysis and reporting module for RAG retrieval validation.

This module processes validation results and generates comprehensive reports.
"""

import json
import logging
from typing import List, Dict, Any
from datetime import datetime
from .retrieval_validator import ValidationResult

# Configure logging
logger = logging.getLogger(__name__)


class ResultAnalyzer:
    """Processes validation results and generates reports."""

    def __init__(self):
        self.validation_results = []

    def add_validation_result(self, result: ValidationResult):
        """
        Add a validation result to the analyzer.

        Args:
            result: ValidationResult object to add
        """
        self.validation_results.append(result)

    def add_validation_results(self, results: List[ValidationResult]):
        """
        Add multiple validation results to the analyzer.

        Args:
            results: List of ValidationResult objects to add
        """
        for result in results:
            self.add_validation_result(result)

    def generate_summary(self) -> Dict[str, Any]:
        """
        Generate a summary of all validation results.

        Returns:
            Dict: Summary statistics and metrics
        """
        if not self.validation_results:
            return {"message": "No validation results to analyze"}

        total_tests = len(self.validation_results)
        passed_tests = sum(1 for r in self.validation_results if r.status == "PASS")
        failed_tests = sum(1 for r in self.validation_results if r.status == "FAIL")
        error_tests = sum(1 for r in self.validation_results if r.status == "ERROR")

        # Calculate average metrics
        avg_latency = sum(r.metrics.get('latency_ms', 0) for r in self.validation_results) / total_tests if total_tests > 0 else 0
        avg_similarity = sum(r.metrics.get('avg_similarity', 0) for r in self.validation_results if 'avg_similarity' in r.metrics) / total_tests if total_tests > 0 else 0
        avg_top_k_recall = sum(r.metrics.get('top_k_recall', 0) for r in self.validation_results if 'top_k_recall' in r.metrics) / total_tests if total_tests > 0 else 0

        # Calculate by category
        category_stats = {}
        for result in self.validation_results:
            category = result.query_category
            if category not in category_stats:
                category_stats[category] = {
                    'total': 0,
                    'passed': 0,
                    'failed': 0,
                    'errors': 0,
                    'avg_latency': 0,
                    'avg_similarity': 0
                }

            category_stats[category]['total'] += 1
            if result.status == "PASS":
                category_stats[category]['passed'] += 1
            elif result.status == "FAIL":
                category_stats[category]['failed'] += 1
            elif result.status == "ERROR":
                category_stats[category]['errors'] += 1

            # Add metrics for averaging
            category_stats[category]['avg_latency'] += result.metrics.get('latency_ms', 0)
            if 'avg_similarity' in result.metrics:
                category_stats[category]['avg_similarity'] += result.metrics['avg_similarity']

        # Calculate averages per category
        for category in category_stats:
            stats = category_stats[category]
            count = stats['total']
            stats['avg_latency'] /= count if count > 0 else 0
            stats['avg_similarity'] /= count if count > 0 else 0
            stats['pass_rate'] = stats['passed'] / count if count > 0 else 0

        summary = {
            "total_tests": total_tests,
            "passed_tests": passed_tests,
            "failed_tests": failed_tests,
            "error_tests": error_tests,
            "pass_rate": passed_tests / total_tests if total_tests > 0 else 0,
            "avg_latency_ms": avg_latency,
            "avg_similarity": avg_similarity,
            "avg_top_k_recall": avg_top_k_recall,
            "category_breakdown": category_stats,
            "timestamp": datetime.now().isoformat()
        }

        return summary

    def generate_detailed_report(self) -> Dict[str, Any]:
        """
        Generate a detailed report of validation results.

        Returns:
            Dict: Detailed report with individual test results
        """
        summary = self.generate_summary()

        detailed_results = []
        for result in self.validation_results:
            detailed_result = {
                "query": result.query,
                "query_category": result.query_category,
                "status": result.status,
                "metrics": result.metrics,
                "result_count": len(result.results),
                "relevant_results": sum(1 for r in result.results if r.get('relevance', False)),
                "timestamp": result.timestamp
            }
            detailed_results.append(detailed_result)

        report = {
            "summary": summary,
            "detailed_results": detailed_results,
            "report_generated_at": datetime.now().isoformat()
        }

        return report

    def create_pass_fail_determination(self) -> Dict[str, Any]:
        """
        Create pass/fail determination based on success criteria.

        Returns:
            Dict: Pass/fail status and reasons
        """
        summary = self.generate_summary()

        # Define success criteria based on the spec
        latency_threshold = 500  # ms
        min_pass_rate = 0.95  # 95%

        is_latency_acceptable = summary.get('avg_latency_ms', float('inf')) <= latency_threshold
        is_pass_rate_acceptable = summary.get('pass_rate', 0) >= min_pass_rate

        overall_status = "PASS" if (is_latency_acceptable and is_pass_rate_acceptable) else "FAIL"

        determination = {
            "overall_status": overall_status,
            "criteria_met": {
                "latency_acceptable": is_latency_acceptable,
                "pass_rate_acceptable": is_pass_rate_acceptable
            },
            "thresholds": {
                "max_avg_latency_ms": latency_threshold,
                "min_pass_rate": min_pass_rate
            },
            "actual_values": {
                "avg_latency_ms": summary.get('avg_latency_ms', 0),
                "pass_rate": summary.get('pass_rate', 0)
            },
            "reasons": []
        }

        if not is_latency_acceptable:
            determination["reasons"].append(f"Average latency {summary.get('avg_latency_ms', 0)}ms exceeds threshold of {latency_threshold}ms")
        if not is_pass_rate_acceptable:
            determination["reasons"].append(f"Pass rate {summary.get('pass_rate', 0):.2%} is below required {min_pass_rate:.0%}")

        return determination

    def generate_error_report(self) -> Dict[str, Any]:
        """
        Generate a report of all failed and error tests.

        Returns:
            Dict: Error report with details about failures
        """
        error_results = [r for r in self.validation_results if r.status in ["FAIL", "ERROR"]]

        if not error_results:
            return {"message": "No errors or failures found in validation results"}

        error_details = []
        for result in error_results:
            error_detail = {
                "query": result.query,
                "query_category": result.query_category,
                "status": result.status,
                "metrics": result.metrics,
                "timestamp": result.timestamp
            }

            # Add error-specific details
            if result.status == "ERROR":
                error_detail["error_message"] = result.metrics.get("error", "Unknown error")

            error_details.append(error_detail)

        error_report = {
            "total_errors_failures": len(error_results),
            "error_details": error_details,
            "timestamp": datetime.now().isoformat()
        }

        return error_report

    def save_report(self, filepath: str, report_format: str = "json"):
        """
        Save the validation report to a file.

        Args:
            filepath: Path to save the report
            report_format: Format to save ('json', 'txt')
        """
        report = self.generate_detailed_report()

        if report_format.lower() == "json":
            with open(filepath, 'w', encoding='utf-8') as f:
                json.dump(report, f, indent=2, ensure_ascii=False)
        elif report_format.lower() == "txt":
            with open(filepath, 'w', encoding='utf-8') as f:
                f.write(f"RAG Retrieval Validation Report\n")
                f.write(f"Generated: {datetime.now().isoformat()}\n")
                f.write(f"Total Tests: {report['summary']['total_tests']}\n")
                f.write(f"Passed: {report['summary']['passed_tests']}\n")
                f.write(f"Failed: {report['summary']['failed_tests']}\n")
                f.write(f"Errors: {report['summary']['error_tests']}\n")
                f.write(f"Pass Rate: {report['summary']['pass_rate']:.2%}\n")
                f.write(f"Average Latency: {report['summary']['avg_latency_ms']:.2f}ms\n")

                f.write(f"\nCategory Breakdown:\n")
                for category, stats in report['summary']['category_breakdown'].items():
                    f.write(f"  {category}: {stats['passed']}/{stats['total']} passed ({stats['pass_rate']:.2%})\n")
        else:
            raise ValueError(f"Unsupported report format: {report_format}")

        logger.info(f"Validation report saved to {filepath}")

    def get_trend_analysis(self) -> Dict[str, Any]:
        """
        Perform trend analysis on validation results over time.

        Returns:
            Dict: Trend analysis results
        """
        if not self.validation_results:
            return {"message": "No validation results for trend analysis"}

        # Group results by timestamp ranges
        hourly_buckets = {}
        for result in self.validation_results:
            # Convert timestamp to hour bucket
            dt = datetime.fromtimestamp(result.timestamp)
            hour_key = dt.strftime("%Y-%m-%d %H:00")

            if hour_key not in hourly_buckets:
                hourly_buckets[hour_key] = {
                    "total": 0,
                    "passed": 0,
                    "failed": 0,
                    "errors": 0,
                    "avg_latency": 0,
                    "avg_similarity": 0
                }

            bucket = hourly_buckets[hour_key]
            bucket["total"] += 1

            if result.status == "PASS":
                bucket["passed"] += 1
            elif result.status == "FAIL":
                bucket["failed"] += 1
            elif result.status == "ERROR":
                bucket["errors"] += 1

            bucket["avg_latency"] += result.metrics.get('latency_ms', 0)
            if 'avg_similarity' in result.metrics:
                bucket["avg_similarity"] += result.metrics['avg_similarity']

        # Calculate averages per bucket
        for bucket in hourly_buckets.values():
            count = bucket["total"]
            bucket["avg_latency"] /= count if count > 0 else 0
            bucket["avg_similarity"] /= count if count > 0 else 0
            bucket["pass_rate"] = bucket["passed"] / count if count > 0 else 0

        trend_analysis = {
            "time_buckets": hourly_buckets,
            "has_improvement_trend": self._analyze_improvement_trend(hourly_buckets),
            "recommendations": self._generate_recommendations(hourly_buckets)
        }

        return trend_analysis

    def _analyze_improvement_trend(self, buckets: Dict) -> bool:
        """Analyze if there's an improvement trend over time."""
        # Convert buckets to chronological order
        sorted_buckets = sorted(buckets.items(), key=lambda x: x[0])

        if len(sorted_buckets) < 2:
            return False  # Need at least 2 data points for trend analysis

        # Compare first and last buckets
        first_bucket = sorted_buckets[0][1]
        last_bucket = sorted_buckets[-1][1]

        # Improvement if pass rate increased and latency decreased
        first_pass_rate = first_bucket.get("pass_rate", 0)
        last_pass_rate = last_bucket.get("pass_rate", 0)
        first_latency = first_bucket.get("avg_latency", float('inf'))
        last_latency = last_bucket.get("avg_latency", float('inf'))

        return last_pass_rate > first_pass_rate and last_latency < first_latency

    def _generate_recommendations(self, buckets: Dict) -> List[str]:
        """Generate recommendations based on trend analysis."""
        recommendations = []

        if not buckets:
            return ["Collect more validation data to enable trend analysis"]

        # Analyze overall performance
        summary = self.generate_summary()

        if summary.get("avg_latency_ms", 0) > 500:
            recommendations.append("Consider optimizing retrieval performance - average latency exceeds 500ms")

        if summary.get("pass_rate", 1.0) < 0.95:
            recommendations.append("Investigate causes of low pass rate and improve retrieval accuracy")

        if not self._analyze_improvement_trend(buckets):
            recommendations.append("No clear improvement trend detected - consider adjusting validation parameters or investigating systematic issues")

        return recommendations if recommendations else ["Overall validation performance is satisfactory"]