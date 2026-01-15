#!/usr/bin/env python3
"""
Demo script for RAG retrieval validation system.

This script demonstrates how to use the validation system to test
RAG retrieval functionality.
"""

import os
import sys
import json
from datetime import datetime

# Add the backend src directory to the Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from validation_orchestrator import ValidationOrchestrator, run_validation_pipeline
from query_generator import QueryGenerator


def main():
    """Main function to run validation demo."""
    print("RAG Retrieval Validation Demo")
    print("=" * 40)

    # Check if required environment variables are set
    required_env_vars = ['QDRANT_URL', 'QDRANT_API_KEY', 'COHERE_API_KEY']
    missing_vars = [var for var in required_env_vars if not os.getenv(var)]

    if missing_vars:
        print(f"Warning: Missing required environment variables: {missing_vars}")
        print("Please set these variables before running the validation.")
        print("\nExample:")
        print("export QDRANT_URL='your-qdrant-url'")
        print("export QDRANT_API_KEY='your-qdrant-api-key'")
        print("export COHERE_API_KEY='your-cohere-api-key'")
        return

    print("All required environment variables are set.")
    print()

    try:
        # Initialize the orchestrator
        print("Initializing validation orchestrator...")
        orchestrator = ValidationOrchestrator()
        print("✓ Validation orchestrator initialized successfully")
        print()

        # Test connection to Qdrant
        print("Testing Qdrant connection...")
        is_connected = orchestrator.retrieval_validator.validate_qdrant_connection()
        if is_connected:
            print("✓ Qdrant connection successful")
        else:
            print("✗ Qdrant connection failed")
            return
        print()

        # Test collection exists
        print("Checking if collection exists...")
        collection_exists = orchestrator.retrieval_validator.check_collection_exists()
        if collection_exists:
            print(f"✓ Collection '{orchestrator.retrieval_validator.collection_name}' exists")
        else:
            print(f"✗ Collection '{orchestrator.retrieval_validator.collection_name}' does not exist")
            return
        print()

        # Generate sample queries
        print("Generating sample queries...")
        query_generator = QueryGenerator()
        sample_queries = query_generator.generate_test_queries({
            'keyword': 2,
            'semantic': 2,
            'section-specific': 1
        })

        print(f"Generated {len(sample_queries)} sample queries:")
        for i, (query, category) in enumerate(sample_queries, 1):
            print(f"  {i}. [{category}] {query}")
        print()

        # Run quick validation
        print("Running quick validation...")
        quick_results = orchestrator.run_content_relevance_validation(sample_queries[:2])
        print(f"✓ Quick validation completed")
        print(f"  - Relevance rate: {quick_results['relevance_rate']:.2%}")
        print(f"  - Total results: {quick_results['total_results']}")
        print(f"  - Relevant results: {quick_results['relevant_results']}")
        print()

        # Run comprehensive validation (first 3 queries to keep it quick)
        print("Running comprehensive validation...")
        comprehensive_results = orchestrator.run_comprehensive_validation(
            custom_queries=sample_queries[:3],  # Use first 3 queries to keep it quick
            validate_metadata=True,
            validate_performance=False,  # Skip performance to keep demo quick
            validate_chunks=True
        )
        print("✓ Comprehensive validation completed")
        print()

        # Display summary
        summary = comprehensive_results['summary']
        print("Validation Summary:")
        print(f"  - Total tests: {summary['total_tests']}")
        print(f"  - Passed: {summary['passed_tests']}")
        print(f"  - Failed: {summary['failed_tests']}")
        print(f"  - Error: {summary['error_tests']}")
        print(f"  - Pass rate: {summary['pass_rate']:.2%}")
        print(f"  - Avg latency: {summary['avg_latency_ms']:.2f}ms")
        print()

        # Display pass/fail determination
        pass_fail = comprehensive_results['pass_fail_determination']
        print("Pass/Fail Determination:")
        print(f"  - Overall status: {pass_fail['overall_status']}")
        print(f"  - Criteria met: {len(pass_fail['criteria_met'])}")
        for criterion, met in pass_fail['criteria_met'].items():
            status = "✓" if met else "✗"
            print(f"    {status} {criterion}: {'Met' if met else 'Not met'}")
        print()

        # Save results
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        results_file = f"rag_validation_results_{timestamp}.json"

        print(f"Saving results to {results_file}...")
        orchestrator.save_validation_results(comprehensive_results, results_file)
        print("✓ Results saved successfully")
        print()

        # Generate and display a simple report
        print("Generated Report:")
        print("-" * 20)
        report = orchestrator._generate_text_report(comprehensive_results)
        print(report)

        print("Demo completed successfully!")
        print(f"Full results saved to: {results_file}")

    except Exception as e:
        print(f"Error during validation: {str(e)}")
        print("Please check your configuration and try again.")


def run_pipeline_demo():
    """Run the validation pipeline demo."""
    print("Running validation pipeline demo...")

    # Check environment variables
    required_env_vars = ['QDRANT_URL', 'QDRANT_API_KEY', 'COHERE_API_KEY']
    missing_vars = [var for var in required_env_vars if not os.getenv(var)]

    if missing_vars:
        print(f"Missing required environment variables: {missing_vars}")
        return

    try:
        # Run the complete pipeline
        results = run_validation_pipeline()

        print("Pipeline completed successfully!")
        print(f"Total tests: {results['summary']['total_tests']}")
        print(f"Pass rate: {results['summary']['pass_rate']:.2%}")

        # Save results
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"pipeline_results_{timestamp}.json"

        with open(filename, 'w') as f:
            json.dump(results, f, indent=2, default=str)

        print(f"Results saved to: {filename}")

    except Exception as e:
        print(f"Pipeline failed: {str(e)}")


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "pipeline":
        run_pipeline_demo()
    else:
        main()