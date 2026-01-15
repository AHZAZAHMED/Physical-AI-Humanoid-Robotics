"""
Test environment setup for RAG retrieval validation.

This module handles test environment configuration and setup.
"""

import os
import tempfile
from typing import Dict, Any
from .config import config


class TestEnvironment:
    """Manages test environment setup and configuration."""

    def __init__(self):
        self.temp_dir = None
        self.test_config = None

    def setup_test_environment(self):
        """Create a temporary directory for test outputs and set up test configuration."""
        self.temp_dir = tempfile.mkdtemp(prefix="rag_validation_test_")
        print(f"Created test environment at: {self.temp_dir}")

        # Set up test-specific configuration
        self.test_config = {
            "output_directory": self.temp_dir,
            "test_mode": True,
            "validation_thresholds": {
                "min_similarity": 0.3,  # Lower threshold for testing
                "max_latency_ms": 2000,  # Higher threshold for testing
                "min_pass_rate": 0.5     # Lower threshold for testing
            }
        }

        return self.temp_dir

    def validate_test_setup(self) -> tuple[bool, str]:
        """
        Validate that the test environment is properly set up.

        Returns:
            tuple: (is_valid, message)
        """
        if not self.temp_dir or not os.path.exists(self.temp_dir):
            return False, "Test environment directory does not exist"

        # Check that required config values are available
        is_config_valid, config_msg = config.validate()
        if not is_config_valid:
            return False, f"Base configuration invalid: {config_msg}"

        return True, "Test environment is properly set up"

    def cleanup_test_environment(self):
        """Clean up the test environment."""
        import shutil
        if self.temp_dir and os.path.exists(self.temp_dir):
            try:
                shutil.rmtree(self.temp_dir)
                print(f"Cleaned up test environment at: {self.temp_dir}")
                self.temp_dir = None
            except Exception as e:
                print(f"Warning: Could not clean up test environment: {str(e)}")

    def get_test_output_path(self, filename: str) -> str:
        """
        Get the full path for a test output file.

        Args:
            filename: Name of the output file

        Returns:
            str: Full path to the test output file
        """
        if not self.temp_dir:
            raise RuntimeError("Test environment not set up. Call setup_test_environment() first.")

        return os.path.join(self.temp_dir, filename)

    def run_pre_flight_checks(self) -> Dict[str, Any]:
        """
        Run pre-flight checks to ensure all required services are available.

        Returns:
            Dict: Results of pre-flight checks
        """
        results = {
            "config_valid": False,
            "qdrant_accessible": False,
            "cohere_accessible": False,
            "collection_exists": False
        }

        # Check configuration
        is_config_valid, config_msg = config.validate()
        results["config_valid"] = is_config_valid
        results["config_msg"] = config_msg

        if is_config_valid:
            # Try to initialize clients to test connectivity
            try:
                from qdrant_client import QdrantClient
                client_params = config.get_qdrant_client_params()
                qdrant_client = QdrantClient(**client_params)

                # Test Qdrant connectivity
                try:
                    qdrant_client.get_collections()
                    results["qdrant_accessible"] = True
                except Exception as e:
                    results["qdrant_error"] = str(e)

                # Test if the expected collection exists
                try:
                    qdrant_client.get_collection(config.collection_name)
                    results["collection_exists"] = True
                except Exception as e:
                    results["collection_error"] = str(e)

            except Exception as e:
                results["qdrant_init_error"] = str(e)

            # Test Cohere connectivity
            try:
                import cohere
                cohere_client = cohere.Client(config.cohere_api_key)

                # Test with a simple embedding call
                test_response = cohere_client.embed(
                    texts=["test"],
                    model="multilingual-22-12"
                )
                results["cohere_accessible"] = True

            except Exception as e:
                results["cohere_error"] = str(e)

        return results


def setup_test_env() -> TestEnvironment:
    """Convenience function to set up the test environment."""
    test_env = TestEnvironment()
    test_env.setup_test_environment()
    return test_env


# For direct execution/testing
if __name__ == "__main__":
    test_env = setup_test_env()

    print("Running pre-flight checks...")
    checks = test_env.run_pre_flight_checks()
    print(f"Pre-flight check results: {checks}")

    print("\nValidating test setup...")
    is_valid, msg = test_env.validate_test_setup()
    print(f"Test setup valid: {is_valid}, Message: {msg}")

    # Cleanup
    test_env.cleanup_test_environment()