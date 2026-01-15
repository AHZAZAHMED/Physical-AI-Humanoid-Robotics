"""
Main validation logic for RAG retrieval pipeline.

This module contains the core validation functionality that executes
similarity searches in Qdrant and validates that retrieved content
is semantically relevant to queries.
"""

import time
import logging
from typing import List, Dict, Any, Optional, Tuple
from qdrant_client import QdrantClient
from qdrant_client.http import models
import cohere
from .config import config

# Configure logging
logger = logging.getLogger(__name__)


class ValidationResult:
    """Represents the result of a single validation test."""

    def __init__(self, query: str, results: List[Dict], metrics: Dict, status: str,
                 query_category: str = ""):
        self.query = query
        self.results = results
        self.metrics = metrics
        self.status = status
        self.query_category = query_category
        self.timestamp = time.time()


class RetrievalValidator:
    """Core class to execute validation tests for RAG retrieval."""

    def __init__(self):
        # Initialize Qdrant client with connection validation
        client_params = config.get_qdrant_client_params()
        self.qdrant_client = QdrantClient(**client_params)

        # Validate Qdrant connection
        if not self.validate_qdrant_connection():
            logger.warning("Could not validate Qdrant connection during initialization")

        # Initialize Cohere client
        self.cohere_client = cohere.Client(config.cohere_api_key)

        # Validation parameters
        self.collection_name = config.collection_name
        self.similarity_threshold = config.similarity_threshold
        self.top_k_results = config.top_k_results

    def validate_qdrant_connection(self) -> bool:
        """
        Validate the Qdrant connection by attempting to access the client.

        Returns:
            bool: True if connection is valid, False otherwise
        """
        try:
            # Try to get collections to verify connection
            collections = self.qdrant_client.get_collections()
            logger.info(f"Successfully connected to Qdrant. Available collections: {[col.name for col in collections.collections]}")
            return True
        except Exception as e:
            logger.error(f"Failed to connect to Qdrant: {str(e)}")
            return False

    def validate_single_query(self, query: str, query_category: str = "general") -> ValidationResult:
        """
        Execute validation for a single query.

        Args:
            query: The query string to validate
            query_category: Category of the query (keyword, semantic, section-specific)

        Returns:
            ValidationResult: Object containing the validation results
        """
        start_time = time.time()

        try:
            # Generate embedding for the query
            response = self.cohere_client.embed(
                texts=[query],
                model="multilingual-22-12"  # Using Cohere's multilingual model
            )
            query_embedding = response.embeddings[0]

            # Execute similarity search in Qdrant
            search_results = self.qdrant_client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=self.top_k_results,
                score_threshold=self.similarity_threshold
            )

            # Process results
            processed_results = []
            for result in search_results:
                processed_result = {
                    'content': result.payload.get('content', ''),
                    'similarity_score': result.score,
                    'metadata': result.payload.get('metadata', {}),
                    'relevance': result.score >= self.similarity_threshold
                }
                processed_results.append(processed_result)

            # Calculate metrics
            end_time = time.time()
            latency_ms = (end_time - start_time) * 1000

            metrics = {
                'latency_ms': latency_ms,
                'top_k_recall': len(processed_results) / self.top_k_results if self.top_k_results > 0 else 0,
                'avg_similarity': sum(r['similarity_score'] for r in processed_results) / len(processed_results) if processed_results else 0,
                'relevant_results_count': sum(1 for r in processed_results if r['relevance']),
                'total_results_returned': len(processed_results)
            }

            # Determine status based on validation criteria
            status = "PASS" if len(processed_results) > 0 and metrics['avg_similarity'] >= self.similarity_threshold else "FAIL"

            return ValidationResult(
                query=query,
                results=processed_results,
                metrics=metrics,
                status=status,
                query_category=query_category
            )

        except Exception as e:
            logger.error(f"Error validating query '{query}': {str(e)}")

            # Return error result
            end_time = time.time()
            latency_ms = (end_time - start_time) * 1000

            error_metrics = {
                'latency_ms': latency_ms,
                'error': str(e)
            }

            return ValidationResult(
                query=query,
                results=[],
                metrics=error_metrics,
                status="ERROR",
                query_category=query_category
            )

    def validate_batch_queries(self, queries: List[Tuple[str, str]]) -> List[ValidationResult]:
        """
        Execute validation for a batch of queries.

        Args:
            queries: List of tuples (query_string, query_category)

        Returns:
            List[ValidationResult]: Results for each query
        """
        results = []
        for query, category in queries:
            result = self.validate_single_query(query, category)
            results.append(result)
        return results

    def check_collection_exists(self) -> bool:
        """
        Check if the validation collection exists in Qdrant.

        Returns:
            bool: True if collection exists, False otherwise
        """
        try:
            self.qdrant_client.get_collection(self.collection_name)
            return True
        except Exception as e:
            logger.error(f"Collection {self.collection_name} does not exist: {str(e)}")
            return False

    def get_collection_info(self) -> Dict[str, Any]:
        """
        Get information about the validation collection.

        Returns:
            Dict: Collection information including vector count and schema
        """
        try:
            collection_info = self.qdrant_client.get_collection(self.collection_name)
            return {
                "name": collection_info.config.params.vectors.size,
                "vectors_count": collection_info.points_count,
                "config": collection_info.config.dict() if hasattr(collection_info.config, 'dict') else str(collection_info.config)
            }
        except Exception as e:
            logger.error(f"Error getting collection info: {str(e)}")
            return {}