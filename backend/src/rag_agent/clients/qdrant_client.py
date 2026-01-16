"""
Qdrant client connection for vector search operations.
"""
from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import PointStruct, VectorParams, Distance, Batch
import requests
import json
import sys
import os

# Handle both direct execution and module imports
try:
    # Try relative imports first (when used as a module)
    from ..config import AgentConfiguration
except ImportError:
    # Fallback for direct execution - add project root to path
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(__file__))))
    from config import AgentConfiguration


class QdrantService:
    """
    Service class for interacting with Qdrant vector database.
    """

    def __init__(self, config: AgentConfiguration):
        self.config = config
        # For cloud instances, we need to force HTTP mode
        # Check if it's a cloud instance (contains .cloud. or .gcp.) and handle appropriately
        if 'cloud' in config.qdrant_host or 'gcp' in config.qdrant_host:
            # For cloud instances, use the raw URL directly with HTTP mode
            # Based on our test, prefer_grpc=False is what works for cloud instances
            self.client = QdrantClient(
                url=config.qdrant_host,  # Use the full URL as provided
                api_key=config.qdrant_api_key,
                prefer_grpc=False,       # Explicitly prefer HTTP for cloud
                timeout=30               # Set timeout
            )
        else:
            self.client = QdrantClient(
                url=config.qdrant_host,
                api_key=config.qdrant_api_key,
                prefer_grpc=True  # Use gRPC for local instances
            )
        self.collection_name = config.qdrant_collection_name

    def initialize_collection(self):
        """
        Initialize the collection if it doesn't exist.
        """
        try:
            # Check if collection exists first - if it does, we don't need to create it
            collection_info = self.client.get_collection(self.collection_name)
            print(f"Collection '{self.collection_name}' already exists with {collection_info.points_count} points")
            # Collection already exists, no need to recreate
        except Exception as e:
            print(f"Could not access collection '{self.collection_name}': {e}")
            # Since the user mentioned the database is already populated with the collection,
            # we'll just log this and continue without creating it
            # The collection should already exist based on the user's statement
            print(f"Note: Expected collection '{self.collection_name}' to exist in Qdrant cloud instance.")

    def search_vectors(
        self,
        query_vector: List[float],
        top_k: int = 5,
        confidence_threshold: float = 0.7,
        filters: Optional[Dict[str, Any]] = None
    ) -> List[Dict[str, Any]]:
        """
        Search for similar vectors in the collection.

        Args:
            query_vector: The query vector to search for
            top_k: Number of results to return
            confidence_threshold: Minimum similarity score threshold
            filters: Optional metadata filters

        Returns:
            List of matching points with their payload and similarity scores
        """
        # Build filters if provided
        search_filter = None
        if filters:
            filter_conditions = []
            for key, value in filters.items():
                if isinstance(value, list):
                    filter_conditions.append(
                        models.FieldCondition(
                            key=key,
                            match=models.MatchAny(any=value)
                        )
                    )
                else:
                    filter_conditions.append(
                        models.FieldCondition(
                            key=key,
                            match=models.MatchValue(value=value)
                        )
                    )

            if filter_conditions:
                search_filter = models.Filter(must=filter_conditions)

        # Perform search - use points_api for newer versions if direct search fails
        try:
            results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                limit=top_k * 2,  # Get more results to account for filtering
                query_filter=search_filter,
                with_payload=True,
                with_vectors=False,
                score_threshold=confidence_threshold
            )
        except AttributeError:
            # Fallback for older API versions or different client configurations
            results = self.client.search_points(
                collection_name=self.collection_name,
                query_vector=query_vector,
                limit=top_k * 2,  # Get more results to account for filtering
                query_filter=search_filter,
                with_payload=True,
                with_vectors=False,
                score_threshold=confidence_threshold
            )

        # Filter results based on confidence threshold and return formatted results
        filtered_results = []
        for result in results:
            if result.score >= confidence_threshold:
                formatted_result = {
                    "id": result.id,
                    "content": result.payload.get("content", ""),
                    "metadata": result.payload.get("metadata", {}),
                    "similarity_score": result.score,
                    "confidence_score": result.score  # In this case, similarity and confidence are the same
                }
                filtered_results.append(formatted_result)

        # Return top_k results after filtering
        return filtered_results[:top_k]

    def add_vectors(self, points: List[PointStruct]):
        """
        Add vectors to the collection.
        """
        self.client.upsert(
            collection_name=self.collection_name,
            points=points
        )

    def health_check(self) -> bool:
        """
        Check if the Qdrant connection is healthy.
        """
        try:
            # Try to get collection info
            self.client.get_collection(self.collection_name)
            return True
        except Exception:
            # Try to ping the server to check if it's accessible at all
            try:
                # Check if the client itself is responsive
                self.client.get_collections()
                return True  # Server is accessible but collection may not exist
            except Exception:
                return False  # Server is not accessible

    def search_with_inference(
        self,
        query_text: str,
        top_k: int = 5,
        confidence_threshold: float = 0.7,
        filters: Optional[Dict[str, Any]] = None
    ) -> List[Dict[str, Any]]:
        """
        Search using external embedding service to generate embeddings and search in Qdrant.
        This is a wrapper that maintains compatibility with the inference API naming.

        Args:
            query_text: The text query to search for
            top_k: Number of results to return
            confidence_threshold: Minimum similarity score threshold
            filters: Optional metadata filters

        Returns:
            List of matching points with their payload and similarity scores
        """
        try:
            # Use the OpenAIService to generate embeddings (supports OpenAI, Cohere, or sentence transformers)
            # We'll create a temporary instance here to avoid circular dependencies
            from ..clients.openai_client import OpenAIService
            from ..config import AgentConfiguration

            config = AgentConfiguration()
            embedding_service = OpenAIService(config)

            # Generate embedding using the configured service
            query_vector = embedding_service.create_embedding(query_text)

            # Call the regular search method with the generated vector
            return self.search_vectors(query_vector, top_k, confidence_threshold, filters)
        except Exception as e:
            print(f"Error in search_with_inference: {e}")
            # Fall back to regular search if embedding generation fails
            return self.search_vectors_fallback(query_text, top_k, confidence_threshold, filters)

    def search_vectors_fallback(
        self,
        query_text: str,
        top_k: int = 5,
        confidence_threshold: float = 0.7,
        filters: Optional[Dict[str, Any]] = None
    ) -> List[Dict[str, Any]]:
        """
        Fallback search method using external embedding service when inference API is not available.
        """
        try:
            # Use the OpenAIService to generate embeddings (supports OpenAI, Cohere, or sentence transformers)
            # We'll create a temporary instance here to avoid circular dependencies
            from ..clients.openai_client import OpenAIService
            from ..config import AgentConfiguration

            config = AgentConfiguration()
            embedding_service = OpenAIService(config)

            # Generate embedding using the configured service
            query_vector = embedding_service.create_embedding(query_text)

            # Call the regular search method with the generated vector
            return self.search_vectors(query_vector, top_k, confidence_threshold, filters)
        except Exception as e:
            print(f"Fallback search also failed: {e}")
            return []