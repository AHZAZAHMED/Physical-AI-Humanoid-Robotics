"""
RAG (Retrieval-Augmented Generation) retriever for the Physical AI Textbook Platform
Handles retrieval of relevant content from the vector database
"""
import os
from typing import List, Dict, Any
from dotenv import load_dotenv
from sentence_transformers import SentenceTransformer
from qdrant_client import QdrantClient
from qdrant_client.http.models import models
import logging

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class RAGRetriever:
    """
    Class for retrieving relevant content using RAG
    """

    def __init__(self):
        # Initialize the embedding model
        logger.info("Loading embedding model...")
        self.model = SentenceTransformer('intfloat/e5-base-v2')
        logger.info("Embedding model loaded successfully")

        # Initialize Qdrant client
        qdrant_url = os.getenv("QDRANT_URL")
        qdrant_api_key = os.getenv("QDRANT_API_KEY")

        if qdrant_url:
            # Use cloud instance
            self.client = QdrantClient(
                url=qdrant_url,
                api_key=qdrant_api_key,
            )
        else:
            # Use local instance
            self.client = QdrantClient(host="localhost", port=6333)

        logger.info("Qdrant client loaded successfully")

        # Collection name for textbook content
        self.COLLECTION_NAME = "textbook_content"

    def retrieve_relevant_content(self, query: str, top_k: int = 5) -> List[Dict[str, Any]]:
        """
        Retrieve the most relevant content chunks for a given query
        """
        try:
            # Prefix the query with "query: " for the e5-base-v2 model
            prefixed_query = f"query: {query}"

            # Generate embedding for the query
            query_embedding = self.model.encode([prefixed_query])[0].tolist()

            # Search in Qdrant
            search_results = self.client.search(
                collection_name=self.COLLECTION_NAME,
                query_vector=query_embedding,
                limit=top_k
            )

            # Format results
            results = []
            for hit in search_results:
                result = {
                    "text": hit.payload.get("text", ""),
                    "score": hit.score,
                    "metadata": hit.payload.get("metadata", {}),
                    "id": hit.id
                }
                results.append(result)

            logger.info(f"Retrieved {len(results)} relevant results for query: {query[:50]}...")
            return results

        except Exception as e:
            logger.error(f"Error retrieving content: {str(e)}")
            raise

    def retrieve_with_context(self, query: str, top_k: int = 5, context_window: int = 2) -> List[Dict[str, Any]]:
        """
        Retrieve content with additional context from surrounding chunks
        """
        try:
            # First, get the most relevant chunks
            relevant_chunks = self.retrieve_relevant_content(query, top_k)

            # For each chunk, potentially retrieve additional context
            # (This would require additional logic to link related chunks)
            return relevant_chunks

        except Exception as e:
            logger.error(f"Error retrieving content with context: {str(e)}")
            raise

    def search_by_metadata(self, metadata_filters: Dict[str, Any], query: str = None, top_k: int = 5) -> List[Dict[str, Any]]:
        """
        Search for content with specific metadata filters
        """
        try:
            # If a query is provided, use semantic search with metadata filters
            if query:
                # Prefix the query with "query: " for the e5-base-v2 model
                prefixed_query = f"query: {query}"

                # Generate embedding for the query
                query_embedding = self.model.encode([prefixed_query])[0].tolist()

                # Create filter conditions
                must_conditions = []
                for key, value in metadata_filters.items():
                    must_conditions.append(
                        models.FieldCondition(
                            key=f"metadata.{key}",
                            match=models.MatchValue(value=value)
                        )
                    )

                # Search in Qdrant with filters
                search_results = self.client.search(
                    collection_name=self.COLLECTION_NAME,
                    query_vector=query_embedding,
                    limit=top_k,
                    query_filter=models.Filter(must=must_conditions)
                )
            else:
                # If no query, perform a filtered search without semantic matching
                must_conditions = []
                for key, value in metadata_filters.items():
                    must_conditions.append(
                        models.FieldCondition(
                            key=f"metadata.{key}",
                            match=models.MatchValue(value=value)
                        )
                    )

                # Scroll through the collection with filters
                scroll_results = self.client.scroll(
                    collection_name=self.COLLECTION_NAME,
                    scroll_filter=models.Filter(must=must_conditions),
                    limit=top_k
                )

                search_results = scroll_results[0] if scroll_results else []

            # Format results
            results = []
            for hit in search_results:
                result = {
                    "text": hit.payload.get("text", ""),
                    "score": getattr(hit, 'score', 0),  # Score might not exist in scroll results
                    "metadata": hit.payload.get("metadata", {}),
                    "id": hit.id
                }
                results.append(result)

            logger.info(f"Retrieved {len(results)} results with metadata filters")
            return results

        except Exception as e:
            logger.error(f"Error searching by metadata: {str(e)}")
            raise

    def check_collection_exists(self) -> bool:
        """
        Check if the collection exists
        """
        try:
            self.client.get_collection(self.COLLECTION_NAME)
            return True
        except:
            return False

    def get_collection_info(self) -> Dict[str, Any]:
        """
        Get information about the collection
        """
        try:
            collection_info = self.client.get_collection(self.COLLECTION_NAME)
            return {
                "name": collection_info.config.params.vectors.size,
                "vectors_count": collection_info.points_count,
                "config": collection_info.config.dict() if hasattr(collection_info.config, 'dict') else str(collection_info.config)
            }
        except Exception as e:
            logger.error(f"Error getting collection info: {str(e)}")
            raise


# Global instance for use in other modules
retriever = RAGRetriever()


def get_relevant_content(query: str, top_k: int = 5) -> List[Dict[str, Any]]:
    """
    Convenience function to retrieve relevant content
    """
    return retriever.retrieve_relevant_content(query, top_k)


def search_content_with_filters(metadata_filters: Dict[str, Any], query: str = None, top_k: int = 5) -> List[Dict[str, Any]]:
    """
    Convenience function to search content with metadata filters
    """
    return retriever.search_by_metadata(metadata_filters, query, top_k)