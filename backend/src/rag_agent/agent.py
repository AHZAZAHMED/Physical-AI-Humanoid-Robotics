"""
Main agent file containing all agent-related functionality for the RAG system using OpenAI Agents SDK.
"""
import time
from typing import List, Dict, Any, Optional
from functools import lru_cache
import hashlib
import sys
import os
from dotenv import load_dotenv

load_dotenv()

api_key = os.getenv("OPENROUTER_API_KEY")


# Handle both direct execution and module imports
try:
    # Try relative imports first (when used as a module)
    from .config import AgentConfiguration
    from .clients.qdrant_client import QdrantService
    from .clients.openai_client import OpenAIService
    from .models.query import Query
    from .models.retrieved_chunk import RetrievedChunk
    from .models.generated_response import GeneratedResponse
    from .exceptions import (
        EmbeddingError, SearchError, GenerationError,
        ValidationError, GroundingError, InsufficientContextError
    )
    from .utils.logging import rag_logger
    from .services.stats_service import get_stats_service, QueryMetrics
except ImportError:
    # Fallback for direct execution - add project root to path
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
    from config import AgentConfiguration
    from clients.qdrant_client import QdrantService
    from clients.openai_client import OpenAIService
    from models.query import Query
    from models.retrieved_chunk import RetrievedChunk
    from models.generated_response import GeneratedResponse
    from exceptions import (
        EmbeddingError, SearchError, GenerationError,
        ValidationError, GroundingError, InsufficientContextError
    )
    from utils.logging import rag_logger
    from services.stats_service import get_stats_service, QueryMetrics

# Import modern OpenAI SDK for function calling and Google Generative AI
from openai import OpenAI
import google.generativeai as genai

class RAGAgent:
    """
    Main RAG Agent class that handles the complete query processing pipeline using modern OpenAI SDK.
    All agent-related functionality is contained in this single file as specified.
    """

    def __init__(self, config: Optional[AgentConfiguration] = None):
        """
        Initialize the RAG Agent with configuration using modern OpenAI SDK.
        """
        self.config = config or AgentConfiguration()
        self.config.validate()

        # Initialize the Gemini client with API key from config
        gemini_api_key = os.getenv("GEMINI_API_KEY")
        if not gemini_api_key:
            raise ValueError("GEMINI_API_KEY environment variable must be set")

        # Configure the Google Generative AI client
        genai.configure(api_key=gemini_api_key)

        # Initialize the OpenAI-compatible client for Gemini
        self.client = OpenAI(
            api_key=gemini_api_key,
            base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
        )

        self.model_name = self.config.agent_model  # This should be a Gemini model like "gemini-1.5-flash"

        self.qdrant_service = QdrantService(self.config)
        self.openai_service = OpenAIService(self.config)

        # Initialize the Qdrant collection
        self.qdrant_service.initialize_collection()

        # Cache for frequently asked queries (in-memory, would use Redis in production)
        self._query_cache = {}
        self._max_cache_size = self.config.max_cache_size

    def _create_qdrant_search_tool(self):
        """
        Create a custom tool for the OpenAI Agent to search in Qdrant.
        This allows the agent to retrieve relevant book content when needed.

        Note: With the modern OpenAI SDK approach, we handle the search internally
        rather than using function calling tools from the deprecated agents package.
        """
        # This method is kept for compatibility but not actively used with the modern SDK approach
        def qdrant_search_tool(query: str, top_k: int = 5) -> str:
            """
            Search the Physical AI & Humanoid Robotics textbook for content relevant to the query.

            Args:
                query: The search query
                top_k: Number of results to return (default: 5)

            Returns:
                JSON string containing the search results
            """
            # Search using Qdrant's inference API to generate embeddings and search
            search_results = self.qdrant_service.search_with_inference(
                query_text=query,
                top_k=top_k,
                confidence_threshold=self.config.default_confidence_threshold
            )

            # Format results
            results = []
            for chunk_data in search_results:
                result = {
                    "content": chunk_data["content"][:500] + "..." if len(chunk_data["content"]) > 500 else chunk_data["content"],  # Truncate long content
                    "source": {
                        "book": chunk_data["metadata"].get("book_title", "Unknown"),
                        "chapter": chunk_data["metadata"].get("chapter", "Unknown"),
                        "section": chunk_data["metadata"].get("section", "Unknown"),
                        "page": chunk_data["metadata"].get("page", 0)
                    },
                    "similarity_score": chunk_data["similarity_score"],
                    "confidence_score": chunk_data["confidence_score"]
                }
                results.append(result)

            import json
            return json.dumps(results)

        return qdrant_search_tool

    def _get_cache_key(self, query_text: str, parameters: Dict[str, Any]) -> str:
        """
        Generate a cache key for a query based on the query text and parameters.

        Args:
            query_text: The query text
            parameters: Query parameters

        Returns:
            Cache key string
        """
        # Create a hash of the query and parameters to use as cache key
        cache_input = f"{query_text}_{str(sorted(parameters.items()))}"
        return hashlib.md5(cache_input.encode()).hexdigest()

    def _check_cache(self, query_text: str, parameters: Dict[str, Any]) -> Optional[GeneratedResponse]:
        """
        Check if the query response is in cache.

        Args:
            query_text: The query text
            parameters: Query parameters

        Returns:
            Cached response if found, None otherwise
        """
        cache_key = self._get_cache_key(query_text, parameters)
        if cache_key in self._query_cache:
            cached_response, timestamp = self._query_cache[cache_key]
            # Check if cache entry is still valid (not expired)
            if time.time() - timestamp < self.config.cache_ttl_seconds:
                rag_logger.info(f"Cache hit for query: {query_text[:50]}...")
                return cached_response
            else:
                # Remove expired entry
                del self._query_cache[cache_key]
        return None

    def _store_in_cache(self, query_text: str, parameters: Dict[str, Any], response: GeneratedResponse):
        """
        Store a response in cache.

        Args:
            query_text: The query text
            parameters: Query parameters
            response: The response to cache
        """
        cache_key = self._get_cache_key(query_text, parameters)

        # Remove oldest entries if cache is full
        if len(self._query_cache) >= self._max_cache_size:
            # Remove oldest entry (based on timestamp)
            oldest_key = min(self._query_cache.keys(),
                           key=lambda k: self._query_cache[k][1])
            del self._query_cache[oldest_key]

        self._query_cache[cache_key] = (response, time.time())
        rag_logger.info(f"Stored query in cache: {query_text[:50]}...")

    def process_query(self, query_text: str, parameters: Optional[Dict[str, Any]] = None) -> GeneratedResponse:
        """
        Process a user query through the complete RAG pipeline using modern OpenAI SDK.

        Args:
            query_text: The user's query text
            parameters: Optional configuration parameters (top_k, confidence_threshold, etc.)

        Returns:
            GeneratedResponse with the response text and sources
        """
        try:
            from .utils.logging import (
                log_query_start, log_query_success, log_query_error,
                log_cache_hit, log_cache_miss, log_cache_store
            )
        except ImportError:
            from utils.logging import (
                log_query_start, log_query_success, log_query_error,
                log_cache_hit, log_cache_miss, log_cache_store
            )

        # Use default parameters if not provided
        if parameters is None:
            parameters = {}

        # Check cache first for frequent queries
        cached_response = self._check_cache(query_text, parameters)
        if cached_response is not None:
            # Update metrics for cached response
            start_time = time.time()
            query_id = f"query_{int(time.time())}"
            metrics = QueryMetrics(
                query_id=query_id,
                response_time_ms=0,  # Cached responses are very fast
                grounding_confidence=cached_response.grounding_confidence,
                top_k_used=parameters.get('top_k', self.config.default_top_k),
                confidence_threshold=parameters.get('confidence_threshold', self.config.default_confidence_threshold),
                retrieved_chunks_count=len(cached_response.sources),
                timestamp=time.time(),
                success=True
            )
            stats_service = get_stats_service()
            stats_service.record_query(metrics)

            log_cache_hit(rag_logger, query_id, query_text)
            return cached_response

        start_time = time.time()
        query_id = f"query_{int(time.time())}"

        # Log query start
        log_query_start(rag_logger, query_text, query_id)

        # Validate input
        if not query_text or len(query_text.strip()) == 0:
            # Record failed query metric
            response_time_ms = int((time.time() - start_time) * 1000)
            metrics = QueryMetrics(
                query_id=query_id,
                response_time_ms=response_time_ms,
                grounding_confidence=0.0,
                top_k_used=self.config.default_top_k,
                confidence_threshold=self.config.default_confidence_threshold,
                retrieved_chunks_count=0,
                timestamp=time.time(),
                success=False,
                error_type="ValidationError"
            )
            stats_service = get_stats_service()
            stats_service.record_query(metrics)

            log_query_error(rag_logger, query_id, ValidationError("Query text must not be empty"), response_time_ms)
            raise ValidationError("Query text must not be empty")

        if len(query_text) > self.config.max_query_length:
            # Record failed query metric
            response_time_ms = int((time.time() - start_time) * 1000)
            metrics = QueryMetrics(
                query_id=query_id,
                response_time_ms=response_time_ms,
                grounding_confidence=0.0,
                top_k_used=self.config.default_top_k,
                confidence_threshold=self.config.default_confidence_threshold,
                retrieved_chunks_count=0,
                timestamp=time.time(),
                success=False,
                error_type="ValidationError"
            )
            stats_service = get_stats_service()
            stats_service.record_query(metrics)

            log_query_error(rag_logger, query_id, ValidationError(f"Query text must be less than {self.config.max_query_length} characters"), response_time_ms)
            raise ValidationError(f"Query text must be less than {self.config.max_query_length} characters")

        # Use default parameters if not provided
        if parameters is None:
            parameters = {}

        top_k = parameters.get('top_k', self.config.default_top_k)
        confidence_threshold = parameters.get('confidence_threshold', self.config.default_confidence_threshold)
        include_metadata = parameters.get('include_metadata', True)

        # Create query object
        query = Query(
            id=query_id,
            text=query_text,
            parameters=parameters
        )
        query.validate()

        try:
            # 1. Use Qdrant's inference API to search for relevant chunks (generates embedding internally)
            rag_logger.info(f"Searching for top {top_k} chunks with confidence threshold {confidence_threshold}", extra={'query_id': query_id})
            retrieved_chunks_data = self.qdrant_service.search_with_inference(
                query_text=query_text,
                top_k=top_k,
                confidence_threshold=confidence_threshold
            )

            # Convert to RetrievedChunk objects
            retrieved_chunks = []
            for chunk_data in retrieved_chunks_data:
                # Ensure required metadata fields exist, provide defaults if missing
                metadata = chunk_data["metadata"].copy()  # Make a copy to avoid modifying original
                if "book_title" not in metadata:
                    metadata["book_title"] = "Physical AI & Humanoid Robotics Textbook"
                if "chapter" not in metadata:
                    metadata["chapter"] = metadata.get("page_title", "Unknown Chapter")
                if "section" not in metadata:
                    metadata["section"] = metadata.get("headings", ["Unknown Section"])[0] if metadata.get("headings") else "Unknown Section"

                chunk = RetrievedChunk(
                    id=chunk_data["id"],
                    content=chunk_data["content"],
                    similarity_score=chunk_data["similarity_score"],
                    confidence_score=chunk_data["confidence_score"],
                    metadata=metadata
                )
                chunk.validate()
                retrieved_chunks.append(chunk)

            # 3. Check if we have sufficient context
            if not retrieved_chunks:
                response_time_ms = int((time.time() - start_time) * 1000)

                # Record failed query metric for insufficient context
                metrics = QueryMetrics(
                    query_id=query_id,
                    response_time_ms=response_time_ms,
                    grounding_confidence=0.0,
                    top_k_used=top_k,
                    confidence_threshold=confidence_threshold,
                    retrieved_chunks_count=len(retrieved_chunks),
                    timestamp=time.time(),
                    success=False,
                    error_type="InsufficientContextError"
                )
                stats_service = get_stats_service()
                stats_service.record_query(metrics)

                log_query_error(rag_logger, query_id, InsufficientContextError("No relevant content found in the book for the given query"), response_time_ms)
                raise InsufficientContextError("No relevant content found in the book for the given query")

            # 4. Format context for the LLM
            context = self._format_context_for_agent(retrieved_chunks)

            # 5. Use the OpenAI API to generate response with the retrieved context
            rag_logger.info("Generating response using OpenAI API with retrieved context", extra={'query_id': query_id})

            # Create the system message with instructions and context
            system_message = f"""You are an educational AI assistant helping users learn about Physical AI and Humanoid Robotics.
- Ground all responses strictly in the provided book content below
- Maintain a clear, teacher-like instructional tone
- Cite specific book sections that support your answers
- If the provided context doesn't contain sufficient information, acknowledge this rather than hallucinating
- Structure responses with clear explanations and practical examples where possible

CONTEXT:
{context}"""

            # Make the API call using the modern OpenAI SDK
            response = self.client.chat.completions.create(
                model=self.model_name,
                messages=[
                    {"role": "system", "content": system_message},
                    {"role": "user", "content": query_text}
                ],
                temperature=0.3,  # Lower temperature for more consistent, factual responses
            )

            response_text = response.choices[0].message.content

            # 6. Validate that response is grounded in provided context
            grounding_confidence = self._validate_response_grounding(response_text, retrieved_chunks)
            if grounding_confidence < self.config.grounding_threshold:
                response_time_ms = int((time.time() - start_time) * 1000)

                # Record failed query metric for grounding error
                metrics = QueryMetrics(
                    query_id=query_id,
                    response_time_ms=response_time_ms,
                    grounding_confidence=grounding_confidence,
                    top_k_used=top_k,
                    confidence_threshold=confidence_threshold,
                    retrieved_chunks_count=len(retrieved_chunks),
                    timestamp=time.time(),
                    success=False,
                    error_type="GroundingError"
                )
                stats_service = get_stats_service()
                stats_service.record_query(metrics)

                error = GroundingError(
                    f"Response is not sufficiently grounded in provided context. "
                    f"Grounding confidence: {grounding_confidence:.2f}, "
                    f"Required: {self.config.grounding_threshold:.2f}"
                )
                log_query_error(rag_logger, query_id, error, response_time_ms)
                raise error

            # 7. Format response with sources
            sources = self._format_sources(retrieved_chunks, include_metadata)

            # 8. Calculate response time
            response_time_ms = int((time.time() - start_time) * 1000)

            # 9. Create and return generated response
            generated_response = GeneratedResponse(
                id=f"response_{int(time.time())}",
                query_id=query.id,
                text=response_text,
                sources=sources,
                grounding_confidence=grounding_confidence,
                response_time_ms=response_time_ms
            )
            generated_response.validate()

            # Store successful response in cache for future queries
            log_cache_store(rag_logger, query_id, query_text)
            self._store_in_cache(query_text, parameters, generated_response)

            # Record successful query metric
            metrics = QueryMetrics(
                query_id=query_id,
                response_time_ms=response_time_ms,
                grounding_confidence=grounding_confidence,
                top_k_used=top_k,
                confidence_threshold=confidence_threshold,
                retrieved_chunks_count=len(retrieved_chunks),
                timestamp=time.time(),
                success=True
            )
            stats_service = get_stats_service()
            stats_service.record_query(metrics)

            # Log successful query completion
            log_query_success(rag_logger, query_id, response_time_ms, len(sources), grounding_confidence)
            return generated_response

        except Exception as e:
            response_time_ms = int((time.time() - start_time) * 1000)

            # Record failed query metric for any other error
            error_type = type(e).__name__ if hasattr(e, '__name__') else str(type(e))
            metrics = QueryMetrics(
                query_id=query_id,
                response_time_ms=response_time_ms,
                grounding_confidence=0.0,
                top_k_used=top_k if 'top_k' in locals() else self.config.default_top_k,
                confidence_threshold=confidence_threshold if 'confidence_threshold' in locals() else self.config.default_confidence_threshold,
                retrieved_chunks_count=len(retrieved_chunks) if 'retrieved_chunks' in locals() else 0,
                timestamp=time.time(),
                success=False,
                error_type=error_type
            )
            stats_service = get_stats_service()
            stats_service.record_query(metrics)

            log_query_error(rag_logger, query_id, e, response_time_ms)
            raise e

    def _format_context_for_agent(self, retrieved_chunks: List[RetrievedChunk]) -> str:
        """
        Format the retrieved chunks into a context string for the agent.
        """
        context_parts = []
        for i, chunk in enumerate(retrieved_chunks):
            context_parts.append(
                f"Source {i+1}:\n"
                f"Book: {chunk.metadata.get('book_title', 'Unknown')}\n"
                f"Chapter: {chunk.metadata.get('chapter', 'Unknown')}\n"
                f"Section: {chunk.metadata.get('section', 'Unknown')}\n"
                f"Content: {chunk.content}\n"
                f"Similarity Score: {chunk.similarity_score:.3f}\n"
                "---\n"
            )
        return "\n".join(context_parts)

    def _validate_response_grounding(self, response: str, retrieved_chunks: List[RetrievedChunk]) -> float:
        """
        Validate that the response is grounded in the provided context.
        This is a simplified implementation - in a real system, this would involve
        more sophisticated NLP techniques to measure grounding.
        """
        # For now, we'll return a confidence score based on whether the response
        # contains key phrases from the retrieved chunks
        response_lower = response.lower()

        # Count how many chunks have content that appears in the response
        relevant_chunks = 0
        total_chunks = len(retrieved_chunks)

        for chunk in retrieved_chunks:
            chunk_content = chunk.content.lower()
            # Simple check: if any 5-word phrase from the chunk appears in the response
            words = chunk_content.split()
            found_in_response = False

            for i in range(len(words) - 4):  # 5-word phrases
                phrase = " ".join(words[i:i+5])
                if phrase in response_lower:
                    found_in_response = True
                    break

            if found_in_response:
                relevant_chunks += 1

        # Return a grounding confidence score
        if total_chunks == 0:
            return 0.0
        return relevant_chunks / total_chunks

    def _format_sources(self, retrieved_chunks: List[RetrievedChunk], include_metadata: bool = True) -> List[Dict[str, Any]]:
        """
        Format the retrieved chunks as sources for the response.
        """
        sources = []
        for chunk in retrieved_chunks:
            source = {
                "content": chunk.content[:200] + "..." if len(chunk.content) > 200 else chunk.content,  # Preview
                "source": {
                    "book": chunk.metadata.get("book_title", "Unknown"),
                    "chapter": chunk.metadata.get("chapter", "Unknown"),
                    "section": chunk.metadata.get("section", "Unknown"),
                    "page": chunk.metadata.get("page", 0)
                },
                "similarity_score": chunk.similarity_score,
                "confidence_score": chunk.confidence_score
            }
            sources.append(source)

        return sources

    def health_check(self) -> Dict[str, Any]:
        """
        Perform a health check of all dependencies.
        """
        qdrant_healthy = self.qdrant_service.health_check()
        openai_healthy = self.openai_service.health_check()

        status = "healthy" if qdrant_healthy and openai_healthy else "unhealthy"

        return {
            "status": status,
            "timestamp": time.time(),
            "dependencies": {
                "qdrant": {
                    "status": "connected" if qdrant_healthy else "disconnected",
                    "response_time_ms": 0  # Would measure actual time in a real implementation
                },
                "openai": {
                    "status": "connected" if openai_healthy else "disconnected",
                    "response_time_ms": 0  # Would measure actual time in a real implementation
                }
            }
        }