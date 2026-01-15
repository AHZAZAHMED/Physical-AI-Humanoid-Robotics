"""
Embedding service client for embedding operations (used by the RAG system).
Supports OpenAI, Cohere, and local sentence transformer embedding services.
"""
import openai
from typing import List, Dict, Any, Optional
import sys
import os
import cohere

# Handle both direct execution and module imports
try:
    # Try relative imports first (when used as a module)
    from ..config import AgentConfiguration
except ImportError:
    # Fallback for direct execution - add project root to path
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(__file__))))
    from config import AgentConfiguration

# Try to import sentence transformers (optional dependency)
SENTENCE_TRANSFORMERS_AVAILABLE = False
try:
    from sentence_transformers import SentenceTransformer
    # Test if the model can be loaded to catch compatibility issues
    test_model = SentenceTransformer('all-MiniLM-L6-v2', device='cpu')
    SENTENCE_TRANSFORMERS_AVAILABLE = True
    del test_model  # Clean up the test model
except ImportError:
    SENTENCE_TRANSFORMERS_AVAILABLE = False
except Exception as e:
    print(f"Sentence transformers available but failed to load model: {e}")
    SENTENCE_TRANSFORMERS_AVAILABLE = False


class OpenAIService:
    """
    Service class for interacting with embedding APIs (OpenAI, Cohere, or local sentence transformers).
    """

    def __init__(self, config: AgentConfiguration):
        self.config = config
        # Initialize both possible clients
        self.openai_api_key = config.openai_api_key
        self.cohere_api_key = os.getenv("COHERE_API_KEY")

        # Priority order: cohere API > sentence transformers > openai API
        if self.cohere_api_key:
            self.embedding_provider = "cohere"
            self.cohere_client = cohere.Client(self.cohere_api_key)
            print("Using Cohere API for embeddings")
        elif SENTENCE_TRANSFORMERS_AVAILABLE:
            self.embedding_provider = "sentence_transformer"
            self.sentence_model = SentenceTransformer('all-MiniLM-L6-v2')
            print("Using local sentence transformers for embeddings")
        elif self.openai_api_key:
            self.embedding_provider = "openai"
            openai.api_key = self.openai_api_key
            print("Using OpenAI API for embeddings")
        else:
            raise ValueError("Either COHERE_API_KEY, OPENAI_API_KEY (or OPENROUTER_API_KEY), or sentence-transformers must be available")

    def create_embedding(self, text: str) -> List[float]:
        """
        Create embedding for the given text using the configured embedding model.

        Args:
            text: The text to create embedding for

        Returns:
            List of floats representing the embedding vector
        """
        if self.embedding_provider == "sentence_transformer":
            # Use local sentence transformer for embeddings
            embeddings = self.sentence_model.encode([text])
            return embeddings[0].tolist()  # Convert to list format
        elif self.embedding_provider == "cohere":
            # Use Cohere for embeddings
            response = self.cohere_client.embed(
                texts=[text],
                model="embed-multilingual-v2.0"  # Using Cohere's multilingual model
            )
            return response.embeddings[0]
        else:
            # Use OpenAI for embeddings
            response = openai.embeddings.create(
                model=self.config.embedding_model,
                input=text
            )
            return response.data[0].embedding

    def create_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Create embeddings for multiple texts.

        Args:
            texts: List of texts to create embeddings for

        Returns:
            List of embedding vectors
        """
        if self.embedding_provider == "sentence_transformer":
            # Use local sentence transformer for embeddings
            embeddings = self.sentence_model.encode(texts)
            return [emb.tolist() for emb in embeddings]
        elif self.embedding_provider == "cohere":
            # Use Cohere for embeddings
            response = self.cohere_client.embed(
                texts=texts,
                model="embed-multilingual-v2.0"  # Using Cohere's multilingual model
            )
            return response.embeddings
        else:
            # Use OpenAI for embeddings
            response = openai.embeddings.create(
                model=self.config.embedding_model,
                input=texts
            )
            return [item.embedding for item in response.data]

    def health_check(self) -> bool:
        """
        Check if the embedding service is healthy by making a simple request.
        """
        try:
            # Create a simple embedding to test the connection
            test_embedding = self.create_embedding("test")
            return len(test_embedding) > 0
        except Exception:
            return False