"""
RAG (Retrieval-Augmented Generation) Service for Physical AI & Humanoid Robotics Textbook

This service provides:
- Text embedding and indexing functionality
- Vector search for relevant textbook content
- Integration with LLM for answer generation
"""

import os
import logging
from typing import List, Dict, Any, Optional
from dotenv import load_dotenv

from fastapi import FastAPI, HTTPException, BackgroundTasks
from pydantic import BaseModel
import uvicorn

from sentence_transformers import SentenceTransformer
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Initialize the embedding model
logger.info("Loading embedding model...")
model = SentenceTransformer('intfloat/e5-base-v2')
logger.info("Embedding model loaded successfully")

# Initialize Qdrant client
qdrant_url = os.getenv("QDRANT_URL")
qdrant_api_key = os.getenv("QDRANT_API_KEY")

if qdrant_url:
    # Use cloud instance
    client = QdrantClient(
        url=qdrant_url,
        api_key=qdrant_api_key,
    )
else:
    # Use local instance
    client = QdrantClient(host="localhost", port=6333)

# Collection name for textbook content
COLLECTION_NAME = "textbook_content"

# Create collection if it doesn't exist
try:
    client.get_collection(COLLECTION_NAME)
    logger.info(f"Collection '{COLLECTION_NAME}' already exists")
except:
    logger.info(f"Creating collection '{COLLECTION_NAME}'")
    client.create_collection(
        collection_name=COLLECTION_NAME,
        vectors_config=VectorParams(size=768, distance=Distance.COSINE),
    )
    logger.info(f"Collection '{COLLECTION_NAME}' created successfully")

app = FastAPI(
    title="Physical AI & Humanoid Robotics RAG API",
    description="API for retrieving and generating answers from the Physical AI & Humanoid Robotics textbook",
    version="1.0.0"
)

# Pydantic models
class ContentChunk(BaseModel):
    text: str
    metadata: Dict[str, Any]

class IndexContentRequest(BaseModel):
    content_chunks: List[ContentChunk]
    collection_name: Optional[str] = COLLECTION_NAME

class QuestionRequest(BaseModel):
    question: str
    top_k: Optional[int] = 5

class SearchResult(BaseModel):
    text: str
    score: float
    metadata: Dict[str, Any]

class AnswerResponse(BaseModel):
    question: str
    answer: str
    sources: List[SearchResult]

# Text splitter utility
def split_text(text: str, chunk_size: int = 150, overlap: int = 35) -> List[str]:
    """
    Split text into chunks of specified size with overlap
    """
    # Simple word-based splitting
    words = text.split()
    chunks = []

    for i in range(0, len(words), chunk_size - overlap):
        chunk = " ".join(words[i:i + chunk_size])
        if chunk.strip():
            chunks.append(chunk)

    return chunks

@app.get("/")
def read_root():
    return {"message": "Physical AI & Humanoid Robotics RAG API"}

@app.post("/index_content")
async def index_content(request: IndexContentRequest):
    """
    Index textbook content into the vector database
    """
    try:
        points = []
        point_id = 0

        for chunk_data in request.content_chunks:
            # Split the content into smaller chunks if needed
            text_chunks = split_text(chunk_data.text)

            for text_chunk in text_chunks:
                # Prefix the text with "passage: " for the e5-base-v2 model
                prefixed_text = f"passage: {text_chunk}"

                # Generate embedding
                embedding = model.encode([prefixed_text])[0].tolist()

                # Create a point for Qdrant
                point = models.PointStruct(
                    id=point_id,
                    vector=embedding,
                    payload={
                        "text": text_chunk,
                        "metadata": chunk_data.metadata
                    }
                )
                points.append(point)
                point_id += 1

        # Upload points to Qdrant
        client.upsert(
            collection_name=request.collection_name,
            points=points
        )

        logger.info(f"Indexed {len(points)} content chunks into {request.collection_name}")

        return {
            "message": f"Successfully indexed {len(points)} content chunks",
            "indexed_count": len(points),
            "collection_name": request.collection_name
        }

    except Exception as e:
        logger.error(f"Error indexing content: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error indexing content: {str(e)}")

@app.post("/search", response_model=List[SearchResult])
async def search_knowledge_base(request: QuestionRequest):
    """
    Search the knowledge base for relevant content
    """
    try:
        # Prefix the query with "query: " for the e5-base-v2 model
        prefixed_query = f"query: {request.question}"

        # Generate embedding for the query
        query_embedding = model.encode([prefixed_query])[0].tolist()

        # Search in Qdrant
        search_results = client.search(
            collection_name=COLLECTION_NAME,
            query_vector=query_embedding,
            limit=request.top_k
        )

        # Format results
        results = []
        for hit in search_results:
            result = SearchResult(
                text=hit.payload["text"],
                score=hit.score,
                metadata=hit.payload.get("metadata", {})
            )
            results.append(result)

        logger.info(f"Search completed, found {len(results)} results")

        return results

    except Exception as e:
        logger.error(f"Error searching knowledge base: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error searching knowledge base: {str(e)}")

@app.post("/answer_question", response_model=AnswerResponse)
async def answer_question(request: QuestionRequest):
    """
    Answer a question using the knowledge base and LLM
    This is a simplified implementation - in a real system, you would integrate
    with an LLM service like OpenAI or Gemini to generate answers based on retrieved content
    """
    try:
        # First, search the knowledge base
        search_results = await search_knowledge_base(request)

        if not search_results:
            return AnswerResponse(
                question=request.question,
                answer="I couldn't find relevant information in the textbook to answer your question.",
                sources=[]
            )

        # In a real implementation, you would send the question and search results
        # to an LLM to generate a comprehensive answer
        # For this example, we'll just return the top result as the answer

        # For demonstration purposes, we'll create a simple answer based on the retrieved content
        context = "\n".join([result.text for result in search_results[:2]])  # Use top 2 results
        answer = f"Based on the textbook content, here's what I found regarding your question '{request.question}':\n\n{context[:500]}..."  # Truncate for demo

        return AnswerResponse(
            question=request.question,
            answer=answer,
            sources=search_results
        )

    except Exception as e:
        logger.error(f"Error answering question: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error answering question: {str(e)}")

@app.get("/health")
async def health_check():
    """
    Health check endpoint
    """
    return {"status": "healthy", "message": "RAG service is running"}

# For local development
if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)