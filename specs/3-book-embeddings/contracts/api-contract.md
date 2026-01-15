# RAG System API Contract

## Overview
API contracts for the RAG system backend services

## Endpoints (Future Implementation)

### Embedding Service
- **POST** `/api/embeddings/generate` - Generate embeddings for text content
  - Request: `{ "text": "content to embed", "model": "cohere-multilingual" }`
  - Response: `{ "embedding": [0.1, 0.2, ...], "model": "cohere-multilingual" }`

### Storage Service
- **POST** `/api/storage/save` - Save embedded content to vector database
  - Request: `{ "content": "text content", "embedding": [0.1, 0.2, ...], "metadata": {...} }`
  - Response: `{ "id": "unique-id", "status": "success" }`

### Search Service
- **POST** `/api/search/query` - Perform semantic search on stored content
  - Request: `{ "query": "search query", "top_k": 5 }`
  - Response: `{ "results": [{"content": "...", "score": 0.9, "source": "..."}] }`

## Current Implementation
For the current scope, these are internal function interfaces in main.py:
- `get_all_urls()` -> List[str]
- `extract_text_from_urls(urls: List[str])` -> List[Dict]
- `chunk_text(content: str, metadata: Dict)` -> List[Dict]
- `embed(text: str)` -> List[float]
- `create_collection(collection_name: str)` -> bool
- `save_chunk_to_qdrant(chunk: Dict)` -> bool
- `main()` -> None