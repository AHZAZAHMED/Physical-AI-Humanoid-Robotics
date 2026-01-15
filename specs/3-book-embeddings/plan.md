# Implementation Plan: Book Embeddings RAG System Backend

**Branch**: `3-book-embeddings` | **Date**: 2025-12-21 | **Spec**: [link to spec]

**Input**: Feature specification for creating a backend RAG system to crawl, extract, and embed textbook content from deployed book URLs

## Summary

Implement a backend system that crawls the Physical AI & Humanoid Robotics textbook at https://ahzazahmed.github.io/Physical-AI-Humanoid-Robotics/,sitemap url : https://ahzazahmed.github.io/Physical-AI-Humanoid-Robotics/sitemap.xml extracts content, chunks it with metadata, generates embeddings using Cohere, and stores them in Qdrant Cloud for semantic search capabilities.

## Technical Context

**Language/Version**: Python 3.11
**Primary Dependencies**: cohere, qdrant-client, beautifulsoup4, requests, python-dotenv, uv (package manager)
**Storage**: Qdrant Cloud vector database
**Testing**: pytest for unit tests
**Target Platform**: Linux server environment
**Project Type**: Backend service
**Performance Goals**: Process textbook content within 10 minutes, handle 100+ pages efficiently
**Constraints**: Must respect rate limits when crawling, handle various HTML structures from Docusaurus deployment
**Scale/Scope**: Single textbook with multiple chapters/pages to be embedded


## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- **Documentation-Driven Development**: All code will be documented with setup instructions and usage examples
- **Cross-Platform Compatibility**: Using standard Python libraries and containerization for compatibility
- **Accessibility and Clarity**: Clear error messages and logging for debugging embedding pipeline issues

## Project Structure

### Documentation (this feature)
```text
specs/3-book-embeddings/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)
```text
backend/
├── src/
│   └── main.py          # Main RAG system implementation with all required functions
├── requirements.txt     # Python dependencies
├── .env.example         # Environment variables template
└── README.md            # Setup and usage instructions
```

**Structure Decision**: Single backend service with main.py containing all RAG functionality as requested by user

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Single file approach | User requirement for simplicity | Modular approach would add complexity to initial implementation |

## Phase 0: Research Tasks

1. **Environment Setup**: Research best practices for Python project initialization with UV
2. **Web Crawling**: Investigate optimal methods for crawling Docusaurus-generated static sites
3. **Text Extraction**: Best practices for extracting clean text from HTML content
4. **Chunking Strategies**: Optimal text chunking for RAG systems with metadata preservation
5. **Cohere Embeddings**: Best practices for using Cohere embedding models
6. **Qdrant Integration**: Proper setup and usage of Qdrant Cloud for vector storage
7. **URL Validation**: Techniques for validating and verifying deployed book URLs

## Phase 1: Design Elements

### Functions to Implement
1. `get_all_urls`: Discover and validate all textbook page URLs
2. `extract_text_from_urls`: Extract clean text content from each URL
3. `chunk_text`: Split text into appropriately sized chunks with metadata
4. `embed`: Generate embeddings using Cohere API
5. `create_collection`: Initialize Qdrant collection named 'rag_embeddings'
6. `save_chunk_to_qdrant`: Store embedded chunks with metadata in Qdrant
7. `main`: Execute the complete pipeline

### Data Model Considerations
- Text chunks with metadata (source URL, section title, page metadata)
- Embedding vectors for semantic search
- Qdrant point structure with payload containing content and metadata

### API/Integration Points
- Cohere API for embeddings
- Qdrant Cloud for vector storage
- Web scraping endpoints for content extraction