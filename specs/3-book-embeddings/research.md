# Research: Book Embeddings RAG System Backend

## Overview
Research findings for implementing a RAG system backend to crawl, extract, and embed textbook content from the deployed book at https://ahzazahmed.github.io/Physical-AI-Humanoid-Robotics/

## Decision: Web Crawling Method
**Rationale**: Need to efficiently discover and crawl all pages of the Docusaurus-deployed textbook
**Alternatives considered**:
- Manual URL listing vs. automated discovery
- Selenium vs. requests/beautifulsoup
- Sitemap parsing vs. link following

**Chosen approach**: Use requests + BeautifulSoup with breadth-first traversal to discover all pages from the root URL, with proper rate limiting to avoid overwhelming the server.

## Decision: Text Extraction Method
**Rationale**: Extract clean, readable text content from Docusaurus-generated HTML
**Alternatives considered**:
- Generic HTML stripping vs. Docusaurus-specific selectors
- Raw HTML parsing vs. structured content extraction

**Chosen approach**: Use BeautifulSoup with Docusaurus-specific CSS selectors to target main content areas while preserving document structure and headings.

## Decision: Text Chunking Strategy
**Rationale**: Split long textbook content into appropriately sized chunks for embedding
**Alternatives considered**:
- Fixed-length vs. semantic chunking
- Overlapping vs. non-overlapping chunks
- Metadata preservation methods

**Chosen approach**: Recursive character text splitter with overlap, preserving section headers and source metadata for context retrieval.

## Decision: Embedding Model Selection
**Rationale**: Choose between Cohere vs. other embedding providers (OpenAI, Hugging Face)
**Alternatives considered**:
- Cohere embeddings vs. OpenAI embeddings vs. open-source models
- Different model sizes and dimensions

**Chosen approach**: Cohere multilingual-22-12 model for its balance of quality, speed, and multilingual support suitable for textbook content.

## Decision: Vector Database Configuration
**Rationale**: Set up Qdrant Cloud for reliable vector storage and retrieval
**Alternatives considered**:
- Qdrant vs. Pinecone vs. Weaviate vs. local solutions
- Similarity metrics (cosine, euclidean, dot product)

**Chosen approach**: Qdrant Cloud with cosine similarity for semantic search, using appropriate vector dimensions matching the Cohere model output.

## Decision: URL Validation Method
**Rationale**: Ensure all textbook URLs are accessible before processing
**Alternatives considered**:
- Head requests vs. full GET requests
- Concurrent vs. sequential validation

**Chosen approach**: Sequential HEAD requests with timeout handling to validate URLs before processing.