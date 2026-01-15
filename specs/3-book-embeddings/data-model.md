# Data Model: Book Embeddings RAG System

## Overview
Data structures and entities for the RAG system backend that handles textbook content embedding and storage.

## Core Entities

### TextChunk
- **id**: Unique identifier for the chunk (UUID)
- **content**: String - The actual text content of the chunk
- **source_url**: String - Original URL where the content was found
- **section_title**: String - Title of the section/chapter
- **page_title**: String - Title of the page
- **metadata**: Dict - Additional metadata (word count, headings, etc.)
- **embedding_vector**: List[float] - Vector representation from embedding model
- **created_at**: DateTime - Timestamp of creation
- **updated_at**: DateTime - Timestamp of last update

### EmbeddingResult
- **chunk_id**: String - Reference to the text chunk
- **vector**: List[float] - The embedding vector
- **model_used**: String - Name of the embedding model used
- **processing_time**: Float - Time taken to generate embedding

### QdrantPoint
- **id**: String - Unique identifier for the Qdrant point
- **vector**: List[float] - Embedding vector
- **payload**: Dict - Contains content, metadata, and source information
  - content: String - The text content
  - source_url: String - Original URL
  - section_title: String - Section title
  - page_title: String - Page title
  - metadata: Dict - Additional metadata

## Relationships
- Each TextChunk corresponds to one EmbeddingResult
- Each EmbeddingResult is stored as one QdrantPoint
- Multiple TextChunks can come from the same source_url

## Validation Rules
- TextChunk.content must be between 50 and 2000 characters
- source_url must be a valid URL format
- embedding_vector length must match the embedding model's output dimension
- All timestamp fields must be in ISO 8601 format

## State Transitions
- TextChunk: CREATED -> EMBEDDED -> STORED_IN_QDRANT
- Processing follows the pipeline: extract -> chunk -> embed -> store