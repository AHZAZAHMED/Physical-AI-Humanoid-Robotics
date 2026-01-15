# Book Content RAG Agent - Architecture

## System Overview

The Book Content RAG Agent is a Retrieval-Augmented Generation system designed to process user queries by converting them to embeddings, searching for relevant book content in a Qdrant vector database, filtering results by confidence, and using an OpenAI agent to generate grounded responses based on the retrieved content.

## Components

### Core Components

- **Query Processing Service**: Converts user queries to embeddings
- **Vector Search Service**: Retrieves top-K relevant book sections from Qdrant
- **Content Filtering Service**: Filters low-confidence results
- **Agent Service**: Generates responses using OpenAI Agent with retrieved context
- **API Gateway**: Exposes REST endpoints for developers

### File Structure

```
src/
├── agent.py          # Main agent class with all RAG functionality
├── config.py         # Configuration management
├── exceptions.py     # Custom exception classes
├── main.py           # FastAPI application entry point
├── api/              # API endpoint definitions
│   ├── query_endpoint.py
│   ├── health_endpoint.py
│   └── stats_endpoint.py
├── clients/          # External service clients
│   ├── openai_client.py
│   └── qdrant_client.py
├── models/           # Data models
│   ├── query.py
│   ├── retrieved_chunk.py
│   ├── generated_response.py
│   └── agent_configuration.py
├── services/         # Business logic services
│   └── stats_service.py
├── middleware/       # API middleware
│   ├── auth.py
│   ├── rate_limit.py
│   └── error_handler.py
├── utils/            # Utility functions
│   └── logging.py
```

## Data Flow

1. **Query Reception**: API receives user query with optional parameters
2. **Input Validation**: Validate query format and parameters against data model
3. **Embedding Generation**: Convert query to 1536-dimensional vector using text-embedding-ada-002
4. **Vector Search**: Query Qdrant collection for top-K most similar book chunks
5. **Content Filtering**: Filter results below confidence threshold (default 0.7)
6. **Context Construction**: Format retrieved chunks for agent context
7. **Agent Processing**: Call GPT-4 Turbo with system prompt and retrieved context
8. **Response Validation**: Verify response is grounded in provided content
9. **Response Formatting**: Structure response with sources and metadata
10. **Response Delivery**: Return formatted response to user with performance metrics

## Grounding Rules

1. **Source Citation**: All responses must cite specific book sections used with book title, chapter, section, and page numbers
2. **Content Fidelity**: Agent must not introduce information not present in retrieved content; if information is not in the context, acknowledge the limitation
3. **Uncertainty Handling**: When insufficient relevant content exists (below confidence threshold), respond with appropriate acknowledgment rather than hallucinating
4. **Hallucination Prevention**: Implement validation to detect and prevent responses not based on provided context; use grounding confidence metrics
5. **Context Adherence**: The agent must strictly adhere to the information provided in the retrieved chunks without extrapolating beyond the provided context
6. **Transparency**: Clearly indicate when the response is based on multiple sources versus a single source

## Response Style Guidelines

1. **Teacher-like**: Clear, instructional, and educational tone that matches the book's pedagogical approach
2. **Structured**: Organized with clear explanations, logical flow, and relevant examples from the book content
3. **Cited**: References specific book sections that support the response with proper attribution
4. **Concise**: Direct and to the point while maintaining completeness and educational value
5. **Helpful**: Addresses the user's specific query or need with practical applications where appropriate
6. **Accessible**: Uses the book's approach to breaking down complex concepts into digestible parts
7. **Consistent**: Maintains the book's brand voice of being approachable yet authoritative