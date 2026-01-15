# Research Findings: Agent-based RAG Reasoning Flow

## Decision: Embedding Model Selection
**Rationale**: After researching various embedding models, OpenAI's text-embedding-ada-002 is the optimal choice for this RAG system. It offers a good balance of performance, cost, and compatibility with the OpenAI Agent SDK. The model has 1536-dimensional vectors which are well-suited for semantic similarity tasks.

**Alternatives considered**:
- Sentence Transformers (all-MiniLM-L6-v2): Open source, but less performant than OpenAI models
- OpenAI text-embedding-3-small: Newer model with potentially better performance but higher cost
- Custom embeddings: More control but increased complexity and maintenance

## Decision: Qdrant Configuration
**Rationale**: Using Qdrant Cloud with a dedicated collection for book content chunks. The collection will use cosine similarity for semantic search, with metadata fields for book title, chapter, section, and page numbers. This structure allows for efficient similarity search and filtering.

**Collection schema**:
- Vector: 1536 dimensions (matching text-embedding-ada-002)
- Metadata: book_title, chapter, section, page, content_type, difficulty_level
- Payload index on metadata fields for fast filtering

## Decision: OpenAI Agent Configuration
**Rationale**: Using GPT-4 Turbo as the primary model for response generation due to its superior reasoning capabilities and context handling. The system prompt will emphasize grounding responses in provided context and maintaining an educational, teacher-like tone.

**System prompt template**:
```
You are an educational AI assistant helping users learn about Physical AI and Humanoid Robotics.
- Ground all responses strictly in the provided book content
- Maintain a clear, teacher-like instructional tone
- Cite specific book sections that support your answers
- If the provided context doesn't contain sufficient information, acknowledge this rather than hallucinating
- Structure responses with clear explanations and practical examples where possible
```

## Decision: Default Retrieval Parameters
**Rationale**: Based on RAG literature and best practices:
- Top-K = 5: Provides sufficient context without overwhelming the agent
- Confidence threshold = 0.7: Balances relevance with recall
- These defaults can be adjusted based on performance testing and user feedback

**Alternatives considered**:
- Higher K values (10-15): More context but potential noise
- Lower threshold (0.5-0.6): More recall but potential irrelevance
- Adaptive thresholds: More complex but potentially better performance

## Decision: Agent Logic Boundaries
**Rationale**: All agent-related logic will reside in a single `agent.py` file to maintain clear separation of concerns and simplify maintenance. This file will contain:
- Query embedding and processing
- Vector search and content retrieval
- Context filtering and formatting
- OpenAI Agent interaction
- Response validation and grounding checks

**Responsibilities of agent.py**:
- Input validation and preprocessing
- Embedding generation using OpenAI API
- Qdrant vector search with filtering
- Context construction for agent
- OpenAI API calls with proper error handling
- Response formatting and source attribution
- Fallback handling for insufficient context