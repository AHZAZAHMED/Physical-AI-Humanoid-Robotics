# Data Model: RAG Retrieval Validation

## Entities

### ValidationResult
- **Fields**:
  - id: string (unique identifier)
  - query: string (the test query used)
  - results: array of RetrievedChunk (retrieved content)
  - metrics: RetrievalMetrics (performance and accuracy metrics)
  - status: enum (PASS, FAIL, ERROR)
  - timestamp: datetime (when validation was run)
  - query_category: string (keyword, semantic, section-specific)

### RetrievedChunk
- **Fields**:
  - content: string (the retrieved text content)
  - similarity_score: float (cosine similarity to query)
  - metadata: object (source URL, title, chunk ID, etc.)
  - relevance: boolean (whether content is relevant to query)

### RetrievalMetrics
- **Fields**:
  - latency_ms: float (time taken for retrieval)
  - top_k_recall: float (percentage of relevant chunks retrieved)
  - avg_similarity: float (average similarity score of results)
  - confidence_score: float (confidence in result quality)
  - throughput: float (queries per second if testing concurrently)

### ValidationError
- **Fields**:
  - id: string (unique identifier)
  - validation_id: string (reference to ValidationResult)
  - error_type: string (METADATA_MISMATCH, LOW_RELEVANCE, TIMEOUT, etc.)
  - message: string (description of the error)
  - severity: enum (CRITICAL, HIGH, MEDIUM, LOW)
  - details: object (specific error details)

### QueryCategory
- **Fields**:
  - name: string (keyword, semantic, section-specific)
  - description: string (what this category tests)
  - examples: array of string (sample queries)
  - expected_results: object (what to expect from this query type)

## Relationships

- ValidationResult contains multiple RetrievedChunk (1 to many)
- ValidationResult has one RetrievalMetrics (1 to 1)
- ValidationResult may have multiple ValidationError (1 to many)
- RetrievedChunk belongs to a ValidationResult (many to 1)

## Validation Rules

### ValidationResult
- Must have a non-empty query
- Must have at least one result or an error status
- Metrics must be present when status is PASS
- Timestamp must be set when created

### RetrievedChunk
- Content must be non-empty
- Similarity score must be between -1 and 1
- Metadata must contain required fields (URL, title)
- Relevance must be determined by comparison to ground truth

### RetrievalMetrics
- Latency must be positive
- Top-k recall must be between 0 and 1
- Average similarity must be between -1 and 1
- Throughput must be positive when testing concurrent queries

## State Transitions

### ValidationResult
- CREATED → EXECUTING (when validation starts)
- EXECUTING → COMPLETED (when validation finishes successfully)
- EXECUTING → FAILED (when validation encounters an error)
- COMPLETED → ANALYZED (when results are processed)