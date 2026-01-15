# Book Content RAG Agent - Getting Started

## Overview

This guide provides a quick setup for the RAG (Retrieval-Augmented Generation) system that processes user queries against book content and generates grounded, educational responses.

## Prerequisites

- Python 3.9+
- OpenAI API key
- Qdrant Cloud account and collection with book embeddings
- Access to book content embeddings (1536-dimensional vectors using text-embedding-ada-002)

## Setup

### 1. Environment Configuration

```bash
# Create virtual environment
python -m venv rag-agent-env
source rag-agent-env/bin/activate  # On Windows: rag-agent-env\Scripts\activate

# Install dependencies
pip install openai qdrant-client python-dotenv
```

### 2. Environment Variables

Create a `.env` file with the following:

```env
OPENAI_API_KEY=your_openai_api_key_here
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_HOST=your_qdrant_cloud_endpoint
QDRANT_COLLECTION_NAME=book_content_chunks
EMBEDDING_MODEL=text-embedding-ada-002
AGENT_MODEL=gpt-4-turbo
```

### 3. Initialize the Agent

```python
from src.agent import RAGAgent
from src.config import AgentConfiguration

# Initialize the agent with default configuration
config = AgentConfiguration()
agent = RAGAgent(config)

# Or initialize with custom configuration
config = AgentConfiguration(
    default_top_k=5,
    default_confidence_threshold=0.7,
    agent_model="gpt-4-turbo",
    embedding_model="text-embedding-ada-002"
)
agent = RAGAgent(config)
```

## Usage Examples

### Basic Query Processing

```python
# Process a simple query
response = agent.process_query("Explain the concept of forward kinematics in robotics")

print(response.text)
print(f"Sources used: {len(response.sources)}")
print(f"Grounding confidence: {response.grounding_confidence}")
```

### Query with Custom Parameters

```python
# Process query with custom parameters
response = agent.process_query(
    query="How do I implement a PID controller for a humanoid robot?",
    parameters={
        "top_k": 7,
        "confidence_threshold": 0.65
    }
)

# Access source information
for source in response.sources:
    print(f"Source: {source['source']['book']} - {source['source']['chapter']}")
    print(f"Similarity: {source['similarity_score']}")
    print(f"Content preview: {source['content'][:200]}...")
```

### API Usage

```bash
# Send a query to the API
curl -X POST http://localhost:8000/api/v1/query \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer your_api_key" \
  -d '{
    "query": "What are the key challenges in humanoid robotics?",
    "parameters": {
      "top_k": 5,
      "confidence_threshold": 0.7
    }
  }'
```

## Configuration

### Default Parameters

- `default_top_k`: 5 (number of results to retrieve)
- `default_confidence_threshold`: 0.7 (minimum similarity score)
- `agent_model`: "gpt-4-turbo"
- `embedding_model`: "text-embedding-ada-002"
- `max_retries`: 3
- `timeout_seconds`: 30

### Updating Configuration via API

```bash
# Update configuration via API
curl -X PUT http://localhost:8000/api/v1/config \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer your_api_key" \
  -d '{
    "default_top_k": 7,
    "default_confidence_threshold": 0.65
  }'
```

## Troubleshooting

### Common Issues

1. **No relevant results found**
   - Try lowering the confidence threshold
   - Verify the query is related to the book content
   - Check that the Qdrant collection has the correct embeddings

2. **Slow response times**
   - Check API key quotas for OpenAI and Qdrant
   - Verify network connectivity to external services
   - Consider caching frequent queries

3. **Hallucinations in responses**
   - Increase the confidence threshold
   - Review the grounding validation logic
   - Ensure retrieved context is relevant to the query