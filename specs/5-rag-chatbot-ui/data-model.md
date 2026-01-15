# Data Model: RAG Chatbot UI Integration

## Entity Definitions

### ChatMessage Entity
**Description**: Represents a single message in the chat conversation

**Fields**:
- `id`: string (required) - Unique identifier for the message
- `role`: "user" | "assistant" (required) - Role of the message sender
- `content`: string (required) - The text content of the message
- `timestamp`: string (ISO 8601 format) (required) - When the message was created
- `sources`: Array<Object> (optional) - Source citations for RAG responses

**Validation Rules**:
- `id` must be a valid UUID or unique string
- `role` must be either "user" or "assistant"
- `content` must be non-empty string with maximum length of 10,000 characters
- `timestamp` must be in ISO 8601 format
- `sources` array items must have `content` and `metadata` properties if present

### ChatRequest Entity
**Description**: Represents a request from the frontend to the backend

**Fields**:
- `query`: string (required) - The user's query text
- `sessionId`: string (optional) - Identifier for conversation continuity
- `userId`: string (optional) - Identifier for user (if authentication is implemented)

**Validation Rules**:
- `query` must be non-empty string with maximum length of 1,000 characters
- `sessionId` must be a valid UUID or similar identifier if provided
- `query` must be sanitized to prevent injection attacks

### ChatResponse Entity
**Description**: Represents a response from the backend to the frontend

**Fields**:
- `response`: string (required) - The AI-generated response text
- `sources`: Array<Object> (optional) - Citations from the RAG system
- `sessionId`: string (required) - Identifier for conversation continuity
- `error`: Object (optional) - Error information if the request failed
- `timestamp`: string (ISO 8601 format) (required) - When the response was generated

**Validation Rules**:
- `response` must be non-empty string
- `sources` array items must have `content` and `metadata` properties if present
- `sessionId` must be a valid identifier
- `error` object must have `message` property if present
- `timestamp` must be in ISO 8601 format

### Source Entity
**Description**: Represents a source citation from the RAG system

**Fields**:
- `content`: string (required) - The content that was referenced
- `metadata`: Object (required) - Additional information about the source
- `similarityScore`: number (optional) - How closely the source matches the query
- `bookTitle`: string (optional) - Title of the book the source came from

**Validation Rules**:
- `content` must be non-empty string
- `metadata` must be a valid JSON object
- `similarityScore` must be between 0 and 1 if provided
- `bookTitle` must be non-empty string if provided

### HealthCheck Entity
**Description**: Represents the health status of the backend service

**Fields**:
- `status`: "healthy" | "unhealthy" (required) - Overall health status
- `timestamp`: string (ISO 8601 format) (required) - When the check was performed
- `services`: Object (optional) - Status of individual services

**Validation Rules**:
- `status` must be either "healthy" or "unhealthy"
- `timestamp` must be in ISO 8601 format
- `services` object properties must have values of "operational" or "degraded"

## State Transitions

### ChatMessage States
- `created` → `sent` → `delivered` → `read`
- For RAG responses: `received_from_agent` → `formatted` → `delivered`

### ChatSession States
- `initialized` → `active` → `inactive` → `archived`
- Sessions become inactive after 30 minutes of inactivity

## Relationships

### ChatMessage Relationship to ChatSession
- Each ChatMessage belongs to one ChatSession (many-to-one)
- ChatSession contains multiple ChatMessages (one-to-many)

### ChatRequest to ChatResponse
- Each ChatRequest corresponds to one ChatResponse (one-to-one)
- Both share the same sessionId for correlation

## Data Flow Patterns

### Request Processing Flow
1. Frontend sends ChatRequest to backend
2. Backend validates the request
3. Backend processes the query using RAG agent
4. Backend constructs ChatResponse with sources
5. Backend sends ChatResponse to frontend

### Error Handling Flow
1. If error occurs during processing, backend creates error response
2. Error response follows same structure as normal response
3. Frontend detects error field and handles appropriately
4. Frontend may offer retry mechanism to user

## Storage Considerations

### Client-Side Storage
- Recent conversation history (limited to current session)
- Session ID for continuity
- User preferences for chat interface

### Server-Side Storage
- Full conversation history (if persistence is required)
- Query logs for debugging and analytics
- Cached responses for frequently asked questions

## API Payload Examples

### Sample Request
```json
{
  "query": "What is Physical AI and Humanoid Robotics?",
  "sessionId": "abc123def456"
}
```

### Sample Response
```json
{
  "response": "Physical AI and Humanoid Robotics refers to the field dedicated to developing embodied intelligence...",
  "sources": [
    {
      "content": "Theoretical Foundations The theoretical underpinnings of Physical AI draw from several disciplines...",
      "metadata": {
        "book_title": "Physical AI & Humanoid Robotics Textbook",
        "chapter": "Unknown Chapter",
        "section": "Unknown Section"
      },
      "similarityScore": 0.95,
      "bookTitle": "Physical AI & Humanoid Robotics Textbook"
    }
  ],
  "sessionId": "abc123def456",
  "timestamp": "2026-01-13T06:18:07.456Z"
}
```

### Sample Error Response
```json
{
  "response": "",
  "sources": [],
  "sessionId": "abc123def456",
  "error": {
    "message": "No relevant content found in the book for the given query",
    "type": "InsufficientContextError"
  },
  "timestamp": "2026-01-13T06:18:07.456Z"
}
```