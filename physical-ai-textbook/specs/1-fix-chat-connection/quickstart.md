# Quickstart Guide: Chat Service Connection

## Overview
This guide provides instructions for setting up and using the chat service connection in the Physical AI & Humanoid Robotics Textbook Platform.

## Prerequisites
- Node.js 18+ for frontend development
- Python 3.9+ for backend development
- Docker (for vector database - Qdrant)
- Access to Gemini API key (for LLM service)

## Setting Up the Backend Service

### 1. Start the Backend Server
```bash
cd physical-ai-textbook/backend
pip install -r requirements.txt
python -m src.main
```

The backend will start on `http://localhost:8000`

### 2. Verify the Chat Endpoint
Test the chat endpoint directly:
```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "Hello, can you explain humanoid robotics?", "session_id": "test-session"}'
```

### 3. Check Service Health
```bash
curl http://localhost:8000/chat/health
```

## Setting Up the Frontend

### 1. Start the Frontend Server
```bash
cd physical-ai-textbook
npm install
npm start
```

The frontend will start on `http://localhost:3000`

## Troubleshooting Common Issues

### Issue: "I'm having trouble connecting to the chat service"
**Cause**: The backend service is not running or the frontend cannot reach it.
**Solution**:
1. Verify the backend is running on `http://localhost:8000`
2. Check that the Qdrant vector database is accessible
3. Confirm the Gemini API key is properly configured

### Issue: Authentication errors when calling chat endpoint
**Cause**: Authentication middleware conflicts.
**Solution**: Ensure the `/chat` endpoint is properly configured as a public route.

### Issue: CORS errors
**Cause**: Cross-origin requests are blocked.
**Solution**: Check that CORS settings in `backend/src/main.py` allow requests from `http://localhost:3000`

## Testing the Connection

### Manual Test
1. Start both frontend and backend services
2. Open the frontend in your browser
3. Open the chat interface and send a message
4. Verify that you receive a response from the textbook content

### API Test
Use the following command to test the API directly:
```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{
    "message": "What is the main concept of Physical AI?",
    "session_id": "quickstart-test"
  }'
```

## Configuration

### Environment Variables
Ensure these variables are set in your `.env` file:

Backend (in `backend/.env`):
```
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
GEMINI_API_KEY=your_gemini_api_key
```

### Frontend Configuration
The frontend is configured to connect to `http://localhost:8000/chat` by default in `src/theme/Chatbot/index.js`.

## Best Practices

1. **Error Handling**: Always implement proper error handling for network requests
2. **Connection Monitoring**: Implement connection status indicators for better user experience
3. **Retry Logic**: Use exponential backoff for retry attempts when connections fail
4. **Security**: Keep API keys secure and never expose them in frontend code
5. **Performance**: Implement proper request timeouts to prevent hanging connections