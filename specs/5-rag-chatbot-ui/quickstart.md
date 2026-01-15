# Quickstart Guide: RAG Chatbot UI Integration

## Prerequisites

- Node.js (v16 or higher)
- Python (v3.8 or higher)
- pip package manager
- Git

## Installation

### 1. Clone the Repository
```bash
git clone <repository-url>
cd <repository-directory>
```

### 2. Set Up Backend
```bash
# Navigate to backend directory
cd backend

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install fastapi uvicorn python-dotenv requests openai agents google-generativeai qdrant-client sentence-transformers cohere

# Set up environment variables
cp .env.example .env
# Edit .env with your API keys and configuration
```

### 3. Set Up Frontend
```bash
# Navigate to the Docusaurus project
cd ../physical-ai-textbook

# Install dependencies
npm install

# Install ChatKit if needed
npm install @chatscope/chat-ui-kit-react @chatscope/chat-ui-kit-styles
```

## Configuration

### Backend Configuration
Update the `.env` file in the `backend/` directory:

```env
# Vector Database Configuration
QDRANT_HOST="https://your-qdrant-instance.com"
QDRANT_API_KEY="your-qdrant-api-key"

# LLM API Configuration
GEMINI_API_KEY="your-gemini-api-key"
COHERE_API_KEY="your-cohere-api-key"

# Application Configuration
PORT=8000
DEBUG=False
```

### Frontend Configuration
The frontend will connect to the backend at `http://localhost:8000` by default.

## Running the Application

### 1. Start the Backend API
```bash
cd backend
source venv/bin/activate  # Activate virtual environment
uvicorn api:app --reload --port 8000
```

### 2. Start the Frontend
```bash
cd physical-ai-textbook
npm start
```

The application will be available at `http://localhost:3000`.

## API Endpoints

### Chat Endpoint
- **POST** `/api/chat`
- Send user queries to the RAG agent
- Request format: `{"query": "your question here", "sessionId": "optional-session-id"}`
- Response includes AI-generated response and source citations

### Health Check
- **GET** `/api/health`
- Check if backend services are running

## Testing the Chatbot

1. Visit the Docusaurus site at `http://localhost:3000`
2. Look for the floating chatbot icon at the bottom of the screen
3. Click the icon to open the chat interface
4. Type a question about Physical AI and Humanoid Robotics
5. Receive a response with citations from the textbook

## Troubleshooting

### Backend Issues
- Ensure all API keys are correctly set in `.env`
- Check that the Qdrant database is accessible
- Verify that the RAG agent is properly initialized

### Frontend Issues
- Check browser console for JavaScript errors
- Verify that the API endpoint is accessible from the frontend
- Ensure CORS settings allow frontend-backend communication

### Common Solutions
- Restart both backend and frontend servers
- Clear browser cache if UI issues persist
- Check network connectivity to API services