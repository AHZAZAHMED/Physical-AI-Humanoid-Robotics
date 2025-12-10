# Quickstart Guide: Physical AI & Humanoid Robotics Textbook Platform

## Prerequisites

- Node.js 18+ and npm/yarn
- Python 3.11+
- Git
- Access to OpenAI API key
- Access to Qdrant Cloud account
- Access to Neon Postgres account

## Local Development Setup

### 1. Clone and Initialize the Repository

```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Backend Setup

```bash
# Navigate to backend directory
cd backend

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install fastapi uvicorn python-multipart openai qdrant-client python-jose[cryptography] passlib[bcrypt] python-dotenv google-generativeai

# Set up environment variables
cp .env.example .env
# Edit .env with your API keys and database URLs
```

### 3. Frontend Setup

```bash
# From project root
cd frontend

# Install dependencies
npm install
# or
yarn install

# Set up environment variables
cp .env.example .env
# Edit .env with your API URLs and keys
```

### 4. Database Setup

1. Set up Qdrant Cloud cluster for vector storage
2. Set up Neon Postgres database for user data
3. Update connection strings in your environment files

### 5. Content Ingestion

```bash
# From backend directory
python scripts/ingest.py
```

This script will:
- Read MDX content files
- Extract text content
- Generate embeddings using OpenAI
- Store in Qdrant vector database

## Running the Applications

### Backend (API Server)

```bash
# From backend directory with virtual environment activated
uvicorn src.api.main:app --reload --port 8000
```

### Frontend (Docusaurus)

```bash
# From frontend directory
npm run start
# or
yarn start
```

The frontend will be available at `http://localhost:3000`

## Key Configuration

### Environment Variables

**Backend (.env):**
```
GEMINI_API_KEY=your_gemini_api_key
QDRANT_URL=your_qdrant_cluster_url
QDRANT_API_KEY=your_qdrant_api_key
DATABASE_URL=your_postgres_connection_string
SECRET_KEY=your_secret_key_for_auth
ALGORITHM=HS256
ACCESS_TOKEN_EXPIRE_MINUTES=30
```

**Frontend (.env):**
```
REACT_APP_API_BASE_URL=http://localhost:8000
REACT_APP_GEMINI_API_KEY=your_gemini_api_key_if_needed_client_side
```

## Project Structure

```
project-root/
├── backend/
│   ├── src/
│   │   ├── models/          # Data models
│   │   ├── services/        # Business logic
│   │   └── api/             # API routes
│   ├── scripts/             # Utility scripts (ingest.py)
│   └── tests/
├── frontend/
│   ├── docs/                # MDX content files
│   ├── src/
│   │   ├── components/      # React components
│   │   ├── pages/           # Page components
│   │   └── services/        # API service functions
│   └── tests/
└── scripts/                 # Deployment and utility scripts
```

## First Steps for Developers

1. Set up your local environment following the steps above
2. Run the content ingestion script to populate the vector database
3. Start both backend and frontend servers
4. Register a new account to test the authentication flow
5. Navigate through the textbook content
6. Test the chat functionality with the RAG system
7. Experiment with difficulty level switching
8. Test the language switching functionality

## Common Commands

```bash
# Run backend tests
cd backend
python -m pytest

# Run frontend tests
cd frontend
npm test
# or
yarn test

# Build frontend for production
cd frontend
npm run build
# or
yarn build

# Deploy to GitHub Pages
cd frontend
npm run deploy
# or
yarn deploy
```