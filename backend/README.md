# RAG System Backend for Physical AI & Humanoid Robotics Textbook

This backend system crawls the Physical AI & Humanoid Robotics textbook, extracts content, generates embeddings using Cohere, and stores them in Qdrant Cloud for semantic search capabilities.

## Features

- Crawls the textbook website at https://ahzazahmed.github.io/Physical-AI-Humanoid-Robotics/
- Extracts clean text content while preserving important metadata
- Chunks text appropriately for embedding
- Generates embeddings using Cohere's multilingual model
- Stores vectors in Qdrant Cloud with cosine similarity
- Verifies storage with sample similarity searches

## Prerequisites

- Python 3.11+
- UV package manager
- Cohere API key
- Qdrant Cloud account and API key

## Setup

1. **Install UV package manager** (if not already installed):
   ```bash
   pip install uv
   ```

2. **Create virtual environment and install dependencies**:
   ```bash
   cd backend
   python3 -m venv .venv_new
   source .venv_new/bin/activate  # On Windows: .venv_new\Scripts\activate
   pip install --break-system-packages -r requirements.txt
   ```

3. **Set up environment variables**:
   ```bash
   cp .env.example .env
   ```

   Edit the `.env` file with your actual credentials:
   ```env
   COHERE_API_KEY=your_cohere_api_key_here
   QDRANT_URL=your_qdrant_cloud_url_here
   QDRANT_API_KEY=your_qdrant_api_key_here
   BOOK_URL=https://ahzazahmed.github.io/Physical-AI-Humanoid-Robotics/
   ```

## Usage

Run the complete embedding pipeline:
```bash
cd backend/src
python main.py
```

The script will:
1. Discover all URLs from the textbook
2. Extract text content from each URL
3. Chunk the text appropriately
4. Generate embeddings using Cohere
5. Create 'rag_embeddings' collection in Qdrant
6. Save chunks to Qdrant with metadata
7. Verify storage with sample similarity searches

## Architecture

The system implements the following functions in a single `main.py` file:

- `get_all_urls()`: Discover all textbook page URLs
- `extract_text_from_urls()`: Extract clean text content from each URL
- `chunk_text()`: Split text into appropriately sized chunks with metadata
- `embed()`: Generate embeddings using Cohere API
- `create_collection()`: Initialize Qdrant collection named 'rag_embeddings'
- `save_chunk_to_qdrant()`: Store embedded chunks with metadata in Qdrant
- `main()`: Execute the complete pipeline

## Verification

After running the pipeline, you can verify successful embedding by:
- Checking the Qdrant Cloud dashboard for the 'rag_embeddings' collection
- Reviewing console output showing successful processing of book content
- Confirming sample similarity search results

## Error Handling

The system includes comprehensive error handling and logging:
- Network request errors are caught and logged
- API errors are handled gracefully
- Progress is reported throughout the process
- Failed chunks are skipped while continuing with others

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request