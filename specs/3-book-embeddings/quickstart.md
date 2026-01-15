# Quickstart: Book Embeddings RAG System Backend

## Overview
Quick setup guide to run the RAG system backend that crawls, extracts, and embeds textbook content.

## Prerequisites
- Python 3.11+
- UV package manager
- Access to Cohere API
- Qdrant Cloud account and API key

## Setup

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. **Create backend directory**
   ```bash
   mkdir backend
   cd backend
   ```

3. **Install UV (if not already installed)**
   ```bash
   pip install uv
   # Or install via other methods: https://docs.astral.sh/uv/getting-started/installation/
   ```

4. **Create virtual environment and install dependencies**
   ```bash
   uv venv
   source .venv/bin/activate  # On Windows: .venv\Scripts\activate
   uv pip install cohere qdrant-client beautifulsoup4 requests python-dotenv
   ```

5. **Set up environment variables**
   ```bash
   cp .env.example .env
   ```

   Edit `.env` file with your credentials:
   ```env
   COHERE_API_KEY=your_cohere_api_key_here
   QDRANT_URL=your_qdrant_cloud_url_here
   QDRANT_API_KEY=your_qdrant_api_key_here
   BOOK_URL=https://ahzazahmed.github.io/Physical-AI-Humanoid-Robotics/
   ```

## Usage

1. **Run the embedding pipeline**
   ```bash
   cd src
   python main.py
   ```

2. **The script will execute the following steps:**
   - Discover all URLs from the book
   - Extract text content from each URL
   - Chunk the text appropriately
   - Generate embeddings using Cohere
   - Create 'rag_embeddings' collection in Qdrant
   - Save chunks to Qdrant with metadata
   - Verify storage with sample similarity searches

## Verification

After running the script, you can verify successful embedding by checking:
- Qdrant Cloud dashboard for the 'rag_embeddings' collection
- Console output showing successful processing of book content
- Sample similarity search results

## Troubleshooting

- **API Key Issues**: Ensure your Cohere and Qdrant API keys are correctly set in `.env`
- **URL Access**: Verify the book URL is accessible and publicly available
- **Rate Limits**: The script includes delays to respect rate limits; be patient during execution
- **Memory Issues**: Large books may require more memory; monitor resource usage