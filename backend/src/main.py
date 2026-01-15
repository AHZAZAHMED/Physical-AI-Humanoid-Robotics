"""
RAG System Backend for Physical AI & Humanoid Robotics Textbook

This module implements the complete pipeline for:
1. Crawling the textbook website
2. Extracting content
3. Generating embeddings
4. Storing in Qdrant vector database
"""

import os
import requests
from bs4 import BeautifulSoup
from urllib.parse import urljoin, urlparse
from typing import List, Dict, Tuple
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
import time
import logging
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def get_all_urls(base_url: str) -> List[str]:
    """
    Discover and return all URLs from the textbook site.

    Args:
        base_url (str): The base URL of the textbook

    Returns:
        List[str]: List of all discovered URLs
    """
    logger.info(f"Starting to crawl: {base_url}")
    urls = set()
    to_visit = [base_url]
    visited = set()

    # Parse the base domain to only crawl within the same domain
    base_domain = urlparse(base_url).netloc

    while to_visit:
        current_url = to_visit.pop(0)

        if current_url in visited:
            continue

        visited.add(current_url)
        logger.info(f"Crawling: {current_url}")

        # Retry logic for network requests
        max_retries = 3
        retry_count = 0
        while retry_count < max_retries:
            try:
                # Add a small delay to be respectful to the server
                time.sleep(0.5)

                response = requests.get(current_url, timeout=10)
                response.raise_for_status()

                # Only process HTML content
                if 'text/html' in response.headers.get('content-type', ''):
                    soup = BeautifulSoup(response.text, 'html.parser')

                    # Add current URL to the list
                    urls.add(current_url)

                    # Find all links on the page
                    for link in soup.find_all('a', href=True):
                        href = link['href']
                        absolute_url = urljoin(current_url, href)

                        # Only add URLs from the same domain and with the base path
                        if urlparse(absolute_url).netloc == base_domain and absolute_url.startswith(base_url):
                            if absolute_url not in visited and absolute_url not in to_visit:
                                to_visit.append(absolute_url)

                break  # Success, break out of retry loop
            except requests.RequestException as e:
                retry_count += 1
                logger.warning(f"Attempt {retry_count} failed for {current_url}: {e}")
                if retry_count < max_retries:
                    time.sleep(2 ** retry_count)  # Exponential backoff
                else:
                    logger.error(f"Failed to crawl {current_url} after {max_retries} attempts: {e}")
                    continue

    logger.info(f"Discovered {len(urls)} URLs")
    return list(urls)


def extract_text_from_urls(urls: List[str]) -> List[Dict]:
    """
    Extract text content from each URL with metadata.

    Args:
        urls (List[str]): List of URLs to extract content from

    Returns:
        List[Dict]: List of dictionaries containing content and metadata
    """
    contents = []

    for i, url in enumerate(urls):
        logger.info(f"Extracting content from {url} ({i+1}/{len(urls)})")

        # Retry logic for network requests
        max_retries = 3
        retry_count = 0
        while retry_count < max_retries:
            try:
                time.sleep(0.5)  # Be respectful to the server
                response = requests.get(url, timeout=10)
                response.raise_for_status()

                soup = BeautifulSoup(response.text, 'html.parser')

                # Remove script and style elements
                for script in soup(["script", "style"]):
                    script.decompose()

                # Try to get main content area (Docusaurus specific selectors)
                main_content = soup.find('main') or soup.find('article') or soup.find('div', class_='container')

                if main_content:
                    text = main_content.get_text(separator=' ', strip=True)
                else:
                    # Fallback to body content
                    body = soup.find('body')
                    text = body.get_text(separator=' ', strip=True) if body else soup.get_text(separator=' ', strip=True)

                # Clean up the text
                lines = (line.strip() for line in text.splitlines())
                chunks = (phrase.strip() for line in lines for phrase in line.split("  "))
                text = ' '.join(chunk for chunk in chunks if chunk)

                # Extract page title
                title_tag = soup.find('title')
                page_title = title_tag.get_text().strip() if title_tag else "No Title"

                # Extract any headings as additional metadata
                headings = []
                for heading in soup.find_all(['h1', 'h2', 'h3', 'h4', 'h5', 'h6']):
                    headings.append(heading.get_text().strip())

                content_data = {
                    'url': url,
                    'content': text,
                    'page_title': page_title,
                    'headings': headings[:10],  # Limit to first 10 headings
                    'word_count': len(text.split())
                }

                contents.append(content_data)
                break  # Success, break out of retry loop

            except requests.RequestException as e:
                retry_count += 1
                logger.warning(f"Attempt {retry_count} failed for {url}: {e}")
                if retry_count < max_retries:
                    time.sleep(2 ** retry_count)  # Exponential backoff
                else:
                    logger.error(f"Failed to extract content from {url} after {max_retries} attempts: {e}")
                    continue
            except Exception as e:
                logger.error(f"Unexpected error extracting content from {url}: {e}")
                break  # Don't retry for unexpected errors

    logger.info(f"Extracted content from {len(contents)} URLs")
    return contents


def chunk_text(content: str, metadata: Dict, chunk_size: int = 1000) -> List[Dict]:
    """
    Split text content into chunks of specified size with overlap.

    Args:
        content (str): The content to chunk
        metadata (Dict): Metadata to include with each chunk
        chunk_size (int): Maximum size of each chunk in characters

    Returns:
        List[Dict]: List of chunk dictionaries with content and metadata
    """
    if len(content) <= chunk_size:
        return [{
            'content': content,
            'metadata': metadata,
            'chunk_index': 0
        }]

    chunks = []
    start = 0
    chunk_index = 0

    while start < len(content):
        end = start + chunk_size

        # If we're not at the end, try to break at a sentence boundary
        if end < len(content):
            # Look for a good breaking point (sentence or paragraph end)
            snippet = content[start:end]
            last_sentence = max(snippet.rfind('.'), snippet.rfind('!'), snippet.rfind('?'), snippet.rfind('\n'))

            if last_sentence > chunk_size // 2:  # Only break if we're not cutting too early
                end = start + last_sentence + 1
            else:
                # Otherwise, look for a space
                last_space = snippet.rfind(' ')
                if last_space > chunk_size // 2:
                    end = start + last_space

        chunk_content = content[start:end].strip()

        if chunk_content:  # Only add non-empty chunks
            chunk_data = {
                'content': chunk_content,
                'metadata': metadata.copy(),
                'chunk_index': chunk_index
            }
            chunks.append(chunk_data)
            chunk_index += 1

        start = end

    logger.info(f"Split content into {len(chunks)} chunks")
    return chunks


def embed(text: str) -> List[float]:
    """
    Generate embedding for the given text using sentence transformers.

    Args:
        text (str): Text to generate embedding for

    Returns:
        List[float]: The embedding vector
    """
    # Use sentence transformers for local embedding generation
    try:
        from sentence_transformers import SentenceTransformer

        # Load the model (in a real implementation, you'd want to cache this)
        model = SentenceTransformer('all-MiniLM-L6-v2')

        # Generate embedding
        embeddings = model.encode([text])
        return embeddings[0].tolist()  # Convert to list format

    except ImportError:
        logger.error("sentence-transformers not installed. Install with: pip install sentence-transformers")
        # Fallback: use a simple hash-based approach for demonstration
        import hashlib
        logger.warning("Using fallback embedding method - this is not a real semantic embedding")
        # Create a fake embedding using hashing (not semantically meaningful)
        hash_obj = hashlib.sha256(text.encode())
        hex_dig = hash_obj.hexdigest()
        # Convert hex digest to a list of floats in a reasonable range
        embedding = []
        for i in range(0, len(hex_dig), 2):
            byte_val = int(hex_dig[i:i+2], 16)
            normalized_val = (byte_val - 128) / 128.0  # Normalize to [-1, 1]
            embedding.append(normalized_val)

        # Pad or truncate to 384 dimensions (all-MiniLM-L6-v2 produces 384-dim vectors)
        while len(embedding) < 384:
            embedding.append(0.0)
        embedding = embedding[:384]

        return embedding


def create_collection(collection_name: str = 'book_content_chunks') -> bool:
    """
    Create a collection in Qdrant for storing embeddings.

    Args:
        collection_name (str): Name of the collection to create

    Returns:
        bool: True if successful, False otherwise
    """
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")

    if not qdrant_url or not qdrant_api_key:
        raise ValueError("QDRANT_URL and QDRANT_API_KEY environment variables must be set")

    client = QdrantClient(
        url=qdrant_url,
        api_key=qdrant_api_key,
        prefer_grpc=False  # Using HTTP for simplicity
    )

    # Determine the embedding dimension based on the configured embedding service
    try:
        from src.rag_agent.clients.openai_client import OpenAIService
        from src.rag_agent.config import AgentConfiguration

        config = AgentConfiguration()
        embedding_service = OpenAIService(config)

        # Test to see what embedding dimension we get
        test_embedding = embedding_service.create_embedding("test")
        embedding_size = len(test_embedding)
        logger.info(f"Using embedding dimension: {embedding_size}")
    except Exception:
        # Default to sentence transformer dimension if there's an error
        embedding_size = 384  # all-MiniLM-L6-v2 dimension
        logger.info(f"Using default embedding dimension: {embedding_size}")

    # Retry logic for Qdrant operations
    max_retries = 3
    retry_count = 0
    while retry_count < max_retries:
        try:
            # Check if collection already exists
            collections = client.get_collections()
            collection_exists = any(col.name == collection_name for col in collections.collections)

            if not collection_exists:
                # Create new collection with cosine similarity using the correct dimension
                client.recreate_collection(
                    collection_name=collection_name,
                    vectors_config=models.VectorParams(size=embedding_size, distance=models.Distance.COSINE),
                )
                logger.info(f"Created new collection: {collection_name} with dimension {embedding_size}")
            else:
                logger.info(f"Collection {collection_name} already exists")

            return True
        except Exception as e:
            retry_count += 1
            logger.warning(f"Attempt {retry_count} failed for creating collection: {e}")
            if retry_count < max_retries:
                time.sleep(2 ** retry_count)  # Exponential backoff
            else:
                logger.error(f"Error creating collection after {max_retries} attempts: {e}")
                return False


def save_chunk_to_qdrant(chunk: Dict, vector: List[float], collection_name: str = 'book_content_chunks') -> bool:
    """
    Save a text chunk with its embedding to Qdrant.

    Args:
        chunk (Dict): The chunk data with content and metadata
        vector (List[float]): The embedding vector
        collection_name (str): Name of the collection to save to

    Returns:
        bool: True if successful, False otherwise
    """
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")

    if not qdrant_url or not qdrant_api_key:
        raise ValueError("QDRANT_URL and QDRANT_API_KEY environment variables must be set")

    client = QdrantClient(
        url=qdrant_url,
        api_key=qdrant_api_key,
        prefer_grpc=False
    )

    # Retry logic for Qdrant operations
    max_retries = 3
    retry_count = 0
    while retry_count < max_retries:
        try:
            # Create a unique ID for this chunk
            import uuid
            point_id = str(uuid.uuid4())

            # Prepare the payload
            payload = {
                'content': chunk['content'],
                'source_url': chunk['metadata']['url'],
                'page_title': chunk['metadata'].get('page_title', ''),
                'headings': chunk['metadata'].get('headings', []),
                'word_count': chunk['metadata'].get('word_count', 0),
                'chunk_index': chunk['chunk_index']
            }

            # Save to Qdrant
            client.upsert(
                collection_name=collection_name,
                points=[
                    models.PointStruct(
                        id=point_id,
                        vector=vector,
                        payload=payload
                    )
                ]
            )

            logger.info(f"Saved chunk to Qdrant with ID: {point_id}")
            return True
        except Exception as e:
            retry_count += 1
            logger.warning(f"Attempt {retry_count} failed for saving to Qdrant: {e}")
            if retry_count < max_retries:
                time.sleep(2 ** retry_count)  # Exponential backoff
            else:
                logger.error(f"Error saving chunk to Qdrant after {max_retries} attempts: {e}")
                return False


def verify_storage(collection_name: str = 'rag_embeddings') -> bool:
    """
    Verify storage by performing sample similarity search queries.

    Args:
        collection_name (str): Name of the collection to verify

    Returns:
        bool: True if verification successful, False otherwise
    """
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")

    if not qdrant_url or not qdrant_api_key:
        raise ValueError("QDRANT_URL and QDRANT_API_KEY environment variables must be set")

    client = QdrantClient(
        url=qdrant_url,
        api_key=qdrant_api_key,
        prefer_grpc=False
    )

    # Retry logic for Qdrant operations
    max_retries = 3
    retry_count = 0
    while retry_count < max_retries:
        try:
            # Get the total count of points in the collection
            count = client.count(collection_name=collection_name)
            logger.info(f"Total points in collection '{collection_name}': {count.count}")

            if count.count == 0:
                logger.warning("No points found in the collection")
                return False

            # Perform a sample search with a test query
            # First, we need to embed a test query
            test_query = "Physical AI and Humanoid Robotics"

            # Use the same embed function we defined above
            query_vector = embed(test_query)

            # Search for similar content
            search_results = client.search(
                collection_name=collection_name,
                query_vector=query_vector,
                limit=3  # Get top 3 results
            )

            logger.info(f"Sample search results for '{test_query}':")
            for i, result in enumerate(search_results):
                logger.info(f"  {i+1}. Score: {result.score:.3f}, Content preview: {result.payload['content'][:100]}...")

            return True
        except Exception as e:
            retry_count += 1
            logger.warning(f"Attempt {retry_count} failed for verifying storage: {e}")
            if retry_count < max_retries:
                time.sleep(2 ** retry_count)  # Exponential backoff
            else:
                logger.error(f"Error verifying storage after {max_retries} attempts: {e}")
                return False


def main():
    """
    Main function to execute the complete RAG system pipeline.
    """
    logger.info("Starting RAG System Pipeline for Physical AI & Humanoid Robotics Textbook")

    # Book URL to process
    book_url = "https://ahzazahmed.github.io/Physical-AI-Humanoid-Robotics/"

    try:
        # Step 1: Get all URLs from the book
        logger.info("Step 1: Discovering all URLs from the textbook...")
        urls = get_all_urls(book_url)
        logger.info(f"Found {len(urls)} URLs to process")

        if not urls:
            logger.error("No URLs found. Please check the base URL and network connection.")
            return

        # Step 2: Extract text content from all URLs
        logger.info("Step 2: Extracting text content from all URLs...")
        contents = extract_text_from_urls(urls)
        logger.info(f"Extracted content from {len(contents)} pages")

        if not contents:
            logger.error("No content extracted. Please check the URLs and network connection.")
            return

        # Step 3: Create Qdrant collection
        logger.info("Step 3: Creating Qdrant collection...")
        if not create_collection('rag_embeddings'):
            logger.error("Failed to create Qdrant collection")
            return
        logger.info("Qdrant collection created successfully")

        # Step 4: Process each content chunk and store in Qdrant
        logger.info("Step 4: Processing content chunks and storing in Qdrant...")
        total_chunks = 0
        successful_saves = 0

        for content_data in contents:
            # Chunk the content
            chunks = chunk_text(content_data['content'], content_data, chunk_size=1000)
            total_chunks += len(chunks)

            for chunk in chunks:
                try:
                    # Generate embedding for the chunk
                    vector = embed(chunk['content'])

                    # Save to Qdrant
                    if save_chunk_to_qdrant(chunk, vector):
                        successful_saves += 1
                except Exception as e:
                    logger.error(f"Error processing chunk: {e}")
                    continue

        logger.info(f"Processed {total_chunks} chunks, successfully saved {successful_saves} to Qdrant")

        # Step 5: Verify storage with sample searches
        logger.info("Step 5: Verifying storage with sample searches...")
        if verify_storage('rag_embeddings'):
            logger.info("Verification successful! RAG system is ready.")
        else:
            logger.warning("Verification had issues, but embedding process completed.")

        logger.info("RAG System Pipeline completed successfully!")

    except Exception as e:
        logger.error(f"Pipeline failed with error: {e}")
        raise


if __name__ == "__main__":
    main()