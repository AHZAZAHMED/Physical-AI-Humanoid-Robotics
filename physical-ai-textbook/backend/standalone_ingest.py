#!/usr/bin/env python3
"""
Standalone Content Ingestion Script for Physical AI & Humanoid Robotics Textbook

This script reads all markdown files from the docs directory and indexes them
directly into the Qdrant vector database.
"""

import os
import glob
from pathlib import Path
import re
import logging
from dotenv import load_dotenv
from sentence_transformers import SentenceTransformer
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def extract_title_from_md(file_path):
    """Extract the title from the markdown file's first heading"""
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Look for the first heading that might contain the title
    match = re.search(r'^#\s+(.+)$|^title:\s*[\'"]?([^\'"\n]+)[\'"]?', content, re.MULTILINE)
    if match:
        return match.group(1) or match.group(2)
    else:
        # If no title found in the content, use the filename
        return Path(file_path).stem.replace('-', ' ').replace('_', ' ').title()

def chunk_text(text, chunk_size=500, overlap=50):
    """Split text into overlapping chunks"""
    words = text.split()
    chunks = []

    for i in range(0, len(words), chunk_size - overlap):
        chunk = " ".join(words[i:i + chunk_size])
        if chunk.strip():
            chunks.append(chunk)

    return chunks

def read_markdown_files(docs_path):
    """Read all markdown files from the docs directory"""
    md_files = glob.glob(os.path.join(docs_path, "**/*.md"), recursive=True)
    content_chunks = []

    for file_path in md_files:
        logger.info(f"Processing file: {file_path}")

        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        # Extract title from the markdown file
        title = extract_title_from_md(file_path)

        # Remove frontmatter if present
        content = re.sub(r'^---\n.*?\n---\n', '', content, flags=re.DOTALL)

        # Create chunks of the content
        text_chunks = chunk_text(content)

        for i, chunk in enumerate(text_chunks):
            content_chunks.append({
                "text": chunk,
                "metadata": {
                    "source_file": os.path.relpath(file_path, docs_path),
                    "title": title,
                    "chunk_id": i,
                    "full_path": file_path
                }
            })

    return content_chunks

def index_content_directly(content_chunks):
    """Index content directly to Qdrant"""
    # Initialize Qdrant client
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")

    if qdrant_url:
        # Use cloud instance
        client = QdrantClient(
            url=qdrant_url,
            api_key=qdrant_api_key,
        )
    else:
        # Use local instance
        client = QdrantClient(host="localhost", port=6333)

    # Initialize the embedding model
    logger.info("Loading embedding model...")
    model = SentenceTransformer('intfloat/e5-base-v2')
    logger.info("Embedding model loaded successfully")

    # Collection name for textbook content
    COLLECTION_NAME = "textbook_content"

    # Create collection if it doesn't exist
    try:
        client.get_collection(COLLECTION_NAME)
        logger.info(f"Collection '{COLLECTION_NAME}' already exists")
    except:
        logger.info(f"Creating collection '{COLLECTION_NAME}'")
        client.create_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=VectorParams(size=768, distance=Distance.COSINE),  # e5-base-v2 produces 768-dim vectors
        )
        logger.info(f"Collection '{COLLECTION_NAME}' created successfully")

    # Prepare points for insertion
    points = []
    point_id = 0

    for chunk_data in content_chunks:
        # Split the content into smaller chunks if needed
        text_chunks = chunk_text(chunk_data["text"])

        for text_chunk in text_chunks:
            # Prefix the text with "passage: " for the e5-base-v2 model
            prefixed_text = f"passage: {text_chunk}"

            # Generate embedding
            embedding = model.encode([prefixed_text])[0].tolist()

            # Create a point for Qdrant
            point = models.PointStruct(
                id=point_id,
                vector=embedding,
                payload={
                    "text": text_chunk,
                    "metadata": chunk_data["metadata"]
                }
            )
            points.append(point)
            point_id += 1

    # Upload points to Qdrant
    client.upsert(
        collection_name=COLLECTION_NAME,
        points=points
    )

    logger.info(f"Successfully indexed {len(points)} content chunks into {COLLECTION_NAME}")
    return {
        "message": f"Successfully indexed {len(points)} content chunks",
        "indexed_count": len(points),
        "collection_name": COLLECTION_NAME
    }

def main():
    # Define the docs path relative to this script
    docs_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "docs")

    if not os.path.exists(docs_path):
        logger.error(f"Docs directory not found at {docs_path}")
        return

    logger.info(f"Reading markdown files from {docs_path}")

    # Read and process all markdown files
    content_chunks = read_markdown_files(docs_path)

    logger.info(f"Total content chunks created: {len(content_chunks)}")

    if not content_chunks:
        logger.warning("No content chunks were created. Check if the docs directory contains markdown files.")
        return

    # Index the content directly to Qdrant
    result = index_content_directly(content_chunks)

    if result:
        logger.info("Content indexing completed successfully!")
        logger.info(f"Result: {result}")
    else:
        logger.error("Content indexing failed!")

if __name__ == "__main__":
    main()