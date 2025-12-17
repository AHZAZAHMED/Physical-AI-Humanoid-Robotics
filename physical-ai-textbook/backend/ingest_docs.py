#!/usr/bin/env python3
"""
Content Ingestion Script for Physical AI & Humanoid Robotics Textbook

This script reads all markdown files from the docs directory and indexes them
into the Qdrant vector database via the RAG service API.
"""

import os
import requests
import glob
from pathlib import Path
import re
import logging

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

def index_content_to_rag(content_chunks, rag_api_url="http://localhost:8000"):
    """Send content chunks to the RAG service for indexing"""
    url = f"{rag_api_url}/index_content"

    # Prepare the payload
    payload = {
        "content_chunks": content_chunks
    }

    logger.info(f"Sending {len(content_chunks)} content chunks to RAG service at {url}")

    try:
        response = requests.post(url, json=payload)
        response.raise_for_status()

        result = response.json()
        logger.info(f"Successfully indexed content: {result}")
        return result
    except requests.exceptions.RequestException as e:
        logger.error(f"Error indexing content: {e}")
        if hasattr(e, 'response') and e.response is not None:
            logger.error(f"Response: {e.response.text}")
        return None

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

    # Index the content to the RAG service
    result = index_content_to_rag(content_chunks)

    if result:
        logger.info("Content indexing completed successfully!")
    else:
        logger.error("Content indexing failed!")

if __name__ == "__main__":
    main()