"""
Qdrant ingestion script for Physical AI & Humanoid Robotics Textbook content
"""
import os
import glob
from pathlib import Path
import re
from typing import List, Dict, Any
from sentence_transformers import SentenceTransformer
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams
import logging
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class ContentIngestor:
    def __init__(self):
        # Initialize the embedding model
        logger.info("Loading embedding model...")
        self.model = SentenceTransformer('intfloat/e5-base-v2')
        logger.info("Embedding model loaded successfully")

        # Initialize Qdrant client
        qdrant_url = os.getenv("QDRANT_URL")
        qdrant_api_key = os.getenv("QDRANT_API_KEY")

        if qdrant_url:
            # Use cloud instance
            self.client = QdrantClient(
                url=qdrant_url,
                api_key=qdrant_api_key,
            )
        else:
            # Use local instance
            self.client = QdrantClient(host="localhost", port=6333)

        logger.info("Qdrant client loaded successfully")

        # Collection name for textbook content
        self.COLLECTION_NAME = "textbook_content"

        # Create collection if it doesn't exist
        try:
            self.client.get_collection(self.COLLECTION_NAME)
            logger.info(f"Collection '{self.COLLECTION_NAME}' already exists")
        except:
            logger.info(f"Creating collection '{self.COLLECTION_NAME}'")
            self.client.create_collection(
                collection_name=self.COLLECTION_NAME,
                vectors_config=VectorParams(size=768, distance=Distance.COSINE),  # e5-base-v2 produces 768-dim vectors
            )
            logger.info(f"Collection '{self.COLLECTION_NAME}' created successfully")

    def extract_title_from_md(self, file_path: str) -> str:
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

    def chunk_text(self, text: str, chunk_size: int = 500, overlap: int = 50) -> List[str]:
        """Split text into overlapping chunks"""
        words = text.split()
        chunks = []

        for i in range(0, len(words), chunk_size - overlap):
            chunk = " ".join(words[i:i + chunk_size])
            if chunk.strip():
                chunks.append(chunk)

        return chunks

    def read_markdown_files(self, docs_path: str) -> List[Dict[str, Any]]:
        """Read all markdown files from the docs directory"""
        md_files = glob.glob(os.path.join(docs_path, "**/*.md"), recursive=True)
        content_chunks = []

        for file_path in md_files:
            logger.info(f"Processing file: {file_path}")

            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()

            # Extract title from the markdown file
            title = self.extract_title_from_md(file_path)

            # Remove frontmatter if present
            content = re.sub(r'^---\n.*?\n---\n', '', content, flags=re.DOTALL)

            # Create chunks of the content
            text_chunks = self.chunk_text(content)

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

    def index_content(self, content_chunks: List[Dict[str, Any]]) -> Dict[str, Any]:
        """Index content chunks into Qdrant vector database"""
        points = []
        point_id = 0

        for chunk_data in content_chunks:
            # Prefix the text with "passage: " for the e5-base-v2 model
            prefixed_text = f"passage: {chunk_data['text']}"

            # Generate embedding
            embedding = self.model.encode([prefixed_text])[0].tolist()

            point = models.PointStruct(
                id=point_id,
                vector=embedding,
                payload={
                    "text": chunk_data['text'],
                    "metadata": chunk_data['metadata']
                }
            )
            points.append(point)
            point_id += 1

        # Upload points to Qdrant
        self.client.upsert(
            collection_name=self.COLLECTION_NAME,
            points=points
        )

        logger.info(f"Indexed {len(points)} content chunks into {self.COLLECTION_NAME}")

        return {
            "message": f"Successfully indexed {len(points)} content chunks",
            "indexed_count": len(points),
            "collection_name": self.COLLECTION_NAME
        }

    def ingest_from_docs(self, docs_path: str) -> Dict[str, Any]:
        """Main method to ingest content from docs directory"""
        logger.info(f"Reading markdown files from {docs_path}")

        # Read and process all markdown files
        content_chunks = self.read_markdown_files(docs_path)

        logger.info(f"Total content chunks created: {len(content_chunks)}")

        if not content_chunks:
            logger.warning("No content chunks were created. Check if the docs directory contains markdown files.")
            return {"error": "No content chunks were created"}

        # Index the content to Qdrant
        result = self.index_content(content_chunks)

        logger.info("Content indexing completed successfully!")
        return result

def main():
    """Main function to run the ingestion process"""
    # Define the docs path relative to the project
    project_root = Path(__file__).parent.parent.parent.parent  # Go up to project root
    docs_path = project_root / "docs"

    if not docs_path.exists():
        logger.error(f"Docs directory not found at {docs_path}")
        return

    # Create content ingestor and run ingestion
    ingestor = ContentIngestor()
    result = ingestor.ingest_from_docs(str(docs_path))

    print(result)

if __name__ == "__main__":
    main()