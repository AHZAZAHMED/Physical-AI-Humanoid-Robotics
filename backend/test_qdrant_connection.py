#!/usr/bin/env python3
"""
Test direct connection to Qdrant cloud instance.
"""

import os
from qdrant_client import QdrantClient
from qdrant_client.http import models
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Get configuration from environment
qdrant_url = os.getenv("QDRANT_URL")
qdrant_api_key = os.getenv("QDRANT_API_KEY")

print(f"QDRANT_URL: {qdrant_url}")
print(f"API Key present: {qdrant_api_key is not None}")

if not qdrant_url or not qdrant_api_key:
    print("Missing QDRANT_URL or QDRANT_API_KEY in environment")
    exit(1)

try:
    print("\nAttempting to connect to Qdrant cloud instance...")

    # Create client with explicit HTTP settings for cloud
    client = QdrantClient(
        url=qdrant_url,
        api_key=qdrant_api_key,
        prefer_grpc=False,  # Force HTTP
        timeout=30
    )

    print("Client created successfully")

    # Try to get collection info
    collection_name = "rag_embeddings"
    print(f"Checking collection: {collection_name}")

    collection_info = client.get_collection(collection_name)
    print(f"Collection found: {collection_info}")
    print(f"Collection vectors count: {collection_info.points_count}")

    print("\nConnection successful!")

except Exception as e:
    print(f"Connection failed: {e}")
    import traceback
    traceback.print_exc()