#!/usr/bin/env python3
"""
Debug the Qdrant client configuration issue.
"""

import os
import sys
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Add the backend src to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from rag_agent.config import AgentConfiguration
from rag_agent.clients.qdrant_client import QdrantService

def debug_qdrant_client():
    print("Loading configuration...")
    config = AgentConfiguration()

    print(f"Qdrant Host: {config.qdrant_host}")
    print(f"Qdrant API Key present: {config.qdrant_api_key is not None}")
    print(f"Collection Name: {config.qdrant_collection_name}")

    print("\nCreating QdrantService...")
    try:
        qdrant_service = QdrantService(config)
        print("QdrantService created successfully")

        print("\nAttempting to access collection...")
        qdrant_service.initialize_collection()
        print("Collection access attempt completed")

    except Exception as e:
        print(f"Error creating/accessing QdrantService: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    debug_qdrant_client()