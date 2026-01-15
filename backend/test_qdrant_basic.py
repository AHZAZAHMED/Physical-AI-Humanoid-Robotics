#!/usr/bin/env python3
"""
Basic test script to test Qdrant connection and basic retrieval functionality for RAG chatbot.
This script tests the connection to Qdrant and performs basic retrieval without requiring Cohere API.
"""

import os
import sys
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Add the src directory to the Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend/src'))

def test_qdrant_connection():
    """Test basic Qdrant connection."""
    print("Testing Qdrant connection...")

    try:
        from qdrant_client import QdrantClient

        # Get environment variables
        qdrant_url = os.getenv("QDRANT_URL")
        qdrant_api_key = os.getenv("QDRANT_API_KEY")

        if not qdrant_url or not qdrant_api_key:
            print("❌ QDRANT_URL and QDRANT_API_KEY environment variables are required")
            return False

        # Initialize Qdrant client
        client = QdrantClient(
            url=qdrant_url,
            api_key=qdrant_api_key,
            prefer_grpc=False
        )

        # Test connection by getting collections
        collections = client.get_collections()
        print(f"✅ Successfully connected to Qdrant")
        print(f"   Available collections: {[col.name for col in collections.collections]}")

        return client

    except ImportError:
        print("❌ qdrant-client not installed. Install with: pip install qdrant-client")
        return False
    except Exception as e:
        print(f"❌ Failed to connect to Qdrant: {e}")
        return False


def test_basic_retrieval(client):
    """Test basic retrieval functionality without embeddings."""
    print("\nTesting basic retrieval functionality...")

    try:
        # Check if the rag_embeddings collection exists
        collection_name = 'rag_embeddings'
        try:
            collection_info = client.get_collection(collection_name)
            print(f"✅ Collection '{collection_name}' exists")
            print(f"   Points count: {collection_info.points_count}")
            print(f"   Vector size: {collection_info.config.params.vectors.size}")
        except Exception as e:
            print(f"❌ Collection '{collection_name}' does not exist: {e}")
            return False

        # If collection is empty, we can't test retrieval properly
        if collection_info.points_count == 0:
            print(f"⚠️  Collection is empty, cannot test retrieval")
            return True  # Connection is working, but no data to retrieve

        # Test basic operations without embeddings
        print(f"   Total points in collection: {collection_info.points_count}")

        # Get a sample of points to verify the collection has data
        sample_points = client.scroll(
            collection_name=collection_name,
            limit=3  # Get 3 sample points
        )

        if sample_points[0]:  # If there are points
            points = sample_points[0]
            print(f"✅ Retrieved {len(points)} sample points")

            for i, point in enumerate(points):
                print(f"   Point {i+1}:")
                print(f"     ID: {point.id}")
                print(f"     Content preview: {point.payload['content'][:100] if 'content' in point.payload else 'N/A'}...")
                print(f"     Source URL: {point.payload.get('source_url', 'N/A')}")
                print(f"     Page title: {point.payload.get('page_title', 'N/A')}")
            return True
        else:
            print("❌ No points found in collection")
            return False

    except Exception as e:
        print(f"❌ Error testing basic retrieval: {e}")
        return False


def test_count_operation(client):
    """Test count operation."""
    print("\nTesting count operation...")

    try:
        collection_name = 'rag_embeddings'
        count = client.count(collection_name=collection_name)
        print(f"✅ Count operation successful: {count.count} points in collection")
        return True
    except Exception as e:
        print(f"❌ Count operation failed: {e}")
        return False


def main():
    """Main function to run the Qdrant basic retrieval test."""
    print("Basic Qdrant Retrieval Test for RAG Chatbot")
    print("=" * 50)

    # Test Qdrant connection
    client = test_qdrant_connection()
    if not client:
        return False

    # Test basic retrieval functionality
    retrieval_success = test_basic_retrieval(client)

    # Test count operation
    count_success = test_count_operation(client)

    print("\n" + "=" * 50)
    if retrieval_success and count_success:
        print("✅ Basic Qdrant retrieval test completed successfully!")
        print("The RAG system's Qdrant database is properly configured and contains data.")
        print("Note: Full semantic search requires embeddings which need the Cohere API.")
    else:
        print("❌ Basic Qdrant retrieval test failed!")
        print("Please check your Qdrant configuration and data.")

    return retrieval_success and count_success


if __name__ == "__main__":
    main()