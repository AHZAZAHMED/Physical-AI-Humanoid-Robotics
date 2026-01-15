#!/usr/bin/env python3
"""
Simple test script to test Qdrant retrieval functionality for RAG chatbot.
This script tests the connection to Qdrant and performs a basic retrieval test.
"""

import os
import sys
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Add the src directory to the Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

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


def test_retrieval(client):
    """Test basic retrieval functionality."""
    print("\nTesting retrieval functionality...")

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

        # Try to get a count of points
        count = client.count(collection_name=collection_name)
        print(f"   Total points in collection: {count.count}")

        # Test embedding generation (requires Cohere)
        try:
            import cohere
            cohere_api_key = os.getenv("COHERE_API_KEY")

            if not cohere_api_key:
                print("⚠️  COHERE_API_KEY not set, cannot test full retrieval pipeline")
                print("   Testing basic Qdrant functionality only...")

                # Just test basic search without embeddings
                # Get a sample of points to verify the collection has data
                sample_points = client.scroll(
                    collection_name=collection_name,
                    limit=1
                )

                if sample_points[0]:  # If there are points
                    point = sample_points[0][0]  # Get first point
                    print(f"✅ Retrieved sample point with ID: {point.id}")
                    print(f"   Sample payload keys: {list(point.payload.keys())}")
                    return True
                else:
                    print("❌ No points found in collection")
                    return False
            else:
                # Full test with embeddings
                co = cohere.Client(cohere_api_key)

                # Test query
                test_query = "Physical AI and Humanoid Robotics"
                print(f"   Generating embedding for test query: '{test_query}'")

                response = co.embed(
                    texts=[test_query],
                    model="multilingual-22-12"
                )
                query_vector = response.embeddings[0]

                print(f"   Query vector length: {len(query_vector)}")

                # Perform similarity search
                search_results = client.search(
                    collection_name=collection_name,
                    query_vector=query_vector,
                    limit=3  # Get top 3 results
                )

                print(f"✅ Successfully performed similarity search")
                print(f"   Retrieved {len(search_results)} results")

                # Display results
                for i, result in enumerate(search_results):
                    print(f"   Result {i+1}:")
                    print(f"     Score: {result.score:.3f}")
                    print(f"     Content preview: {result.payload['content'][:100]}...")
                    print(f"     Source URL: {result.payload['source_url']}")

                return True

        except ImportError:
            print("⚠️  Cohere not installed. Install with: pip install cohere")
            print("   Testing basic Qdrant functionality only...")

            # Just test basic search without embeddings
            sample_points = client.scroll(
                collection_name=collection_name,
                limit=1
            )

            if sample_points[0]:  # If there are points
                point = sample_points[0][0]  # Get first point
                print(f"✅ Retrieved sample point with ID: {point.id}")
                print(f"   Sample payload keys: {list(point.payload.keys())}")
                return True
            else:
                print("❌ No points found in collection")
                return False
        except Exception as e:
            print(f"❌ Error during retrieval test: {e}")
            return False

    except Exception as e:
        print(f"❌ Error testing retrieval: {e}")
        return False


def main():
    """Main function to run the Qdrant retrieval test."""
    print("Qdrant Retrieval Test for RAG Chatbot")
    print("=" * 50)

    # Test Qdrant connection
    client = test_qdrant_connection()
    if not client:
        return False

    # Test retrieval functionality
    success = test_retrieval(client)

    print("\n" + "=" * 50)
    if success:
        print("✅ Qdrant retrieval test completed successfully!")
        print("The RAG system is properly configured and can retrieve data from Qdrant.")
    else:
        print("❌ Qdrant retrieval test failed!")
        print("Please check your configuration and data.")

    return success


if __name__ == "__main__":
    main()