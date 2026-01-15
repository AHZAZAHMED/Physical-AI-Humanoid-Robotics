#!/usr/bin/env python3
"""
Test the RAG agent with a specific example query about Physical AI.
"""

import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '.'))

from src.rag_agent.agent import RAGAgent
from src.rag_agent.config import AgentConfiguration

def test_specific_query():
    """Test the RAG agent with a specific example query."""
    print("Initializing RAG Agent...")

    try:
        # Create configuration
        config = AgentConfiguration()

        # Create the RAG agent
        agent = RAGAgent(config)

        print("RAG Agent initialized successfully!")

        # Specific example query
        test_query = "Explain the role of control theory in Physical AI and humanoid robotics."

        print(f"\nProcessing query: '{test_query}'")

        # Process the query
        response = agent.process_query(test_query)

        print("\n=== RAG Response ===")
        print(f"Response: {response.text}")
        print(f"Sources: {len(response.sources)} sources found")
        print(f"Grounding Confidence: {response.grounding_confidence}")
        print(f"Response Time: {response.response_time_ms}ms")

        if response.sources:
            print("\nTop sources:")
            for i, source in enumerate(response.sources[:3]):  # Show top 3 sources
                print(f"  {i+1}. Book: {source['source']['book']}")
                print(f"     Chapter: {source['source']['chapter']}")
                print(f"     Section: {source['source']['section']}")
                print(f"     Similarity: {source['similarity_score']:.3f}")
                print(f"     Content preview: {source['content'][:100]}...")
                print()
        else:
            print("No sources found")

    except Exception as e:
        print(f"Error during RAG pipeline test: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_specific_query()