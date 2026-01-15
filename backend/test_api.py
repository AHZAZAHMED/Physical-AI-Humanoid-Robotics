"""
Test script to verify the RAG Chatbot API functionality.
"""
import requests
import json

def test_api_endpoints():
    """Test the basic API functionality."""
    base_url = "http://localhost:8000"

    print("Testing API endpoints...")

    # Test health endpoint
    try:
        response = requests.get(f"{base_url}/api/health")
        if response.status_code == 200:
            print("✓ Health endpoint working")
            print(f"  Response: {response.json()}")
        else:
            print(f"✗ Health endpoint failed with status {response.status_code}")
    except Exception as e:
        print(f"✗ Health endpoint error: {e}")

    # Test root endpoint
    try:
        response = requests.get(f"{base_url}/")
        if response.status_code == 200:
            print("✓ Root endpoint working")
            print(f"  Response: {response.json()}")
        else:
            print(f"✗ Root endpoint failed with status {response.status_code}")
    except Exception as e:
        print(f"✗ Root endpoint error: {e}")

    # Test chat endpoint with a sample request
    try:
        sample_request = {
            "query": "What is Physical AI and Humanoid Robotics?",
            "sessionId": "test-session-123"
        }

        response = requests.post(
            f"{base_url}/api/chat",
            json=sample_request,
            headers={"Content-Type": "application/json"}
        )

        if response.status_code == 200:
            print("✓ Chat endpoint working")
            response_data = response.json()
            print(f"  Response keys: {list(response_data.keys())}")
            if "response" in response_data:
                print(f"  Response preview: {response_data['response'][:100]}...")
        else:
            print(f"✗ Chat endpoint failed with status {response.status_code}")
            print(f"  Error: {response.text}")
    except Exception as e:
        print(f"✗ Chat endpoint error: {e}")
        print("  (This may be expected if the backend service is not running)")


if __name__ == "__main__":
    test_api_endpoints()