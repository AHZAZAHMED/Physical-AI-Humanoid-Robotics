"""
Integration tests for the API endpoints.
"""
import pytest
import asyncio
from fastapi.testclient import TestClient
from src.main import app
from unittest.mock import patch, MagicMock
from src.models.generated_response import GeneratedResponse
import json


@pytest.fixture
def client():
    """Create a test client for the API."""
    return TestClient(app)


class TestQueryEndpoint:
    """Test class for the query endpoint."""

    def test_query_endpoint_success(self, client):
        """Test successful query processing."""
        # Mock the agent to avoid actual API calls
        with patch('src.api.query_endpoint.RAGAgent') as mock_agent_class:
            mock_agent_instance = MagicMock()
            mock_agent_class.return_value = mock_agent_instance

            # Create a mock response
            mock_response = GeneratedResponse(
                id="response-123",
                query_id="query-123",
                text="This is a test response",
                sources=[{
                    "content": "Test source content",
                    "source": {"book": "Test Book", "chapter": "1", "section": "1.1"},
                    "similarity_score": 0.9,
                    "confidence_score": 0.85
                }],
                grounding_confidence=0.92,
                response_time_ms=150
            )
            mock_agent_instance.process_query.return_value = mock_response

            response = client.post(
                "/api/v1/query",
                json={
                    "query": "Test query",
                    "parameters": {
                        "top_k": 5,
                        "confidence_threshold": 0.7
                    }
                }
            )

            assert response.status_code == 200
            data = response.json()
            assert data["text"] == "This is a test response"
            assert data["grounding_confidence"] == 0.92

    def test_query_endpoint_missing_query(self, client):
        """Test query endpoint with missing query parameter."""
        response = client.post(
            "/api/v1/query",
            json={}
        )

        assert response.status_code == 422  # Validation error

    def test_query_endpoint_empty_query(self, client):
        """Test query endpoint with empty query."""
        response = client.post(
            "/api/v1/query",
            json={"query": ""}
        )

        assert response.status_code == 400  # Validation error

    def test_query_endpoint_invalid_parameters(self, client):
        """Test query endpoint with invalid parameters."""
        response = client.post(
            "/api/v1/query",
            json={
                "query": "Test query",
                "parameters": {
                    "top_k": -1,  # Invalid value
                    "confidence_threshold": 1.5  # Invalid value
                }
            }
        )

        # Status code might vary depending on validation implementation
        # If the validation happens in the agent, it might return 500 or 400
        assert response.status_code in [400, 422, 500]


class TestHealthEndpoint:
    """Test class for the health endpoint."""

    def test_health_endpoint_success(self, client):
        """Test successful health check."""
        with patch('src.api.health_endpoint.RAGAgent') as mock_agent_class:
            mock_agent_instance = MagicMock()
            mock_agent_class.return_value = mock_agent_instance

            # Mock the health check method
            mock_health_status = {
                "status": "healthy",
                "timestamp": 1234567890.0,
                "dependencies": {
                    "qdrant": {"status": "connected", "response_time_ms": 10},
                    "openai": {"status": "connected", "response_time_ms": 20}
                }
            }
            mock_agent_instance.health_check.return_value = mock_health_status

            response = client.get("/api/v1/health")

            assert response.status_code == 200
            data = response.json()
            assert data["status"] == "healthy"
            assert "dependencies" in data


class TestStatsEndpoint:
    """Test class for the stats endpoint."""

    def test_stats_endpoint_success(self, client):
        """Test successful stats retrieval."""
        with patch('src.api.stats_endpoint.get_stats_service') as mock_get_stats_service:
            mock_stats_service = MagicMock()
            mock_get_stats_service.return_value = mock_stats_service

            # Mock the stats service response
            mock_stats = {
                "total_queries": 100,
                "successful_responses": 95,
                "insufficient_context": 3,
                "avg_response_time_ms": 1200.0,
                "p95_response_time_ms": 1800,
                "grounding_accuracy": 0.92,
                "timestamp": 1234567890.0,
                "active_metrics_count": 50
            }
            mock_stats_service.get_statistics.return_value = mock_stats

            response = client.get("/api/v1/stats")

            assert response.status_code == 200
            data = response.json()
            assert data["total_queries"] == 100
            assert data["successful_responses"] == 95
            assert data["grounding_accuracy"] == 0.92

    def test_detailed_stats_endpoint_success(self, client):
        """Test successful detailed stats retrieval."""
        with patch('src.api.stats_endpoint.get_stats_service') as mock_get_stats_service:
            mock_stats_service = MagicMock()
            mock_get_stats_service.return_value = mock_stats_service

            # Mock the detailed stats service response
            mock_detailed_stats = {
                "total_queries": 100,
                "successful_responses": 95,
                "insufficient_context": 3,
                "avg_response_time_ms": 1200.0,
                "p95_response_time_ms": 1800,
                "grounding_accuracy": 0.92,
                "timestamp": 1234567890.0,
                "active_metrics_count": 50,
                "recent_queries": [
                    {
                        "query_id": "query-1",
                        "response_time_ms": 1500,
                        "grounding_confidence": 0.85,
                        "top_k_used": 5,
                        "confidence_threshold": 0.7,
                        "retrieved_chunks_count": 3,
                        "timestamp": 1234567889.0,
                        "success": True
                    }
                ]
            }
            mock_stats_service.get_detailed_stats.return_value = mock_detailed_stats

            response = client.get("/api/v1/stats/detailed")

            assert response.status_code == 200
            data = response.json()
            assert data["total_queries"] == 100
            assert "recent_queries" in data
            assert len(data["recent_queries"]) == 1


class TestRootEndpoint:
    """Test class for the root endpoint."""

    def test_root_endpoint_success(self, client):
        """Test the root endpoint."""
        response = client.get("/")

        assert response.status_code == 200
        data = response.json()
        assert "message" in data
        assert "Book Content RAG Agent API" in data["message"]
        assert "endpoints" in data
        assert isinstance(data["endpoints"], list)
        assert len(data["endpoints"]) > 0


class TestAPIIntegration:
    """Test class for API integration scenarios."""

    def test_full_query_workflow(self, client):
        """Test a full query workflow with mocked services."""
        with patch('src.api.query_endpoint.RAGAgent') as mock_agent_class:
            mock_agent_instance = MagicMock()
            mock_agent_class.return_value = mock_agent_instance

            # Mock a successful response
            mock_response = GeneratedResponse(
                id="response-456",
                query_id="query-456",
                text="Detailed response based on book content",
                sources=[
                    {
                        "content": "First source content preview...",
                        "source": {"book": "Robotics Fundamentals", "chapter": "3", "section": "3.2", "page": 45},
                        "similarity_score": 0.92,
                        "confidence_score": 0.88
                    },
                    {
                        "content": "Second source content preview...",
                        "source": {"book": "Robotics Fundamentals", "chapter": "4", "section": "4.1", "page": 67},
                        "similarity_score": 0.87,
                        "confidence_score": 0.82
                    }
                ],
                grounding_confidence=0.91,
                response_time_ms=1250
            )
            mock_agent_instance.process_query.return_value = mock_response

            # Make a query request
            query_data = {
                "query": "Explain the concept of forward kinematics in robotics",
                "parameters": {
                    "top_k": 5,
                    "confidence_threshold": 0.7
                }
            }
            response = client.post("/api/v1/query", json=query_data)

            # Verify the response
            assert response.status_code == 200
            data = response.json()
            assert data["text"] == "Detailed response based on book content"
            assert data["grounding_confidence"] == 0.91
            assert len(data["sources"]) == 2
            assert data["sources"][0]["similarity_score"] == 0.92
            assert data["sources"][1]["source"]["chapter"] == "4"

    def test_multiple_api_endpoints_consistency(self, client):
        """Test consistency across multiple API endpoints."""
        # Test health endpoint
        with patch('src.api.health_endpoint.RAGAgent') as mock_agent_health:
            mock_agent_health_instance = MagicMock()
            mock_agent_health.return_value = mock_agent_health_instance
            mock_agent_health_instance.health_check.return_value = {
                "status": "healthy",
                "timestamp": 1234567890.0,
                "dependencies": {"qdrant": {"status": "connected"}, "openai": {"status": "connected"}}
            }

            health_response = client.get("/api/v1/health")
            assert health_response.status_code == 200

        # Test stats endpoint
        with patch('src.api.stats_endpoint.get_stats_service') as mock_get_stats:
            mock_stats_service = MagicMock()
            mock_get_stats.return_value = mock_stats_service
            mock_stats_service.get_statistics.return_value = {
                "total_queries": 0,
                "successful_responses": 0,
                "insufficient_context": 0,
                "avg_response_time_ms": 0.0,
                "p95_response_time_ms": 0,
                "grounding_accuracy": 0.0,
                "timestamp": 1234567890.0,
                "active_metrics_count": 0
            }

            stats_response = client.get("/api/v1/stats")
            assert stats_response.status_code == 200

        # Test root endpoint
        root_response = client.get("/")
        assert root_response.status_code == 200

        # All endpoints should return successfully
        assert all([
            health_response.status_code == 200,
            stats_response.status_code == 200,
            root_response.status_code == 200
        ])