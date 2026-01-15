"""
API endpoint for health checks.
"""
from fastapi import APIRouter
from typing import Dict, Any
from ..agent import RAGAgent
from ..config import AgentConfiguration


# Create API router
router = APIRouter()


@router.get("/api/v1/health")
async def health_check():
    """
    Check the health status of the service and its dependencies.
    """
    # Create a temporary agent to perform health check
    # In a production system, you'd want to use a shared instance
    config = AgentConfiguration()
    agent = RAGAgent(config)

    health_status = agent.health_check()

    return health_status