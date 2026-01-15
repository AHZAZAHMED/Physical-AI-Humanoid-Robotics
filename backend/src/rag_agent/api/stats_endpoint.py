"""
API endpoint for retrieving usage statistics.
"""
from fastapi import APIRouter
from typing import Dict, Any
import time
from ..services.stats_service import get_stats_service


# Create API router
router = APIRouter()


@router.get("/api/v1/stats")
async def get_stats():
    """
    Retrieve usage statistics and performance metrics.
    """
    stats_service = get_stats_service()
    stats = stats_service.get_statistics()
    return stats


@router.get("/api/v1/stats/detailed")
async def get_detailed_stats():
    """
    Retrieve detailed usage statistics including recent query performance.
    """
    stats_service = get_stats_service()
    stats = stats_service.get_detailed_stats()
    return stats