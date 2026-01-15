"""
Main application entry point for the RAG agent system.
"""
from fastapi import FastAPI
from contextlib import asynccontextmanager
from fastapi.middleware.cors import CORSMiddleware
from fastapi.middleware.trustedhost import TrustedHostMiddleware
import os
from .api.query_endpoint import router as query_router
from .api.health_endpoint import router as health_router
from .api.stats_endpoint import router as stats_router
from .middleware.error_handler import add_error_handlers
from .utils.logging import setup_logging
from .config import AgentConfiguration


@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Application lifespan manager for startup and shutdown events.
    """
    # Startup
    print("Starting RAG Agent System...")

    # Setup logging
    setup_logging(level=os.getenv("LOG_LEVEL", "INFO"))

    # Validate configuration
    config = AgentConfiguration()
    try:
        config.validate()
        print("Configuration validated successfully")
    except Exception as e:
        print(f"Configuration error: {e}")
        raise

    yield

    # Shutdown
    print("Shutting down RAG Agent System...")


# Create FastAPI app with lifespan
app = FastAPI(
    title="Book Content RAG Agent API",
    description="API for retrieving book content and generating grounded, instructional responses",
    version="1.0.0",
    lifespan=lifespan,
    # Add additional security-related settings
    docs_url="/docs",  # Set to None in production to disable docs
    redoc_url="/redoc",  # Set to None in production to disable redoc
)

# Add security middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, specify exact origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
    # In production, specify exact headers and origins
)

# Add trusted host middleware to prevent HTTP Host Header attacks
app.add_middleware(
    TrustedHostMiddleware,
    allowed_hosts=["*"],  # In production, specify exact hosts
)

# Add error handlers
add_error_handlers(app)

# Include API routers
app.include_router(query_router)
app.include_router(health_router)
app.include_router(stats_router)

# Add middleware
from .middleware.auth import auth_middleware
from .middleware.rate_limit import rate_limit_middleware

# Note: In a real implementation, you'd add these as proper FastAPI middleware
# For this implementation, the authentication and rate limiting are handled
# within the individual endpoints


@app.get("/")
async def root():
    """
    Root endpoint for the API.
    """
    return {
        "message": "Welcome to the Book Content RAG Agent API",
        "version": "1.0.0",
        "endpoints": [
            "/api/v1/query - Process user queries through RAG pipeline",
            "/api/v1/health - Check service health",
            "/api/v1/stats - Get usage statistics",
            "/api/v1/config - Get/update configuration"
        ]
    }


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "backend.src.rag_agent.main:app",
        host="0.0.0.0",
        port=int(os.getenv("PORT", 8000)),
        reload=True
    )