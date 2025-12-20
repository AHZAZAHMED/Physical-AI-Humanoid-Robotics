"""
Main FastAPI application for the Physical AI Textbook Platform
"""
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from slowapi import Limiter, _rate_limit_exceeded_handler
from slowapi.util import get_remote_address
from slowapi.errors import RateLimitExceeded
from src.api import auth, chat, health
from src.middleware.auth import AuthMiddleware
from src.db.connection import init_db
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Initialize rate limiter
limiter = Limiter(key_func=get_remote_address)
app = FastAPI(
    title="Physical AI Textbook Platform API",
    description="Backend API for the Physical AI & Humanoid Robotics Textbook Platform",
    version="1.0.0"
)

# Set up rate limit handler
app.state.limiter = limiter
app.add_exception_handler(RateLimitExceeded, _rate_limit_exceeded_handler)

# Initialize database connection
init_db()

# Configure allowed origins
# In production, replace with specific frontend URL
allowed_origins = os.getenv("ALLOWED_ORIGINS", "*").split(",")
if "*" in allowed_origins:
    # If wildcard is used, we allow credentials but this should be changed in production
    allow_credentials = True
    allow_origins = ["*"]
else:
    allow_credentials = True
    allow_origins = [origin.strip() for origin in allowed_origins]

# Add CORS middleware to allow frontend communication
app.add_middleware(
    CORSMiddleware,
    allow_origins=allow_origins,
    allow_credentials=allow_credentials,
    allow_methods=["GET", "POST", "PUT", "DELETE", "OPTIONS", "PATCH"],
    allow_headers=["*"],
)

# Add authentication middleware
app.add_middleware(AuthMiddleware)

# Include API routes
app.include_router(auth.router, prefix="/auth", tags=["authentication"])
app.include_router(chat.router, prefix="/chat", tags=["chat"])
app.include_router(health.router, prefix="/health", tags=["health"])

@app.get("/")
def read_root():
    return {"message": "Physical AI Textbook Platform API"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)