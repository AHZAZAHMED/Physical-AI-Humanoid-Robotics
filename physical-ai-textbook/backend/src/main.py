"""
Main FastAPI application for the Physical AI Textbook Platform
"""
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from src.api import auth, chat, health
from src.middleware.auth import AuthMiddleware
from src.db.connection import init_db
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

app = FastAPI(
    title="Physical AI Textbook Platform API",
    description="Backend API for the Physical AI & Humanoid Robotics Textbook Platform",
    version="1.0.0"
)

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