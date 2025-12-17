"""
Database connection management for the Physical AI Textbook Platform
"""
from sqlalchemy import create_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from sqlalchemy.pool import QueuePool
import os
from dotenv import load_dotenv
import logging

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Database URL from environment
DATABASE_URL = os.getenv("NEON_DATABASE_URL")

if not DATABASE_URL:
    logger.warning("NEON_DATABASE_URL not found in environment. Using fallback.")
    # Fallback to a local SQLite for development if no database URL is provided
    DATABASE_URL = "sqlite:///./physical_ai_textbook.db"

# Create the database engine with connection pooling
engine = create_engine(
    DATABASE_URL,
    poolclass=QueuePool,
    pool_size=10,  # Number of connections to maintain in the pool
    max_overflow=20,  # Number of connections that can be created beyond pool_size
    pool_pre_ping=True,  # Verify connections before use
    pool_recycle=300,  # Recycle connections after 5 minutes
    echo=False  # Set to True for SQL query logging
)

# Create session factory
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

# Base class for models
Base = declarative_base()

def init_db():
    """
    Initialize the database by creating all tables
    """
    logger.info("Initializing database connection...")
    try:
        # Import all models to ensure they are registered with Base.metadata
        from src.auth.models_db import User, Session  # Import the actual DB models

        # Create all tables
        Base.metadata.create_all(bind=engine)
        logger.info("Database tables created successfully")
    except Exception as e:
        logger.error(f"Error initializing database: {str(e)}")
        raise

def get_db():
    """
    Dependency function to get database session
    """
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()

# Test connection function
def test_connection():
    """
    Test the database connection
    """
    try:
        with engine.connect() as connection:
            result = connection.execute("SELECT 1")
            logger.info("Database connection test successful")
            return True
    except Exception as e:
        logger.error(f"Database connection test failed: {str(e)}")
        return False

if __name__ == "__main__":
    # Test the connection when running this module directly
    test_connection()