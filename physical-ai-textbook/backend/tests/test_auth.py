"""
API tests for authentication endpoints in the Physical AI Textbook Platform
"""
import pytest
from fastapi.testclient import TestClient
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from sqlalchemy.pool import StaticPool
import sys
import os

# Add the src directory to the path so we can import our modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from src.main import app
from src.db.connection import Base, get_db
from src.auth.models_db import User

# Use an in-memory SQLite database for testing
SQLALCHEMY_DATABASE_URL = "sqlite:///:memory:"

engine = create_engine(
    SQLALCHEMY_DATABASE_URL,
    connect_args={"check_same_thread": False},
    poolclass=StaticPool,
)
TestingSessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

# Create the database tables
Base.metadata.create_all(bind=engine)

# Override the get_db dependency
def override_get_db():
    try:
        db = TestingSessionLocal()
        yield db
    finally:
        db.close()

app.dependency_overrides[get_db] = override_get_db

client = TestClient(app)

def test_register_user():
    """
    Test user registration endpoint
    """
    response = client.post(
        "/auth/register",
        json={
            "email": "test@example.com",
            "password": "testpassword123",
            "background_info": "Software developer interested in robotics",
            "software_expertise": "Python, JavaScript",
            "hardware_expertise": "Basic electronics"
        }
    )
    assert response.status_code == 200
    data = response.json()
    assert data["email"] == "test@example.com"
    assert "id" in data
    assert data["background_info"] == "Software developer interested in robotics"


def test_register_user_invalid_email():
    """
    Test user registration with invalid email format
    """
    response = client.post(
        "/auth/register",
        json={
            "email": "invalid-email",
            "password": "testpassword123",
            "background_info": "Software developer interested in robotics"
        }
    )
    assert response.status_code == 400


def test_register_user_weak_password():
    """
    Test user registration with weak password
    """
    response = client.post(
        "/auth/register",
        json={
            "email": "weakpass@example.com",
            "password": "123",
            "background_info": "Software developer interested in robotics"
        }
    )
    assert response.status_code == 400


def test_login_user():
    """
    Test user login endpoint
    """
    # First register a user
    client.post(
        "/auth/register",
        json={
            "email": "login@example.com",
            "password": "testpassword123",
            "background_info": "Testing login functionality"
        }
    )

    # Then try to log in
    response = client.post(
        "/auth/login",
        json={
            "email": "login@example.com",
            "password": "testpassword123"
        }
    )
    assert response.status_code == 200
    data = response.json()
    assert "access_token" in data
    assert data["token_type"] == "bearer"


def test_login_user_invalid_credentials():
    """
    Test user login with invalid credentials
    """
    response = client.post(
        "/auth/login",
        json={
            "email": "nonexistent@example.com",
            "password": "wrongpassword"
        }
    )
    assert response.status_code == 401


def test_get_current_user():
    """
    Test getting current user information
    """
    # Register a user
    register_response = client.post(
        "/auth/register",
        json={
            "email": "profile@example.com",
            "password": "testpassword123",
            "background_info": "Testing profile functionality"
        }
    )
    assert register_response.status_code == 200
    user_data = register_response.json()
    user_id = user_data["id"]

    # Log in to get a token
    login_response = client.post(
        "/auth/login",
        json={
            "email": "profile@example.com",
            "password": "testpassword123"
        }
    )
    assert login_response.status_code == 200
    token_data = login_response.json()
    token = token_data["access_token"]

    # Use the token to get user profile
    response = client.get(
        "/auth/me",
        headers={"Authorization": f"Bearer {token}"}
    )
    assert response.status_code == 200
    profile_data = response.json()
    assert profile_data["email"] == "profile@example.com"
    assert profile_data["background_info"] == "Testing profile functionality"


if __name__ == "__main__":
    pytest.main()