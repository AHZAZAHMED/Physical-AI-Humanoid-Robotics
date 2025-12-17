"""
Authentication configuration for the Physical AI Textbook Platform
"""
from passlib.context import CryptContext
from datetime import datetime, timedelta
from typing import Optional
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Password hashing context
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")

# Secret key for JWT tokens (should be set in environment)
SECRET_KEY = os.getenv("AUTH_SECRET", "fallback_secret_key_for_development")
ALGORITHM = "HS256"
ACCESS_TOKEN_EXPIRE_MINUTES = 30

def verify_password(plain_password: str, hashed_password: str) -> bool:
    """
    Verify a plain password against a hashed password
    """
    return pwd_context.verify(plain_password, hashed_password)

def get_password_hash(password: str) -> str:
    """
    Generate a hash for a plain password
    """
    return pwd_context.hash(password)

def create_access_token(data: dict, expires_delta: Optional[timedelta] = None):
    """
    Create a JWT access token
    """
    from datetime import datetime
    from jose import jwt

    to_encode = data.copy()
    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(minutes=15)
    to_encode.update({"exp": expire})
    encoded_jwt = jwt.encode(to_encode, SECRET_KEY, algorithm=ALGORITHM)
    return encoded_jwt