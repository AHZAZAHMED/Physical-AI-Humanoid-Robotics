"""
Session management for the Physical AI Textbook Platform
"""
from datetime import datetime, timedelta
from typing import Optional
import secrets
from sqlalchemy.orm import Session
from jose import jwt, JWTError
from fastapi import HTTPException, status, Depends
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials

from src.auth.models_db import User, Session as DBSession
from src.auth.config import SECRET_KEY, ALGORITHM, ACCESS_TOKEN_EXPIRE_MINUTES
from src.db.connection import get_db
from src.auth.models import TokenData

security = HTTPBearer()

def create_session_token(user_id: int, db: Session) -> str:
    """
    Create a new session token for a user
    """
    # Generate a secure random token
    token = secrets.token_urlsafe(32)

    # Set session expiration (e.g., 24 hours from now)
    expires_at = datetime.utcnow() + timedelta(hours=24)

    # Create a new session record in the database
    db_session = DBSession(
        user_id=user_id,
        session_token=token,
        expires_at=expires_at,
        is_active=True
    )

    db.add(db_session)
    db.commit()
    db.refresh(db_session)

    return token

def verify_session_token(token: str, db: Session) -> Optional[User]:
    """
    Verify a session token and return the associated user
    """
    try:
        # Find the session in the database
        db_session = db.query(DBSession).filter(
            DBSession.session_token == token,
            DBSession.is_active == True,
            DBSession.expires_at > datetime.utcnow()
        ).first()

        if not db_session:
            return None

        # Update last accessed time
        db_session.last_accessed_at = datetime.utcnow()
        db.commit()

        # Return the associated user
        return db_session.user

    except Exception:
        return None

def get_current_user_from_token(token: str, db: Session) -> Optional[User]:
    """
    Get the current user from a token (either session token or JWT)
    """
    # First try to verify as a session token
    user = verify_session_token(token, db)
    if user:
        return user

    # If not a session token, try to verify as a JWT
    try:
        payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
        email: str = payload.get("sub")
        if email is None:
            return None
        token_data = TokenData(email=email)
    except JWTError:
        return None

    # Find user by email from JWT
    user = db.query(User).filter(User.email == token_data.email).first()
    return user

def get_current_user(credentials: HTTPAuthorizationCredentials = Depends(security), db: Session = Depends(get_db)) -> User:
    """
    Dependency to get the current user from the authorization header
    """
    token = credentials.credentials
    user = get_current_user_from_token(token, db)

    if user is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )

    return user

def invalidate_session(token: str, db: Session) -> bool:
    """
    Invalidate a session token (logout)
    """
    try:
        # Find the session in the database
        db_session = db.query(DBSession).filter(
            DBSession.session_token == token
        ).first()

        if not db_session:
            return False

        # Mark as inactive
        db_session.is_active = False
        db.commit()

        return True
    except Exception:
        return False

def create_access_token_with_data(data: dict, expires_delta: Optional[timedelta] = None):
    """
    Create an access token with custom data
    """
    to_encode = data.copy()
    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)

    to_encode.update({"exp": expire, "type": "access"})
    encoded_jwt = jwt.encode(to_encode, SECRET_KEY, algorithm=ALGORITHM)
    return encoded_jwt

def create_refresh_token(data: dict):
    """
    Create a refresh token
    """
    to_encode = data.copy()
    expire = datetime.utcnow() + timedelta(days=7)  # Refresh tokens last 7 days
    to_encode.update({"exp": expire, "type": "refresh"})
    encoded_jwt = jwt.encode(to_encode, SECRET_KEY, algorithm=ALGORITHM)
    return encoded_jwt