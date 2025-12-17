"""
Authentication middleware for the Physical AI Textbook Platform
"""
from fastapi import HTTPException, status, Request
from fastapi.security.utils import get_authorization_scheme_param
from sqlalchemy.orm import Session
from jose import jwt, JWTError
from datetime import datetime
import logging

from src.db.connection import SessionLocal, engine
from src.auth.models_db import User, Session as DBSession

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class AuthMiddleware:
    """
    Authentication middleware to handle token validation for protected routes
    """
    def __init__(self, app):
        self.app = app

    async def __call__(self, scope, receive, send):
        if scope["type"] != "http":
            return await self.app(scope, receive, send)

        request = Request(scope)
        # Skip authentication for public routes
        public_routes = [
            "/",
            "/docs",
            "/redoc",
            "/openapi.json",
            "/health",
            "/auth/register",
            "/auth/login",
            "/chat",
            "/chat/health",
            "/index_content",
            "/search",
            "/answer_question"
        ]

        # Check if the current route is public
        if request.url.path in public_routes:
            return await self.app(scope, receive, send)

        # For protected routes, check for authentication
        authorization = request.headers.get("Authorization")
        if not authorization:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Authorization header is required"
            )

        # Get the token from the authorization header
        scheme, token = get_authorization_scheme_param(authorization)
        if not token or scheme.lower() != "bearer":
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid authentication credentials"
            )

        # Validate the token
        user = self.validate_token(token)
        if not user:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid or expired token"
            )

        # Add user info to request state for use in route handlers
        request.state.user = user
        scope['user'] = user

        # Continue with the request
        return await self.app(scope, receive, send)

    def validate_token(self, token: str) -> User:
        """
        Validate the token and return the associated user
        """
        db = SessionLocal()
        try:
            # First try to validate as a session token
            db_session = db.query(DBSession).filter(
                DBSession.session_token == token,
                DBSession.is_active == True,
                DBSession.expires_at > datetime.utcnow()
            ).first()

            if db_session:
                # Update last accessed time
                db_session.last_accessed_at = datetime.utcnow()
                db.commit()
                return db_session.user

            # If not a session token, try to validate as a JWT
            try:
                payload = jwt.decode(token, "fallback_secret_key_for_development", algorithms=["HS256"])
                email: str = payload.get("sub")
                if email is None:
                    return None

                user = db.query(User).filter(User.email == email).first()
                return user
            except JWTError:
                return None

        except Exception as e:
            logger.error(f"Error validating token: {str(e)}")
            return None
        finally:
            db.close()