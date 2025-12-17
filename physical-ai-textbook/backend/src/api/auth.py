"""
Authentication API endpoints for the Physical AI Textbook Platform
"""
from fastapi import APIRouter, HTTPException, Depends, status
from fastapi.security import HTTPBearer
from sqlalchemy.orm import Session
from datetime import timedelta
import re

from src.db.connection import get_db
from src.auth.models import UserCreate, UserLogin, User, Token, UserPublic
from src.auth.models_db import User as DBUser
from src.auth.config import verify_password, get_password_hash, create_access_token
from src.auth.session import create_session_token
from src.auth import models

router = APIRouter()

# Regex for email validation
EMAIL_REGEX = r'^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}$'

@router.post("/register", response_model=UserPublic)
async def register_user(user_data: UserCreate, db: Session = Depends(get_db)):
    """
    Register a new user with email, password, and background information
    """
    try:
        # Validate email format
        if not re.match(EMAIL_REGEX, user_data.email):
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Invalid email format"
            )

        # Check if user already exists
        existing_user = db.query(DBUser).filter(DBUser.email == user_data.email).first()
        if existing_user:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="User with this email already exists"
            )

        # Validate password strength (at least 8 characters)
        if len(user_data.password) < 8:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Password must be at least 8 characters long"
            )

        # Hash the password
        hashed_password = get_password_hash(user_data.password)

        # Create new user
        db_user = DBUser(
            email=user_data.email,
            hashed_password=hashed_password,
            background_info=user_data.background_info,
            software_expertise=user_data.software_expertise,
            hardware_expertise=user_data.hardware_expertise
        )

        db.add(db_user)
        db.commit()
        db.refresh(db_user)

        # Return public user information
        return UserPublic(
            id=db_user.id,
            email=db_user.email,
            background_info=db_user.background_info,
            software_expertise=db_user.software_expertise,
            hardware_expertise=db_user.hardware_expertise,
            is_active=db_user.is_active,
            is_verified=db_user.is_verified,
            role=db_user.role,
            created_at=db_user.created_at
        )

    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error registering user: {str(e)}"
        )


@router.post("/login", response_model=Token)
async def login_user(user_data: UserLogin, db: Session = Depends(get_db)):
    """
    Login a user with email and password
    """
    try:
        # Find the user by email
        db_user = db.query(DBUser).filter(DBUser.email == user_data.email).first()

        if not db_user or not verify_password(user_data.password, db_user.hashed_password):
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Incorrect email or password",
                headers={"WWW-Authenticate": "Bearer"},
            )

        if not db_user.is_active:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Inactive user account",
                headers={"WWW-Authenticate": "Bearer"},
            )

        # Create access token
        access_token_expires = timedelta(minutes=30)
        access_token = create_access_token(
            data={"sub": db_user.email, "user_id": db_user.id},
            expires_delta=access_token_expires
        )

        return {"access_token": access_token, "token_type": "bearer"}

    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error logging in user: {str(e)}"
        )


@router.post("/logout")
async def logout_user(token: str = Depends(HTTPBearer()), db: Session = Depends(get_db)):
    """
    Logout a user by invalidating their session
    """
    from src.auth.session import invalidate_session

    try:
        # Try to invalidate the session (if it's a session token)
        success = invalidate_session(token.credentials, db)
        if success:
            return {"message": "Successfully logged out"}

        # If it's a JWT token, we can't invalidate it server-side
        # (JWT tokens are stateless), so we just return a success message
        return {"message": "Successfully logged out (JWT token cannot be invalidated server-side)"}

    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error logging out user: {str(e)}"
        )


# Dependency to get current user from database
async def get_current_user_from_db(token: str = Depends(HTTPBearer()), db: Session = Depends(get_db)):
    """
    Dependency to get the current user from the token
    """
    from src.auth.session import get_current_user_from_token

    user = get_current_user_from_token(token.credentials, db)
    if not user:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )
    return user


@router.get("/me", response_model=UserPublic)
async def get_current_user(current_user: DBUser = Depends(get_current_user_from_db)):
    """
    Get current user information
    """
    return UserPublic(
        id=current_user.id,
        email=current_user.email,
        background_info=current_user.background_info,
        software_expertise=current_user.software_expertise,
        hardware_expertise=current_user.hardware_expertise,
        is_active=current_user.is_active,
        is_verified=current_user.is_verified,
        role=current_user.role,
        created_at=current_user.created_at
    )


@router.put("/me")
async def update_current_user(
    user_update: models.UserUpdate,
    current_user: DBUser = Depends(get_current_user_from_db),
    db: Session = Depends(get_db)
):
    """
    Update current user information
    """
    try:
        # Update user fields if provided
        if user_update.background_info is not None:
            current_user.background_info = user_update.background_info
        if user_update.software_expertise is not None:
            current_user.software_expertise = user_update.software_expertise
        if user_update.hardware_expertise is not None:
            current_user.hardware_expertise = user_update.hardware_expertise

        # Update the timestamp
        from datetime import datetime
        current_user.updated_at = datetime.utcnow()

        db.commit()
        db.refresh(current_user)

        return UserPublic(
            id=current_user.id,
            email=current_user.email,
            background_info=current_user.background_info,
            software_expertise=current_user.software_expertise,
            hardware_expertise=current_user.hardware_expertise,
            is_active=current_user.is_active,
            is_verified=current_user.is_verified,
            role=current_user.role,
            created_at=current_user.created_at
        )
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error updating user: {str(e)}"
        )