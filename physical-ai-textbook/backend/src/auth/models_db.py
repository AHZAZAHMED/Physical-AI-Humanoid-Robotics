"""
SQLAlchemy database models for the Physical AI Textbook Platform
"""
from sqlalchemy import Column, Integer, String, Text, Boolean, DateTime, ForeignKey
from sqlalchemy.sql import func
from sqlalchemy.orm import relationship
from src.db.connection import Base
import uuid
from datetime import datetime

class User(Base):
    """
    User database model
    """
    __tablename__ = "users"

    id = Column(Integer, primary_key=True, index=True)
    email = Column(String, unique=True, index=True, nullable=False)
    hashed_password = Column(String, nullable=False)
    background_info = Column(Text, nullable=True)
    software_expertise = Column(String, nullable=True)
    hardware_expertise = Column(String, nullable=True)
    is_active = Column(Boolean, default=True)
    is_verified = Column(Boolean, default=False)
    role = Column(String, default="user")  # user, admin, moderator
    created_at = Column(DateTime, default=func.now())
    updated_at = Column(DateTime, default=func.now(), onupdate=func.now())

    # Relationship to sessions
    sessions = relationship("Session", back_populates="user")

class Session(Base):
    """
    Session database model for tracking user sessions
    """
    __tablename__ = "sessions"

    id = Column(Integer, primary_key=True, index=True)
    user_id = Column(Integer, ForeignKey("users.id"), nullable=False)
    session_token = Column(String, unique=True, nullable=False, index=True)
    expires_at = Column(DateTime, nullable=False)
    created_at = Column(DateTime, default=func.now())
    last_accessed_at = Column(DateTime, default=func.now(), onupdate=func.now())
    device_info = Column(String, nullable=True)
    ip_address = Column(String, nullable=True)
    is_active = Column(Boolean, default=True)

    # Relationship to user
    user = relationship("User", back_populates="sessions")