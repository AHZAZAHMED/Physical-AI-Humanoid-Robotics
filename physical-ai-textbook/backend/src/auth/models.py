"""
User models for the Physical AI Textbook Platform
"""
from pydantic import BaseModel, EmailStr
from typing import Optional
from datetime import datetime
from enum import Enum

class UserRole(str, Enum):
    USER = "user"
    ADMIN = "admin"
    MODERATOR = "moderator"

class UserBase(BaseModel):
    email: EmailStr
    background_info: Optional[str] = None

class UserCreate(UserBase):
    password: str
    # Hardware/Software expertise fields
    software_expertise: Optional[str] = None
    hardware_expertise: Optional[str] = None

class UserUpdate(BaseModel):
    background_info: Optional[str] = None
    software_expertise: Optional[str] = None
    hardware_expertise: Optional[str] = None

class UserInDB(UserBase):
    id: int
    hashed_password: str
    is_active: bool = True
    is_verified: bool = False
    role: UserRole = UserRole.USER
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True

class User(UserBase):
    id: int
    is_active: bool
    is_verified: bool
    role: UserRole
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True

class UserLogin(BaseModel):
    email: EmailStr
    password: str

class UserPublic(BaseModel):
    id: int
    email: EmailStr
    background_info: Optional[str]
    software_expertise: Optional[str]
    hardware_expertise: Optional[str]
    is_active: bool
    is_verified: bool
    role: UserRole
    created_at: datetime

    class Config:
        from_attributes = True

class Token(BaseModel):
    access_token: str
    token_type: str

class TokenData(BaseModel):
    email: Optional[str] = None
    user_id: Optional[int] = None