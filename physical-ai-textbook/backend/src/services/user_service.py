"""
User service for the Physical AI Textbook Platform
Handles business logic for user operations
"""
from sqlalchemy.orm import Session
from typing import Optional
import re
from datetime import datetime

from src.auth.models_db import User as DBUser
from src.auth.models import UserCreate, UserUpdate


class UserService:
    """
    Service class for user-related operations
    """

    @staticmethod
    def validate_email(email: str) -> bool:
        """
        Validate email format using regex
        """
        email_regex = r'^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}$'
        return re.match(email_regex, email) is not None

    @staticmethod
    def validate_password_strength(password: str) -> tuple[bool, str]:
        """
        Validate password strength
        Returns (is_valid, error_message)
        """
        if len(password) < 8:
            return False, "Password must be at least 8 characters long"

        if not re.search(r"[A-Z]", password):
            return False, "Password must contain at least one uppercase letter"

        if not re.search(r"[a-z]", password):
            return False, "Password must contain at least one lowercase letter"

        if not re.search(r"\d", password):
            return False, "Password must contain at least one digit"

        return True, ""

    @staticmethod
    def check_email_uniqueness(email: str, db: Session) -> bool:
        """
        Check if email is unique in the database
        """
        existing_user = db.query(DBUser).filter(DBUser.email == email).first()
        return existing_user is None

    @staticmethod
    def create_user(user_data: UserCreate, hashed_password: str, db: Session) -> DBUser:
        """
        Create a new user in the database
        """
        # Validate email format
        if not UserService.validate_email(user_data.email):
            raise ValueError("Invalid email format")

        # Check if email already exists
        if not UserService.check_email_uniqueness(user_data.email, db):
            raise ValueError("User with this email already exists")

        # Validate password strength
        is_valid, error_msg = UserService.validate_password_strength(user_data.password)
        if not is_valid:
            raise ValueError(error_msg)

        # Create the user
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

        return db_user

    @staticmethod
    def update_user(user_id: int, user_update: UserUpdate, db: Session) -> Optional[DBUser]:
        """
        Update user information
        """
        db_user = db.query(DBUser).filter(DBUser.id == user_id).first()

        if not db_user:
            return None

        # Update fields if provided
        if user_update.background_info is not None:
            db_user.background_info = user_update.background_info
        if user_update.software_expertise is not None:
            db_user.software_expertise = user_update.software_expertise
        if user_update.hardware_expertise is not None:
            db_user.hardware_expertise = user_update.hardware_expertise

        # Update the timestamp
        db_user.updated_at = datetime.utcnow()

        db.commit()
        db.refresh(db_user)

        return db_user

    @staticmethod
    def get_user_by_email(email: str, db: Session) -> Optional[DBUser]:
        """
        Get a user by email
        """
        return db.query(DBUser).filter(DBUser.email == email).first()

    @staticmethod
    def get_user_by_id(user_id: int, db: Session) -> Optional[DBUser]:
        """
        Get a user by ID
        """
        return db.query(DBUser).filter(DBUser.id == user_id).first()

    @staticmethod
    def deactivate_user(user_id: int, db: Session) -> bool:
        """
        Deactivate a user account
        """
        db_user = db.query(DBUser).filter(DBUser.id == user_id).first()

        if not db_user:
            return False

        db_user.is_active = False
        db.commit()

        return True

    @staticmethod
    def activate_user(user_id: int, db: Session) -> bool:
        """
        Activate a user account
        """
        db_user = db.query(DBUser).filter(DBUser.id == user_id).first()

        if not db_user:
            return False

        db_user.is_active = True
        db.commit()

        return True


# Convenience functions for use in API endpoints
def create_user_with_validation(user_data: UserCreate, hashed_password: str, db: Session) -> DBUser:
    """
    Create a user with validation
    """
    return UserService.create_user(user_data, hashed_password, db)


def update_user_with_validation(user_id: int, user_update: UserUpdate, db: Session) -> Optional[DBUser]:
    """
    Update a user with validation
    """
    return UserService.update_user(user_id, user_update, db)


def validate_email_format(email: str) -> bool:
    """
    Validate email format
    """
    return UserService.validate_email(email)


def check_email_exists(email: str, db: Session) -> bool:
    """
    Check if email already exists
    """
    return not UserService.check_email_uniqueness(email, db)