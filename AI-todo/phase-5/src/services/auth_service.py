"""Authentication service for user registration and login."""
from datetime import datetime, timedelta
from typing import Optional
from uuid import UUID

from sqlmodel import Session, select
import bcrypt

from models.user_models import User, UserCreate, UserUpdate


def hash_password(password: str) -> str:
    """Hash a password using bcrypt.

    Args:
        password: Plain text password (will be truncated to 72 bytes).

    Returns:
        Hashed password string.
    """
    password_bytes = password.encode('utf-8')[:72]
    return bcrypt.hashpw(password_bytes, bcrypt.gensalt()).decode('utf-8')


def verify_password(plain_password: str, hashed_password: str) -> bool:
    """Verify a password against a hash.

    Args:
        plain_password: Plain text password to verify.
        hashed_password: Hashed password to compare against.

    Returns:
        True if password matches, False otherwise.
    """
    password_bytes = plain_password.encode('utf-8')[:72]
    return bcrypt.checkpw(password_bytes, hashed_password.encode('utf-8'))


def get_user_by_email(db: Session, email: str) -> Optional[User]:
    """Get a user by email address.

    Args:
        db: Database session.
        email: User's email address.

    Returns:
        User if found, None otherwise.
    """
    statement = select(User).where(User.email == email.lower().strip())
    result = db.exec(statement)
    return result.first()


def get_user_by_id(db: Session, user_id: UUID) -> Optional[User]:
    """Get a user by ID.

    Args:
        db: Database session.
        user_id: User's unique identifier.

    Returns:
        User if found, None otherwise.
    """
    statement = select(User).where(User.id == user_id)
    result = db.exec(statement)
    return result.first()


def register_user(db: Session, data: UserCreate) -> Optional[User]:
    """Register a new user.

    Args:
        db: Database session.
        data: User registration data (email, password, name).

    Returns:
        Newly created User if successful, None if email already exists.
    """
    # Check if email already exists
    existing_user = get_user_by_email(db, data.email)
    if existing_user is not None:
        return None

    # Create new user with hashed password
    user = User(
        email=data.email.lower().strip(),
        name=data.name,
        password_hash=hash_password(data.password),
        is_active=True
    )
    db.add(user)
    db.commit()
    db.refresh(user)

    return user


def authenticate_user(db: Session, email: str, password: str) -> Optional[User]:
    """Authenticate a user with email and password.

    Args:
        db: Database session.
        email: User's email address.
        password: Plain text password.

    Returns:
        User if credentials are valid, None otherwise.
    """
    # Normalize email
    email = email.lower().strip()

    user = get_user_by_email(db, email)
    if user is None:
        # Verify password anyway to prevent timing attacks
        try:
            verify_password(password, "$2b$12$dummy_hash_for_timing_attack_prevention")
        except:
            pass
        return None

    if not verify_password(password, user.password_hash):
        return None

    return user


def update_user_profile(db: Session, user: User, data: UserUpdate) -> User:
    """Update user profile fields.

    Args:
        db: Database session.
        user: Current user to update.
        data: Profile update data (partial update - only provided fields are changed).

    Returns:
        Updated User object.
    """
    # Only update fields that were explicitly provided
    update_data = data.model_dump(exclude_unset=True)

    for field, value in update_data.items():
        setattr(user, field, value)

    db.add(user)
    db.commit()
    db.refresh(user)
    return user