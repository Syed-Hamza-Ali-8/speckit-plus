"""Authentication service for user registration and login."""

from uuid import UUID

from sqlmodel import select
from sqlmodel.ext.asyncio.session import AsyncSession

from app.core.security import hash_password, verify_password
from app.models.user import User
from app.schemas.auth import UserCreate


async def get_user_by_email(db: AsyncSession, email: str) -> User | None:
    """Get a user by email address.

    Args:
        db: Async database session.
        email: User's email address.

    Returns:
        User if found, None otherwise.
    """
    statement = select(User).where(User.email == email.lower().strip())
    result = await db.exec(statement)
    return result.first()


async def get_user_by_id(db: AsyncSession, user_id: UUID) -> User | None:
    """Get a user by ID.

    Args:
        db: Async database session.
        user_id: User's unique identifier.

    Returns:
        User if found, None otherwise.
    """
    statement = select(User).where(User.id == user_id)
    result = await db.exec(statement)
    return result.first()


async def register_user(db: AsyncSession, data: UserCreate) -> User | None:
    """Register a new user.

    Args:
        db: Async database session.
        data: User registration data (email, password).

    Returns:
        Newly created User if successful, None if email already exists.
    """
    # Check if email already exists
    existing_user = await get_user_by_email(db, data.email)
    if existing_user is not None:
        return None

    # Create new user with hashed password
    user = User(
        email=data.email.lower().strip(),
        hashed_password=hash_password(data.password),
        is_active=True,
    )
    db.add(user)
    await db.commit()
    await db.refresh(user)
    return user


async def authenticate_user(
    db: AsyncSession,
    email: str,
    password: str,
) -> User | None:
    """Authenticate a user with email and password.

    Args:
        db: Async database session.
        email: User's email address.
        password: Plain text password.

    Returns:
        User if credentials are valid, None otherwise.
    """
    user = await get_user_by_email(db, email)
    if user is None:
        return None

    if not verify_password(password, user.hashed_password):
        return None

    return user
