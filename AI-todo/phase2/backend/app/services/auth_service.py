"""Authentication service for user registration and login."""

from uuid import UUID

from sqlmodel import select
from sqlmodel.ext.asyncio.session import AsyncSession

from app.core.security import hash_password, verify_password
from app.models.user import User
from app.schemas.auth import UserCreate, UserSettingsUpdate, UserUpdate
from app.services import notification_service


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
        data: User registration data (email, password, first_name, last_name).

    Returns:
        Newly created User if successful, None if email already exists.
    """
    # Check if email already exists
    existing_user = await get_user_by_email(db, data.email)
    if existing_user is not None:
        return None

    # Create new user with hashed password and optional name fields
    user = User(
        email=data.email.lower().strip(),
        hashed_password=hash_password(data.password),
        first_name=data.first_name,
        last_name=data.last_name,
        is_active=True,
    )
    db.add(user)
    await db.commit()
    await db.refresh(user)

    # Create welcome notification for new user
    await notification_service.create_notification(
        db,
        user.id,
        "welcome",
        "Welcome to TaskFlow!",
        "Get started by creating your first task.",
        "/tasks",
    )

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


async def update_user_profile(
    db: AsyncSession,
    user: User,
    data: UserUpdate,
) -> User:
    """Update user profile fields.

    Args:
        db: Async database session.
        user: Current user to update.
        data: Profile update data (partial update - only provided fields are changed).

    Returns:
        Updated User object.
    """
    # Only update fields that were explicitly provided (exclude_unset=True)
    update_data = data.model_dump(exclude_unset=True)

    for field, value in update_data.items():
        setattr(user, field, value)

    db.add(user)
    await db.commit()
    await db.refresh(user)
    return user


async def update_user_settings(
    db: AsyncSession,
    user: User,
    data: UserSettingsUpdate,
) -> User:
    """Update user settings (theme, email notifications).

    Args:
        db: Async database session.
        user: Current user to update.
        data: Settings update data (partial update - only provided fields are changed).

    Returns:
        Updated User object.
    """
    update_data = data.model_dump(exclude_unset=True)

    for field, value in update_data.items():
        setattr(user, field, value)

    db.add(user)
    await db.commit()
    await db.refresh(user)
    return user


async def change_password(
    db: AsyncSession,
    user: User,
    current_password: str,
    new_password: str,
) -> bool:
    """Change user password after verifying current password.

    Args:
        db: Async database session.
        user: Current user.
        current_password: Current password for verification.
        new_password: New password to set.

    Returns:
        True if password was changed, False if current password is incorrect.
    """
    if not verify_password(current_password, user.hashed_password):
        return False

    user.hashed_password = hash_password(new_password)
    db.add(user)
    await db.commit()
    return True


async def delete_user(db: AsyncSession, user: User) -> None:
    """Delete user account and all associated data.

    Args:
        db: Async database session.
        user: User to delete.

    Note:
        Tasks are cascade-deleted via FK constraint.
    """
    await db.delete(user)
    await db.commit()
