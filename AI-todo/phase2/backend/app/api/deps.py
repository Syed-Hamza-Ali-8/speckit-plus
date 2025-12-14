"""FastAPI dependencies for authentication and database access."""

from typing import Annotated
from uuid import UUID

from fastapi import Depends, HTTPException, status
from fastapi.security import OAuth2PasswordBearer
from sqlmodel import select
from sqlmodel.ext.asyncio.session import AsyncSession

from app.core.database import get_async_session
from app.core.security import decode_token
from app.models.user import User

# T021: OAuth2 Bearer token scheme
oauth2_scheme = OAuth2PasswordBearer(tokenUrl="/auth/login")


# T018: Get current user from JWT token
async def get_current_user(
    token: Annotated[str, Depends(oauth2_scheme)],
    db: Annotated[AsyncSession, Depends(get_async_session)],
) -> User:
    """Extract and validate the current user from JWT token.

    Args:
        token: JWT token from Authorization header.
        db: Async database session.

    Returns:
        Authenticated User instance.

    Raises:
        HTTPException: 401 if token is invalid or user not found.
    """
    credentials_exception = HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail="Could not validate credentials",
        headers={"WWW-Authenticate": "Bearer"},
    )

    payload = decode_token(token)
    if payload is None:
        raise credentials_exception

    user_id_str = payload.get("sub")
    if user_id_str is None:
        raise credentials_exception

    try:
        user_id = UUID(user_id_str)
    except ValueError:
        raise credentials_exception

    statement = select(User).where(User.id == user_id)
    result = await db.exec(statement)
    user = result.first()

    if user is None:
        raise credentials_exception

    return user


# T019: Get current active user (checks is_active)
async def get_current_active_user(
    current_user: Annotated[User, Depends(get_current_user)],
) -> User:
    """Verify the current user is active.

    Args:
        current_user: User from get_current_user dependency.

    Returns:
        Active User instance.

    Raises:
        HTTPException: 403 if user account is inactive.
    """
    if not current_user.is_active:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Inactive user account",
        )
    return current_user


# Type alias for cleaner dependency injection
CurrentUser = Annotated[User, Depends(get_current_active_user)]
DbSession = Annotated[AsyncSession, Depends(get_async_session)]
