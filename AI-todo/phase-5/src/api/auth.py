"""Authentication endpoints for registration, login, and profile."""
from fastapi import APIRouter, Depends, HTTPException, status, Header
from sqlmodel import Session
from typing import Optional

from config.database import get_session
from core.security import create_access_token, decode_access_token
from models.user_models import User
from schemas.auth import UserCreate, UserLogin
from services.auth_service import authenticate_user, register_user, get_user_by_id
from uuid import UUID

router = APIRouter(prefix="/auth", tags=["auth"])


@router.post("/register")
def register(
    user_data: UserCreate,
    session: Session = Depends(get_session)
):
    """Register a new user account."""
    # Convert Pydantic model to SQLModel for database operations
    from models.user_models import UserCreate as UserCreateModel
    user_create_db = UserCreateModel(
        email=user_data.email,
        name=user_data.name,
        password=user_data.password
    )

    user = register_user(session, user_create_db)
    if user is None:
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail="Email already registered",
        )

    # Create access token
    access_token = create_access_token(
        user_id=user.id,
        email=user.email,
        name=user.name,
    )

    return {
        "access_token": access_token,
        "token_type": "bearer",
        "user": {
            "id": user.id,
            "email": user.email,
            "name": user.name,
            "is_active": user.is_active
        }
    }


@router.post("/login")
def login(
    credentials: UserLogin,
    session: Session = Depends(get_session)
):
    """Authenticate user and return JWT token."""
    user = authenticate_user(session, credentials.email, credentials.password)
    if user is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid email or password",
        )

    if not user.is_active:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Account is inactive",
        )

    # Create access token
    access_token = create_access_token(
        user_id=user.id,
        email=user.email,
        name=user.name,
    )

    return {
        "access_token": access_token,
        "token_type": "bearer",
        "user": {
            "id": user.id,
            "email": user.email,
            "name": user.name,
            "is_active": user.is_active
        }
    }


@router.get("/me")
def get_current_user(
    authorization: Optional[str] = Header(None),
    session: Session = Depends(get_session)
):
    """Get current authenticated user's profile.

    Validates JWT token from Authorization header and returns user info.
    """
    if not authorization or not authorization.startswith("Bearer "):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Missing or invalid authorization header",
        )

    # Extract token from "Bearer <token>"
    token = authorization.split(" ")[1]

    # Decode and validate token
    payload = decode_access_token(token)
    if payload is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid or expired token",
        )

    # Get user_id from token
    user_id_str = payload.get("sub")
    if not user_id_str:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid token payload",
        )

    try:
        user_id = UUID(user_id_str)
    except ValueError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid user ID in token",
        )

    # Fetch user from database
    user = get_user_by_id(session, user_id)
    if user is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="User not found",
        )

    if not user.is_active:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="User account is inactive",
        )

    return {
        "id": str(user.id),
        "email": user.email,
        "name": user.name,
        "is_active": user.is_active,
        "created_at": user.created_at.isoformat() if user.created_at else None,
    }


@router.patch("/me")
def update_current_user(
    update_data: dict,
    authorization: Optional[str] = Header(None),
    session: Session = Depends(get_session)
):
    """Update current authenticated user's profile.

    Allows updating the user's name.
    Validates JWT token from Authorization header.
    """
    if not authorization or not authorization.startswith("Bearer "):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Missing or invalid authorization header",
        )

    # Extract token from "Bearer <token>"
    token = authorization.split(" ")[1]

    # Decode and validate token
    payload = decode_access_token(token)
    if payload is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid or expired token",
        )

    # Get user_id from token
    user_id_str = payload.get("sub")
    if not user_id_str:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid token payload",
        )

    try:
        user_id = UUID(user_id_str)
    except ValueError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid user ID in token",
        )

    # Fetch user from database
    user = get_user_by_id(session, user_id)
    if user is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="User not found",
        )

    if not user.is_active:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="User account is inactive",
        )

    # Update user fields
    if "name" in update_data and update_data["name"] is not None:
        user.name = update_data["name"]

    # Save changes
    session.add(user)
    session.commit()
    session.refresh(user)

    return {
        "id": str(user.id),
        "email": user.email,
        "name": user.name,
        "is_active": user.is_active,
        "created_at": user.created_at.isoformat() if user.created_at else None,
    }