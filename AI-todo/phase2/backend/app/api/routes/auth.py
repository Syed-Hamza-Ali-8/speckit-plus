"""Authentication endpoints for registration, login, and profile."""

from fastapi import APIRouter, HTTPException, Request, status

from app.api.deps import CurrentUser, DbSession
from app.core.security import create_access_token
from app.middleware.rate_limit import (
    LOGIN_RATE_LIMIT,
    REGISTER_RATE_LIMIT,
    limiter,
)
from app.schemas.auth import (
    LoginRequest,
    MessageResponse,
    PasswordChangeRequest,
    TokenResponse,
    UserCreate,
    UserResponse,
    UserSettingsResponse,
    UserSettingsUpdate,
    UserUpdate,
)
from app.services import auth_service

router = APIRouter(prefix="/auth", tags=["auth"])


# T011: POST /auth/register endpoint
# T012: Rate limit 5/min
@router.post(
    "/register",
    response_model=UserResponse,
    status_code=status.HTTP_201_CREATED,
    responses={
        409: {"description": "Email already registered"},
        429: {"description": "Rate limit exceeded"},
    },
)
@limiter.limit(REGISTER_RATE_LIMIT)
async def register(
    request: Request,
    data: UserCreate,
    db: DbSession,
) -> UserResponse:
    """Register a new user account.

    Creates a new user with the provided email, password, and optional name fields.
    Password is hashed using Argon2id before storage.

    Rate limited to 5 requests per minute.
    """
    user = await auth_service.register_user(db, data)
    if user is None:
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail="Email already registered",
        )
    return UserResponse.model_validate(user)


# T016: POST /auth/login endpoint
# T017: Rate limit 10/min
@router.post(
    "/login",
    response_model=TokenResponse,
    responses={
        401: {"description": "Invalid credentials"},
        429: {"description": "Rate limit exceeded"},
    },
)
@limiter.limit(LOGIN_RATE_LIMIT)
async def login(
    request: Request,
    data: LoginRequest,
    db: DbSession,
) -> TokenResponse:
    """Authenticate user and return JWT token.

    Validates email and password, returns JWT access token if successful.
    Token expires after 30 minutes (configurable).
    Token payload includes user's display_name.

    Rate limited to 10 requests per minute.
    """
    user = await auth_service.authenticate_user(db, data.email, data.password)
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

    access_token = create_access_token(
        user_id=user.id,
        email=user.email,
        display_name=user.display_name,
    )
    return TokenResponse(access_token=access_token, token_type="bearer")


# T020: GET /auth/me endpoint
@router.get(
    "/me",
    response_model=UserResponse,
    responses={
        401: {"description": "Not authenticated"},
    },
)
async def get_current_user_profile(
    current_user: CurrentUser,
) -> UserResponse:
    """Get current authenticated user's profile.

    Requires valid JWT token in Authorization header.
    Returns user profile including first_name, last_name, and display_name.
    """
    return UserResponse.model_validate(current_user)


# Dynamic Profile Page: PATCH /auth/me endpoint
@router.patch(
    "/me",
    response_model=UserResponse,
    responses={
        401: {"description": "Not authenticated"},
        422: {"description": "Validation error"},
    },
)
async def update_current_user_profile(
    data: UserUpdate,
    current_user: CurrentUser,
    db: DbSession,
) -> UserResponse:
    """Update current authenticated user's profile.

    Supports partial updates - only provided fields are updated.
    Email cannot be changed via this endpoint.

    Updatable fields:
    - first_name: User's first name (max 100 chars, null to clear)
    - last_name: User's last name (max 100 chars, null to clear)
    - avatar_url: Avatar as base64 data URL (null to clear)
    """
    user = await auth_service.update_user_profile(db, current_user, data)
    return UserResponse.model_validate(user)


# Settings endpoints
@router.patch(
    "/me/settings",
    response_model=UserSettingsResponse,
    responses={
        401: {"description": "Not authenticated"},
        422: {"description": "Validation error"},
    },
)
async def update_settings(
    data: UserSettingsUpdate,
    current_user: CurrentUser,
    db: DbSession,
) -> UserSettingsResponse:
    """Update current user's settings.

    Supports partial updates - only provided fields are updated.

    Updatable fields:
    - theme: Theme preference (light, dark, or system)
    - email_notifications: Email notification preference (true/false)
    """
    user = await auth_service.update_user_settings(db, current_user, data)
    return UserSettingsResponse.model_validate(user)


@router.post(
    "/change-password",
    response_model=MessageResponse,
    responses={
        401: {"description": "Current password is incorrect"},
        422: {"description": "Validation error"},
    },
)
async def change_password(
    data: PasswordChangeRequest,
    current_user: CurrentUser,
    db: DbSession,
) -> MessageResponse:
    """Change current user's password.

    Requires current password verification before setting new password.
    New password must be at least 8 characters.
    """
    success = await auth_service.change_password(
        db, current_user, data.current_password, data.new_password
    )
    if not success:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Current password is incorrect",
        )
    return MessageResponse(message="Password changed successfully")


@router.delete(
    "/me",
    status_code=status.HTTP_204_NO_CONTENT,
    responses={
        401: {"description": "Not authenticated"},
    },
)
async def delete_account(
    current_user: CurrentUser,
    db: DbSession,
) -> None:
    """Delete current user's account and all associated data.

    This action is irreversible. All user data including tasks will be deleted.
    """
    await auth_service.delete_user(db, current_user)
