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
    TokenResponse,
    UserCreate,
    UserResponse,
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

    Creates a new user with the provided email and password.
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

    access_token = create_access_token(user_id=user.id, email=user.email)
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
    Returns user profile (id, email, is_active, created_at).
    """
    return UserResponse.model_validate(current_user)
