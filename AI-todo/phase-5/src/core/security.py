"""Security utilities for password hashing and JWT token management."""
from datetime import datetime, timedelta, timezone
from typing import Any, Optional
from uuid import UUID

from jose import JWTError, jwt
from passlib.context import CryptContext

# Password hashing context
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")


def hash_password(password: str) -> str:
    """Hash a password using bcrypt.

    Args:
        password: Plain text password to hash.

    Returns:
        bcrypt hash string.
    """
    return pwd_context.hash(password)


def verify_password(plain_password: str, hashed_password: str) -> bool:
    """Verify a password against its hash.

    Args:
        plain_password: Plain text password to verify.
        hashed_password: bcrypt hash to verify against.

    Returns:
        True if password matches, False otherwise.
    """
    return pwd_context.verify(plain_password, hashed_password)


def create_access_token(
    user_id: UUID,
    email: str,
    name: str,
    expires_delta: Optional[timedelta] = None,
) -> str:
    """Create a JWT access token.

    Args:
        user_id: User's UUID to encode in 'sub' claim.
        email: User's email to include in token.
        name: User's name to include in token.
        expires_delta: Optional custom expiration time.

    Returns:
        Encoded JWT token string.
    """
    now = datetime.now(timezone.utc)

    if expires_delta:
        expire = now + expires_delta
    else:
        # Default to 30 minutes expiration
        expire = now + timedelta(minutes=30)

    to_encode: dict[str, Any] = {
        "sub": str(user_id),
        "email": email,
        "name": name,
        "iat": now.timestamp(),
        "exp": expire.timestamp(),
    }

    # Using a default secret key for development - in production this should be in environment variables
    secret_key = "taskgpt-secret-key-change-in-production"
    algorithm = "HS256"

    encoded_jwt = jwt.encode(
        to_encode,
        secret_key,
        algorithm=algorithm,
    )
    return encoded_jwt


def decode_access_token(token: str) -> Optional[dict]:
    """Decode and validate a JWT access token.

    Args:
        token: JWT token string to decode.

    Returns:
        Decoded token payload if valid, None otherwise.
    """
    try:
        secret_key = "taskgpt-secret-key-change-in-production"
        algorithm = "HS256"

        payload = jwt.decode(token, secret_key, algorithms=[algorithm])
        return payload
    except JWTError:
        return None