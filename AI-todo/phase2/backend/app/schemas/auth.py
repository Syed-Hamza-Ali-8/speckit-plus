"""Authentication schemas for request/response validation."""

from datetime import datetime
from uuid import UUID

from pydantic import BaseModel, EmailStr, Field


# T009: Registration schemas
class UserCreate(BaseModel):
    """Schema for user registration request."""

    email: EmailStr = Field(
        ...,
        description="User email address",
        json_schema_extra={"example": "user@example.com"},
    )
    password: str = Field(
        ...,
        min_length=8,
        description="User password (minimum 8 characters)",
        json_schema_extra={"example": "securepassword123"},
    )


class UserResponse(BaseModel):
    """Schema for user data in responses."""

    id: UUID = Field(..., description="Unique user identifier")
    email: str = Field(..., description="User email address")
    is_active: bool = Field(..., description="Account active status")
    created_at: datetime = Field(..., description="Account creation timestamp")

    model_config = {"from_attributes": True}


# T014: Login schemas
class LoginRequest(BaseModel):
    """Schema for login request."""

    email: EmailStr = Field(
        ...,
        description="User email address",
        json_schema_extra={"example": "user@example.com"},
    )
    password: str = Field(
        ...,
        description="User password",
        json_schema_extra={"example": "securepassword123"},
    )


class TokenResponse(BaseModel):
    """Schema for JWT token response."""

    access_token: str = Field(..., description="JWT access token")
    token_type: str = Field(default="bearer", description="Token type")
