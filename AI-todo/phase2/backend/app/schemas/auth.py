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
    first_name: str | None = Field(
        default=None,
        max_length=100,
        description="User's first name",
        json_schema_extra={"example": "John"},
    )
    last_name: str | None = Field(
        default=None,
        max_length=100,
        description="User's last name",
        json_schema_extra={"example": "Doe"},
    )


class UserResponse(BaseModel):
    """Schema for user data in responses."""

    id: UUID = Field(..., description="Unique user identifier")
    email: str = Field(..., description="User email address")
    first_name: str | None = Field(None, description="User's first name")
    last_name: str | None = Field(None, description="User's last name")
    display_name: str = Field(..., description="Computed display name")
    avatar_url: str | None = Field(None, description="Avatar URL or base64 data URL")
    theme: str = Field(..., description="Theme preference: light, dark, or system")
    email_notifications: bool = Field(..., description="Email notification preference")
    is_active: bool = Field(..., description="Account active status")
    created_at: datetime = Field(..., description="Account creation timestamp")

    model_config = {"from_attributes": True}


class UserUpdate(BaseModel):
    """Schema for profile update request (partial update)."""

    first_name: str | None = Field(
        default=None,
        max_length=100,
        description="User's first name (null to clear)",
        json_schema_extra={"example": "Jane"},
    )
    last_name: str | None = Field(
        default=None,
        max_length=100,
        description="User's last name (null to clear)",
        json_schema_extra={"example": "Smith"},
    )
    avatar_url: str | None = Field(
        default=None,
        description="Avatar as base64 data URL (max 2MB)",
    )


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


# Settings schemas
class UserSettingsUpdate(BaseModel):
    """Schema for settings update request (partial update)."""

    theme: str | None = Field(
        default=None,
        pattern="^(light|dark|system)$",
        description="Theme preference: light, dark, or system",
    )
    email_notifications: bool | None = Field(
        default=None,
        description="Email notification preference",
    )


class UserSettingsResponse(BaseModel):
    """Schema for settings response."""

    theme: str = Field(..., description="Current theme preference")
    email_notifications: bool = Field(..., description="Email notifications enabled")

    model_config = {"from_attributes": True}


class PasswordChangeRequest(BaseModel):
    """Schema for password change request."""

    current_password: str = Field(
        ...,
        min_length=1,
        description="Current password for verification",
    )
    new_password: str = Field(
        ...,
        min_length=8,
        description="New password (minimum 8 characters)",
    )


class MessageResponse(BaseModel):
    """Generic message response."""

    message: str = Field(..., description="Response message")
