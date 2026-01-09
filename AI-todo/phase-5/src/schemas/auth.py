"""Authentication schemas for request/response validation."""

from pydantic import BaseModel, EmailStr, Field


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
    name: str = Field(
        ...,
        max_length=100,
        description="User's full name",
        json_schema_extra={"example": "John Doe"},
    )


class UserLogin(BaseModel):
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
