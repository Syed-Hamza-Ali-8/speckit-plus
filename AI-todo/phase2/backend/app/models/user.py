"""User model for authentication."""

from datetime import datetime, timezone
from uuid import UUID, uuid4

from pydantic import field_validator
from sqlalchemy import Column, DateTime, Index, Text, func
from sqlmodel import Field, SQLModel


class User(SQLModel, table=True):
    """User account for authentication.

    Attributes:
        id: Unique user identifier (UUID).
        email: User email address (unique, indexed for login lookups).
        hashed_password: Argon2id password hash.
        first_name: User's first name (optional).
        last_name: User's last name (optional).
        is_active: Account active status (inactive users cannot authenticate).
        created_at: Account creation timestamp (UTC).
    """

    model_config = {"validate_assignment": True}

    __tablename__ = "users"
    __table_args__ = (Index("ix_users_email", "email", unique=True),)

    id: UUID = Field(
        default_factory=uuid4,
        primary_key=True,
        description="Unique user identifier",
    )
    email: str = Field(
        max_length=255,
        nullable=False,
        description="User email address (unique)",
    )
    hashed_password: str = Field(
        max_length=255,
        nullable=False,
        description="Argon2id password hash",
    )
    first_name: str | None = Field(
        default=None,
        max_length=100,
        nullable=True,
        description="User's first name",
    )
    last_name: str | None = Field(
        default=None,
        max_length=100,
        nullable=True,
        description="User's last name",
    )
    avatar_url: str | None = Field(
        default=None,
        sa_column=Column(Text, nullable=True),
        description="Avatar image as base64 data URL",
    )
    theme: str = Field(
        default="system",
        max_length=20,
        nullable=False,
        description="Theme preference: light, dark, or system",
    )
    email_notifications: bool = Field(
        default=True,
        nullable=False,
        description="Email notification preference",
    )
    is_active: bool = Field(
        default=True,
        nullable=False,
        description="Account active status",
    )
    created_at: datetime = Field(
        default_factory=lambda: datetime.now(timezone.utc),
        sa_column=Column(
            DateTime(timezone=True),
            nullable=False,
            server_default=func.now(),
        ),
        description="Account creation timestamp (UTC)",
    )

    @property
    def display_name(self) -> str:
        """Compute display name from first/last name or email.

        Returns:
            Full name if both names present, single name if one present,
            or email username as fallback.
        """
        if self.first_name and self.last_name:
            return f"{self.first_name} {self.last_name}"
        elif self.first_name:
            return self.first_name
        elif self.last_name:
            return self.last_name
        return self.email.split("@")[0]

    @field_validator("email", mode="before")
    @classmethod
    def email_not_empty(cls, v: str) -> str:
        """Validate email is not empty or whitespace-only."""
        if not v or not v.strip():
            raise ValueError("Email cannot be empty")
        return v.lower().strip()

    @field_validator("first_name", "last_name", mode="before")
    @classmethod
    def normalize_name(cls, v: str | None) -> str | None:
        """Normalize name fields - strip whitespace, convert empty to None."""
        if v is None:
            return None
        stripped = v.strip()
        return stripped if stripped else None
