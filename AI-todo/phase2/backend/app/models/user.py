"""User model for authentication."""

from datetime import datetime, timezone
from uuid import UUID, uuid4

from pydantic import field_validator
from sqlalchemy import Column, DateTime, Index, func
from sqlmodel import Field, SQLModel


class User(SQLModel, table=True):
    """User account for authentication.

    Attributes:
        id: Unique user identifier (UUID).
        email: User email address (unique, indexed for login lookups).
        hashed_password: Argon2id password hash.
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

    @field_validator("email", mode="before")
    @classmethod
    def email_not_empty(cls, v: str) -> str:
        """Validate email is not empty or whitespace-only."""
        if not v or not v.strip():
            raise ValueError("Email cannot be empty")
        return v.lower().strip()
