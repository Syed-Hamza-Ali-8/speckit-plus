"""Recurring task pattern model."""

from datetime import date, datetime, timezone
from enum import Enum
from uuid import UUID, uuid4
from typing import List, Optional

from pydantic import field_validator
from sqlalchemy import CheckConstraint, Column, Date, DateTime, Index, JSON, func
from sqlalchemy import Enum as SAEnum
from sqlmodel import Field, SQLModel


class RecurrencePattern(str, Enum):
    """Types of recurrence patterns."""
    DAILY = "daily"
    WEEKLY = "weekly"
    MONTHLY = "monthly"
    YEARLY = "yearly"
    CUSTOM = "custom"


class RecurringTaskPattern(SQLModel, table=True):
    """Recurring task pattern entity representing a template for recurring tasks."""

    model_config = {"validate_assignment": True}

    __tablename__ = "recurring_task_patterns"
    __table_args__ = (
        CheckConstraint(
            "length(trim(base_task_title)) > 0",
            name="ck_recurring_patterns_title_not_empty",
        ),
        Index("ix_recurring_task_patterns_user_id", "user_id"),
    )

    id: UUID = Field(
        default_factory=uuid4,
        primary_key=True,
        description="Unique recurring pattern identifier",
    )
    user_id: UUID = Field(
        description="Owner's user ID",
    )
    base_task_title: str = Field(
        max_length=200,
        description="Base title for recurring tasks",
    )
    base_task_description: str | None = Field(
        default=None,
        max_length=1000,
        description="Base description for recurring tasks",
    )
    pattern_type: RecurrencePattern = Field(
        sa_column=Column(
            SAEnum(RecurrencePattern, native_enum=False, length=20),
            nullable=False,
        ),
        description="Type of recurrence pattern",
    )
    interval: int = Field(
        default=1,
        ge=1,
        description="Interval between occurrences (e.g., every 2 weeks)",
    )
    start_date: date = Field(
        description="Start date for the recurring pattern",
    )
    end_date: date | None = Field(
        default=None,
        description="Optional end date for the recurring pattern",
    )
    weekdays: List[int] = Field(
        default_factory=list,
        sa_column=Column(
            JSON,
            nullable=False,
            default=list,
        ),
        description="List of weekdays for weekly patterns (0=Sunday, 6=Saturday)",
    )
    days_of_month: List[int] = Field(
        default_factory=list,
        sa_column=Column(
            JSON,
            nullable=False,
            default=list,
        ),
        description="List of days of month for monthly patterns",
    )
    created_at: datetime = Field(
        default_factory=lambda: datetime.now(timezone.utc),
        sa_column=Column(
            DateTime(timezone=True),
            nullable=False,
            server_default=func.now(),  # ensures DB default in UTC
        ),
        description="Creation timestamp (UTC)",
    )
    updated_at: datetime = Field(
        default_factory=lambda: datetime.now(timezone.utc),
        sa_column=Column(
            DateTime(timezone=True),
            nullable=False,
            server_default=func.now(),
            onupdate=func.now(),  # auto-update on modification
        ),
        description="Last modification timestamp (UTC)",
    )

    @field_validator("base_task_title", mode="before")
    @classmethod
    def base_task_title_not_empty(cls, v: str) -> str:
        """Validate base_task_title is not empty or whitespace-only."""
        if not v or not v.strip():
            raise ValueError("Base task title cannot be empty or whitespace-only")
        return v

    @field_validator("weekdays", mode="before")
    @classmethod
    def validate_weekdays(cls, v: List[int]) -> List[int]:
        """Validate weekdays are in range 0-6."""
        if v:
            for day in v:
                if not 0 <= day <= 6:
                    raise ValueError("Weekdays must be between 0 (Sunday) and 6 (Saturday)")
        return v

    @field_validator("days_of_month", mode="before")
    @classmethod
    def validate_days_of_month(cls, v: List[int]) -> List[int]:
        """Validate days of month are in range 1-31."""
        if v:
            for day in v:
                if not 1 <= day <= 31:
                    raise ValueError("Days of month must be between 1 and 31")
        return v