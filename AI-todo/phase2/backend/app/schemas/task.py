"""Task schemas for request/response validation."""

from datetime import date, datetime
from uuid import UUID

from pydantic import BaseModel, Field

from app.models.task import TaskStatus


class TaskCreate(BaseModel):
    """Schema for task creation request."""

    title: str = Field(
        ...,
        min_length=1,
        max_length=200,
        description="Task title (1-200 characters)",
        json_schema_extra={"example": "Buy groceries"},
    )
    description: str | None = Field(
        default=None,
        max_length=1000,
        description="Optional task description",
        json_schema_extra={"example": "Milk, eggs, bread"},
    )
    due_date: date | None = Field(
        default=None,
        description="Optional due date for the task",
        json_schema_extra={"example": "2025-12-31"},
    )


class TaskUpdate(BaseModel):
    """Schema for task update request (all fields optional)."""

    title: str | None = Field(
        default=None,
        min_length=1,
        max_length=200,
        description="Updated task title",
    )
    description: str | None = Field(
        default=None,
        max_length=1000,
        description="Updated task description",
    )
    status: TaskStatus | None = Field(
        default=None,
        description="Updated task status",
    )
    due_date: date | None = Field(
        default=None,
        description="Updated due date",
    )


class TaskResponse(BaseModel):
    """Schema for task data in responses (includes user_id for backward compatibility)."""

    id: UUID = Field(..., description="Unique task identifier")
    user_id: UUID = Field(..., description="Owner user ID")
    title: str = Field(..., description="Task title")
    description: str | None = Field(..., description="Task description")
    status: TaskStatus = Field(..., description="Task status")
    due_date: date | None = Field(..., description="Due date")
    created_at: datetime = Field(..., description="Creation timestamp")
    updated_at: datetime = Field(..., description="Last update timestamp")

    model_config = {"from_attributes": True}


class TaskRead(BaseModel):
    """Schema for task data in responses (excludes user_id for security).

    This schema hides user_id from API responses as the user context
    is already implied by authentication.
    """

    id: UUID = Field(..., description="Unique task identifier")
    title: str = Field(..., description="Task title")
    description: str | None = Field(..., description="Task description")
    status: TaskStatus = Field(..., description="Task status")
    due_date: date | None = Field(None, description="Due date")
    created_at: datetime = Field(..., description="Creation timestamp")
    updated_at: datetime = Field(..., description="Last update timestamp")

    model_config = {"from_attributes": True}


class PaginatedTaskResponse(BaseModel):
    """Schema for paginated task list responses."""

    items: list[TaskRead] = Field(..., description="List of tasks")
    total: int = Field(..., description="Total count of matching tasks")
    limit: int = Field(..., description="Page size")
    offset: int = Field(..., description="Current offset")
