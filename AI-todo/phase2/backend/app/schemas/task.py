"""Task schemas for request/response validation."""

from datetime import datetime
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


class TaskResponse(BaseModel):
    """Schema for task data in responses."""

    id: UUID = Field(..., description="Unique task identifier")
    user_id: UUID = Field(..., description="Owner user ID")
    title: str = Field(..., description="Task title")
    description: str | None = Field(..., description="Task description")
    status: TaskStatus = Field(..., description="Task status")
    created_at: datetime = Field(..., description="Creation timestamp")
    updated_at: datetime = Field(..., description="Last update timestamp")

    model_config = {"from_attributes": True}
