"""Task schemas for request/response validation."""

from datetime import date, datetime
from uuid import UUID
from typing import List, Optional

from pydantic import BaseModel, Field

from app.models.task import TaskStatus, PriorityLevel


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
    # Phase V: Advanced features
    priority: PriorityLevel | None = Field(
        default=PriorityLevel.MEDIUM,
        description="Priority level for the task",
    )
    tags: List[str] = Field(
        default_factory=list,
        description="List of tags for the task",
    )
    is_recurring: bool = Field(
        default=False,
        description="Whether this task is part of a recurring pattern",
    )
    recurring_pattern_id: UUID | None = Field(
        default=None,
        description="ID of the recurring pattern this task belongs to",
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
    # Phase V: Advanced features
    priority: PriorityLevel | None = Field(
        default=None,
        description="Updated priority level",
    )
    tags: List[str] | None = Field(
        default=None,
        description="Updated list of tags",
    )


class TaskResponse(BaseModel):
    """Schema for task data in responses (includes user_id for backward compatibility)."""

    id: UUID = Field(..., description="Unique task identifier")
    user_id: UUID = Field(..., description="Owner user ID")
    title: str = Field(..., description="Task title")
    description: str | None = Field(..., description="Task description")
    status: TaskStatus = Field(..., description="Task status")
    due_date: date | None = Field(..., description="Due date")
    # Phase V: Advanced features
    priority: PriorityLevel = Field(..., description="Priority level")
    tags: List[str] = Field(..., description="List of tags")
    is_recurring: bool = Field(..., description="Whether this task is part of a recurring pattern")
    recurring_pattern_id: UUID | None = Field(None, description="ID of the recurring pattern this task belongs to")
    is_reminder_sent: bool = Field(..., description="Whether a reminder has been sent")
    parent_task_id: UUID | None = Field(None, description="ID of the parent task (for recurring tasks)")
    next_occurrence_id: UUID | None = Field(None, description="ID of the next occurrence (for recurring tasks)")
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
    # Phase V: Advanced features
    priority: PriorityLevel = Field(..., description="Priority level")
    tags: List[str] = Field(..., description="List of tags")
    is_recurring: bool = Field(..., description="Whether this task is part of a recurring pattern")
    recurring_pattern_id: UUID | None = Field(None, description="ID of the recurring pattern this task belongs to")
    is_reminder_sent: bool = Field(..., description="Whether a reminder has been sent")
    parent_task_id: UUID | None = Field(None, description="ID of the parent task (for recurring tasks)")
    next_occurrence_id: UUID | None = Field(None, description="ID of the next occurrence (for recurring tasks)")
    created_at: datetime = Field(..., description="Creation timestamp")
    updated_at: datetime = Field(..., description="Last update timestamp")

    model_config = {"from_attributes": True}


class PaginatedTaskResponse(BaseModel):
    """Schema for paginated task list responses."""

    items: list[TaskRead] = Field(..., description="List of tasks")
    total: int = Field(..., description="Total count of matching tasks")
    limit: int = Field(..., description="Page size")
    offset: int = Field(..., description="Current offset")


# Phase V: Advanced features schemas
class SetTaskPriorityRequest(BaseModel):
    """Schema for setting task priority request."""

    priority: PriorityLevel = Field(..., description="New priority level")


class AddTaskTagsRequest(BaseModel):
    """Schema for adding task tags request."""

    tags: List[str] = Field(..., description="List of tags to add")


class RemoveTaskTagsRequest(BaseModel):
    """Schema for removing task tags request."""

    tags: List[str] = Field(..., description="List of tags to remove")


class SearchTasksRequest(BaseModel):
    """Schema for searching tasks request."""

    query: str = Field(..., min_length=1, description="Search query string")
    status: str | None = Field(default=None, description="Filter by status (all, pending, completed)")
    priority: PriorityLevel | None = Field(default=None, description="Filter by priority level")
    tags: List[str] = Field(default_factory=list, description="Filter by tags")
    due_before: date | None = Field(default=None, description="Filter by due date before")
    due_after: date | None = Field(default=None, description="Filter by due date after")
    sort_by: str = Field(default="created_at", description="Field to sort by")
    order: str = Field(default="desc", description="Sort order (asc or desc)")
    page: int = Field(default=1, ge=1, description="Page number")
    per_page: int = Field(default=20, ge=1, le=100, description="Number of items per page")


class CreateRecurringTaskPatternRequest(BaseModel):
    """Schema for creating recurring task pattern request."""

    base_task_title: str = Field(..., min_length=1, max_length=200, description="Base title for recurring tasks")
    base_task_description: str | None = Field(default=None, max_length=1000, description="Base description for recurring tasks")
    pattern_type: str = Field(..., description="Type of recurrence pattern (daily, weekly, monthly, yearly, custom)")
    interval: int = Field(default=1, ge=1, description="Interval between occurrences")
    start_date: date = Field(..., description="Start date for the recurring pattern")
    end_date: date | None = Field(default=None, description="Optional end date for the recurring pattern")
    weekdays: List[int] | None = Field(default=None, description="List of weekdays for weekly patterns (0=Sunday, 6=Saturday)")
    days_of_month: List[int] | None = Field(default=None, description="List of days of month for monthly patterns")
