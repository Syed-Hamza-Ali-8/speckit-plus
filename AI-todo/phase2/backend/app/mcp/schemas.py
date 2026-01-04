"""MCP tool input/output schemas.

Defines Pydantic models for MCP tool inputs and outputs,
following MCP schema conventions for future SDK compatibility.
"""

from datetime import date
from enum import Enum
from typing import Any, Literal, List
from uuid import UUID

from pydantic import BaseModel, Field


class MCPToolResult(BaseModel):
    """Standard result wrapper for all MCP tool invocations.

    Attributes:
        success: Whether the operation completed successfully.
        data: Operation result data (varies by tool).
        error: Human-readable error message if failed.
        error_code: Machine-readable error code for programmatic handling.
    """

    success: bool
    data: dict[str, Any] | list[Any] | None = None
    error: str | None = None
    error_code: str | None = None


class ListTasksInput(BaseModel):
    """Input schema for list_tasks MCP tool.

    Attributes:
        status: Optional filter by task status.
        limit: Maximum number of tasks to return.
        offset: Number of tasks to skip for pagination.
    """

    status: Literal["pending", "completed"] | None = None
    limit: int = Field(default=20, ge=1, le=100)
    offset: int = Field(default=0, ge=0)


class CreateTaskInput(BaseModel):
    """Input schema for create_task MCP tool.

    Attributes:
        title: Task title (required).
        description: Optional task description.
        due_date: Optional due date for the task.
        priority: Optional priority level (low, medium, high).
        tags: Optional list of tags for the task.
        is_recurring: Whether this is a recurring task.
        recurring_pattern_id: Optional recurring pattern ID.
    """

    title: str = Field(..., min_length=1, max_length=200)
    description: str | None = Field(default=None, max_length=1000)
    due_date: date | None = None
    priority: Literal["low", "medium", "high"] | None = "medium"
    tags: List[str] = Field(default_factory=list)
    is_recurring: bool = False
    recurring_pattern_id: UUID | None = None


class UpdateTaskInput(BaseModel):
    """Input schema for update_task MCP tool.

    Attributes:
        task_id: UUID of the task to update.
        title: New task title (optional).
        description: New task description (optional).
        due_date: New due date (optional).
        priority: New priority level (optional).
        tags: New list of tags (optional).
    """

    task_id: UUID
    title: str | None = Field(default=None, min_length=1, max_length=200)
    description: str | None = Field(default=None, max_length=1000)
    due_date: date | None = None
    priority: Literal["low", "medium", "high"] | None = None
    tags: List[str] | None = None


class CompleteTaskInput(BaseModel):
    """Input schema for complete_task MCP tool.

    Attributes:
        task_id: UUID of the task to mark as complete.
    """

    task_id: UUID


class DeleteTaskInput(BaseModel):
    """Input schema for delete_task MCP tool.

    Attributes:
        task_id: UUID of the task to delete.
    """

    task_id: UUID


# Phase V: Advanced Features Schemas
class PriorityLevel(str, Enum):
    """Priority levels for tasks."""
    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"


class SetTaskPriorityInput(BaseModel):
    """Input schema for set_task_priority MCP tool.

    Attributes:
        task_id: UUID of the task to set priority for.
        priority: Priority level (low, medium, high).
    """

    task_id: UUID
    priority: Literal["low", "medium", "high"]


class AddTaskTagsInput(BaseModel):
    """Input schema for add_task_tags MCP tool.

    Attributes:
        task_id: UUID of the task to add tags to.
        tags: List of tags to add.
    """

    task_id: UUID
    tags: List[str]


class RemoveTaskTagsInput(BaseModel):
    """Input schema for remove_task_tags MCP tool.

    Attributes:
        task_id: UUID of the task to remove tags from.
        tags: List of tags to remove.
    """

    task_id: UUID
    tags: List[str]


class SearchTasksInput(BaseModel):
    """Input schema for search_tasks MCP tool.

    Attributes:
        query: Search query string.
        status: Optional filter by task status.
        priority: Optional filter by priority level.
        tags: Optional filter by tags.
        due_before: Optional filter for tasks due before this date.
        due_after: Optional filter by tasks due after this date.
        sort_by: Field to sort by (default: created_at).
        order: Sort order (asc or desc, default: desc).
        page: Page number for pagination (default: 1).
        per_page: Number of items per page (default: 20).
    """

    query: str = Field(..., min_length=1)
    status: Literal["all", "pending", "completed"] | None = None
    priority: Literal["low", "medium", "high"] | None = None
    tags: List[str] = Field(default_factory=list)
    due_before: date | None = None
    due_after: date | None = None
    sort_by: Literal["created_at", "title", "due_date", "priority"] = "created_at"
    order: Literal["asc", "desc"] = "desc"
    page: int = Field(default=1, ge=1)
    per_page: int = Field(default=20, ge=1, le=100)


class CreateRecurringTaskInput(BaseModel):
    """Input schema for create_recurring_task MCP tool.

    Attributes:
        base_task_title: Title for the base recurring task.
        base_task_description: Description for the base recurring task.
        pattern_type: Type of recurrence (daily, weekly, monthly, yearly).
        interval: Interval between occurrences (default: 1).
        start_date: Start date for the recurring pattern.
        end_date: Optional end date for the recurring pattern.
        weekdays: Optional list of weekdays for weekly patterns (0=Sunday, 6=Saturday).
        days_of_month: Optional list of days of month for monthly patterns.
    """

    base_task_title: str = Field(..., min_length=1, max_length=200)
    base_task_description: str | None = Field(default=None, max_length=1000)
    pattern_type: Literal["daily", "weekly", "monthly", "yearly", "custom"]
    interval: int = Field(default=1, ge=1)
    start_date: date
    end_date: date | None = None
    weekdays: List[int] | None = Field(default=None, min_items=1, max_items=7)
    days_of_month: List[int] | None = Field(default=None, min_items=1, max_items=31)
