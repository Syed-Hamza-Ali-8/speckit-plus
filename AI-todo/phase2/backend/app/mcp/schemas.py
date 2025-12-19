"""MCP tool input/output schemas.

Defines Pydantic models for MCP tool inputs and outputs,
following MCP schema conventions for future SDK compatibility.
"""

from datetime import date
from typing import Any, Literal
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
    """

    title: str = Field(..., min_length=1, max_length=200)
    description: str | None = Field(default=None, max_length=1000)
    due_date: date | None = None


class UpdateTaskInput(BaseModel):
    """Input schema for update_task MCP tool.

    Attributes:
        task_id: UUID of the task to update.
        title: New task title (optional).
        description: New task description (optional).
        due_date: New due date (optional).
    """

    task_id: UUID
    title: str | None = Field(default=None, min_length=1, max_length=200)
    description: str | None = Field(default=None, max_length=1000)
    due_date: date | None = None


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
