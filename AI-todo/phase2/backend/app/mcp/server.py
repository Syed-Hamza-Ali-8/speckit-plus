"""MCP Server implementation using Official MCP SDK.

This module creates a proper MCP server that exposes task operations
as tools following the Model Context Protocol specification.
"""

from datetime import date
from typing import Any
from uuid import UUID

from mcp.server.fastmcp import FastMCP

# Create the MCP server instance
mcp = FastMCP(name="Todo MCP Server")

# In-memory storage for database session (will be injected)
_db_session: Any = None
_user_id: UUID | None = None


def set_context(db_session: Any, user_id: UUID) -> None:
    """Set the database session and user context for MCP tools.

    Args:
        db_session: Async database session.
        user_id: Current user's UUID.
    """
    global _db_session, _user_id
    _db_session = db_session
    _user_id = user_id


@mcp.tool()
async def add_task(
    title: str,
    description: str | None = None,
    due_date: str | None = None,
) -> dict:
    """Create a new task for the user.

    Args:
        title: The title of the task (required).
        description: Optional description of the task.
        due_date: Optional due date in YYYY-MM-DD format.

    Returns:
        dict with task_id, status, and title of created task.
    """
    from app.schemas.task import TaskCreate
    from app.services import task_service

    if _db_session is None or _user_id is None:
        return {"error": "Context not set", "status": "error"}

    try:
        # Parse due_date if provided
        parsed_due_date: date | None = None
        if due_date:
            parsed_due_date = date.fromisoformat(due_date)

        task_create = TaskCreate(
            title=title,
            description=description,
            due_date=parsed_due_date,
        )

        task = await task_service.create_task(
            db=_db_session,
            user_id=_user_id,
            data=task_create,
        )

        return {
            "task_id": str(task.id),
            "status": "created",
            "title": task.title,
        }
    except Exception as e:
        return {"error": str(e), "status": "error"}


@mcp.tool()
async def list_tasks(
    status: str = "all",
) -> list[dict]:
    """List all tasks for the user.

    Args:
        status: Filter by status - "all", "pending", or "completed".

    Returns:
        List of task objects with id, title, description, completed status.
    """
    from app.models.task import TaskStatus
    from app.services import task_service

    if _db_session is None or _user_id is None:
        return [{"error": "Context not set", "status": "error"}]

    try:
        # Convert status string to enum
        status_filter: TaskStatus | None = None
        if status == "pending":
            status_filter = TaskStatus.PENDING
        elif status == "completed":
            status_filter = TaskStatus.COMPLETED

        tasks, _total = await task_service.get_tasks(
            db=_db_session,
            user_id=_user_id,
            status=status_filter,
            limit=100,
            offset=0,
        )

        return [
            {
                "id": str(task.id),
                "title": task.title,
                "description": task.description,
                "completed": task.status == TaskStatus.COMPLETED,
                "due_date": task.due_date.isoformat() if task.due_date else None,
            }
            for task in tasks
        ]
    except Exception as e:
        return [{"error": str(e), "status": "error"}]


@mcp.tool()
async def complete_task(
    task_id: str,
) -> dict:
    """Mark a task as complete.

    Args:
        task_id: The UUID of the task to mark as complete.

    Returns:
        dict with task_id, status, and title of completed task.
    """
    from app.models.task import TaskStatus
    from app.schemas.task import TaskUpdate
    from app.services import task_service

    if _db_session is None or _user_id is None:
        return {"error": "Context not set", "status": "error"}

    try:
        task_uuid = UUID(task_id)
        task_update = TaskUpdate(status=TaskStatus.COMPLETED)

        task = await task_service.update_task(
            db=_db_session,
            task_id=task_uuid,
            user_id=_user_id,
            data=task_update,
        )

        if task is None:
            return {
                "task_id": task_id,
                "status": "not_found",
                "error": "Task not found or not owned by user",
            }

        return {
            "task_id": str(task.id),
            "status": "completed",
            "title": task.title,
        }
    except ValueError:
        return {"error": "Invalid task_id format", "status": "error"}
    except Exception as e:
        return {"error": str(e), "status": "error"}


@mcp.tool()
async def delete_task(
    task_id: str,
) -> dict:
    """Delete a task from the list.

    Args:
        task_id: The UUID of the task to delete.

    Returns:
        dict with task_id and status of deletion.
    """
    from app.services import task_service

    if _db_session is None or _user_id is None:
        return {"error": "Context not set", "status": "error"}

    try:
        task_uuid = UUID(task_id)

        # Get task title before deletion for response
        task = await task_service.get_task(
            db=_db_session,
            task_id=task_uuid,
            user_id=_user_id,
        )

        if task is None:
            return {
                "task_id": task_id,
                "status": "not_found",
                "error": "Task not found or not owned by user",
            }

        title = task.title

        deleted = await task_service.delete_task(
            db=_db_session,
            task_id=task_uuid,
            user_id=_user_id,
        )

        if not deleted:
            return {
                "task_id": task_id,
                "status": "error",
                "error": "Failed to delete task",
            }

        return {
            "task_id": task_id,
            "status": "deleted",
            "title": title,
        }
    except ValueError:
        return {"error": "Invalid task_id format", "status": "error"}
    except Exception as e:
        return {"error": str(e), "status": "error"}


@mcp.tool()
async def update_task(
    task_id: str,
    title: str | None = None,
    description: str | None = None,
    due_date: str | None = None,
) -> dict:
    """Update a task's title, description, or due date.

    Args:
        task_id: The UUID of the task to update.
        title: New title for the task (optional).
        description: New description for the task (optional).
        due_date: New due date in YYYY-MM-DD format (optional).

    Returns:
        dict with task_id, status, and title of updated task.
    """
    from app.schemas.task import TaskUpdate
    from app.services import task_service

    if _db_session is None or _user_id is None:
        return {"error": "Context not set", "status": "error"}

    try:
        task_uuid = UUID(task_id)

        # Build update data
        update_fields: dict[str, Any] = {}
        if title is not None:
            update_fields["title"] = title
        if description is not None:
            update_fields["description"] = description
        if due_date is not None:
            update_fields["due_date"] = date.fromisoformat(due_date)

        if not update_fields:
            return {
                "task_id": task_id,
                "status": "error",
                "error": "No fields to update",
            }

        task_update = TaskUpdate(**update_fields)

        task = await task_service.update_task(
            db=_db_session,
            task_id=task_uuid,
            user_id=_user_id,
            data=task_update,
        )

        if task is None:
            return {
                "task_id": task_id,
                "status": "not_found",
                "error": "Task not found or not owned by user",
            }

        return {
            "task_id": str(task.id),
            "status": "updated",
            "title": task.title,
        }
    except ValueError as e:
        return {"error": f"Invalid input: {e}", "status": "error"}
    except Exception as e:
        return {"error": str(e), "status": "error"}


# Entry point for running as standalone MCP server
if __name__ == "__main__":
    mcp.run(transport="stdio")
