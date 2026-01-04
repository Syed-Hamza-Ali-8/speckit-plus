"""MCP Server implementation using Official MCP SDK.

This module creates a proper MCP server that exposes task operations
as tools following the Model Context Protocol specification.
"""

from datetime import date
from typing import Any, List
from uuid import UUID

from mcp.server.fastmcp import FastMCP
from app.services import notification_service

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
    priority: str | None = "medium",  # Phase V: Advanced feature
    tags: List[str] | None = None,   # Phase V: Advanced feature
    is_recurring: bool | None = False,  # Phase V: Advanced feature
    recurring_pattern_id: str | None = None,  # Phase V: Advanced feature
) -> dict:
    """Create a new task for the user.

    Args:
        title: The title of the task (required).
        description: Optional description of the task.
        due_date: Optional due date in YYYY-MM-DD format.
        priority: Priority level - "low", "medium", or "high" (default: "medium").
        tags: Optional list of tags for the task.
        is_recurring: Whether this task is part of a recurring pattern (default: False).
        recurring_pattern_id: Optional ID of the recurring pattern this task belongs to.

    Returns:
        dict with task_id, status, and title of created task.
    """
    from app.models.task import PriorityLevel
    from app.schemas.task import TaskCreate
    from app.services import task_service

    if _db_session is None or _user_id is None:
        return {"error": "Context not set", "status": "error"}

    try:
        # Parse due_date if provided
        parsed_due_date: date | None = None
        if due_date:
            parsed_due_date = date.fromisoformat(due_date)

        # Parse priority if provided
        parsed_priority: PriorityLevel | None = None
        if priority:
            try:
                parsed_priority = PriorityLevel(priority)
            except ValueError:
                return {"error": f"Invalid priority: {priority}", "status": "error"}

        # Parse recurring pattern ID if provided
        parsed_recurring_pattern_id: UUID | None = None
        if recurring_pattern_id:
            try:
                parsed_recurring_pattern_id = UUID(recurring_pattern_id)
            except ValueError:
                return {"error": f"Invalid recurring_pattern_id: {recurring_pattern_id}", "status": "error"}

        task_create = TaskCreate(
            title=title,
            description=description,
            due_date=parsed_due_date,
            # Phase V: Advanced features
            priority=parsed_priority,
            tags=tags or [],
            is_recurring=is_recurring,
            recurring_pattern_id=parsed_recurring_pattern_id,
        )

        task = await task_service.create_task(
            db=_db_session,
            user_id=_user_id,
            data=task_create,
        )

        # Create notification for task creation
        print(f"[MCP Server] Creating notification for task: {task.title}")
        notification = await notification_service.notify_task_created(
            db=_db_session,
            user_id=_user_id,
            task_title=task.title,
            task_id=task.id,
            due_date=task.due_date,
        )
        print(f"[MCP Server] Notification created with ID: {notification.id}")

        return {
            "task_id": str(task.id),
            "status": "created",
            "title": task.title,
            "priority": task.priority.value,
            "tags": task.tags,
        }
    except Exception as e:
        return {"error": str(e), "status": "error"}


@mcp.tool()
async def list_tasks(
    status: str = "all",
    priority: str | None = None,  # Phase V: Advanced feature
    tags: List[str] | None = None,  # Phase V: Advanced feature
) -> list[dict]:
    """List all tasks for the user.

    Args:
        status: Filter by status - "all", "pending", or "completed".
        priority: Filter by priority - "low", "medium", or "high" (optional).
        tags: Filter by tags - list of tags to match (optional).

    Returns:
        List of task objects with id, title, description, completed status, priority, and tags.
    """
    from app.models.task import TaskStatus, PriorityLevel
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

        # Convert priority string to enum if provided
        priority_filter: PriorityLevel | None = None
        if priority:
            try:
                priority_filter = PriorityLevel(priority)
            except ValueError:
                return [{"error": f"Invalid priority: {priority}", "status": "error"}]

        tasks, _total = await task_service.get_tasks(
            db=_db_session,
            user_id=_user_id,
            status=status_filter,
            priority=priority_filter,
            tags=tags,
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
                "priority": task.priority.value,
                "tags": task.tags,
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

        # Create notification for task completion
        await notification_service.notify_task_completed(
            db=_db_session,
            user_id=_user_id,
            task_title=task.title,
            task_id=task.id,
        )

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

        # Create notification for task deletion
        await notification_service.notify_task_deleted(
            db=_db_session,
            user_id=_user_id,
            task_title=title,
        )

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
    priority: str | None = None,  # Phase V: Advanced feature
    tags: List[str] | None = None,  # Phase V: Advanced feature
) -> dict:
    """Update a task's title, description, due date, priority, or tags.

    Args:
        task_id: The UUID of the task to update.
        title: New title for the task (optional).
        description: New description for the task (optional).
        due_date: New due date in YYYY-MM-DD format (optional).
        priority: New priority level - "low", "medium", or "high" (optional).
        tags: New list of tags for the task (optional).

    Returns:
        dict with task_id, status, and title of updated task.
    """
    from app.models.task import PriorityLevel
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
        if priority is not None:
            try:
                update_fields["priority"] = PriorityLevel(priority)
            except ValueError:
                return {"error": f"Invalid priority: {priority}", "status": "error"}
        if tags is not None:
            update_fields["tags"] = tags

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

        # Create notification for task update
        await notification_service.notify_task_updated(
            db=_db_session,
            user_id=_user_id,
            task_title=task.title,
            task_id=task.id,
        )

        return {
            "task_id": str(task.id),
            "status": "updated",
            "title": task.title,
            "priority": task.priority.value,
            "tags": task.tags,
        }
    except ValueError as e:
        return {"error": f"Invalid input: {e}", "status": "error"}
    except Exception as e:
        return {"error": str(e), "status": "error"}


# Phase V: Advanced Features MCP Tools
@mcp.tool()
async def set_task_priority(
    task_id: str,
    priority: str,
) -> dict:
    """Set the priority of a task.

    Args:
        task_id: The UUID of the task to update.
        priority: Priority level - "low", "medium", or "high".

    Returns:
        dict with task_id, status, title, and new priority of the updated task.
    """
    from app.models.task import PriorityLevel
    from app.services import task_service

    if _db_session is None or _user_id is None:
        return {"error": "Context not set", "status": "error"}

    try:
        task_uuid = UUID(task_id)

        # Validate priority
        try:
            priority_level = PriorityLevel(priority)
        except ValueError:
            return {"error": f"Invalid priority: {priority}", "status": "error"}

        task = await task_service.set_task_priority(
            db=_db_session,
            task_id=task_uuid,
            user_id=_user_id,
            priority=priority_level,
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
            "priority": task.priority.value,
        }
    except ValueError as e:
        return {"error": f"Invalid input: {e}", "status": "error"}
    except Exception as e:
        return {"error": str(e), "status": "error"}


@mcp.tool()
async def add_task_tags(
    task_id: str,
    tags: List[str],
) -> dict:
    """Add tags to a task.

    Args:
        task_id: The UUID of the task to update.
        tags: List of tags to add to the task.

    Returns:
        dict with task_id, status, title, and updated tags of the task.
    """
    from app.services import task_service

    if _db_session is None or _user_id is None:
        return {"error": "Context not set", "status": "error"}

    try:
        task_uuid = UUID(task_id)

        task = await task_service.add_task_tags(
            db=_db_session,
            task_id=task_uuid,
            user_id=_user_id,
            tags=tags,
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
            "tags": task.tags,
        }
    except ValueError as e:
        return {"error": f"Invalid input: {e}", "status": "error"}
    except Exception as e:
        return {"error": str(e), "status": "error"}


@mcp.tool()
async def remove_task_tags(
    task_id: str,
    tags: List[str],
) -> dict:
    """Remove tags from a task.

    Args:
        task_id: The UUID of the task to update.
        tags: List of tags to remove from the task.

    Returns:
        dict with task_id, status, title, and updated tags of the task.
    """
    from app.services import task_service

    if _db_session is None or _user_id is None:
        return {"error": "Context not set", "status": "error"}

    try:
        task_uuid = UUID(task_id)

        task = await task_service.remove_task_tags(
            db=_db_session,
            task_id=task_uuid,
            user_id=_user_id,
            tags=tags,
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
            "tags": task.tags,
        }
    except ValueError as e:
        return {"error": f"Invalid input: {e}", "status": "error"}
    except Exception as e:
        return {"error": str(e), "status": "error"}


@mcp.tool()
async def search_tasks(
    query: str,
    status: str | None = None,
    priority: str | None = None,
    tags: List[str] | None = None,
    due_before: str | None = None,
    due_after: str | None = None,
    sort_by: str = "created_at",
    order: str = "desc",
    page: int = 1,
    per_page: int = 20,
) -> dict:
    """Search tasks with various filters.

    Args:
        query: Search query string to match in title or description.
        status: Filter by status - "all", "pending", or "completed" (optional).
        priority: Filter by priority - "low", "medium", or "high" (optional).
        tags: Filter by tags - list of tags to match (optional).
        due_before: Filter for tasks due before this date (YYYY-MM-DD format, optional).
        due_after: Filter for tasks due after this date (YYYY-MM-DD format, optional).
        sort_by: Field to sort by - "created_at", "title", "due_date", or "priority" (default: "created_at").
        order: Sort order - "asc" or "desc" (default: "desc").
        page: Page number for pagination (default: 1).
        per_page: Number of items per page (default: 20).

    Returns:
        dict with tasks list, total count, and pagination info.
    """
    from app.models.task import TaskStatus, PriorityLevel
    from app.services import task_service

    if _db_session is None or _user_id is None:
        return {"error": "Context not set", "status": "error"}

    try:
        # Parse dates if provided
        parsed_due_before: date | None = None
        if due_before:
            parsed_due_before = date.fromisoformat(due_before)

        parsed_due_after: date | None = None
        if due_after:
            parsed_due_after = date.fromisoformat(due_after)

        # Validate status if provided
        validated_status = None
        if status and status not in ["all", "pending", "completed"]:
            return {"error": f"Invalid status: {status}", "status": "error"}

        # Validate priority if provided
        validated_priority: PriorityLevel | None = None
        if priority:
            try:
                validated_priority = PriorityLevel(priority)
            except ValueError:
                return {"error": f"Invalid priority: {priority}", "status": "error"}

        # Validate sort_by and order
        if sort_by not in ["created_at", "title", "due_date", "priority"]:
            return {"error": f"Invalid sort_by: {sort_by}", "status": "error"}
        if order not in ["asc", "desc"]:
            return {"error": f"Invalid order: {order}", "status": "error"}

        tasks, total = await task_service.search_tasks(
            db=_db_session,
            user_id=_user_id,
            query=query,
            status=status,
            priority=validated_priority,
            tags=tags,
            due_before=parsed_due_before,
            due_after=parsed_due_after,
            sort_by=sort_by,
            order=order,
            page=page,
            per_page=per_page,
        )

        return {
            "tasks": [
                {
                    "id": str(task.id),
                    "title": task.title,
                    "description": task.description,
                    "status": task.status.value,
                    "due_date": task.due_date.isoformat() if task.due_date else None,
                    "priority": task.priority.value,
                    "tags": task.tags,
                    "created_at": task.created_at.isoformat(),
                    "updated_at": task.updated_at.isoformat(),
                }
                for task in tasks
            ],
            "total": total,
            "page": page,
            "per_page": per_page,
        }
    except ValueError as e:
        return {"error": f"Invalid input: {e}", "status": "error"}
    except Exception as e:
        return {"error": str(e), "status": "error"}


@mcp.tool()
async def create_recurring_task_pattern(
    base_task_title: str,
    base_task_description: str | None = None,
    pattern_type: str = "daily",
    interval: int = 1,
    start_date: str | None = None,
    end_date: str | None = None,
    weekdays: List[int] | None = None,
    days_of_month: List[int] | None = None,
) -> dict:
    """Create a recurring task pattern.

    Args:
        base_task_title: Title for the base recurring task.
        base_task_description: Description for the base recurring task (optional).
        pattern_type: Type of recurrence - "daily", "weekly", "monthly", "yearly", or "custom" (default: "daily").
        interval: Interval between occurrences (default: 1).
        start_date: Start date for the recurring pattern in YYYY-MM-DD format (default: today).
        end_date: End date for the recurring pattern in YYYY-MM-DD format (optional).
        weekdays: List of weekdays for weekly patterns (0=Sunday, 6=Saturday, optional).
        days_of_month: List of days of month for monthly patterns (optional).

    Returns:
        dict with pattern_id, status, and details of the created recurring pattern.
    """
    from app.models.recurring_task_pattern import RecurrencePattern
    from app.services import task_service

    if _db_session is None or _user_id is None:
        return {"error": "Context not set", "status": "error"}

    try:
        # Parse dates if provided
        parsed_start_date: date | None = None
        if start_date:
            parsed_start_date = date.fromisoformat(start_date)
        else:
            from datetime import datetime
            parsed_start_date = datetime.now().date()

        parsed_end_date: date | None = None
        if end_date:
            parsed_end_date = date.fromisoformat(end_date)

        # Validate pattern type
        try:
            validated_pattern_type = RecurrencePattern(pattern_type)
        except ValueError:
            return {"error": f"Invalid pattern_type: {pattern_type}", "status": "error"}

        # Validate interval
        if interval < 1:
            return {"error": "Interval must be at least 1", "status": "error"}

        # Validate weekdays if provided
        if weekdays:
            for day in weekdays:
                if not 0 <= day <= 6:
                    return {"error": f"Weekday must be between 0 (Sunday) and 6 (Saturday): {day}", "status": "error"}

        # Validate days_of_month if provided
        if days_of_month:
            for day in days_of_month:
                if not 1 <= day <= 31:
                    return {"error": f"Day of month must be between 1 and 31: {day}", "status": "error"}

        recurring_pattern = await task_service.create_recurring_task_pattern(
            db=_db_session,
            user_id=_user_id,
            base_task_title=base_task_title,
            base_task_description=base_task_description,
            pattern_type=validated_pattern_type,
            interval=interval,
            start_date=parsed_start_date,
            end_date=parsed_end_date,
            weekdays=weekdays,
            days_of_month=days_of_month,
        )

        return {
            "pattern_id": str(recurring_pattern.id),
            "status": "created",
            "base_task_title": recurring_pattern.base_task_title,
            "pattern_type": recurring_pattern.pattern_type.value,
            "interval": recurring_pattern.interval,
            "start_date": recurring_pattern.start_date.isoformat(),
            "end_date": recurring_pattern.end_date.isoformat() if recurring_pattern.end_date else None,
            "weekdays": recurring_pattern.weekdays,
            "days_of_month": recurring_pattern.days_of_month,
        }
    except ValueError as e:
        return {"error": f"Invalid input: {e}", "status": "error"}
    except Exception as e:
        return {"error": str(e), "status": "error"}


# Entry point for running as standalone MCP server
if __name__ == "__main__":
    mcp.run(transport="stdio")
