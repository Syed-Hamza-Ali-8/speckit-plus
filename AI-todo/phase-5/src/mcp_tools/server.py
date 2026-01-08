"""MCP Server implementation for Phase 5 Todo App.

This module creates an MCP server that exposes task operations as tools
following the Model Context Protocol specification.

Adapted from Phase 2 to work with Phase 5's synchronous database architecture.
"""

from datetime import datetime
from typing import Any, List
from uuid import UUID

from mcp.server.fastmcp import FastMCP

# Create the MCP server instance
mcp = FastMCP(name="Phase 5 Todo MCP Server")

# Global context storage (will be injected per request)
_db_session: Any = None
_user_id: UUID | None = None


def set_context(db_session: Any, user_id: UUID) -> None:
    """Set the database session and user context for MCP tools.

    Args:
        db_session: Database session (synchronous Session).
        user_id: Current user's UUID.
    """
    global _db_session, _user_id
    _db_session = db_session
    _user_id = user_id


@mcp.tool()
async def add_task(
    title: str,
    description: str | None = None,
    priority: str | None = "medium",
) -> dict:
    """Create a new task for the user.

    Args:
        title: The title of the task (required).
        description: Optional description of the task.
        priority: Priority level - "low", "medium", or "high" (default: "medium").

    Returns:
        dict with task_id, status, and title of created task.
    """
    from models.task_models import Task, TaskStatus, PriorityLevel

    if _db_session is None or _user_id is None:
        return {"error": "Context not set", "status": "error"}

    try:
        # Map priority string to enum
        priority_map = {
            "low": PriorityLevel.LOW,
            "medium": PriorityLevel.MEDIUM,
            "high": PriorityLevel.HIGH
        }
        priority_level = priority_map.get(priority, PriorityLevel.MEDIUM)

        # Create task
        new_task = Task(
            title=title,
            description=description,
            user_id=_user_id,
            created_by=_user_id,
            updated_by=_user_id,
            status=TaskStatus.PENDING,
            priority=priority_level
        )

        _db_session.add(new_task)
        _db_session.commit()
        _db_session.refresh(new_task)

        return {
            "task_id": str(new_task.id),
            "status": "created",
            "title": new_task.title,
            "priority": new_task.priority.value if hasattr(new_task.priority, 'value') else str(new_task.priority),
        }
    except Exception as e:
        _db_session.rollback()
        return {"error": str(e), "status": "error"}


@mcp.tool()
async def list_tasks(
    status: str = "all",
    priority: str | None = None,
) -> list[dict]:
    """List all tasks for the user.

    Args:
        status: Filter by status - "all", "pending", or "completed".
        priority: Filter by priority - "low", "medium", or "high" (optional).

    Returns:
        List of task objects with id, title, description, status, and priority.
    """
    from sqlmodel import select
    from models.task_models import Task, TaskStatus, PriorityLevel

    if _db_session is None or _user_id is None:
        return [{"error": "Context not set", "status": "error"}]

    try:
        # Build query
        statement = select(Task).where(Task.user_id == _user_id)

        # Apply status filter
        if status == "pending":
            statement = statement.where(Task.status == TaskStatus.PENDING)
        elif status == "completed":
            statement = statement.where(Task.status == TaskStatus.COMPLETED)

        # Apply priority filter if provided
        if priority:
            priority_map = {
                "low": PriorityLevel.LOW,
                "medium": PriorityLevel.MEDIUM,
                "high": PriorityLevel.HIGH
            }
            if priority in priority_map:
                statement = statement.where(Task.priority == priority_map[priority])

        statement = statement.limit(100)
        tasks = _db_session.exec(statement).all()

        return [
            {
                "id": str(task.id),
                "title": task.title,
                "description": task.description,
                "status": task.status.value if hasattr(task.status, 'value') else str(task.status),
                "priority": task.priority.value if hasattr(task.priority, 'value') else str(task.priority),
                "completed": task.status == TaskStatus.COMPLETED,
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
    from models.task_models import Task, TaskStatus

    if _db_session is None or _user_id is None:
        return {"error": "Context not set", "status": "error"}

    try:
        task_uuid = UUID(task_id)

        # Get task
        task = _db_session.get(Task, task_uuid)

        if task is None or task.user_id != _user_id:
            return {
                "task_id": task_id,
                "status": "not_found",
                "error": "Task not found or not owned by user",
            }

        # Update task status
        task.status = TaskStatus.COMPLETED
        task.updated_at = datetime.utcnow()

        _db_session.add(task)
        _db_session.commit()

        return {
            "task_id": str(task.id),
            "status": "completed",
            "title": task.title,
        }
    except ValueError:
        return {"error": "Invalid task_id format", "status": "error"}
    except Exception as e:
        _db_session.rollback()
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
    from models.task_models import Task

    if _db_session is None or _user_id is None:
        return {"error": "Context not set", "status": "error"}

    try:
        task_uuid = UUID(task_id)

        # Get task
        task = _db_session.get(Task, task_uuid)

        if task is None or task.user_id != _user_id:
            return {
                "task_id": task_id,
                "status": "not_found",
                "error": "Task not found or not owned by user",
            }

        title = task.title

        # Delete task
        _db_session.delete(task)
        _db_session.commit()

        return {
            "task_id": task_id,
            "status": "deleted",
            "title": title,
        }
    except ValueError:
        return {"error": "Invalid task_id format", "status": "error"}
    except Exception as e:
        _db_session.rollback()
        return {"error": str(e), "status": "error"}


@mcp.tool()
async def update_task(
    task_id: str,
    title: str | None = None,
    description: str | None = None,
    priority: str | None = None,
) -> dict:
    """Update a task's title, description, or priority.

    Args:
        task_id: The UUID of the task to update.
        title: New title for the task (optional).
        description: New description for the task (optional).
        priority: New priority level - "low", "medium", or "high" (optional).

    Returns:
        dict with task_id, status, and title of updated task.
    """
    from models.task_models import Task, PriorityLevel

    print(f"[UPDATE_TASK] Called with task_id={task_id}, title={title}, description={description}, priority={priority}")
    print(f"[UPDATE_TASK] Context: user_id={_user_id}, db_session={_db_session is not None}")

    if _db_session is None or _user_id is None:
        print(f"[UPDATE_TASK] ERROR: Context not set")
        return {"error": "Context not set", "status": "error"}

    try:
        task_uuid = UUID(task_id)

        # Get task
        task = _db_session.get(Task, task_uuid)
        print(f"[UPDATE_TASK] Found task: {task.title if task else 'None'}")
        print(f"[UPDATE_TASK] Task user_id: {task.user_id if task else 'None'}, Current user_id: {_user_id}")

        if task is None or task.user_id != _user_id:
            print(f"[UPDATE_TASK] ERROR: Task not found or not owned by user")
            return {
                "task_id": task_id,
                "status": "not_found",
                "error": "Task not found or not owned by user",
            }

        # Update fields if provided
        if title is not None:
            task.title = title
        if description is not None:
            task.description = description
        if priority is not None:
            priority_map = {
                "low": PriorityLevel.LOW,
                "medium": PriorityLevel.MEDIUM,
                "high": PriorityLevel.HIGH
            }
            if priority in priority_map:
                task.priority = priority_map[priority]

        task.updated_at = datetime.utcnow()

        _db_session.add(task)
        _db_session.commit()

        return {
            "task_id": str(task.id),
            "status": "updated",
            "title": task.title,
            "priority": task.priority.value if hasattr(task.priority, 'value') else str(task.priority),
        }
    except ValueError:
        return {"error": "Invalid task_id format", "status": "error"}
    except Exception as e:
        _db_session.rollback()
        return {"error": str(e), "status": "error"}


# ============================================================================
# HIGH-LEVEL MCP TOOLS (Handle multi-step workflows internally)
# ============================================================================

@mcp.tool()
async def update_task_by_name(
    task_name: str,
    new_description: str | None = None,
    new_title: str | None = None,
    new_priority: str | None = None,
) -> dict:
    """Update a task by its name (handles UUID lookup internally).

    This is a high-level tool that finds the task by name and updates it.
    If multiple tasks match, returns a list for clarification.

    Args:
        task_name: The name/title of the task to update.
        new_description: New description for the task (optional).
        new_title: New title for the task (optional).
        new_priority: New priority level - "low", "medium", or "high" (optional).

    Returns:
        dict with status and result. If multiple matches, includes list of tasks.
    """
    print(f"[UPDATE_BY_NAME] Called with task_name='{task_name}', new_description='{new_description}', new_title='{new_title}', new_priority='{new_priority}'")

    if _db_session is None or _user_id is None:
        return {"error": "Context not set", "status": "error"}

    try:
        # Step 1: Get all tasks
        all_tasks = await list_tasks(status="all")
        print(f"[UPDATE_BY_NAME] Found {len(all_tasks)} total tasks")

        # Step 2: Find matching tasks (case-insensitive partial match)
        task_name_lower = task_name.lower().strip()
        matching_tasks = [
            task for task in all_tasks
            if task_name_lower in task["title"].lower() or task["title"].lower() in task_name_lower
        ]

        print(f"[UPDATE_BY_NAME] Found {len(matching_tasks)} matching tasks")

        # Step 3: Handle results
        if len(matching_tasks) == 0:
            return {
                "status": "not_found",
                "message": f"No task found matching '{task_name}'",
            }

        if len(matching_tasks) > 1:
            # Multiple matches - return for clarification
            return {
                "status": "clarification_needed",
                "message": f"Found {len(matching_tasks)} tasks matching '{task_name}'. Please be more specific.",
                "matching_tasks": [
                    {
                        "id": task["id"],
                        "title": task["title"],
                        "description": task.get("description"),
                        "status": task["status"],
                        "priority": task["priority"],
                    }
                    for task in matching_tasks
                ],
            }

        # Step 4: Single match - update it
        task = matching_tasks[0]
        task_id = task["id"]

        print(f"[UPDATE_BY_NAME] Updating task {task_id} ('{task['title']}')")

        result = await update_task(
            task_id=task_id,
            title=new_title,
            description=new_description,
            priority=new_priority,
        )

        print(f"[UPDATE_BY_NAME] Update result: {result}")
        return result

    except Exception as e:
        print(f"[UPDATE_BY_NAME] Error: {e}")
        import traceback
        print(traceback.format_exc())
        return {"error": str(e), "status": "error"}


@mcp.tool()
async def delete_task_by_name(task_name: str) -> dict:
    """Delete a task by its name (handles UUID lookup internally).

    This is a high-level tool that finds the task by name and deletes it.
    If multiple tasks match, returns a list for clarification.

    Args:
        task_name: The name/title of the task to delete.

    Returns:
        dict with status and result. If multiple matches, includes list of tasks.
    """
    print(f"[DELETE_BY_NAME] Called with task_name='{task_name}'")

    if _db_session is None or _user_id is None:
        return {"error": "Context not set", "status": "error"}

    try:
        # Step 1: Get all tasks
        all_tasks = await list_tasks(status="all")
        print(f"[DELETE_BY_NAME] Found {len(all_tasks)} total tasks")

        # Step 2: Find matching tasks (case-insensitive partial match)
        task_name_lower = task_name.lower().strip()
        matching_tasks = [
            task for task in all_tasks
            if task_name_lower in task["title"].lower() or task["title"].lower() in task_name_lower
        ]

        print(f"[DELETE_BY_NAME] Found {len(matching_tasks)} matching tasks")

        # Step 3: Handle results
        if len(matching_tasks) == 0:
            return {
                "status": "not_found",
                "message": f"No task found matching '{task_name}'",
            }

        if len(matching_tasks) > 1:
            # Multiple matches - return for clarification
            return {
                "status": "clarification_needed",
                "message": f"Found {len(matching_tasks)} tasks matching '{task_name}'. Please be more specific.",
                "matching_tasks": [
                    {
                        "id": task["id"],
                        "title": task["title"],
                        "description": task.get("description"),
                        "status": task["status"],
                        "priority": task["priority"],
                    }
                    for task in matching_tasks
                ],
            }

        # Step 4: Single match - delete it
        task = matching_tasks[0]
        task_id = task["id"]

        print(f"[DELETE_BY_NAME] Deleting task {task_id} ('{task['title']}')")

        result = await delete_task(task_id=task_id)

        print(f"[DELETE_BY_NAME] Delete result: {result}")
        return result

    except Exception as e:
        print(f"[DELETE_BY_NAME] Error: {e}")
        import traceback
        print(traceback.format_exc())
        return {"error": str(e), "status": "error"}


@mcp.tool()
async def complete_task_by_name(task_name: str) -> dict:
    """Mark a task as complete by its name (handles UUID lookup internally).

    This is a high-level tool that finds the task by name and marks it complete.
    If multiple tasks match, returns a list for clarification.

    Args:
        task_name: The name/title of the task to complete.

    Returns:
        dict with status and result. If multiple matches, includes list of tasks.
    """
    print(f"[COMPLETE_BY_NAME] Called with task_name='{task_name}'")

    if _db_session is None or _user_id is None:
        return {"error": "Context not set", "status": "error"}

    try:
        # Step 1: Get all pending tasks
        all_tasks = await list_tasks(status="pending")
        print(f"[COMPLETE_BY_NAME] Found {len(all_tasks)} pending tasks")

        # Step 2: Find matching tasks (case-insensitive partial match)
        task_name_lower = task_name.lower().strip()
        matching_tasks = [
            task for task in all_tasks
            if task_name_lower in task["title"].lower() or task["title"].lower() in task_name_lower
        ]

        print(f"[COMPLETE_BY_NAME] Found {len(matching_tasks)} matching tasks")

        # Step 3: Handle results
        if len(matching_tasks) == 0:
            return {
                "status": "not_found",
                "message": f"No pending task found matching '{task_name}'",
            }

        if len(matching_tasks) > 1:
            # Multiple matches - return for clarification
            return {
                "status": "clarification_needed",
                "message": f"Found {len(matching_tasks)} pending tasks matching '{task_name}'. Please be more specific.",
                "matching_tasks": [
                    {
                        "id": task["id"],
                        "title": task["title"],
                        "description": task.get("description"),
                        "status": task["status"],
                        "priority": task["priority"],
                    }
                    for task in matching_tasks
                ],
            }

        # Step 4: Single match - complete it
        task = matching_tasks[0]
        task_id = task["id"]

        print(f"[COMPLETE_BY_NAME] Completing task {task_id} ('{task['title']}')")

        result = await complete_task(task_id=task_id)

        print(f"[COMPLETE_BY_NAME] Complete result: {result}")
        return result

    except Exception as e:
        print(f"[COMPLETE_BY_NAME] Error: {e}")
        import traceback
        print(traceback.format_exc())
        return {"error": str(e), "status": "error"}


# Entry point for running as standalone MCP server
if __name__ == "__main__":
    mcp.run(transport="stdio")
