"""MCP tool implementations for task operations.

Provides a thin wrapper around existing task_service functions,
adapting them to MCP tool conventions with consistent error handling.
"""

from uuid import UUID

from sqlmodel.ext.asyncio.session import AsyncSession

from app.mcp.schemas import (
    CompleteTaskInput,
    CreateTaskInput,
    DeleteTaskInput,
    ListTasksInput,
    MCPToolResult,
    UpdateTaskInput,
)
from app.models.task import TaskStatus
from app.schemas.task import TaskCreate, TaskRead, TaskUpdate
from app.services import task_service
from app.services import notification_service


class TaskTools:
    """Static MCP tool implementations for task operations.

    Each tool follows MCP conventions:
    - Accepts typed input schema
    - Returns MCPToolResult with success/error status
    - Handles exceptions gracefully
    """

    @staticmethod
    async def list_tasks(
        db: AsyncSession,
        user_id: UUID,
        input_data: ListTasksInput,
    ) -> MCPToolResult:
        """List tasks for a user with optional filtering.

        Args:
            db: Async database session.
            user_id: Owner's user ID.
            input_data: List parameters (status, limit, offset).

        Returns:
            MCPToolResult with tasks list and total count.
        """
        try:
            # Convert status string to enum if provided
            status_filter: TaskStatus | None = None
            if input_data.status is not None:
                status_filter = TaskStatus(input_data.status)

            tasks, total = await task_service.get_tasks(
                db=db,
                user_id=user_id,
                status=status_filter,
                limit=input_data.limit,
                offset=input_data.offset,
            )

            # Convert to response format
            task_list = [
                TaskRead.model_validate(task).model_dump(mode="json")
                for task in tasks
            ]

            return MCPToolResult(
                success=True,
                data={
                    "tasks": task_list,
                    "total": total,
                    "limit": input_data.limit,
                    "offset": input_data.offset,
                },
            )
        except Exception as e:
            return MCPToolResult(
                success=False,
                error=str(e),
                error_code="TOOL_ERROR",
            )

    @staticmethod
    async def create_task(
        db: AsyncSession,
        user_id: UUID,
        input_data: CreateTaskInput,
    ) -> MCPToolResult:
        """Create a new task for a user.

        Args:
            db: Async database session.
            user_id: Owner's user ID.
            input_data: Task creation data.

        Returns:
            MCPToolResult with created task data.
        """
        try:
            task_create = TaskCreate(
                title=input_data.title,
                description=input_data.description,
                due_date=input_data.due_date,
            )

            task = await task_service.create_task(
                db=db,
                user_id=user_id,
                data=task_create,
            )

            # Create notification for task creation
            await notification_service.notify_task_created(
                db=db,
                user_id=user_id,
                task_title=task.title,
                task_id=task.id,
                due_date=task.due_date,
            )

            return MCPToolResult(
                success=True,
                data=TaskRead.model_validate(task).model_dump(mode="json"),
            )
        except ValueError as e:
            return MCPToolResult(
                success=False,
                error=str(e),
                error_code="VALIDATION_ERROR",
            )
        except Exception as e:
            return MCPToolResult(
                success=False,
                error=str(e),
                error_code="TOOL_ERROR",
            )

    @staticmethod
    async def update_task(
        db: AsyncSession,
        user_id: UUID,
        input_data: UpdateTaskInput,
    ) -> MCPToolResult:
        """Update an existing task.

        Args:
            db: Async database session.
            user_id: Owner's user ID.
            input_data: Task update data including task_id.

        Returns:
            MCPToolResult with updated task data.
        """
        try:
            # Build update data from non-None fields
            update_fields: dict[str, str | None] = {}
            if input_data.title is not None:
                update_fields["title"] = input_data.title
            if input_data.description is not None:
                update_fields["description"] = input_data.description
            if input_data.due_date is not None:
                update_fields["due_date"] = input_data.due_date  # type: ignore[assignment]

            task_update = TaskUpdate(**update_fields)

            task = await task_service.update_task(
                db=db,
                task_id=input_data.task_id,
                user_id=user_id,
                data=task_update,
            )

            if task is None:
                return MCPToolResult(
                    success=False,
                    error="Task not found or not owned by user",
                    error_code="NOT_FOUND",
                )

            # Create notification for task update
            await notification_service.notify_task_updated(
                db=db,
                user_id=user_id,
                task_title=task.title,
                task_id=task.id,
            )

            return MCPToolResult(
                success=True,
                data=TaskRead.model_validate(task).model_dump(mode="json"),
            )
        except ValueError as e:
            return MCPToolResult(
                success=False,
                error=str(e),
                error_code="VALIDATION_ERROR",
            )
        except Exception as e:
            return MCPToolResult(
                success=False,
                error=str(e),
                error_code="TOOL_ERROR",
            )

    @staticmethod
    async def complete_task(
        db: AsyncSession,
        user_id: UUID,
        input_data: CompleteTaskInput,
    ) -> MCPToolResult:
        """Mark a task as completed.

        Args:
            db: Async database session.
            user_id: Owner's user ID.
            input_data: Task ID to complete.

        Returns:
            MCPToolResult with completed task data.
        """
        try:
            task_update = TaskUpdate(status=TaskStatus.COMPLETED)

            task = await task_service.update_task(
                db=db,
                task_id=input_data.task_id,
                user_id=user_id,
                data=task_update,
            )

            if task is None:
                return MCPToolResult(
                    success=False,
                    error="Task not found or not owned by user",
                    error_code="NOT_FOUND",
                )

            # Create notification for task completion
            await notification_service.notify_task_completed(
                db=db,
                user_id=user_id,
                task_title=task.title,
                task_id=task.id,
            )

            return MCPToolResult(
                success=True,
                data=TaskRead.model_validate(task).model_dump(mode="json"),
            )
        except Exception as e:
            return MCPToolResult(
                success=False,
                error=str(e),
                error_code="TOOL_ERROR",
            )

    @staticmethod
    async def delete_task(
        db: AsyncSession,
        user_id: UUID,
        input_data: DeleteTaskInput,
    ) -> MCPToolResult:
        """Delete a task.

        Args:
            db: Async database session.
            user_id: Owner's user ID.
            input_data: Task ID to delete.

        Returns:
            MCPToolResult indicating success or failure.
        """
        try:
            # Fetch task before deletion to get title for notification
            task = await task_service.get_task(
                db=db,
                task_id=input_data.task_id,
                user_id=user_id,
            )

            if task is None:
                return MCPToolResult(
                    success=False,
                    error="Task not found or not owned by user",
                    error_code="NOT_FOUND",
                )

            task_title = task.title

            deleted = await task_service.delete_task(
                db=db,
                task_id=input_data.task_id,
                user_id=user_id,
            )

            if not deleted:
                return MCPToolResult(
                    success=False,
                    error="Task not found or not owned by user",
                    error_code="NOT_FOUND",
                )

            # Create notification for task deletion
            await notification_service.notify_task_deleted(
                db=db,
                user_id=user_id,
                task_title=task_title,
            )

            return MCPToolResult(
                success=True,
                data={"deleted": True, "task_id": str(input_data.task_id)},
            )
        except Exception as e:
            return MCPToolResult(
                success=False,
                error=str(e),
                error_code="TOOL_ERROR",
            )
