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
    # Phase V: Advanced features
    SetTaskPriorityInput,
    AddTaskTagsInput,
    RemoveTaskTagsInput,
    SearchTasksInput,
    CreateRecurringTaskInput,
)
from app.models.task import TaskStatus, PriorityLevel
from app.schemas.task import TaskCreate, TaskRead, TaskUpdate, SetTaskPriorityRequest, AddTaskTagsRequest, RemoveTaskTagsRequest, SearchTasksRequest, CreateRecurringTaskPatternRequest
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
                # Phase V: Advanced features
                priority=input_data.priority,
                tags=input_data.tags,
                is_recurring=input_data.is_recurring,
                recurring_pattern_id=input_data.recurring_pattern_id,
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
            if input_data.priority is not None:
                update_fields["priority"] = input_data.priority  # type: ignore[assignment]
            if input_data.tags is not None:
                update_fields["tags"] = input_data.tags  # type: ignore[assignment]

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

    # Phase V: Advanced Features MCP Tools
    @staticmethod
    async def set_task_priority(
        db: AsyncSession,
        user_id: UUID,
        input_data: SetTaskPriorityInput,
    ) -> MCPToolResult:
        """Set the priority of a task.

        Args:
            db: Async database session.
            user_id: Owner's user ID.
            input_data: Task ID and priority level.

        Returns:
            MCPToolResult with updated task data.
        """
        try:
            task = await task_service.set_task_priority(
                db=db,
                task_id=input_data.task_id,
                user_id=user_id,
                priority=input_data.priority,
            )

            if task is None:
                return MCPToolResult(
                    success=False,
                    error="Task not found or not owned by user",
                    error_code="NOT_FOUND",
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
    async def add_task_tags(
        db: AsyncSession,
        user_id: UUID,
        input_data: AddTaskTagsInput,
    ) -> MCPToolResult:
        """Add tags to a task.

        Args:
            db: Async database session.
            user_id: Owner's user ID.
            input_data: Task ID and list of tags to add.

        Returns:
            MCPToolResult with updated task data.
        """
        try:
            task = await task_service.add_task_tags(
                db=db,
                task_id=input_data.task_id,
                user_id=user_id,
                tags=input_data.tags,
            )

            if task is None:
                return MCPToolResult(
                    success=False,
                    error="Task not found or not owned by user",
                    error_code="NOT_FOUND",
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
    async def remove_task_tags(
        db: AsyncSession,
        user_id: UUID,
        input_data: RemoveTaskTagsInput,
    ) -> MCPToolResult:
        """Remove tags from a task.

        Args:
            db: Async database session.
            user_id: Owner's user ID.
            input_data: Task ID and list of tags to remove.

        Returns:
            MCPToolResult with updated task data.
        """
        try:
            task = await task_service.remove_task_tags(
                db=db,
                task_id=input_data.task_id,
                user_id=user_id,
                tags=input_data.tags,
            )

            if task is None:
                return MCPToolResult(
                    success=False,
                    error="Task not found or not owned by user",
                    error_code="NOT_FOUND",
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
    async def search_tasks(
        db: AsyncSession,
        user_id: UUID,
        input_data: SearchTasksInput,
    ) -> MCPToolResult:
        """Search tasks with various filters.

        Args:
            db: Async database session.
            user_id: Owner's user ID.
            input_data: Search parameters including query and filters.

        Returns:
            MCPToolResult with search results and total count.
        """
        try:
            tasks, total = await task_service.search_tasks(
                db=db,
                user_id=user_id,
                query=input_data.query,
                status=input_data.status,
                priority=input_data.priority,
                tags=input_data.tags,
                due_before=input_data.due_before,
                due_after=input_data.due_after,
                sort_by=input_data.sort_by,
                order=input_data.order,
                page=input_data.page,
                per_page=input_data.per_page,
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
                    "page": input_data.page,
                    "per_page": input_data.per_page,
                },
            )
        except Exception as e:
            return MCPToolResult(
                success=False,
                error=str(e),
                error_code="TOOL_ERROR",
            )

    @staticmethod
    async def create_recurring_task(
        db: AsyncSession,
        user_id: UUID,
        input_data: CreateRecurringTaskInput,
    ) -> MCPToolResult:
        """Create a recurring task pattern.

        Args:
            db: Async database session.
            user_id: Owner's user ID.
            input_data: Recurring task pattern creation data.

        Returns:
            MCPToolResult with created recurring pattern data.
        """
        try:
            from app.models.recurring_task_pattern import RecurrencePattern
            from app.services.task_service import create_recurring_task_pattern

            # Convert string pattern type to enum
            pattern_type = RecurrencePattern(input_data.pattern_type)

            recurring_pattern = await create_recurring_task_pattern(
                db=db,
                user_id=user_id,
                base_task_title=input_data.base_task_title,
                base_task_description=input_data.base_task_description,
                pattern_type=pattern_type,
                interval=input_data.interval,
                start_date=input_data.start_date,
                end_date=input_data.end_date,
                weekdays=input_data.weekdays,
                days_of_month=input_data.days_of_month,
            )

            # Convert to response format (we'll need to create a schema for this)
            pattern_data = {
                "id": str(recurring_pattern.id),
                "user_id": str(recurring_pattern.user_id),
                "base_task_title": recurring_pattern.base_task_title,
                "base_task_description": recurring_pattern.base_task_description,
                "pattern_type": recurring_pattern.pattern_type.value,
                "interval": recurring_pattern.interval,
                "start_date": recurring_pattern.start_date.isoformat(),
                "end_date": recurring_pattern.end_date.isoformat() if recurring_pattern.end_date else None,
                "weekdays": recurring_pattern.weekdays,
                "days_of_month": recurring_pattern.days_of_month,
                "created_at": recurring_pattern.created_at.isoformat(),
                "updated_at": recurring_pattern.updated_at.isoformat(),
            }

            return MCPToolResult(
                success=True,
                data=pattern_data,
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
