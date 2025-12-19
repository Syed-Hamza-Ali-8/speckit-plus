"""TaskManagerAgent - Handles CRUD operations for tasks.

Processes read, create, update, complete, and delete intents
using MCP tools with user confirmation for destructive actions.
"""

import time
from uuid import UUID

from sqlmodel.ext.asyncio.session import AsyncSession

from app.agents.base import ActionTaken, AgentRequest, AgentResponse, BaseAgent
from app.mcp.schemas import (
    CompleteTaskInput,
    CreateTaskInput,
    DeleteTaskInput,
    ListTasksInput,
    UpdateTaskInput,
)
from app.mcp.tools import TaskTools


class TaskManagerAgent(BaseAgent):
    """Agent for task CRUD operations.

    Handles:
    - read: List tasks with optional filtering
    - create: Create new tasks from extracted data
    - update: Modify existing tasks
    - complete: Mark tasks as done
    - delete: Remove tasks (requires confirmation)
    """

    name = "TaskManagerAgent"

    async def process(
        self,
        request: AgentRequest,
        db: AsyncSession,
    ) -> AgentResponse:
        """Process task-related intents.

        Args:
            request: Agent request with intent and context.
            db: Database session.

        Returns:
            AgentResponse with result or confirmation prompt.
        """
        handlers = {
            "read": self._handle_read,
            "create": self._handle_create,
            "update": self._handle_update,
            "complete": self._handle_complete,
            "delete": self._handle_delete,
        }

        handler = handlers.get(request.intent)
        if not handler:
            return AgentResponse(
                success=False,
                message="I don't know how to handle that request.",
                error=f"Unknown intent: {request.intent}",
            )

        return await handler(request, db)

    async def _handle_read(
        self,
        request: AgentRequest,
        db: AsyncSession,
    ) -> AgentResponse:
        """Handle read intent - list user's tasks."""
        start_time = time.time()

        # Parse status filter from message if present
        status_filter = None
        message_lower = request.message.lower()
        if "pending" in message_lower or "incomplete" in message_lower:
            status_filter = "pending"
        elif "completed" in message_lower or "done" in message_lower:
            status_filter = "completed"

        result = await TaskTools.list_tasks(
            db=db,
            user_id=request.user_id,
            input_data=ListTasksInput(status=status_filter),
        )

        duration_ms = int((time.time() - start_time) * 1000)

        if not result.success:
            return AgentResponse(
                success=False,
                message="Sorry, I couldn't retrieve your tasks.",
                error=result.error,
                actions_taken=[
                    ActionTaken(
                        tool="list_tasks",
                        success=False,
                        summary=result.error or "Failed to list tasks",
                        duration_ms=duration_ms,
                    )
                ],
            )

        tasks = result.data.get("tasks", []) if result.data else []
        total = result.data.get("total", 0) if result.data else 0

        # Format response message
        if total == 0:
            message = "You don't have any tasks yet. Would you like to create one?"
        else:
            task_list = "\n".join(
                [f"- {t['title']} ({t['status']})" for t in tasks[:10]]
            )
            message = f"You have {total} task(s):\n\n{task_list}"
            if total > 10:
                message += f"\n\n...and {total - 10} more."

        return AgentResponse(
            success=True,
            message=message,
            data=result.data,
            actions_taken=[
                ActionTaken(
                    tool="list_tasks",
                    success=True,
                    summary=f"Listed {total} tasks",
                    duration_ms=duration_ms,
                )
            ],
        )

    async def _handle_create(
        self,
        request: AgentRequest,
        db: AsyncSession,
    ) -> AgentResponse:
        """Handle create intent - create a new task."""
        start_time = time.time()

        # Extract task data from context (set by ConversationAgent)
        task_data = {}
        if request.context and request.context.extracted_data:
            task_data = request.context.extracted_data

        # Fallback: use the message as the title
        title = task_data.get("title") or request.message
        # Clean up common prefixes
        for prefix in ["add a task to ", "create a task to ", "add task ", "create task "]:
            if title.lower().startswith(prefix):
                title = title[len(prefix):].strip()
                break

        description = task_data.get("description")
        due_date = task_data.get("due_date")

        result = await TaskTools.create_task(
            db=db,
            user_id=request.user_id,
            input_data=CreateTaskInput(
                title=title,
                description=description,
                due_date=due_date,
            ),
        )

        duration_ms = int((time.time() - start_time) * 1000)

        if not result.success:
            return AgentResponse(
                success=False,
                message=f"Sorry, I couldn't create the task: {result.error}",
                error=result.error,
                actions_taken=[
                    ActionTaken(
                        tool="create_task",
                        success=False,
                        summary=result.error or "Failed to create task",
                        duration_ms=duration_ms,
                    )
                ],
            )

        task = result.data
        message = f"Created task: '{task['title']}'"
        if task.get("due_date"):
            message += f" (due {task['due_date']})"

        # Store task ID in context for reference
        if request.context:
            task_id = UUID(task["id"]) if isinstance(task["id"], str) else task["id"]
            request.context.last_task_ids = [task_id]

        return AgentResponse(
            success=True,
            message=message,
            data=task,
            actions_taken=[
                ActionTaken(
                    tool="create_task",
                    success=True,
                    summary=f"Created '{task['title']}'",
                    duration_ms=duration_ms,
                )
            ],
        )

    async def _handle_update(
        self,
        request: AgentRequest,
        db: AsyncSession,
    ) -> AgentResponse:
        """Handle update intent - modify an existing task."""
        start_time = time.time()

        # Get task ID from context
        task_id = None
        if request.context and request.context.last_task_ids:
            task_id = request.context.last_task_ids[0]

        if not task_id:
            return AgentResponse(
                success=False,
                message="Which task would you like to update? Please list your tasks first.",
                error="No task ID in context",
            )

        # Extract update data from context
        update_data = {}
        if request.context and request.context.extracted_data:
            update_data = request.context.extracted_data

        result = await TaskTools.update_task(
            db=db,
            user_id=request.user_id,
            input_data=UpdateTaskInput(
                task_id=task_id,
                title=update_data.get("title"),
                description=update_data.get("description"),
                due_date=update_data.get("due_date"),
            ),
        )

        duration_ms = int((time.time() - start_time) * 1000)

        if not result.success:
            return AgentResponse(
                success=False,
                message=f"Sorry, I couldn't update the task: {result.error}",
                error=result.error,
                actions_taken=[
                    ActionTaken(
                        tool="update_task",
                        success=False,
                        summary=result.error or "Failed to update task",
                        duration_ms=duration_ms,
                    )
                ],
            )

        task = result.data
        return AgentResponse(
            success=True,
            message=f"Updated task: '{task['title']}'",
            data=task,
            actions_taken=[
                ActionTaken(
                    tool="update_task",
                    success=True,
                    summary=f"Updated '{task['title']}'",
                    duration_ms=duration_ms,
                )
            ],
        )

    async def _handle_complete(
        self,
        request: AgentRequest,
        db: AsyncSession,
    ) -> AgentResponse:
        """Handle complete intent - mark a task as done."""
        start_time = time.time()

        # Get task ID from context
        task_id = None
        if request.context and request.context.last_task_ids:
            task_id = request.context.last_task_ids[0]

        if not task_id:
            return AgentResponse(
                success=False,
                message="Which task would you like to complete? Please list your tasks first.",
                error="No task ID in context",
            )

        result = await TaskTools.complete_task(
            db=db,
            user_id=request.user_id,
            input_data=CompleteTaskInput(task_id=task_id),
        )

        duration_ms = int((time.time() - start_time) * 1000)

        if not result.success:
            return AgentResponse(
                success=False,
                message=f"Sorry, I couldn't complete the task: {result.error}",
                error=result.error,
                actions_taken=[
                    ActionTaken(
                        tool="complete_task",
                        success=False,
                        summary=result.error or "Failed to complete task",
                        duration_ms=duration_ms,
                    )
                ],
            )

        task = result.data
        return AgentResponse(
            success=True,
            message=f"Marked '{task['title']}' as complete!",
            data=task,
            actions_taken=[
                ActionTaken(
                    tool="complete_task",
                    success=True,
                    summary=f"Completed '{task['title']}'",
                    duration_ms=duration_ms,
                )
            ],
        )

    async def _handle_delete(
        self,
        request: AgentRequest,
        db: AsyncSession,
    ) -> AgentResponse:
        """Handle delete intent - remove a task (requires confirmation)."""
        start_time = time.time()

        # Get task ID from context
        task_id = None
        if request.context and request.context.last_task_ids:
            task_id = request.context.last_task_ids[0]

        if not task_id:
            return AgentResponse(
                success=False,
                message="Which task would you like to delete? Please list your tasks first.",
                error="No task ID in context",
            )

        # Check if confirmation is provided
        if request.context and request.context.pending_confirmation:
            confirmation = request.context.pending_confirmation
            if confirmation.action == "delete" and task_id in confirmation.target_ids:
                # Execute the deletion
                result = await TaskTools.delete_task(
                    db=db,
                    user_id=request.user_id,
                    input_data=DeleteTaskInput(task_id=task_id),
                )

                duration_ms = int((time.time() - start_time) * 1000)

                if not result.success:
                    return AgentResponse(
                        success=False,
                        message=f"Sorry, I couldn't delete the task: {result.error}",
                        error=result.error,
                        actions_taken=[
                            ActionTaken(
                                tool="delete_task",
                                success=False,
                                summary=result.error or "Failed to delete task",
                                duration_ms=duration_ms,
                            )
                        ],
                    )

                return AgentResponse(
                    success=True,
                    message="Task deleted successfully.",
                    data=result.data,
                    actions_taken=[
                        ActionTaken(
                            tool="delete_task",
                            success=True,
                            summary="Deleted task",
                            duration_ms=duration_ms,
                        )
                    ],
                )

        # No confirmation yet - request it
        return AgentResponse(
            success=True,
            message="Are you sure you want to delete this task? This action cannot be undone. Reply 'yes' to confirm.",
            requires_confirmation=True,
            confirmation_prompt="Are you sure you want to delete this task? Reply 'yes' to confirm.",
        )
