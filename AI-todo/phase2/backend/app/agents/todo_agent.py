"""Todo Agent implementation using OpenAI Agents SDK with Gemini.

This module creates an AI agent that uses MCP tools to manage
user's todo tasks through natural language conversation.

Uses OpenAIChatCompletionsModel to connect to Gemini via OpenAI-compatible API.
"""

from typing import Any
from uuid import UUID

from openai import AsyncOpenAI
from agents import Agent, Runner, function_tool
from agents.models.openai_chatcompletions import OpenAIChatCompletionsModel
from agents.run import RunConfig
from sqlmodel.ext.asyncio.session import AsyncSession

from app.core.config import get_settings
from app.mcp.server import set_context


# System prompt for the Todo Assistant
TODO_AGENT_SYSTEM_PROMPT = """You are a helpful todo task management assistant. Your role is to help users manage their tasks through natural language conversation.

You have access to the following tools:
- add_task_tool: Create a new task with title, optional description, due date, priority, tags, and recurring options
- list_tasks_tool: View all tasks, optionally filtered by status (all, pending, completed), priority, and tags
- complete_task_tool: Mark a task as completed
- delete_task_tool: Remove a task from the list
- update_task_tool: Modify a task's title, description, due date, priority, or tags
- set_task_priority_tool: Set priority level (low, medium, high) for tasks
- add_task_tags_tool: Add tags to organize tasks
- remove_task_tags_tool: Remove tags from tasks
- search_tasks_tool: Find tasks by keywords and filters
- create_recurring_task_pattern_tool: Create tasks that repeat on a schedule (daily, weekly, monthly, yearly)

Guidelines:
1. Be helpful and conversational
2. When creating tasks, extract the title from the user's message
3. When listing tasks, present them in a clear, readable format
4. For destructive actions (delete), confirm the task details before proceeding
5. If a user's request is ambiguous, ask clarifying questions
6. Always confirm actions after they're completed
7. For recurring tasks, ask for the pattern (daily, weekly, monthly, etc.)
8. For priority levels, offer low, medium, or high options
9. For tags, suggest relevant tags based on the task content

Examples of user requests you should handle:
- "Add a task to buy groceries" -> Use add_task_tool with title="Buy groceries"
- "Create a recurring task to water plants every Monday" -> Use create_recurring_task_pattern_tool
- "Show my high priority tasks" -> Use list_tasks_tool with priority filter
- "Find tasks with 'meeting' in the title" -> Use search_tasks_tool
- "Add 'work' tag to this task" -> Use add_task_tags_tool
- "Set this task to high priority" -> Use set_task_priority_tool
- "Remove 'personal' tag from this task" -> Use remove_task_tags_tool
"""


def _create_gemini_client() -> AsyncOpenAI:
    """Create an AsyncOpenAI client configured for Gemini API.

    Returns:
        AsyncOpenAI client pointing to Gemini's OpenAI-compatible endpoint.
    """
    settings = get_settings()
    return AsyncOpenAI(
        api_key=settings.openai_api_key,
        base_url=settings.openai_base_url or "https://generativelanguage.googleapis.com/v1beta/openai/",
    )


def _create_gemini_model() -> OpenAIChatCompletionsModel:
    """Create an OpenAIChatCompletionsModel configured for Gemini.

    Returns:
        OpenAIChatCompletionsModel that uses Gemini via chat completions API.
    """
    settings = get_settings()
    client = _create_gemini_client()

    return OpenAIChatCompletionsModel(
        model=settings.openai_model or "gemini-2.0-flash",
        openai_client=client,
    )


class GeminiTodoAgent:
    """Todo Agent using OpenAI Agents SDK via Chat Completions.

    This implementation uses OpenAIChatCompletionsModel to connect to
    any OpenAI-compatible API (OpenRouter, Gemini, etc.).
    """

    def __init__(self) -> None:
        """Initialize the Todo Agent."""
        pass  # Model created fresh each request to pick up config changes

    def _get_model(self) -> OpenAIChatCompletionsModel:
        """Get the model, creating fresh each time to pick up config changes."""
        return _create_gemini_model()

    async def process_message(
        self,
        message: str,
        user_id: UUID,
        db: AsyncSession,
        conversation_history: list[dict[str, str]] | None = None,
    ) -> dict[str, Any]:
        """Process a user message using Gemini via OpenAI Agents SDK.

        Args:
            message: The user's natural language message.
            user_id: The user's UUID.
            db: Async database session.
            conversation_history: Optional list of previous messages.

        Returns:
            dict containing the agent's response and metadata.
        """
        # Set context for MCP tools (they use global context)
        set_context(db, user_id)

        # Import MCP tool functions
        from app.mcp.server import (
            add_task,
            list_tasks,
            complete_task,
            delete_task,
            update_task,
            set_task_priority,
            add_task_tags,
            remove_task_tags,
            search_tasks,
            create_recurring_task_pattern,
        )

        # Define function tools - must be defined inside the method
        # to capture the current db context
        @function_tool
        async def add_task_tool(
            title: str,
            description: str | None = None,
            due_date: str | None = None,
        ) -> dict:
            """Create a new task for the user.

            Args:
                title: The title of the task (required).
                description: Optional description of the task.
                due_date: Optional due date in YYYY-MM-DD format.
            """
            return await add_task(title, description, due_date)

        @function_tool
        async def list_tasks_tool(status: str = "all") -> list[dict]:
            """List all tasks for the user.

            Args:
                status: Filter by status - 'all', 'pending', or 'completed'.
            """
            return await list_tasks(status)

        @function_tool
        async def complete_task_tool(task_id: str) -> dict:
            """Mark a task as complete.

            Args:
                task_id: The UUID of the task to mark as complete.
            """
            return await complete_task(task_id)

        @function_tool
        async def delete_task_tool(task_id: str) -> dict:
            """Delete a task from the list.

            Args:
                task_id: The UUID of the task to delete.
            """
            return await delete_task(task_id)

        @function_tool
        async def update_task_tool(
            task_id: str,
            title: str | None = None,
            description: str | None = None,
            due_date: str | None = None,
            priority: str | None = None,
            tags: list[str] | None = None,
        ) -> dict:
            """Update a task's title, description, due date, priority, or tags.

            Args:
                task_id: The UUID of the task to update.
                title: New title for the task (optional).
                description: New description for the task (optional).
                due_date: New due date in YYYY-MM-DD format (optional).
                priority: New priority level - "low", "medium", or "high" (optional).
                tags: New list of tags for the task (optional).
            """
            return await update_task(task_id, title, description, due_date, priority, tags)

        @function_tool
        async def set_task_priority_tool(
            task_id: str,
            priority: str,
        ) -> dict:
            """Set the priority of a task.

            Args:
                task_id: The UUID of the task to update.
                priority: Priority level - "low", "medium", or "high".
            """
            return await set_task_priority(task_id, priority)

        @function_tool
        async def add_task_tags_tool(
            task_id: str,
            tags: list[str],
        ) -> dict:
            """Add tags to a task.

            Args:
                task_id: The UUID of the task to update.
                tags: List of tags to add to the task.
            """
            return await add_task_tags(task_id, tags)

        @function_tool
        async def remove_task_tags_tool(
            task_id: str,
            tags: list[str],
        ) -> dict:
            """Remove tags from a task.

            Args:
                task_id: The UUID of the task to update.
                tags: List of tags to remove from the task.
            """
            return await remove_task_tags(task_id, tags)

        @function_tool
        async def search_tasks_tool(
            query: str,
            status: str | None = None,
            priority: str | None = None,
            tags: list[str] | None = None,
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
            """
            return await search_tasks(query, status, priority, tags, due_before, due_after, sort_by, order, page, per_page)

        @function_tool
        async def create_recurring_task_pattern_tool(
            base_task_title: str,
            base_task_description: str | None = None,
            pattern_type: str = "daily",
            interval: int = 1,
            start_date: str | None = None,
            end_date: str | None = None,
            weekdays: list[int] | None = None,
            days_of_month: list[int] | None = None,
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
            """
            return await create_recurring_task_pattern(base_task_title, base_task_description, pattern_type, interval, start_date, end_date, weekdays, days_of_month)

        try:
            # Create agent with model via OpenAIChatCompletionsModel
            agent = Agent(
                name="Todo Assistant",
                instructions=TODO_AGENT_SYSTEM_PROMPT,
                model=self._get_model(),
                tools=[
                    add_task_tool,
                    list_tasks_tool,
                    complete_task_tool,
                    delete_task_tool,
                    update_task_tool,
                    set_task_priority_tool,
                    add_task_tags_tool,
                    remove_task_tags_tool,
                    search_tasks_tool,
                    create_recurring_task_pattern_tool,
                ],
            )

            # Create run config with tracing disabled (avoids OpenAI tracing issues)
            run_config = RunConfig(
                tracing_disabled=True,
            )

            # Build input with conversation history if available
            if conversation_history and len(conversation_history) > 1:
                # Include recent conversation for context (last 10 messages)
                history_text = "\n".join([
                    f"{msg['role'].upper()}: {msg['content']}"
                    for msg in conversation_history[-10:-1]  # Exclude current message
                ])
                input_with_context = f"Previous conversation:\n{history_text}\n\nCurrent message: {message}"
            else:
                input_with_context = message

            # Run the agent with the message
            result = await Runner.run(
                agent,
                input_with_context,
                run_config=run_config,
            )

            # Extract tool calls from result
            tool_calls = []
            if hasattr(result, "new_items"):
                for item in result.new_items:
                    tool_name = None

                    # Try different ways to get tool name
                    if hasattr(item, "name"):
                        tool_name = item.name
                    elif hasattr(item, "tool_name"):
                        tool_name = item.tool_name
                    elif hasattr(item, "raw_item"):
                        raw = item.raw_item
                        if hasattr(raw, "name"):
                            tool_name = raw.name
                        elif hasattr(raw, "function") and hasattr(raw.function, "name"):
                            tool_name = raw.function.name

                    # Check item type
                    item_type = getattr(item, "type", None)
                    if item_type in ("tool_call_item", "function_call_output"):
                        if tool_name:
                            tool_calls.append({
                                "tool": tool_name,
                                "success": True,
                            })
                    elif tool_name and "task" in tool_name.lower():
                        # Fallback: if we found a tool name with "task" in it
                        tool_calls.append({
                            "tool": tool_name,
                            "success": True,
                        })

            return {
                "response": result.final_output or "I processed your request.",
                "tool_calls": tool_calls,
                "success": True,
            }

        except Exception as e:
            import traceback
            print(f"GeminiTodoAgent error: {e}")
            print(traceback.format_exc())
            return {
                "response": "I encountered an error while processing your request. Please try again.",
                "error": str(e),
                "success": False,
            }


# Singleton instance
_todo_agent: GeminiTodoAgent | None = None


def get_todo_agent() -> GeminiTodoAgent:
    """Get the singleton Todo Agent instance.

    Returns:
        GeminiTodoAgent instance configured for Gemini via chat completions.
    """
    global _todo_agent
    if _todo_agent is None:
        _todo_agent = GeminiTodoAgent()
    return _todo_agent
