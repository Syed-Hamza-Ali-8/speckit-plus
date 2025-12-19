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
- add_task_tool: Create a new task with title, optional description, and due date
- list_tasks_tool: View all tasks, optionally filtered by status (all, pending, completed)
- complete_task_tool: Mark a task as completed
- delete_task_tool: Remove a task from the list
- update_task_tool: Modify a task's title, description, or due date

Guidelines:
1. Be helpful and conversational
2. When creating tasks, extract the title from the user's message
3. When listing tasks, present them in a clear, readable format
4. For destructive actions (delete), confirm the task details before proceeding
5. If a user's request is ambiguous, ask clarifying questions
6. Always confirm actions after they're completed

Examples of user requests you should handle:
- "Add a task to buy groceries" -> Use add_task_tool with title="Buy groceries"
- "Show my tasks" -> Use list_tasks_tool with status="all"
- "What's pending?" -> Use list_tasks_tool with status="pending"
- "Mark task X as done" -> Use complete_task_tool
- "Delete the groceries task" -> First list tasks to find it, then delete_task_tool
- "Change task X to Y" -> Use update_task_tool
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
        ) -> dict:
            """Update a task's title, description, or due date.

            Args:
                task_id: The UUID of the task to update.
                title: New title for the task (optional).
                description: New description for the task (optional).
                due_date: New due date in YYYY-MM-DD format (optional).
            """
            return await update_task(task_id, title, description, due_date)

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
