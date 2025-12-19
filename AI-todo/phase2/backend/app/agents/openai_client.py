"""OpenAI client helper with async streaming support.

Provides a configured AsyncOpenAI client singleton and helper functions
for chat completions with streaming and function calling.
"""

from collections.abc import AsyncGenerator
from functools import lru_cache
from typing import Any

from openai import AsyncOpenAI
from openai.types.chat import (
    ChatCompletionMessageParam,
    ChatCompletionToolParam,
)

from app.core.config import get_settings


@lru_cache
def get_openai_client() -> AsyncOpenAI:
    """Get cached AsyncOpenAI client instance.

    Supports OpenRouter and other OpenAI-compatible APIs via base_url.

    Returns:
        Configured AsyncOpenAI client.

    Raises:
        ValueError: If OPENAI_API_KEY is not configured.
    """
    settings = get_settings()
    if not settings.openai_api_key:
        raise ValueError("OPENAI_API_KEY is not configured")

    # Build client kwargs
    client_kwargs: dict = {
        "api_key": settings.openai_api_key,
        "timeout": settings.openai_timeout,
    }

    # Add base_url for OpenRouter or other providers
    if settings.openai_base_url:
        client_kwargs["base_url"] = settings.openai_base_url

    return AsyncOpenAI(**client_kwargs)


async def chat_completion(
    messages: list[ChatCompletionMessageParam],
    tools: list[ChatCompletionToolParam] | None = None,
    tool_choice: str | dict[str, Any] | None = None,
    stream: bool = False,
) -> str | AsyncGenerator[str, None]:
    """Execute a chat completion request.

    Args:
        messages: List of chat messages.
        tools: Optional list of function tools.
        tool_choice: Optional tool selection strategy.
        stream: Whether to stream the response.

    Returns:
        Complete response string if not streaming,
        otherwise an async generator yielding chunks.
    """
    settings = get_settings()
    client = get_openai_client()

    if stream:
        return _stream_completion(messages, tools, tool_choice)

    response = await client.chat.completions.create(
        model=settings.openai_model,
        messages=messages,
        tools=tools,
        tool_choice=tool_choice,
    )

    content = response.choices[0].message.content
    return content if content else ""


async def _stream_completion(
    messages: list[ChatCompletionMessageParam],
    tools: list[ChatCompletionToolParam] | None = None,
    tool_choice: str | dict[str, Any] | None = None,
) -> AsyncGenerator[str, None]:
    """Stream chat completion response chunks.

    Args:
        messages: List of chat messages.
        tools: Optional list of function tools.
        tool_choice: Optional tool selection strategy.

    Yields:
        Response content chunks as they arrive.
    """
    settings = get_settings()
    client = get_openai_client()

    stream = await client.chat.completions.create(
        model=settings.openai_model,
        messages=messages,
        tools=tools,
        tool_choice=tool_choice,
        stream=True,
    )

    async for chunk in stream:
        if chunk.choices and chunk.choices[0].delta.content:
            yield chunk.choices[0].delta.content


async def function_call_completion(
    messages: list[ChatCompletionMessageParam],
    functions: list[dict[str, Any]],
    function_name: str | None = None,
) -> dict[str, Any]:
    """Execute a chat completion with function calling.

    Args:
        messages: List of chat messages.
        functions: List of function definitions.
        function_name: Optional specific function to call.

    Returns:
        Parsed function call arguments as a dictionary.

    Raises:
        ValueError: If no function call is returned.
    """
    import json

    settings = get_settings()
    client = get_openai_client()

    # Convert functions to tools format
    tools: list[ChatCompletionToolParam] = [
        {"type": "function", "function": func} for func in functions
    ]

    tool_choice: str | dict[str, Any] = "auto"
    if function_name:
        tool_choice = {"type": "function", "function": {"name": function_name}}

    response = await client.chat.completions.create(
        model=settings.openai_model,
        messages=messages,
        tools=tools,
        tool_choice=tool_choice,
    )

    message = response.choices[0].message

    if not message.tool_calls:
        raise ValueError("No function call returned from OpenAI")

    tool_call = message.tool_calls[0]
    return json.loads(tool_call.function.arguments)


# Pre-defined system prompts for agents
INTENT_CLASSIFIER_PROMPT = """You are an intent classifier for a todo task management application.
Analyze the user's message and classify their intent into one of these categories:
- read: User wants to view/list their tasks
- create: User wants to create a new task
- update: User wants to modify an existing task
- delete: User wants to delete a task
- complete: User wants to mark a task as done
- plan: User wants help planning or prioritizing their tasks
- chat: General conversation not related to task operations

Also extract any relevant entities like task titles, descriptions, or due dates."""

TASK_EXTRACTOR_PROMPT = """You are a task data extractor. From the user's message, extract:
- title: The task title (required for create operations)
- description: Optional task description
- due_date: Due date in YYYY-MM-DD format if mentioned

Be concise and extract only what is explicitly mentioned."""

PLANNER_SYSTEM_PROMPT = """You are a productivity assistant helping users plan their tasks.
Given a list of tasks and a user query, provide helpful planning advice such as:
- Daily task schedules
- Weekly planning
- Priority recommendations
- Time management tips

Be concise and actionable in your responses."""
