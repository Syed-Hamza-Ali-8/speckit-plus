"""Chat API router with POST /chat endpoint.

Provides the main chat endpoint for processing natural
language messages through the OpenAI Agents SDK with MCP tools.
"""

import time
from typing import Literal

from fastapi import APIRouter, HTTPException, Request

from app.agents.base import ChatMessage
from app.agents.todo_agent import get_todo_agent
from app.api.deps import CurrentUser, DbSession
from app.chat.schemas import (
    ActionTakenResponse,
    ChatErrorResponse,
    ChatMetadata,
    ChatRequest,
    ChatResponse,
)
from app.chat.session import get_session_store
from app.core.config import get_settings
from app.middleware.rate_limit import limiter

router = APIRouter(prefix="/chat", tags=["chat"])


@router.post(
    "",
    response_model=ChatResponse,
    responses={
        400: {"model": ChatErrorResponse, "description": "Bad request"},
        401: {"model": ChatErrorResponse, "description": "Unauthorized"},
        429: {"model": ChatErrorResponse, "description": "Rate limited"},
        502: {"model": ChatErrorResponse, "description": "AI API error"},
        504: {"model": ChatErrorResponse, "description": "Tool timeout"},
    },
)
@limiter.limit("60/minute")
async def chat(
    request: Request,
    body: ChatRequest,
    user: CurrentUser,
    db: DbSession,
) -> ChatResponse:
    """Process a chat message using OpenAI Agents SDK with MCP tools.

    This endpoint uses the official OpenAI Agents SDK and MCP (Model Context
    Protocol) to process natural language requests for task management.

    Args:
        request: FastAPI request (for rate limiting).
        body: Chat request body with message and optional session_id.
        user: Authenticated user from JWT.
        db: Database session.

    Returns:
        ChatResponse with AI response and metadata.

    Raises:
        HTTPException: For various error conditions.
    """
    start_time = time.time()
    settings = get_settings()
    session_store = get_session_store()
    todo_agent = get_todo_agent()

    # Get or create session
    session = session_store.get_or_create(
        user_id=user.id,
        session_id=body.session_id,
    )

    # Add user message to session
    user_message = ChatMessage(
        role="user",
        content=body.message,
    )
    session_store.add_message(session, user_message)

    # Build conversation history from session context
    conversation_history = [
        {"role": msg.role, "content": msg.content}
        for msg in session.context.messages[-20:]  # Last 20 messages
    ]

    # Process through Todo Agent (OpenAI Agents SDK with MCP tools)
    try:
        result = await todo_agent.process_message(
            message=body.message,
            user_id=user.id,
            db=db,
            conversation_history=conversation_history,
        )
    except Exception as e:
        # Log error for debugging
        print(f"Agent error: {str(e)}")

        # Check for API errors
        error_str = str(e).lower()
        if "api" in error_str or "openai" in error_str or "gemini" in error_str:
            raise HTTPException(
                status_code=502,
                detail={
                    "error": True,
                    "code": "AI_API_ERROR",
                    "message": "AI service temporarily unavailable. Please try again.",
                },
            )

        # Check for timeout
        if "timeout" in error_str:
            raise HTTPException(
                status_code=504,
                detail={
                    "error": True,
                    "code": "TOOL_TIMEOUT",
                    "message": "The operation took too long. Please try again.",
                },
            )

        # Generic error
        raise HTTPException(
            status_code=500,
            detail={
                "error": True,
                "code": "INTERNAL_ERROR",
                "message": "An unexpected error occurred. Please try again.",
            },
        )

    # Extract response
    response_text = result.get("response", "I couldn't process that request.")

    # Add assistant message to session
    assistant_message = ChatMessage(
        role="assistant",
        content=response_text,
    )
    session_store.add_message(session, assistant_message)

    # Calculate processing time
    processing_time_ms = int((time.time() - start_time) * 1000)

    # Build actions from tool calls
    actions = []
    for tc in result.get("tool_calls", []):
        actions.append(
            ActionTakenResponse(
                tool=tc.get("tool", "unknown"),
                success=tc.get("success", True),
                summary=f"Executed {tc.get('tool', 'tool')}",
            )
        )

    # Determine intent from tool calls or response content
    intent: Literal["read", "create", "update", "delete", "complete", "plan", "chat"] = "chat"

    # First try to detect from tool calls
    if actions:
        tool_name = actions[0].tool.lower()
        if "list" in tool_name:
            intent = "read"
        elif "add" in tool_name or "create" in tool_name:
            intent = "create"
        elif "update" in tool_name:
            intent = "update"
        elif "delete" in tool_name:
            intent = "delete"
        elif "complete" in tool_name:
            intent = "complete"

    # Fallback: detect intent from response text if still "chat"
    if intent == "chat" and response_text:
        response_lower = response_text.lower()
        if any(phrase in response_lower for phrase in ["here are your", "your tasks", "you have", "tasks:"]):
            intent = "read"
        elif any(phrase in response_lower for phrase in ["added", "created", "new task"]):
            intent = "create"
        elif any(phrase in response_lower for phrase in ["marked as complete", "completed", "done"]):
            intent = "complete"
        elif any(phrase in response_lower for phrase in ["deleted", "removed"]):
            intent = "delete"
        elif any(phrase in response_lower for phrase in ["updated", "changed", "modified"]):
            intent = "update"

    # Build response
    return ChatResponse(
        response=response_text,
        session_id=session.session_id,
        intent=intent,
        actions=actions,
        metadata=ChatMetadata(
            processing_time_ms=processing_time_ms,
            agent_chain=["TodoAgent (OpenAI Agents SDK)", "MCP Tools"],
            model=settings.openai_model,
        ),
    )


@router.get("/info")
async def chat_info() -> dict:
    """Get information about the chat endpoint.

    Returns:
        dict with information about the chat system.
    """
    settings = get_settings()
    return {
        "version": "2.0",
        "description": "Chat endpoint using OpenAI Agents SDK with MCP tools",
        "model": settings.openai_model,
        "features": [
            "Official MCP SDK integration",
            "OpenAI Agents SDK",
            "Function tools for task operations",
            "Session-based conversation history",
        ],
        "tools": [
            "add_task",
            "list_tasks",
            "complete_task",
            "delete_task",
            "update_task",
        ],
    }
