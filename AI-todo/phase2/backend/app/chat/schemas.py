"""Chat request/response schemas.

Defines Pydantic models for chat API request validation
and response serialization.
"""

from typing import Literal
from uuid import UUID

from pydantic import BaseModel, Field


class ChatRequest(BaseModel):
    """Schema for chat request body.

    Attributes:
        message: User's natural language message.
        session_id: Optional session ID for context continuity.
    """

    message: str = Field(
        ...,
        min_length=1,
        max_length=2000,
        description="User's natural language message",
    )
    session_id: UUID | None = Field(
        default=None,
        description="Optional session ID for context continuity",
    )


class ActionTakenResponse(BaseModel):
    """Schema for an executed MCP tool action.

    Attributes:
        tool: Name of the MCP tool invoked.
        success: Whether the tool succeeded.
        summary: Human-readable result summary.
    """

    tool: str
    success: bool
    summary: str


class ChatMetadata(BaseModel):
    """Schema for chat response metadata.

    Attributes:
        processing_time_ms: Total processing time in milliseconds.
        agent_chain: List of agents involved in processing.
        model: LLM model used for generation.
    """

    processing_time_ms: int
    agent_chain: list[str]
    model: str


class ChatResponse(BaseModel):
    """Schema for chat response body.

    Attributes:
        response: AI-generated response text.
        session_id: Session ID for follow-up messages.
        intent: Detected user intent.
        actions: List of MCP tools invoked.
        metadata: Processing metadata.
    """

    response: str
    session_id: UUID
    intent: Literal["read", "create", "update", "delete", "complete", "plan", "chat"]
    actions: list[ActionTakenResponse]
    metadata: ChatMetadata


class ChatErrorResponse(BaseModel):
    """Schema for chat error response.

    Attributes:
        error: Always True for error responses.
        code: Machine-readable error code.
        message: Human-readable error message.
    """

    error: bool = True
    code: str
    message: str


# WebSocket message schemas
class WSIncomingMessage(BaseModel):
    """Schema for WebSocket messages from client.

    Attributes:
        type: Message type ('message' or 'ping').
        content: Message content for 'message' type.
        session_id: Optional session ID.
    """

    type: Literal["message", "ping"]
    content: str | None = None
    session_id: UUID | None = None


class WSOutgoingMessage(BaseModel):
    """Schema for WebSocket messages to client.

    Attributes:
        type: Message type.
        content: Message content.
        metadata: Additional metadata.
    """

    type: Literal["token", "tool_start", "tool_end", "complete", "error", "pong"]
    content: str
    metadata: dict | None = None
