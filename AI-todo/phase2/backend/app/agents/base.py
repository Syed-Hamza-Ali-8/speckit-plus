"""Base agent classes and shared schemas.

Defines the core abstractions for the multi-agent architecture:
- AgentRequest: Input to agent processing
- AgentResponse: Output from agent processing
- ActionTaken: Record of MCP tool invocation
- SessionContext: Conversation context for continuity
- BaseAgent: Abstract base class for all agents
"""

from abc import ABC, abstractmethod
from datetime import datetime
from typing import Literal
from uuid import UUID

from pydantic import BaseModel, Field
from sqlmodel.ext.asyncio.session import AsyncSession


class ActionTaken(BaseModel):
    """Record of an MCP tool invocation during agent processing.

    Attributes:
        tool: Name of the MCP tool invoked.
        success: Whether the tool execution succeeded.
        summary: Human-readable summary of the result.
        duration_ms: Execution time in milliseconds (optional).
    """

    tool: str
    success: bool
    summary: str
    duration_ms: int | None = None


class Confirmation(BaseModel):
    """Pending destructive action awaiting user confirmation.

    Attributes:
        action: Type of destructive action.
        target_ids: IDs of entities to be affected.
        prompt: Confirmation message shown to user.
        expires_at: Auto-clear time for the confirmation.
    """

    action: Literal["delete", "bulk_update"]
    target_ids: list[UUID]
    prompt: str
    expires_at: datetime


class ChatMessage(BaseModel):
    """Individual message in conversation history.

    Attributes:
        role: Message sender (user or assistant).
        content: Message text content.
        timestamp: When the message was sent.
    """

    role: Literal["user", "assistant"]
    content: str
    timestamp: datetime = Field(default_factory=datetime.utcnow)


class SessionContext(BaseModel):
    """Conversation context within a session.

    Maintains state across multiple messages in a conversation
    for context resolution and confirmation flows.

    Attributes:
        messages: Rolling history of recent messages.
        last_intent: Most recent classified intent.
        last_task_ids: Recently referenced task IDs.
        pending_confirmation: Awaiting user confirmation for destructive action.
        extracted_data: Task data extracted from messages.
    """

    messages: list[ChatMessage] = Field(default_factory=list)
    last_intent: str | None = None
    last_task_ids: list[UUID] = Field(default_factory=list)
    pending_confirmation: Confirmation | None = None
    extracted_data: dict | None = None


class AgentRequest(BaseModel):
    """Input to agent processing pipeline.

    Attributes:
        intent: Classified intent (may be empty before classification).
        user_id: Authenticated user's ID.
        session_id: Session ID for context tracking.
        message: User's natural language message.
        context: Conversation context from session.
    """

    intent: str
    user_id: UUID
    session_id: UUID
    message: str
    context: SessionContext | None = None


class AgentResponse(BaseModel):
    """Output from agent processing.

    Attributes:
        success: Whether the agent completed successfully.
        message: Response message to display to user.
        data: Additional response data (varies by agent).
        error: Error message if failed.
        actions_taken: List of MCP tools invoked.
        requires_confirmation: Whether user confirmation is needed.
        confirmation_prompt: Prompt to display for confirmation.
    """

    success: bool
    message: str = ""
    data: dict | None = None
    error: str | None = None
    actions_taken: list[ActionTaken] = Field(default_factory=list)
    requires_confirmation: bool = False
    confirmation_prompt: str | None = None


class BaseAgent(ABC):
    """Abstract base class for all agents.

    Defines the contract that all agents must implement:
    - name: Unique identifier for the agent
    - process: Main processing method

    Subclasses should implement domain-specific logic
    while maintaining the standard request/response interface.
    """

    name: str = "BaseAgent"

    @abstractmethod
    async def process(
        self,
        request: AgentRequest,
        db: AsyncSession,
    ) -> AgentResponse:
        """Process an agent request and return a response.

        Args:
            request: Input request with user message and context.
            db: Async database session for data operations.

        Returns:
            AgentResponse with result data or error.
        """
        pass
