"""GuardrailAgent - Safety layer for destructive actions.

Validates and gates destructive operations, requiring user
confirmation before proceeding with delete operations.
"""

from datetime import datetime, timedelta

from sqlmodel.ext.asyncio.session import AsyncSession

from app.agents.base import (
    AgentRequest,
    AgentResponse,
    BaseAgent,
    Confirmation,
)


class GuardrailAgent(BaseAgent):
    """Safety agent for validating destructive actions.

    Responsibilities:
    - Identify destructive intents (delete, bulk operations)
    - Require explicit user confirmation
    - Gate execution until confirmation received
    - Handle confirmation timeout
    """

    name = "GuardrailAgent"

    # Intents that require confirmation
    DESTRUCTIVE_INTENTS = {"delete"}

    # Confirmation phrases that count as "yes"
    CONFIRM_PHRASES = {
        "yes",
        "y",
        "yeah",
        "yep",
        "sure",
        "ok",
        "okay",
        "confirm",
        "do it",
        "go ahead",
        "proceed",
        "delete it",
        "remove it",
    }

    # Denial phrases that count as "no"
    DENY_PHRASES = {
        "no",
        "n",
        "nope",
        "cancel",
        "stop",
        "don't",
        "dont",
        "nevermind",
        "never mind",
        "wait",
        "hold on",
        "abort",
    }

    async def process(
        self,
        request: AgentRequest,
        db: AsyncSession,
    ) -> AgentResponse:
        """Validate request and check for destructive actions.

        Args:
            request: Agent request with intent.
            db: Database session.

        Returns:
            AgentResponse allowing or blocking the action.
        """
        # Check if this is a confirmation response to a pending action
        if request.context and request.context.pending_confirmation:
            return await self._handle_pending_confirmation(request)

        # Check if intent requires confirmation
        if request.intent in self.DESTRUCTIVE_INTENTS:
            return await self._request_confirmation(request)

        # Non-destructive action - allow to proceed
        return AgentResponse(
            success=True,
            message="",
            data={"allowed": True, "requires_confirmation": False},
        )

    async def validate_and_execute(
        self,
        request: AgentRequest,
        db: AsyncSession,
        task_manager,  # Type hint omitted to avoid circular import
    ) -> AgentResponse:
        """Validate the action and execute through TaskManagerAgent if allowed.

        Args:
            request: Agent request.
            db: Database session.
            task_manager: TaskManagerAgent instance.

        Returns:
            AgentResponse from validation or task execution.
        """
        # First, validate through guardrails
        validation = await self.process(request, db)

        # If confirmation is required, return that response
        if validation.requires_confirmation:
            return validation

        # If explicitly blocked or cancelled
        if validation.data and validation.data.get("cancelled"):
            return validation

        # If allowed, pass through to task manager
        if validation.success and validation.data and validation.data.get("allowed"):
            return await task_manager.process(request, db)

        # If confirmed, execute the pending action
        if (
            validation.success
            and validation.data
            and validation.data.get("confirmed")
        ):
            return await task_manager.process(request, db)

        return validation

    async def _handle_pending_confirmation(
        self,
        request: AgentRequest,
    ) -> AgentResponse:
        """Handle a message when there's a pending confirmation.

        Args:
            request: Agent request with pending confirmation.

        Returns:
            AgentResponse based on user's confirmation choice.
        """
        if not request.context or not request.context.pending_confirmation:
            return AgentResponse(
                success=True,
                message="",
                data={"allowed": True},
            )

        confirmation = request.context.pending_confirmation

        # Check if confirmation has expired
        if datetime.utcnow() > confirmation.expires_at:
            request.context.pending_confirmation = None
            return AgentResponse(
                success=True,
                message="The confirmation has expired. Please try your request again.",
                data={"expired": True},
            )

        message_lower = request.message.lower().strip()

        # Check for confirmation
        if message_lower in self.CONFIRM_PHRASES or any(
            phrase in message_lower for phrase in self.CONFIRM_PHRASES
        ):
            # Keep confirmation for TaskManagerAgent to process
            return AgentResponse(
                success=True,
                message="",
                data={"confirmed": True, "allowed": True},
            )

        # Check for denial
        if message_lower in self.DENY_PHRASES or any(
            phrase in message_lower for phrase in self.DENY_PHRASES
        ):
            # Clear the pending confirmation
            request.context.pending_confirmation = None
            return AgentResponse(
                success=True,
                message="Okay, I've cancelled that action. Your task is safe!",
                data={"cancelled": True},
            )

        # Unclear response - ask again
        return AgentResponse(
            success=True,
            message="I need a clear answer. Please reply 'yes' to confirm the deletion, or 'no' to cancel.",
            requires_confirmation=True,
            confirmation_prompt=confirmation.prompt,
        )

    async def _request_confirmation(
        self,
        request: AgentRequest,
    ) -> AgentResponse:
        """Request user confirmation for a destructive action.

        Args:
            request: Agent request with destructive intent.

        Returns:
            AgentResponse requesting confirmation.
        """
        # Determine the action type and target
        action_descriptions = {
            "delete": "delete this task",
        }

        action_desc = action_descriptions.get(request.intent, "perform this action")
        prompt = f"Are you sure you want to {action_desc}? This cannot be undone. Reply 'yes' to confirm or 'no' to cancel."

        # Set pending confirmation in context
        if request.context:
            target_ids = (
                request.context.last_task_ids if request.context.last_task_ids else []
            )
            request.context.pending_confirmation = Confirmation(
                action=request.intent,  # type: ignore[arg-type]
                target_ids=target_ids,
                prompt=prompt,
                expires_at=datetime.utcnow() + timedelta(minutes=5),
            )

        return AgentResponse(
            success=True,
            message=prompt,
            requires_confirmation=True,
            confirmation_prompt=prompt,
        )

    def clear_confirmation(self, request: AgentRequest) -> None:
        """Clear any pending confirmation from the session.

        Args:
            request: Agent request with context.
        """
        if request.context:
            request.context.pending_confirmation = None
