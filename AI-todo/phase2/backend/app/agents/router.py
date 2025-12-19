"""AgentRouter - Orchestrates multi-agent flow for chat processing.

Central coordinator that routes requests through the appropriate
agent pipeline based on classified intent.
"""

from sqlmodel.ext.asyncio.session import AsyncSession

from app.agents.base import AgentRequest, AgentResponse
from app.agents.conversation import ConversationAgent
from app.agents.guardrails import GuardrailAgent
from app.agents.planner import PlannerAgent
from app.agents.task_manager import TaskManagerAgent


class AgentRouter:
    """Router for orchestrating multi-agent chat processing.

    Flow:
    1. ConversationAgent classifies intent
    2. GuardrailAgent validates destructive actions
    3. TaskManagerAgent or PlannerAgent executes
    4. Response returned to caller
    """

    def __init__(self) -> None:
        """Initialize all sub-agents."""
        self.conversation = ConversationAgent()
        self.guardrails = GuardrailAgent()
        self.task_manager = TaskManagerAgent()
        self.planner = PlannerAgent()

    async def route(
        self,
        request: AgentRequest,
        db: AsyncSession,
    ) -> AgentResponse:
        """Route a request through the agent pipeline.

        Args:
            request: Agent request with user message.
            db: Database session.

        Returns:
            Final AgentResponse from the appropriate agent.
        """
        # Track agent chain for metadata
        agent_chain = [self.conversation.name]

        # Step 1: Classify intent via ConversationAgent
        conv_response = await self.conversation.process(request, db)

        # If ConversationAgent returned a direct response (chat, error, confirmation)
        if conv_response.message:
            conv_response.data = conv_response.data or {}
            conv_response.data["agent_chain"] = agent_chain
            return conv_response

        # Get classified intent
        intent = request.intent or (
            conv_response.data.get("intent") if conv_response.data else "chat"
        )
        request.intent = intent

        # Step 2: Route based on intent
        if intent == "plan":
            # Planning requests go directly to PlannerAgent
            agent_chain.append(self.planner.name)
            response = await self.planner.process(request, db)
            response.data = response.data or {}
            response.data["agent_chain"] = agent_chain
            return response

        if intent == "chat":
            # Chat was already handled by ConversationAgent
            agent_chain.append("DirectResponse")
            return conv_response

        # Step 3: Task operations go through guardrails
        agent_chain.append(self.guardrails.name)

        # For destructive actions, check guardrails first
        if intent in self.guardrails.DESTRUCTIVE_INTENTS:
            guardrail_response = await self.guardrails.process(request, db)

            # If confirmation is required, return that
            if guardrail_response.requires_confirmation:
                guardrail_response.data = guardrail_response.data or {}
                guardrail_response.data["agent_chain"] = agent_chain
                return guardrail_response

            # If cancelled, return cancellation message
            if guardrail_response.data and guardrail_response.data.get("cancelled"):
                guardrail_response.data["agent_chain"] = agent_chain
                return guardrail_response

            # If not allowed for some reason
            if (
                guardrail_response.data
                and not guardrail_response.data.get("allowed")
                and not guardrail_response.data.get("confirmed")
            ):
                guardrail_response.data["agent_chain"] = agent_chain
                return guardrail_response

        # Step 4: Execute via TaskManagerAgent
        agent_chain.append(self.task_manager.name)
        task_response = await self.task_manager.process(request, db)

        # If task manager requires confirmation (for delete)
        if task_response.requires_confirmation:
            # Set up pending confirmation in context
            if request.context and request.context.last_task_ids:
                self.conversation.set_pending_confirmation(
                    request=request,
                    action=intent,
                    target_ids=request.context.last_task_ids,
                    prompt=task_response.confirmation_prompt or "",
                )

        task_response.data = task_response.data or {}
        task_response.data["agent_chain"] = agent_chain
        return task_response

    async def route_with_context(
        self,
        request: AgentRequest,
        db: AsyncSession,
    ) -> tuple[AgentResponse, list[str]]:
        """Route request and return response with agent chain.

        Args:
            request: Agent request.
            db: Database session.

        Returns:
            Tuple of (AgentResponse, agent_chain list).
        """
        response = await self.route(request, db)
        agent_chain = (
            response.data.get("agent_chain", [])
            if response.data
            else [self.conversation.name]
        )
        return response, agent_chain


# Singleton instance
_router: AgentRouter | None = None


def get_agent_router() -> AgentRouter:
    """Get singleton AgentRouter instance.

    Returns:
        AgentRouter instance.
    """
    global _router
    if _router is None:
        _router = AgentRouter()
    return _router
