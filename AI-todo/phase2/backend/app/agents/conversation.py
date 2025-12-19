"""ConversationAgent - Handles intent classification and message routing.

Entry point for all chat messages. Classifies user intent using
OpenAI function calling and routes to appropriate sub-agents.
"""

from datetime import datetime, timedelta

from sqlmodel.ext.asyncio.session import AsyncSession

from app.agents.base import AgentRequest, AgentResponse, BaseAgent, Confirmation
from app.agents.schemas import SYSTEM_PROMPTS


class ConversationAgent(BaseAgent):
    """Agent for intent classification and conversation routing.

    Responsibilities:
    - Classify user intent (read, create, update, delete, complete, plan, chat)
    - Extract task-related entities
    - Handle general conversation
    - Detect confirmation responses
    """

    name = "ConversationAgent"

    async def process(
        self,
        request: AgentRequest,
        db: AsyncSession,
    ) -> AgentResponse:
        """Process incoming message and classify intent.

        Args:
            request: Agent request with user message.
            db: Database session (passed through for sub-agents).

        Returns:
            AgentResponse with classified intent, or response for chat intent.
        """
        # Check if there's a pending confirmation
        if request.context and request.context.pending_confirmation:
            return await self._handle_confirmation_check(request)

        # Classify intent using OpenAI function calling
        try:
            classification = await self._classify_intent(request.message)
        except Exception as e:
            return AgentResponse(
                success=False,
                message="I'm having trouble understanding that. Could you rephrase?",
                error=f"Intent classification failed: {str(e)}",
            )

        intent = classification.get("intent", "chat")
        confidence = classification.get("confidence", 0.5)

        # Update request with classified intent
        request.intent = intent

        # Store extracted data in context if available
        if request.context:
            request.context.last_intent = intent
            # Store extracted task data for create/update operations
            extracted_data = {
                "title": classification.get("task_title"),
                "description": classification.get("task_description"),
                "due_date": self._parse_due_date(classification.get("due_date")),
            }
            # Filter out None values
            request.context.extracted_data = {k: v for k, v in extracted_data.items() if v}

        # Handle chat intent directly
        if intent == "chat":
            return self._handle_chat(request.message)

        # For task operations, return with classified intent for routing
        return AgentResponse(
            success=True,
            message="",
            data={
                "intent": intent,
                "confidence": confidence,
                "extracted_data": request.context.extracted_data
                if request.context and request.context.extracted_data
                else {},
            },
        )

    async def _classify_intent(self, message: str) -> dict:
        """Classify user intent using LLM.

        Args:
            message: User's message text.

        Returns:
            Classification result with intent and extracted entities.
        """
        import json

        classification_prompt = f"""{SYSTEM_PROMPTS["intent_classifier"]}

User message: "{message}"

Respond with ONLY a JSON object (no markdown, no explanation) in this exact format:
{{"intent": "one of: read, create, update, delete, complete, plan, chat", "confidence": 0.0 to 1.0, "task_title": "extracted title if any", "task_description": "extracted description if any", "due_date": "YYYY-MM-DD if mentioned"}}

JSON response:"""

        messages = [
            {"role": "user", "content": classification_prompt},
        ]

        try:
            from app.agents.openai_client import chat_completion
            response = await chat_completion(messages=messages, stream=False)

            # Parse JSON response
            response_text = response.strip() if isinstance(response, str) else ""

            # Clean up markdown code blocks if present
            if response_text.startswith("```"):
                lines = response_text.split("\n")
                response_text = "\n".join(
                    line for line in lines
                    if not line.startswith("```")
                )

            result = json.loads(response_text)
            return result
        except (json.JSONDecodeError, Exception) as e:
            # Fallback: simple keyword-based classification
            message_lower = message.lower()

            if any(w in message_lower for w in ["show", "list", "what", "view", "see", "my task"]):
                return {"intent": "read", "confidence": 0.7}
            elif any(w in message_lower for w in ["add", "create", "new", "remind", "need to"]):
                return {"intent": "create", "confidence": 0.7, "task_title": message}
            elif any(w in message_lower for w in ["delete", "remove", "get rid"]):
                return {"intent": "delete", "confidence": 0.7}
            elif any(w in message_lower for w in ["done", "complete", "finish", "mark"]):
                return {"intent": "complete", "confidence": 0.7}
            elif any(w in message_lower for w in ["update", "change", "edit", "modify"]):
                return {"intent": "update", "confidence": 0.7}
            elif any(w in message_lower for w in ["plan", "prioritize", "schedule", "organize"]):
                return {"intent": "plan", "confidence": 0.7}
            else:
                return {"intent": "chat", "confidence": 0.8}

    async def _handle_confirmation_check(
        self,
        request: AgentRequest,
    ) -> AgentResponse:
        """Check if user is confirming or denying a pending action.

        Uses simple keyword matching for reliability with any LLM backend.

        Args:
            request: Agent request with pending confirmation.

        Returns:
            AgentResponse indicating confirmation status.
        """
        message_lower = request.message.lower().strip()

        # Confirmation keywords
        confirm_words = ["yes", "yeah", "yep", "yup", "sure", "ok", "okay", "confirm", "do it", "proceed", "go ahead", "y"]
        # Denial keywords
        deny_words = ["no", "nope", "nah", "cancel", "stop", "don't", "dont", "never", "n"]

        is_confirmation = any(word in message_lower for word in confirm_words)
        is_denial = any(word in message_lower for word in deny_words)

        # If both detected, prioritize denial for safety
        if is_confirmation and is_denial:
            is_confirmation = False

        if is_confirmation:
            # Keep the pending confirmation and signal to proceed
            return AgentResponse(
                success=True,
                message="",
                data={"confirmed": True, "proceed_with_action": True},
            )
        elif is_denial:
            # Clear the pending confirmation
            if request.context:
                request.context.pending_confirmation = None
            return AgentResponse(
                success=True,
                message="Okay, I've cancelled that action.",
                data={"confirmed": False, "cancelled": True},
            )
        else:
            # Unclear response, ask again
            return AgentResponse(
                success=True,
                message="I didn't catch that. Please reply 'yes' to confirm or 'no' to cancel.",
                requires_confirmation=True,
            )

    def _handle_chat(self, message: str) -> AgentResponse:
        """Handle general chat messages.

        Args:
            message: User's message.

        Returns:
            AgentResponse with conversational reply.
        """
        message_lower = message.lower().strip()

        # Greeting responses
        greetings = ["hi", "hello", "hey", "howdy", "greetings"]
        if any(g in message_lower for g in greetings):
            return AgentResponse(
                success=True,
                message="Hello! I'm your task assistant. I can help you manage your todos - just ask me to show your tasks, create new ones, or help you plan your day!",
            )

        # Thanks responses
        thanks = ["thank", "thanks", "thx", "appreciate"]
        if any(t in message_lower for t in thanks):
            return AgentResponse(
                success=True,
                message="You're welcome! Let me know if you need anything else.",
            )

        # Help requests
        help_words = ["help", "how", "what can you"]
        if any(h in message_lower for h in help_words):
            return AgentResponse(
                success=True,
                message="""I can help you with:
- **View tasks**: "Show my tasks", "What do I have to do?"
- **Create tasks**: "Add a task to buy groceries", "Remind me to call mom tomorrow"
- **Complete tasks**: "Mark the first task as done", "I finished buying groceries"
- **Delete tasks**: "Delete that task", "Remove the groceries task"
- **Plan**: "Help me plan my day", "What should I focus on?"

Just type naturally and I'll understand!""",
            )

        # Default response
        return AgentResponse(
            success=True,
            message="I'm not sure what you mean. Try asking me to show your tasks, create a new task, or help you plan your day!",
        )

    def _parse_due_date(self, date_str: str | None) -> str | None:
        """Parse due date string to YYYY-MM-DD format.

        Args:
            date_str: Date string (may be relative like 'tomorrow').

        Returns:
            Date in YYYY-MM-DD format or None.
        """
        if not date_str:
            return None

        date_lower = date_str.lower().strip()
        today = datetime.now().date()

        # Handle relative dates
        if date_lower == "today":
            return today.isoformat()
        elif date_lower == "tomorrow":
            return (today + timedelta(days=1)).isoformat()
        elif date_lower == "next week":
            return (today + timedelta(weeks=1)).isoformat()
        elif date_lower == "next month":
            return (today + timedelta(days=30)).isoformat()

        # Try to parse as ISO date
        try:
            datetime.strptime(date_str, "%Y-%m-%d")
            return date_str
        except ValueError:
            pass

        return None

    def set_pending_confirmation(
        self,
        request: AgentRequest,
        action: str,
        target_ids: list,
        prompt: str,
    ) -> None:
        """Set a pending confirmation in the session context.

        Args:
            request: Agent request with context.
            action: The action requiring confirmation ('delete', 'bulk_update').
            target_ids: IDs of entities affected.
            prompt: Confirmation prompt shown to user.
        """
        if request.context:
            request.context.pending_confirmation = Confirmation(
                action=action,  # type: ignore[arg-type]
                target_ids=target_ids,
                prompt=prompt,
                expires_at=datetime.utcnow() + timedelta(minutes=5),
            )
