"""PlannerAgent - Generates task planning and prioritization advice.

Uses LLM to analyze tasks and provide planning recommendations
such as daily schedules, weekly plans, and priority suggestions.
"""

import json
import time

from sqlmodel.ext.asyncio.session import AsyncSession

from app.agents.base import ActionTaken, AgentRequest, AgentResponse, BaseAgent
from app.agents.openai_client import (
    PLANNER_SYSTEM_PROMPT,
    chat_completion,
)
from app.mcp.schemas import ListTasksInput
from app.mcp.tools import TaskTools


class PlannerAgent(BaseAgent):
    """Agent for task planning and prioritization.

    Capabilities:
    - Daily plan generation
    - Weekly planning
    - Priority matrix creation
    - Time management suggestions
    """

    name = "PlannerAgent"

    async def process(
        self,
        request: AgentRequest,
        db: AsyncSession,
    ) -> AgentResponse:
        """Generate a plan based on user's tasks and query.

        Args:
            request: Agent request with planning query.
            db: Database session.

        Returns:
            AgentResponse with generated plan.
        """
        start_time = time.time()

        # First, fetch all user's pending tasks
        result = await TaskTools.list_tasks(
            db=db,
            user_id=request.user_id,
            input_data=ListTasksInput(status="pending", limit=100),
        )

        list_duration = int((time.time() - start_time) * 1000)

        if not result.success:
            return AgentResponse(
                success=False,
                message="Sorry, I couldn't retrieve your tasks for planning.",
                error=result.error,
                actions_taken=[
                    ActionTaken(
                        tool="list_tasks",
                        success=False,
                        summary=result.error or "Failed to list tasks",
                        duration_ms=list_duration,
                    )
                ],
            )

        tasks = result.data.get("tasks", []) if result.data else []

        if not tasks:
            return AgentResponse(
                success=True,
                message="You don't have any pending tasks to plan. Create some tasks first, then I can help you organize them!",
                actions_taken=[
                    ActionTaken(
                        tool="list_tasks",
                        success=True,
                        summary="No pending tasks found",
                        duration_ms=list_duration,
                    )
                ],
            )

        # Generate plan using LLM
        plan_start = time.time()
        try:
            plan = await self._generate_plan(tasks, request.message)
            plan_duration = int((time.time() - plan_start) * 1000)
        except Exception as e:
            return AgentResponse(
                success=False,
                message=f"Sorry, I couldn't generate a plan: {str(e)}",
                error=str(e),
                actions_taken=[
                    ActionTaken(
                        tool="list_tasks",
                        success=True,
                        summary=f"Listed {len(tasks)} tasks",
                        duration_ms=list_duration,
                    )
                ],
            )

        total_duration = int((time.time() - start_time) * 1000)

        return AgentResponse(
            success=True,
            message=plan,
            data={"plan": plan, "task_count": len(tasks)},
            actions_taken=[
                ActionTaken(
                    tool="list_tasks",
                    success=True,
                    summary=f"Analyzed {len(tasks)} tasks",
                    duration_ms=list_duration,
                )
            ],
        )

    async def _generate_plan(self, tasks: list, query: str) -> str:
        """Generate a plan using OpenAI.

        Args:
            tasks: List of user's tasks.
            query: User's planning request.

        Returns:
            Generated plan text.
        """
        # Format tasks for the prompt
        task_summary = json.dumps(
            [
                {
                    "title": t["title"],
                    "description": t.get("description"),
                    "due_date": t.get("due_date"),
                    "status": t["status"],
                }
                for t in tasks
            ],
            indent=2,
        )

        messages = [
            {"role": "system", "content": PLANNER_SYSTEM_PROMPT},
            {
                "role": "user",
                "content": f"Here are my current tasks:\n\n{task_summary}\n\nRequest: {query}",
            },
        ]

        response = await chat_completion(messages=messages, stream=False)

        # Ensure we return a string
        if isinstance(response, str):
            return response
        return str(response)

    async def generate_daily_plan(
        self,
        request: AgentRequest,
        db: AsyncSession,
    ) -> AgentResponse:
        """Generate a daily plan focusing on today's priorities."""
        # Modify request message to focus on daily planning
        request.message = "Help me plan my day. What should I focus on today?"
        return await self.process(request, db)

    async def generate_weekly_plan(
        self,
        request: AgentRequest,
        db: AsyncSession,
    ) -> AgentResponse:
        """Generate a weekly plan with day-by-day breakdown."""
        request.message = "Help me plan my week. Create a day-by-day schedule for my tasks."
        return await self.process(request, db)

    async def generate_priority_matrix(
        self,
        request: AgentRequest,
        db: AsyncSession,
    ) -> AgentResponse:
        """Generate an Eisenhower matrix priority analysis."""
        request.message = (
            "Organize my tasks into an Eisenhower Matrix with four quadrants: "
            "1) Urgent & Important, 2) Important but Not Urgent, "
            "3) Urgent but Not Important, 4) Neither. "
            "Explain which tasks to focus on first."
        )
        return await self.process(request, db)
