"""Todo Agent implementation using OpenAI with MCP tools.

This module creates an AI agent that uses MCP (Model Context Protocol) tools
to manage user's todo tasks through natural language conversation.
"""

import json
import os
from typing import Any, Dict
from uuid import UUID

from openai import OpenAI
from sqlmodel import Session

from mcp_tools.server import mcp, set_context


# System prompt for the Todo Assistant
TODO_AGENT_SYSTEM_PROMPT = """You are a friendly task management assistant.

Tools available:
- add_task: Create tasks
- list_tasks: View tasks
- update_task_by_name, delete_task_by_name, complete_task_by_name: Modify tasks by name (preferred)
- update_task, delete_task, complete_task: Modify tasks by UUID (fallback)
- create_recurring_task, list_recurring_tasks, delete_recurring_task_by_name, update_recurring_task_by_name: Manage recurring tasks

Key rules:
1. Use *_by_name tools when user provides task names (e.g., "Update Hello World")
2. Use UUID tools only when user provides UUIDs
3. For recurring tasks: daily/weekly/monthly patterns, parse days/weekdays/dates from user input
4. When multiple tasks match, list them and ask which one
5. If you asked a clarification question, the next message is likely the answer
6. Confirm actions after completion"""


# Convert MCP tools to OpenAI function calling format
def get_openai_tools() -> list[dict]:
    """Get MCP tools in OpenAI function calling format."""
    return [
        {
            "type": "function",
            "function": {
                "name": "add_task",
                "description": "Create a new task for the user",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "title": {
                            "type": "string",
                            "description": "The title of the task"
                        },
                        "description": {
                            "type": "string",
                            "description": "Optional description of the task"
                        },
                        "priority": {
                            "type": "string",
                            "enum": ["low", "medium", "high"],
                            "description": "Priority level of the task"
                        }
                    },
                    "required": ["title"]
                }
            }
        },
        {
            "type": "function",
            "function": {
                "name": "list_tasks",
                "description": "Get a list of the user's tasks",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "status": {
                            "type": "string",
                            "enum": ["all", "pending", "completed"],
                            "description": "Filter tasks by status"
                        },
                        "priority": {
                            "type": "string",
                            "enum": ["low", "medium", "high"],
                            "description": "Filter tasks by priority"
                        }
                    },
                    "required": []
                }
            }
        },
        {
            "type": "function",
            "function": {
                "name": "complete_task",
                "description": "Mark a task as completed",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "task_id": {
                            "type": "string",
                            "description": "The UUID of the task to complete"
                        }
                    },
                    "required": ["task_id"]
                }
            }
        },
        {
            "type": "function",
            "function": {
                "name": "delete_task",
                "description": "Delete a task from the list",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "task_id": {
                            "type": "string",
                            "description": "The UUID of the task to delete"
                        }
                    },
                    "required": ["task_id"]
                }
            }
        },
        {
            "type": "function",
            "function": {
                "name": "update_task",
                "description": "Update a task's title, description, or priority",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "task_id": {
                            "type": "string",
                            "description": "The UUID of the task to update"
                        },
                        "title": {
                            "type": "string",
                            "description": "New title for the task"
                        },
                        "description": {
                            "type": "string",
                            "description": "New description for the task"
                        },
                        "priority": {
                            "type": "string",
                            "enum": ["low", "medium", "high"],
                            "description": "New priority level"
                        }
                    },
                    "required": ["task_id"]
                }
            }
        },
        {
            "type": "function",
            "function": {
                "name": "update_task_by_name",
                "description": "Update a task by its name (automatically finds the task). Use this instead of update_task when you know the task name but not the UUID.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "task_name": {
                            "type": "string",
                            "description": "The name/title of the task to update"
                        },
                        "new_description": {
                            "type": "string",
                            "description": "New description for the task"
                        },
                        "new_title": {
                            "type": "string",
                            "description": "New title for the task"
                        },
                        "new_priority": {
                            "type": "string",
                            "enum": ["low", "medium", "high"],
                            "description": "New priority level"
                        }
                    },
                    "required": ["task_name"]
                }
            }
        },
        {
            "type": "function",
            "function": {
                "name": "delete_task_by_name",
                "description": "Delete a task by its name (automatically finds the task). Use this instead of delete_task when you know the task name but not the UUID.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "task_name": {
                            "type": "string",
                            "description": "The name/title of the task to delete"
                        }
                    },
                    "required": ["task_name"]
                }
            }
        },
        {
            "type": "function",
            "function": {
                "name": "complete_task_by_name",
                "description": "Mark a task as complete by its name (automatically finds the task). Use this instead of complete_task when you know the task name but not the UUID.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "task_name": {
                            "type": "string",
                            "description": "The name/title of the task to complete"
                        }
                    },
                    "required": ["task_name"]
                }
            }
        },
        {
            "type": "function",
            "function": {
                "name": "create_recurring_task",
                "description": "Create a recurring task pattern that generates tasks automatically on a schedule (daily, weekly, or monthly).",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "title": {
                            "type": "string",
                            "description": "The title for tasks generated from this pattern"
                        },
                        "pattern_type": {
                            "type": "string",
                            "enum": ["daily", "weekly", "monthly"],
                            "description": "Type of recurrence: daily, weekly, or monthly"
                        },
                        "start_date": {
                            "type": "string",
                            "description": "Start date in YYYY-MM-DD format (e.g., 2026-01-09)"
                        },
                        "description": {
                            "type": "string",
                            "description": "Optional description for generated tasks"
                        },
                        "interval": {
                            "type": "integer",
                            "description": "Interval between occurrences (default: 1)"
                        },
                        "end_date": {
                            "type": "string",
                            "description": "Optional end date in YYYY-MM-DD format"
                        },
                        "weekdays": {
                            "type": "string",
                            "description": "For weekly patterns: comma-separated day names (e.g., 'Monday,Wednesday,Friday')"
                        },
                        "days_of_month": {
                            "type": "string",
                            "description": "For monthly patterns: comma-separated day numbers (e.g., '1,15,30')"
                        }
                    },
                    "required": ["title", "pattern_type", "start_date"]
                }
            }
        },
        {
            "type": "function",
            "function": {
                "name": "list_recurring_tasks",
                "description": "List all recurring task patterns for the user.",
                "parameters": {
                    "type": "object",
                    "properties": {}
                }
            }
        },
        {
            "type": "function",
            "function": {
                "name": "delete_recurring_task_by_name",
                "description": "Delete a recurring task pattern by its name.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "pattern_name": {
                            "type": "string",
                            "description": "The title/name of the recurring pattern to delete"
                        }
                    },
                    "required": ["pattern_name"]
                }
            }
        },
        {
            "type": "function",
            "function": {
                "name": "update_recurring_task_by_name",
                "description": "Update a recurring task pattern by its name.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "pattern_name": {
                            "type": "string",
                            "description": "The title/name of the recurring pattern to update"
                        },
                        "new_title": {
                            "type": "string",
                            "description": "New title for the pattern"
                        },
                        "new_description": {
                            "type": "string",
                            "description": "New description for generated tasks"
                        },
                        "new_interval": {
                            "type": "integer",
                            "description": "New interval between occurrences"
                        },
                        "new_end_date": {
                            "type": "string",
                            "description": "New end date in YYYY-MM-DD format"
                        }
                    },
                    "required": ["pattern_name"]
                }
            }
        }
    ]


class TodoAgent:
    """Todo Agent using OpenAI with MCP tools."""

    def __init__(self) -> None:
        """Initialize the Todo Agent."""
        self.openai_client = OpenAI(
            api_key=os.getenv("OPENAI_API_KEY"),
            base_url=os.getenv("OPENAI_BASE_URL")
        )
        self.model = os.getenv("OPENAI_MODEL", "gpt-3.5-turbo")

    async def process_message(
        self,
        message: str,
        user_id: UUID,
        db: Session,
        conversation_history: list[dict[str, str]] | None = None,
        awaiting_clarification: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        """Process a user message using OpenAI with MCP tools.

        Args:
            message: The user's natural language message.
            user_id: The user's UUID.
            db: Database session.
            conversation_history: Optional list of previous messages.
            awaiting_clarification: Optional dict indicating if we're waiting for clarification.

        Returns:
            dict containing the agent's response and metadata.
        """
        # Set context for MCP tools
        set_context(db, user_id)

        try:
            # Build messages
            messages = [
                {"role": "system", "content": TODO_AGENT_SYSTEM_PROMPT},
            ]

            # If we're awaiting clarification, add context to help the model understand
            if awaiting_clarification:
                clarification_context = f"\n\n[CONTEXT: You previously asked the user to clarify which task they meant. The user is now responding to your clarification question. Their response '{message}' is answering your question about: {awaiting_clarification.get('question', 'task selection')}]"

                # Add conversation history
                if conversation_history:
                    messages.extend(conversation_history[-20:])

                # Modify the last user message to include clarification context
                if messages and messages[-1]["role"] == "user":
                    messages[-1]["content"] += clarification_context
            else:
                # Add conversation history if provided (includes current message)
                if conversation_history:
                    messages.extend(conversation_history[-20:])  # Last 20 messages (10 exchanges)

            # Debug logging
            print(f"\n=== OpenAI Request ===")
            print(f"User message: {message}")
            print(f"Awaiting clarification: {awaiting_clarification}")
            print(f"Conversation history length: {len(conversation_history) if conversation_history else 0}")
            print(f"Messages being sent to OpenAI:")
            for i, msg in enumerate(messages):
                print(f"  {i}. {msg['role']}: {msg['content'][:100]}...")
            print(f"=====================\n")

            # Call OpenAI with MCP tools
            response = self.openai_client.chat.completions.create(
                model=self.model,
                messages=messages,
                tools=get_openai_tools(),
                tool_choice="auto",
                max_tokens=200,
                temperature=0.7
            )

            response_message = response.choices[0].message
            tool_calls = response_message.tool_calls
            actions = []

            # If OpenAI wants to call MCP tools
            if tool_calls:
                # Import MCP tools
                from mcp_tools.server import (
                    add_task,
                    list_tasks,
                    complete_task,
                    delete_task,
                    update_task,
                    update_task_by_name,
                    delete_task_by_name,
                    complete_task_by_name,
                    create_recurring_task,
                    list_recurring_tasks,
                    delete_recurring_task_by_name,
                    update_recurring_task_by_name,
                )

                # Map tool names to MCP functions
                tool_map = {
                    "add_task": add_task,
                    "list_tasks": list_tasks,
                    "complete_task": complete_task,
                    "delete_task": delete_task,
                    "update_task": update_task,
                    "update_task_by_name": update_task_by_name,
                    "delete_task_by_name": delete_task_by_name,
                    "complete_task_by_name": complete_task_by_name,
                    "create_recurring_task": create_recurring_task,
                    "list_recurring_tasks": list_recurring_tasks,
                    "delete_recurring_task_by_name": delete_recurring_task_by_name,
                    "update_recurring_task_by_name": update_recurring_task_by_name,
                }

                # Execute each tool call
                for tool_call in tool_calls:
                    function_name = tool_call.function.name
                    function_args = json.loads(tool_call.function.arguments)

                    # Execute the MCP tool
                    if function_name in tool_map:
                        result = await tool_map[function_name](**function_args)
                        actions.append({
                            "type": function_name,
                            "result": result
                        })

                # Get final response from OpenAI with tool results
                messages.append(response_message)

                # Add tool results
                for tool_call, action in zip(tool_calls, actions):
                    messages.append({
                        "role": "tool",
                        "tool_call_id": tool_call.id,
                        "name": tool_call.function.name,
                        "content": json.dumps(action["result"])
                    })

                # Get final response
                final_response = self.openai_client.chat.completions.create(
                    model=self.model,
                    messages=messages,
                    max_tokens=200,
                    temperature=0.7
                )

                response_text = final_response.choices[0].message.content
            else:
                # No tool calls, just return the response
                response_text = response_message.content

            # Detect if we're asking for clarification and extract matching tasks
            awaiting_clarification_result = None

            # Check if we called list_tasks and got multiple matching results
            if actions and any(action.get("type") == "list_tasks" for action in actions):
                list_result = next((action.get("result") for action in actions if action.get("type") == "list_tasks"), None)

                if list_result and isinstance(list_result, list):
                    all_tasks = list_result

                    # Extract search term from original message
                    # Remove action words to get the task name
                    search_term = message.lower()
                    for word in ["delete", "remove", "update", "change", "modify", "complete", "finish", "done", "the", "task", "a"]:
                        search_term = search_term.replace(word, "")
                    search_term = search_term.strip()

                    # Filter tasks that match the search term
                    matching_tasks = [
                        task for task in all_tasks
                        if search_term in task["title"].lower() or task["title"].lower() in search_term
                    ]

                    # Check if the response is asking for clarification
                    is_question = response_text and "?" in response_text
                    mentions_task_action = response_text and any(word in response_text.lower() for word in ["delete", "remove", "update", "change", "task"])

                    if is_question and mentions_task_action and len(matching_tasks) > 1:
                        # Determine the action from the original message
                        action_type = None
                        if any(word in message.lower() for word in ["delete", "remove"]):
                            action_type = "delete"
                        elif any(word in message.lower() for word in ["update", "change", "modify"]):
                            action_type = "update"
                        elif any(word in message.lower() for word in ["complete", "finish", "done"]):
                            action_type = "complete"

                        if action_type:
                            awaiting_clarification_result = {
                                "question": response_text,
                                "original_message": message,
                                "action": action_type,
                                "matching_tasks": [
                                    {
                                        "id": str(task["id"]),
                                        "title": task["title"],
                                        "status": task.get("status", "unknown"),
                                        "priority": task.get("priority", "unknown")
                                    }
                                    for task in matching_tasks
                                ]
                            }
                            print(f"[CLARIFICATION DETECTED] Action: {action_type}, Matching tasks: {len(matching_tasks)}")
                            print(f"[CLARIFICATION DETECTED] Tasks: {[t['title'] for t in matching_tasks]}")

            return {
                "response": response_text,
                "tool_calls": actions,
                "success": True,
                "awaiting_clarification": awaiting_clarification_result,
            }

        except Exception as e:
            import traceback
            print(f"TodoAgent error: {e}")
            print(traceback.format_exc())
            return {
                "response": "I encountered an error while processing your request. Please try again.",
                "error": str(e),
                "success": False,
            }


# Singleton instance
_todo_agent: TodoAgent | None = None


def get_todo_agent() -> TodoAgent:
    """Get the singleton Todo Agent instance.

    Returns:
        TodoAgent instance configured with OpenAI and MCP tools.
    """
    global _todo_agent
    if _todo_agent is None:
        _todo_agent = TodoAgent()
    return _todo_agent
