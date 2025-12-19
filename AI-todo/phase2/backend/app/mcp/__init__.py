"""MCP (Model Context Protocol) tool layer for task operations.

This module provides MCP-compatible tool schemas and implementations
for AI-driven task management operations.

Now includes official MCP SDK server implementation in server.py.
"""

from app.mcp.schemas import (
    CompleteTaskInput,
    CreateTaskInput,
    DeleteTaskInput,
    ListTasksInput,
    MCPToolResult,
    UpdateTaskInput,
)
from app.mcp.tools import TaskTools
from app.mcp.server import (
    mcp,
    set_context,
    add_task,
    list_tasks,
    complete_task,
    delete_task,
    update_task,
)

__all__ = [
    # Schemas (legacy)
    "MCPToolResult",
    "ListTasksInput",
    "CreateTaskInput",
    "UpdateTaskInput",
    "CompleteTaskInput",
    "DeleteTaskInput",
    "TaskTools",
    # Official MCP SDK server
    "mcp",
    "set_context",
    "add_task",
    "list_tasks",
    "complete_task",
    "delete_task",
    "update_task",
]
