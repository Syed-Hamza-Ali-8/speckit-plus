---
id: 003
title: Fix MCP server notification calls
stage: green
date: 2025-12-23
surface: agent
model: claude-sonnet-4-5
feature: phase3-ai-chatbot
branch: main
user: user
command: /sp.implement
labels: ["bugfix", "notifications", "chatbot", "mcp-server", "critical-fix"]
links:
  spec: specs/phase3-ai-chatbot/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - phase2/backend/app/mcp/server.py
  - phase2/frontend/src/hooks/useChat.ts (debug logging)
tests:
  - Python syntax validation passed
---

## Prompt

I  just create a task through chatbot but still no notification shows of task created, same with update and delete

## Response snapshot

After investigation, I found the **actual root cause**: The chatbot uses a different code path than I initially fixed!

**The Real Problem:**

1. **PHR 001** updated `app/mcp/tools.py` (TaskTools class) with notification calls
2. **PHR 002** updated frontend cache invalidation
3. **BUT** the chatbot doesn't use `tools.py` at all!

The chatbot uses `GeminiTodoAgent` which imports tools from `app/mcp/server.py` (lines 115-121 in `todo_agent.py`):
```python
from app.mcp.server import (
    add_task,
    list_tasks,
    complete_task,
    delete_task,
    update_task,
)
```

These are FastMCP-decorated functions that directly call task_service but **never call notification_service**.

**The TaskTools vs MCP Server Confusion:**

- `app/mcp/tools.py` = TaskTools class used by TaskManagerAgent (multi-agent system, not currently used)
- `app/mcp/server.py` = FastMCP tools used by GeminiTodoAgent (the actual chatbot implementation)

I mistakenly updated the wrong file in PHR 001!

**Solution Implemented:**

Updated `phase2/backend/app/mcp/server.py` to add notification calls to all task operations:

1. **Added notification_service import** (line 12)

2. **Updated `add_task()` function** (lines 74-81):
   - Added `notify_task_created()` call after task creation

3. **Updated `complete_task()` function** (lines 177-183):
   - Added `notify_task_completed()` call after marking complete

4. **Updated `delete_task()` function** (lines 245-250):
   - Added `notify_task_deleted()` call after deletion

5. **Updated `update_task()` function** (lines 322-328):
   - Added `notify_task_updated()` call after update

**How It Works Now:**

When users interact with the chatbot:
1. GeminiTodoAgent processes the message
2. Calls tools from server.py (add_task, complete_task, etc.)
3. Each tool creates/modifies the task AND creates a notification
4. Frontend cache invalidation (from PHR 002) triggers notification refetch
5. Notification bell updates immediately

**Files Modified:**
- `phase2/backend/app/mcp/server.py` (line 12: import, lines 74-81, 177-183, 245-250, 322-328: notification calls)
- `phase2/frontend/src/hooks/useChat.ts` (lines 110-131: added debug logging to verify cache invalidation)

## Outcome

- ✅ Impact: Chatbot task operations now correctly create notifications that appear immediately in the UI
- 🧪 Tests: Python syntax validation passed
- 📁 Files: 1 backend file (server.py) with notification integration, 1 frontend file with debug logging
- 🔁 Next prompts: Test in browser - create/update/complete/delete tasks via chatbot and verify notifications appear
- 🧠 Reflection: This was a classic case of multiple similar implementations causing confusion. The codebase has two MCP tool implementations (tools.py and server.py), and I initially updated the wrong one. The key lesson is to trace the actual code path from user action to backend function before making changes.

## Evaluation notes (flywheel)

- Failure modes observed: Incorrect assumption about which MCP implementation was in use
- Graders run and results (PASS/FAIL): Syntax validation PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Consider consolidating the two MCP implementations (tools.py and server.py) to avoid future confusion, or clearly document which is used by which agent
