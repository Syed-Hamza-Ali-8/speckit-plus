# Example: MCP Tools Usage for Task Operations

## Goal

Expose backend task operations as MCP tools
that can be safely used by AI agents.

---

## Scenario

An AI agent needs to list a user's tasks.

---

## What Happens

1. Agent calls MCP tool `list_tasks`
2. MCP server validates input schema
3. Task data is fetched from backend
4. Structured result is returned to the agent

---

## Outcome

The agent receives task data without direct
database or API access.
