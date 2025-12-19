# Skill Name: MCP-Server-Tools

## Purpose

This skill is responsible for implementing an **MCP (Model Context Protocol) Server**
using the **Official MCP SDK**.

It exposes **task-related operations as MCP tools** so that AI agents can safely
interact with backend functionality in a standardized, inspectable way.

This skill contains **NO AI reasoning**.

---

## What This Skill Must Do

- Implement an MCP server using the official SDK
- Expose task operations as MCP tools
- Define strict input/output schemas for tools
- Execute backend task operations
- Return structured tool results

---

## MCP Tools Owned by This Skill

- list_tasks
- create_task
- update_task
- complete_task
- delete_task

---

## What This Skill Must NOT Do

- ❌ Perform AI reasoning
- ❌ Manage chat sessions
- ❌ Render UI
- ❌ Bypass guardrails
- ❌ Call OpenAI APIs directly

---

## Inputs Expected by This Skill

From agents (via MCP):
- tool name
- validated input payload
- authenticated user context

---

## Output of This Skill

- Structured MCP tool responses
- Deterministic results
- Errors in standardized MCP format

---

## Constraints

- MUST use the Official MCP SDK
- MUST be stateless
- MUST validate schemas strictly
- MUST NOT store conversational context

---

## Owner Subagent

- **MCPServerAgent**
