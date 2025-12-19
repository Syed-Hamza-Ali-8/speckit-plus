# Skill Name: OpenAI-Agents-SDK-Integrator

## Purpose

Generate Python code using the **OpenAI Agents SDK** correctly.

Supports:
- Agent
- Tool
- Runner
- Result
- Session & Memory
- MCP compatibility
- Voice agents (STT/TTS)

---

## What This Skill Must Do

- Define Agents with instructions
- Register Tools (Pydantic schemas)
- Use Runner (sync/async)
- Use Session for context
- Attach Memory safely
- Return normalized Result objects

---

## What This Skill Must NOT Do

- ❌ Use raw OpenAI APIs
- ❌ Hardcode keys
- ❌ Bypass Guardrails

---

## SDK Concepts Used

- Agent
- Tool
- Result
- Session
- Memory
- MCP-aware execution
- Voice pipelines

---

## Owner Subagent

- PlannerAgent
