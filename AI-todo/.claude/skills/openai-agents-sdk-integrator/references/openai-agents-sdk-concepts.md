# OpenAI Agents SDK – Core Concepts Reference

Official Documentation:
https://openai.github.io/openai-agents-python/

---

## Agent

An Agent defines:
- identity (name)
- instructions
- tool access

Agents reason and decide when to call tools.

---

## Tool

A Tool is a callable function with:
- a strict schema
- validated input
- structured output

Tools are the ONLY way agents interact with external systems.

---

## Runner

The Runner executes agents:
- synchronously or asynchronously
- handles reasoning loops
- returns a Result object

---

## Result

Result includes:
- final_output (user-facing text)
- tool calls and observations
- execution metadata

Always check `final_output`.

---

## Session & Context

Session:
- maintains conversational context
- scoped to a chat session
- supports ephemeral memory

Use session.set / session.get.

---

## Memory

Memory types:
- short-term (session)
- long-term (optional, persisted)

Phase 3 uses ONLY short-term memory.

---

## Model Context Protocol (MCP)

MCP allows:
- model abstraction
- multi-model or hosted model backends
- safe context exchange

Agents must not assume a single model implementation.

---

## Voice Agents

Voice pipelines include:
- STT → Agent → TTS
- streaming audio support
- tool-based integration

Voice is optional but SDK-supported.

---

## Guardrails

Guardrails:
- restrict tool usage
- enforce confirmation
- validate results
- stop unsafe loops

Must be applied before and after Runner execution.  