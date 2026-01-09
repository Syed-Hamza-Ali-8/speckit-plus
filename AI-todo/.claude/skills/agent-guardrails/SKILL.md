# Skill Name: AgentGuardrails

## Purpose

This skill enforces **safety, confirmation, and execution boundaries**
for all OpenAI Agents.

It ensures:
- destructive actions require confirmation
- tools are not misused
- agents do not hallucinate permissions
- SDK Result objects are validated before execution

This skill operates as a **pre- and post-execution guard layer**.

---

## What This Skill Must Do

- Block destructive tool calls without confirmation
- Validate tool input schemas
- Enforce user authorization boundaries
- Stop agent loops if unsafe behavior is detected
- Normalize SDK Result output

---

## Guardrails Enforced

### 1. Destructive Action Confirmation
Delete/update operations MUST:
- require explicit confirmation
- fail fast if missing

### 2. Tool Invocation Control
- Only registered tools may be invoked
- No raw code execution

### 3. Result Validation
- Ensure `Result.final_output` exists
- Ensure tool outputs match schemas

### 4. Context Safety
- Prevent leaking internal state
- Prevent cross-user context contamination

---

## What This Skill Must NOT Do

- ❌ Modify data itself
- ❌ Call OpenAI APIs
- ❌ Generate UI

---

## Owner Subagent

- **TaskManagerAgent**
