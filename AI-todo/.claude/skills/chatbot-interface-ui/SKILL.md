# Skill Name: ChatbotInterfaceUI (Chatkit)

## Purpose

This skill integrates **Chatkit** as the chat user interface for the Todo AI Chatbot.

It is responsible for:
- configuring Chatkit
- connecting Chatkit to the backend chat endpoint
- enabling streaming and optional voice support

This skill does NOT implement or generate custom chat UI components.

---

## What This Skill Must Do

- Use Chatkit as the chat UI layer
- Configure Chatkit to send messages to `/api/chat`
- Display assistant responses returned by backend agents
- Support streaming responses via Chatkit
- Optionally support voice input/output via backend tools

---

## What This Skill Must NOT Do

- ❌ Build custom chat UI components
- ❌ Contain AI reasoning logic
- ❌ Call OpenAI APIs directly
- ❌ Bypass backend guardrails or agents
- ❌ Implement business logic in the frontend

---

## Architecture Position

Chatkit is **UI-only**.

All intelligence lives in:
- backend agents
- OpenAI Agents SDK
- guardrails
- tools

---

## Allowed Responsibilities

- UI configuration
- Endpoint wiring
- UX-level loading and error handling (via Chatkit defaults)

---

## Owner Subagent

- **ConversationAgent**
