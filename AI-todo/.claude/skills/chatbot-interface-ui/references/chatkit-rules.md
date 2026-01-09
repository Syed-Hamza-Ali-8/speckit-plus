# Chatkit Integration Rules

## Core Rules

- Chatkit is UI-only
- All AI logic lives in backend agents
- All messages MUST go through `/api/chat`
- Guardrails MUST execute before agent logic
- Chatkit must not access databases or tools directly

---

## Streaming

- Streaming is handled by Chatkit
- Backend should stream agent responses when supported
- UI must not implement custom streaming logic

---

## Constraints

- Do not modify Chatkit internals
- Do not embed business logic in UI
- Do not call OpenAI APIs from the frontend
