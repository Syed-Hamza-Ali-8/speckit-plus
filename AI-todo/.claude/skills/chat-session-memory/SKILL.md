# Skill Name: ChatSessionMemory

## Purpose

This skill is responsible for maintaining **short-term conversational context**
within a single chat session.

It allows the chatbot to:
- Understand follow-up commands
- Resolve references like “that task” or “the last one”
- Maintain continuity in multi-step interactions

This skill does **not** provide long-term memory.

---

## What This Skill Must Do

- Track recent user intents
- Track recently referenced task IDs
- Store temporary conversational context
- Provide context for follow-up messages

---

## What This Skill Must NOT Do

- ❌ Persist memory across sessions
- ❌ Store sensitive user data long-term
- ❌ Modify tasks
- ❌ Perform planning or summarization

---

## Inputs Expected by This Skill

- Recent user messages
- Recent agent actions
- Task IDs referenced in the session

Example user messages:
- “Delete that task”
- “Yes, do it”
- “Mark it as done”

---

## Output of This Skill

- Context object containing:
  - last_intent
  - last_task_id
  - confirmation_state

This output is consumed by:
- TaskManagerAgent
- ConversationAgent

---

## Memory Scope Rules

- Memory is scoped to a single chat session
- Memory is cleared when the session ends
- Only store what is necessary for context resolution

---

## Error Handling

- If context is missing, request clarification
- Do not guess task references

---

## Security & Privacy Rules

- Never store sensitive task content unnecessarily
- Never persist memory beyond the session
- Never expose memory to the user

---

## Constraints

- MUST be session-scoped
- MUST be ephemeral
- MUST not replace database state

---

## Owner Subagent

- **ConversationAgent**
