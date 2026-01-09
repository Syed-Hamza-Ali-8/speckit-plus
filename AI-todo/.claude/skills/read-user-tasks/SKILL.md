# Skill Name: ReadUserTasks

## Purpose

This skill is responsible for reading and retrieving a user’s todo tasks in response
to natural language chat requests.

It enables the AI chatbot to answer questions like:
- “What are my tasks?”
- “What do I need to do today?”
- “Show my pending todos”
- “What have I completed?”

This skill is strictly READ-ONLY.

---

## What This Skill Must Do

- Fetch tasks belonging to the authenticated user
- Support common filters:
  - status (pending, completed)
  - due date (today, upcoming)
  - priority (if supported)
- Convert chat intent into structured read queries
- Return structured task data for other agents to consume

---

## What This Skill Must NOT Do

- ❌ Create tasks
- ❌ Update tasks
- ❌ Delete tasks
- ❌ Modify database state
- ❌ Generate conversational responses

All write operations belong to **ModifyUserTasks**.

---

## Inputs Expected by This Skill

From chat or orchestration agent:
- user_id
- optional filters:
  - status
  - due_date
  - priority

Example user messages:
- “Show my tasks”
- “What are my tasks for today?”
- “List completed tasks”

---

## Output of This Skill

A structured task list, for example:

- task_id
- title
- status
- due_date
- priority

This output is passed to:
- PlannerAgent (for summaries)
- ConversationAgent (for friendly responses)

---

## Error Handling

If reading tasks fails:
- Return a structured error object
- Do not retry automatically
- Do not generate user-facing text

Example:
TASK_READ_FAILED

---

## Security Rules

- Only read tasks belonging to the authenticated user
- Respect all permission checks
- Never expose internal system details

---

## Constraints

- MUST be read-only
- MUST reuse Phase 2 backend / CRUD logic
- MUST follow spec-driven behavior
- MUST NOT invent filters not supported by backend

---

## Owner Subagent

- **TaskManagerAgent**