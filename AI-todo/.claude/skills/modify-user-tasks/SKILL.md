# Skill Name: ModifyUserTasks

## Purpose

This skill is responsible for **creating, updating, and deleting user todo tasks**
based on natural language chat commands.

It enables the AI chatbot to safely perform **write operations** on tasks while
enforcing authentication, authorization, and confirmation for destructive actions.

---

## What This Skill Must Do

- Create new tasks
- Update existing tasks (title, status, due date, priority)
- Delete tasks (only with explicit confirmation)
- Convert chat intent into structured mutation requests
- Call existing Phase 2 backend services or CRUD agents

---

## What This Skill Must NOT Do

- ❌ Read tasks for display (handled by ReadUserTasks)
- ❌ Plan or summarize tasks
- ❌ Render UI
- ❌ Access the database directly

---

## Inputs Expected by This Skill

From orchestration agent:
- user_id
- action: create | update | delete
- task_id (required for update/delete)
- task_fields (title, status, due_date, priority)
- confirmation (required for delete)

Example user messages:
- “Add a task to revise math tonight”
- “Mark task 3 as completed”
- “Delete task 5”

---

## Output of This Skill

A structured mutation result:

- success: true | false
- task_id
- updated_fields
- error (if any)

This output is passed to:
- ConversationAgent
- PlannerAgent (if follow-up is needed)

---

## Safety Rules

- Delete operations MUST require explicit confirmation
- Ambiguous instructions MUST request clarification
- Missing required fields MUST NOT be assumed

---

## Error Handling

Return structured errors only.

Examples:
TASK_UPDATE_FAILED
TASK_DELETE_REQUIRES_CONFIRMATION

Do not retry destructive actions automatically.

---

## Security Rules

- Operate only on authenticated user tasks
- Enforce permission checks via Phase 2 agents
- Never expose internal database or stack errors

---

## Constraints

- MUST reuse Phase 2 CRUD / API infrastructure
- MUST follow spec-driven behavior
- MUST NOT invent task fields or defaults
- MUST NOT bypass confirmation rules

---

## Owner Subagent

- **TaskManagerAgent**