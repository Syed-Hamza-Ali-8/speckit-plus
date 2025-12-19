# Skill Name: PlanAndSummarizeTasks

## Purpose

This skill is responsible for **planning, prioritizing, and summarizing**
a user’s todo tasks in a conversational and helpful way.

It enables the AI chatbot to:
- Summarize task lists
- Create daily or weekly plans
- Suggest priorities
- Highlight overdue or important tasks

This skill does **NOT** modify tasks.

---

## What This Skill Must Do

- Accept structured task data as input
- Generate:
  - concise summaries
  - daily plans
  - priority suggestions
- Adapt output tone for conversational responses
- Handle empty or large task lists gracefully

---

## What This Skill Must NOT Do

- ❌ Create, update, or delete tasks
- ❌ Call backend APIs directly
- ❌ Modify database state
- ❌ Invent tasks not provided in input

---

## Inputs Expected by This Skill

From TaskManagerAgent or orchestration agent:
- task_list (from ReadUserTasks)
- optional timeframe:
  - today
  - this week
- optional user goal:
  - “focus on study”
  - “reduce workload”

Example user messages:
- “Plan my day”
- “Summarize my tasks”
- “What should I focus on today?”

---

## Output of This Skill

Human-readable planning output, such as:
- a short summary
- a prioritized list
- simple recommendations

This output is passed directly to:
- **ConversationAgent**

---

## Reasoning Rules

- Prefer clarity over verbosity
- Keep plans realistic
- Highlight urgent or overdue tasks first
- Avoid guilt-inducing language

---

## Error Handling

- If no tasks are provided, respond gracefully:
  - “You don’t have any tasks yet.”

---

## Constraints

- MUST be read-only
- MUST use provided task data only
- MUST not assume missing context
- MUST follow spec-driven behavior

---

## Owner Subagent

- **PlannerAgent**
