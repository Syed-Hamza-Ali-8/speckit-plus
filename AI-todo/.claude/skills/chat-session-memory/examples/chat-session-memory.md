# ChatSessionMemory – Examples

## Example 1: Follow-up Delete
User: "Delete task 3"
Assistant: "Are you sure?"
User: "Yes"

Memory:
- last_task_id: 3
- confirmation_state: true

---

## Example 2: Reference Resolution
User: "Mark it as completed"

Memory:
- last_task_id: 5
