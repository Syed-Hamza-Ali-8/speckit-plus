# ReadUserTasks – Examples

## Example 1: List All Tasks
User: "Show my tasks"

Action:
Use skill: ReadUserTasks
Filters: none

---

## Example 2: Today’s Tasks
User: "What do I need to do today?"

Action:
Use skill: ReadUserTasks
Filters:
- due_date: today

---

## Example 3: Pending Tasks
User: "What tasks are still pending?"

Action:
Use skill: ReadUserTasks
Filters:
- status: pending
