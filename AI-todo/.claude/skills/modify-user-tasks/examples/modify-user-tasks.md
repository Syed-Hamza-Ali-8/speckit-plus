# ModifyUserTasks – Examples

## Example 1: Create Task
User: "Add a task to study physics tomorrow"

Action:
Use skill: ModifyUserTasks
Action: create
Fields:
- title: study physics
- due_date: tomorrow

---

## Example 2: Update Task
User: "Mark task 2 as done"

Action:
Use skill: ModifyUserTasks
Action: update
Task ID: 2
Fields:
- status: completed

---

## Example 3: Delete Task
User: "Delete task 4"

Action:
Use skill: ModifyUserTasks
Action: delete
Task ID: 4
Confirmation: required
