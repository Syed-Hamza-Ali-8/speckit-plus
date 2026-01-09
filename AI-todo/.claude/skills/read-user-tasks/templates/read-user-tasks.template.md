# ReadUserTasks Execution Template

Input:
- user_id
- filters:
  - status
  - due_date
  - priority

Steps:
1. Validate authentication
2. Apply only provided filters
3. Call Phase 2 read APIs or CRUD agent
4. Return structured task list
5. Do not format conversational output

Output:
- tasks[]:
  - id
  - title
  - status
  - due_date
  - priority
