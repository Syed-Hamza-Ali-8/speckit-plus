# ModifyUserTasks Execution Template

Input:
- user_id
- action: create | update | delete
- task_id (if required)
- fields
- confirmation (if delete)

Steps:
1. Validate authentication
2. Validate permissions
3. Check confirmation for delete
4. Call Phase 2 CRUD or API agent
5. Return structured result

Output:
- success
- task_id
- updated_fields
- error (if any)
