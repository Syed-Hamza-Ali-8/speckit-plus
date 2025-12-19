# ChatSessionMemory Execution Template

Input:
- recent_messages
- recent_actions

Steps:
1. Extract last intent
2. Extract referenced task IDs
3. Track confirmation state
4. Provide context to next agent

Output:
- context:
  - last_intent
  - last_task_id
  - confirmation_state
