# Example: OpenAI Agent with Tools, Session, and Guardrails

## Goal

Create an OpenAI Agent that:
- reads user tasks via a Tool
- uses session-based context
- respects guardrails
- returns a structured Result

---

## Agent Description

Agent Name: TaskAgent  
Purpose: Assist users with task-related queries using tools safely.

---

## Execution Flow

1. User sends: "What tasks do I have today?"
2. Agent reasons and decides to call `read_user_tasks`
3. Tool is invoked with validated schema
4. Tool returns structured data
5. Agent summarizes result
6. Runner returns `Result.final_output`

---

## Example Pseudocode

```python
agent = Agent(
  name="TaskAgent",
  instructions="Use tools to fetch tasks. Respect guardrails."
)

agent.register_tool(read_user_tasks)

session = Session()

result = Runner.run_sync(
  agent,
  input="What tasks do I have today?",
  session=session
)

print(result.final_output)
