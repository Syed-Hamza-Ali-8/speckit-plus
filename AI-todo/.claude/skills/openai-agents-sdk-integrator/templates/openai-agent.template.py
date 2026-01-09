from agents import Agent, Runner, Tool, Session
from pydantic import BaseModel

class Input(BaseModel):
    user_id: str

def example_tool(payload: Input):
    return {"ok": True}

tool = Tool(
    name="example_tool",
    func=example_tool,
    description="Example SDK tool"
)

agent = Agent(
    name="ExampleAgent",
    instructions="Use tools safely and respect guardrails."
)

agent.register_tool(tool)

session = Session()

result = Runner.run_sync(agent, input="Run example", session=session)

print(result.final_output)
