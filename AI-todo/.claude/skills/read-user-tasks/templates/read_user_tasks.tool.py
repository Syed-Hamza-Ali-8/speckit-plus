from pydantic import BaseModel
from typing import List

class ReadTasksInput(BaseModel):
    user_id: str

class ReadTasksOutput(BaseModel):
    tasks: List[dict]

def read_user_tasks_tool(payload: ReadTasksInput) -> ReadTasksOutput:
    tasks = api.get_tasks(payload.user_id)
    return ReadTasksOutput(tasks=tasks)
