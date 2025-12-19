from pydantic import BaseModel

class ModifyTaskInput(BaseModel):
    user_id: str
    action: str
    task_id: str | None
    confirm: bool | None = False

def modify_user_tasks_tool(payload: ModifyTaskInput):
    if payload.action == "delete" and not payload.confirm:
        raise Exception("CONFIRMATION_REQUIRED")
    return api.modify_task(**payload.dict())
