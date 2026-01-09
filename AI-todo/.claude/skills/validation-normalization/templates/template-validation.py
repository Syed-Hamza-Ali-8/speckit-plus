from pydantic import BaseModel, EmailStr, ValidationError
from typing import Optional

class UserPayload(BaseModel):
    username: str
    email: EmailStr
    age: Optional[int] = None

class TaskPayload(BaseModel):
    title: str
    description: Optional[str] = ""
    assigned_to: str

def validate_payload(schema: BaseModel, data: dict):
    try:
        validated = schema(**data)
        return validated, None
    except ValidationError as e:
        return None, e.errors()
