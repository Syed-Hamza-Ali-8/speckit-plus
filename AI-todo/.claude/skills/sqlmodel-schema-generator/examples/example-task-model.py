from sqlmodel import SQLModel, Field, Relationship
from datetime import datetime
from typing import Optional
from uuid import uuid4

class Task(SQLModel, table=True):
    id: str = Field(default_factory=lambda: str(uuid4()), primary_key=True)
    title: str
    description: Optional[str] = None
    completed: bool = False
    user_id: str = Field(foreign_key="user.id")
    created_at: datetime = Field(default_factory=datetime.utcnow)

class TaskCreate(SQLModel):
    title: str
    description: Optional[str] = None

class TaskUpdate(SQLModel):
    title: Optional[str] = None
    description: Optional[str] = None
    completed: Optional[bool] = None

class TaskRead(SQLModel):
    id: str
    title: str
    description: Optional[str]
    completed: bool
    created_at: datetime
