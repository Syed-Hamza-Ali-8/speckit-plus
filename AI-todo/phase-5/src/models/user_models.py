from datetime import datetime
from typing import Optional
from uuid import UUID, uuid4
from sqlmodel import SQLModel, Field
from pydantic import BaseModel


class UserBase(SQLModel):
    email: str = Field(unique=True, nullable=False)
    name: str = Field(max_length=100)
    is_active: bool = True


class UserCreate(UserBase):
    password: str = Field(min_length=4)  # Shortened for local testing


class UserUpdate(SQLModel):
    name: Optional[str] = Field(default=None, max_length=100)
    email: Optional[str] = Field(default=None)
    is_active: Optional[bool] = None


class User(UserBase, table=True):
    id: UUID = Field(default_factory=uuid4, primary_key=True)
    password_hash: str = Field(nullable=False)  # In real app, store hashed password
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)


class UserRead(UserBase):
    id: UUID
    created_at: datetime
    updated_at: datetime