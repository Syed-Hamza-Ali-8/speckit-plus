from sqlmodel import SQLModel, Field, Relationship
from datetime import datetime
from typing import Optional, List
from uuid import uuid4

class MODEL_NAME(SQLModel, table=True):
    id: str = Field(default_factory=lambda: str(uuid4()), primary_key=True)
    FIELD_ROWS

class MODEL_NAMECreate(SQLModel):
    FIELD_ROWS_CREATE

class MODEL_NAMEUpdate(SQLModel):
    FIELD_ROWS_UPDATE

class MODEL_NAMERead(SQLModel):
    id: str
    FIELD_ROWS_READ
