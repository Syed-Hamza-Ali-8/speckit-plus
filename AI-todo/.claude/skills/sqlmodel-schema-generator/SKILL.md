Skill Name: SQLModel Schema Generator
Skill Type: Database Model Creation Skill
Purpose

This skill generates SQLModel ORM models for tables defined in:

/specs/database/*.md


It converts database specs → complete SQLModel models including:

fields

relationships

default values

validators

Pydantic read/write models

base models (Create/Update/Read)

What This Skill Must Do

Read schema specs (example: @specs/database/task-schema.md)

Generate model files inside:

backend/models/


Create base classes:

Task(BaseModel + SQLModel)

TaskCreate

TaskUpdate

TaskRead

Apply database constraints:

primary keys

foreign keys

indexes

relationships

Ensure every model references the proper user_id for multi-user support.

Inputs Expected

User may provide:

Database spec reference
@specs/database/task.md

Fields list

Required props

Relationships

File to generate
backend/models/task.py

Output of This Skill

SQLModel classes

Related Pydantic models

Type definitions

Relationship definitions

Table metadata

This skill ONLY generates database models.

Implementation Rules
1. Follow the DB spec exactly

Correct field names

Correct types

Correct FK relationships

Correct nullability

2. Use SQLModel patterns

Example:

class Task(SQLModel, table=True):
    id: str = Field(default_factory=uuid4, primary_key=True)
    title: str
    user_id: str = Field(foreign_key="user.id")

3. Must include Pydantic models

For every table:

<Name>Create

<Name>Update

<Name>Read

4. Must define timestamps if specified
created_at: datetime = Field(default_factory=datetime.utcnow)

5. Must import proper dependencies
from sqlmodel import SQLModel, Field, Relationship
from typing import Optional, List

6. Must include relationship definitions

Example:

user: Optional["User"] = Relationship(back_populates="tasks")

7. Must NOT create database engine or migrations

This skill is for models only.

Example Usage
Example 1
Use skill: sqlmodel-schema-generator
Spec: @specs/database/task-schema.md
Generate: backend/models/task.py

Example 2
Use skill: sqlmodel-schema-generator
Spec: @specs/database/user-schema.md
Generate: backend/models/user.py

Constraints

MUST NOT modify routes

MUST NOT modify frontend

MUST NOT create migrations

MUST NOT generate SQL queries (handled by CRUD skill)