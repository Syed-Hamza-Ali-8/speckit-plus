# Data Model: Phase 2 Part 2 - Authentication

**Date**: 2025-12-14
**Feature**: User Authentication & Task Ownership

## Entity Relationship Diagram

```text
┌─────────────────────────────────────┐
│              User                   │
├─────────────────────────────────────┤
│ id: UUID (PK)                       │
│ email: VARCHAR(255) UNIQUE          │
│ hashed_password: VARCHAR(255)       │
│ is_active: BOOLEAN                  │
│ created_at: TIMESTAMP               │
└─────────────────────────────────────┘
                 │
                 │ 1:N
                 ▼
┌─────────────────────────────────────┐
│              Task                   │
├─────────────────────────────────────┤
│ id: UUID (PK)                       │
│ user_id: UUID (FK → User.id)        │
│ title: VARCHAR(200)                 │
│ description: VARCHAR(1000) NULL     │
│ status: VARCHAR(50)                 │
│ created_at: TIMESTAMP               │
│ updated_at: TIMESTAMP               │
└─────────────────────────────────────┘
```

## Entity Definitions

### User

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | PRIMARY KEY, DEFAULT uuid4 | Unique user identifier |
| email | VARCHAR(255) | UNIQUE, NOT NULL, INDEX | User email (login credential) |
| hashed_password | VARCHAR(255) | NOT NULL | Argon2id password hash |
| is_active | BOOLEAN | NOT NULL, DEFAULT TRUE | Account active status |
| created_at | TIMESTAMP(TZ) | NOT NULL, DEFAULT NOW() | Account creation time |

**Indexes**:
- `ix_users_email` - B-tree index on email for login lookups

**Constraints**:
- Email must be unique (enforced at DB and app level)
- Email format validated at Pydantic schema level

### Task (Updated)

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | PRIMARY KEY, DEFAULT uuid4 | Unique task identifier |
| user_id | UUID | FK → users.id, NOT NULL, INDEX | Owner user reference |
| title | VARCHAR(200) | NOT NULL, CHECK(length(trim(title)) > 0) | Task title |
| description | VARCHAR(1000) | NULL | Optional description |
| status | VARCHAR(50) | NOT NULL, DEFAULT 'pending' | Task status enum |
| created_at | TIMESTAMP(TZ) | NOT NULL, DEFAULT NOW() | Creation timestamp |
| updated_at | TIMESTAMP(TZ) | NOT NULL, DEFAULT NOW(), ON UPDATE NOW() | Last update |

**Indexes**:
- `ix_tasks_user_id` - B-tree index for user task queries

**Foreign Key**:
- `fk_tasks_user_id` → `users.id` ON DELETE CASCADE

## SQLModel Definitions

### User Model

```python
from datetime import datetime, timezone
from uuid import UUID, uuid4

from sqlmodel import Field, SQLModel


class User(SQLModel, table=True):
    """User account for authentication."""

    __tablename__ = "users"

    id: UUID = Field(
        default_factory=uuid4,
        primary_key=True,
        description="Unique user identifier",
    )
    email: str = Field(
        max_length=255,
        unique=True,
        index=True,
        nullable=False,
        description="User email address (unique)",
    )
    hashed_password: str = Field(
        max_length=255,
        nullable=False,
        description="Argon2id password hash",
    )
    is_active: bool = Field(
        default=True,
        nullable=False,
        description="Account active status",
    )
    created_at: datetime = Field(
        default_factory=lambda: datetime.now(timezone.utc),
        nullable=False,
        description="Account creation timestamp",
    )
```

### Task Model (Updated)

```python
from datetime import datetime, timezone
from uuid import UUID, uuid4

from sqlmodel import Field, Relationship, SQLModel


class Task(SQLModel, table=True):
    """Task with user ownership."""

    __tablename__ = "tasks"

    id: UUID = Field(default_factory=uuid4, primary_key=True)
    user_id: UUID = Field(
        foreign_key="users.id",
        index=True,
        nullable=False,
        description="Owner user ID",
    )
    title: str = Field(max_length=200, nullable=False)
    description: str | None = Field(default=None, max_length=1000)
    status: str = Field(default="pending", nullable=False)
    created_at: datetime = Field(
        default_factory=lambda: datetime.now(timezone.utc),
        nullable=False,
    )
    updated_at: datetime = Field(
        default_factory=lambda: datetime.now(timezone.utc),
        nullable=False,
    )

    # Relationship (optional, for eager loading)
    # owner: "User" = Relationship(back_populates="tasks")
```

## Pydantic Schemas

### Auth Schemas

```python
from datetime import datetime
from uuid import UUID

from pydantic import BaseModel, EmailStr, Field


class UserCreate(BaseModel):
    """User registration request."""

    email: EmailStr
    password: str = Field(min_length=8, max_length=128)


class LoginRequest(BaseModel):
    """User login request."""

    email: EmailStr
    password: str


class TokenResponse(BaseModel):
    """JWT token response."""

    access_token: str
    token_type: str = "bearer"


class UserResponse(BaseModel):
    """User profile response (excludes password)."""

    id: UUID
    email: str
    is_active: bool
    created_at: datetime

    model_config = {"from_attributes": True}
```

### Task Schemas

```python
from datetime import datetime
from uuid import UUID

from pydantic import BaseModel, Field


class TaskCreate(BaseModel):
    """Task creation request."""

    title: str = Field(min_length=1, max_length=200)
    description: str | None = Field(default=None, max_length=1000)


class TaskUpdate(BaseModel):
    """Task update request (all fields optional)."""

    title: str | None = Field(default=None, min_length=1, max_length=200)
    description: str | None = Field(default=None, max_length=1000)
    status: str | None = Field(default=None, pattern="^(pending|completed)$")


class TaskResponse(BaseModel):
    """Task response."""

    id: UUID
    user_id: UUID
    title: str
    description: str | None
    status: str
    created_at: datetime
    updated_at: datetime

    model_config = {"from_attributes": True}
```

## Validation Rules

### User Validation

| Field | Rule | Error Message |
|-------|------|---------------|
| email | Valid email format | "Invalid email format" |
| email | Unique in database | "Email already registered" |
| password | 8-128 characters | "Password must be 8-128 characters" |

### Task Validation

| Field | Rule | Error Message |
|-------|------|---------------|
| title | 1-200 characters | "Title must be 1-200 characters" |
| title | Not empty/whitespace | "Title cannot be empty" |
| description | Max 1000 characters | "Description too long" |
| status | "pending" or "completed" | "Invalid status" |

## State Transitions

### User States

```text
                    ┌─────────────┐
   Register         │   Active    │
   ─────────────────▶│ is_active=T │
                    └──────┬──────┘
                           │
                    Admin  │  Deactivate
                           ▼
                    ┌─────────────┐
                    │  Inactive   │
                    │ is_active=F │
                    └──────┬──────┘
                           │
                    Admin  │  Reactivate
                           ▼
                    ┌─────────────┐
                    │   Active    │
                    └─────────────┘
```

### Task States

```text
                    ┌─────────────┐
    Create          │   Pending   │
    ────────────────▶│status=pending│
                    └──────┬──────┘
                           │
                    Toggle │  Complete
                           ▼
                    ┌─────────────┐
                    │  Completed  │
                    │status=done  │
                    └──────┬──────┘
                           │
                    Toggle │  Uncomplete
                           ▼
                    ┌─────────────┐
                    │   Pending   │
                    └─────────────┘
```

## Migration Plan

### Migration 002: Add Users Table

```python
"""Add users table and FK constraint

Revision ID: 002
Revises: 001
"""

def upgrade() -> None:
    # 1. Create users table
    op.create_table(
        "users",
        sa.Column("id", sa.UUID(), primary_key=True),
        sa.Column("email", sa.String(255), unique=True, nullable=False),
        sa.Column("hashed_password", sa.String(255), nullable=False),
        sa.Column("is_active", sa.Boolean(), default=True, nullable=False),
        sa.Column("created_at", sa.DateTime(timezone=True), nullable=False),
    )
    op.create_index("ix_users_email", "users", ["email"])

    # 2. Add FK constraint to tasks (user_id already exists from 001)
    op.create_foreign_key(
        "fk_tasks_user_id",
        "tasks", "users",
        ["user_id"], ["id"],
        ondelete="CASCADE"
    )


def downgrade() -> None:
    op.drop_constraint("fk_tasks_user_id", "tasks", type_="foreignkey")
    op.drop_index("ix_users_email", "users")
    op.drop_table("users")
```

**Migration Order**:
1. `001_create_tasks_table.py` - Existing (has user_id column)
2. `002_add_users_table.py` - New (creates users, adds FK)

**Data Migration Note**: If tasks exist without valid user_id, they will fail FK constraint. Handle by:
- Option A: Delete orphan tasks before migration
- Option B: Create placeholder user, assign orphan tasks
