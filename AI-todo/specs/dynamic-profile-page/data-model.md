# Data Model — Dynamic Profile Page

**Date**: 2025-12-15
**Feature**: dynamic-profile-page

## Entity Definitions

### User (Existing - Extended)

**Table**: `users`

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `id` | UUID | PK, default uuid4 | Unique user identifier |
| `email` | VARCHAR(255) | NOT NULL, UNIQUE, INDEX | User email (immutable) |
| `hashed_password` | VARCHAR(255) | NOT NULL | Argon2id hash |
| `first_name` | VARCHAR(100) | NULLABLE | User's first name |
| `last_name` | VARCHAR(100) | NULLABLE | User's last name |
| `avatar_url` | TEXT | NULLABLE | **NEW** - Avatar as base64 data URL |
| `is_active` | BOOLEAN | NOT NULL, default true | Account status |
| `created_at` | TIMESTAMP(tz) | NOT NULL, default now() | Registration date |

**Computed Properties**:
- `display_name`: `first_name + last_name` or `email.split('@')[0]`

---

## Schema Definitions

### Backend (Python/SQLModel)

#### User Model Extension

```python
# app/models/user.py - ADD field
avatar_url: str | None = Field(
    default=None,
    sa_column=Column(Text, nullable=True),
    description="Avatar image as base64 data URL",
)
```

#### UserUpdate Schema (NEW)

```python
# app/schemas/auth.py - ADD schema
class UserUpdate(BaseModel):
    """Schema for profile update request."""

    first_name: str | None = Field(
        default=None,
        max_length=100,
        description="User's first name",
    )
    last_name: str | None = Field(
        default=None,
        max_length=100,
        description="User's last name",
    )
    avatar_url: str | None = Field(
        default=None,
        description="Avatar as base64 data URL (max 2MB)",
    )
```

#### UserResponse Extension

```python
# app/schemas/auth.py - MODIFY schema
class UserResponse(BaseModel):
    # ... existing fields ...
    avatar_url: str | None = Field(None, description="Avatar URL or base64")
```

---

### Frontend (TypeScript)

#### User Types

```typescript
// src/types/user.ts
export interface User {
  id: string;
  email: string;
  first_name: string | null;
  last_name: string | null;
  display_name: string;
  avatar_url: string | null;
  is_active: boolean;
  created_at: string;
}

export interface UserUpdateRequest {
  first_name?: string | null;
  last_name?: string | null;
  avatar_url?: string | null;
}
```

#### Task Stats (Computed)

```typescript
// src/types/user.ts
export interface TaskStats {
  total: number;
  completed: number;
  pending: number;
  completionRate: number; // percentage 0-100
}
```

---

## Validation Rules

### Profile Update

| Field | Rule | Error Message |
|-------|------|---------------|
| `first_name` | max 100 chars | "First name must be less than 100 characters" |
| `last_name` | max 100 chars | "Last name must be less than 100 characters" |
| `avatar_url` | valid data URL or null | "Invalid avatar format" |
| `avatar_url` | max ~2.7MB (2MB image in base64) | "Avatar too large (max 2MB)" |

### Zod Schema (Frontend)

```typescript
// src/lib/validations/profile.ts
import { z } from 'zod';

export const profileSchema = z.object({
  firstName: z
    .string()
    .max(100, 'First name must be less than 100 characters')
    .nullable()
    .optional(),
  lastName: z
    .string()
    .max(100, 'Last name must be less than 100 characters')
    .nullable()
    .optional(),
});

export type ProfileFormData = z.infer<typeof profileSchema>;
```

---

## State Transitions

### Profile Update Flow

```
┌─────────────┐     PATCH /auth/me      ┌─────────────┐
│   Viewing   │ ──────────────────────► │  Updating   │
│   Profile   │                         │   Profile   │
└─────────────┘                         └─────────────┘
      ▲                                       │
      │         Success: UserResponse         │
      │◄──────────────────────────────────────┤
      │                                       │
      │         Error: 400/401/500            │
      │◄──────────────────────────────────────┘
```

### Avatar Upload Flow

```
┌────────────┐   Select File   ┌────────────┐   Convert   ┌────────────┐
│   Empty    │ ──────────────► │  Preview   │ ──────────► │  Base64    │
│   Avatar   │                 │   Image    │  FileReader │   String   │
└────────────┘                 └────────────┘             └────────────┘
                                    │                          │
                                    │ Cancel                   │ Submit
                                    ▼                          ▼
                               ┌────────────┐           ┌────────────┐
                               │   Reset    │           │   PATCH    │
                               │            │           │  /auth/me  │
                               └────────────┘           └────────────┘
```

---

## Database Migration

### Alembic Migration Script

```python
# alembic/versions/YYYYMMDD_add_user_avatar.py
"""Add avatar_url to users table

Revision ID: xxxx
Revises: previous_revision
Create Date: 2025-12-15
"""

from alembic import op
import sqlalchemy as sa

def upgrade() -> None:
    op.add_column(
        'users',
        sa.Column('avatar_url', sa.Text(), nullable=True)
    )

def downgrade() -> None:
    op.drop_column('users', 'avatar_url')
```

---

## Relationships

```
┌─────────────────────────────────────────────────────────────┐
│                         User                                │
│  id, email, first_name, last_name, avatar_url, is_active   │
└─────────────────────────────────────────────────────────────┘
                              │
                              │ 1:N (owner_id)
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                         Task                                │
│    id, title, description, status, owner_id, created_at    │
└─────────────────────────────────────────────────────────────┘
```

Task stats are computed by counting tasks with `owner_id = current_user.id` grouped by `status`.
