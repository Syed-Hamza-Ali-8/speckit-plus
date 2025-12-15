# Data Model — Dynamic Settings Page

**Date**: 2025-12-15
**Feature**: dynamic-settings-page

## Entity Definitions

### User (Extended)

**Table**: `users`

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `id` | UUID | PK | Unique user identifier |
| `email` | VARCHAR(255) | NOT NULL, UNIQUE | User email |
| `hashed_password` | VARCHAR(255) | NOT NULL | Argon2id hash |
| `first_name` | VARCHAR(100) | NULLABLE | User's first name |
| `last_name` | VARCHAR(100) | NULLABLE | User's last name |
| `avatar_url` | TEXT | NULLABLE | Avatar base64 data URL |
| `theme` | VARCHAR(20) | NOT NULL, default "system" | **NEW** - light/dark/system |
| `email_notifications` | BOOLEAN | NOT NULL, default true | **NEW** - Email prefs |
| `is_active` | BOOLEAN | NOT NULL, default true | Account status |
| `created_at` | TIMESTAMP(tz) | NOT NULL | Registration date |

---

## Schema Definitions

### Backend (Python/SQLModel)

#### User Model Extension

```python
# app/models/user.py - ADD fields
theme: str = Field(
    default="system",
    max_length=20,
    nullable=False,
    description="Theme preference: light, dark, or system",
)
email_notifications: bool = Field(
    default=True,
    nullable=False,
    description="Email notification preference",
)
```

#### New Schemas

```python
# app/schemas/auth.py - ADD schemas

class UserSettingsUpdate(BaseModel):
    """Schema for settings update request (partial update)."""

    theme: str | None = Field(
        default=None,
        pattern="^(light|dark|system)$",
        description="Theme preference",
    )
    email_notifications: bool | None = Field(
        default=None,
        description="Email notification preference",
    )


class UserSettingsResponse(BaseModel):
    """Schema for settings response."""

    theme: str = Field(..., description="Current theme")
    email_notifications: bool = Field(..., description="Email notifications enabled")

    model_config = {"from_attributes": True}


class PasswordChangeRequest(BaseModel):
    """Schema for password change request."""

    current_password: str = Field(
        ...,
        min_length=1,
        description="Current password for verification",
    )
    new_password: str = Field(
        ...,
        min_length=8,
        description="New password (minimum 8 characters)",
    )


class MessageResponse(BaseModel):
    """Generic message response."""

    message: str
```

#### UserResponse Update

```python
# app/schemas/auth.py - MODIFY UserResponse
class UserResponse(BaseModel):
    # ... existing fields ...
    theme: str = Field(..., description="Theme preference")
    email_notifications: bool = Field(..., description="Email notifications")
```

---

### Frontend (TypeScript)

#### Settings Types

```typescript
// src/types/settings.ts

/**
 * Theme options
 */
export type Theme = 'light' | 'dark' | 'system';

/**
 * User settings from backend
 */
export interface UserSettings {
  theme: Theme;
  email_notifications: boolean;
}

/**
 * Settings update request
 */
export interface UserSettingsUpdate {
  theme?: Theme;
  email_notifications?: boolean;
}

/**
 * Password change request
 */
export interface PasswordChangeRequest {
  current_password: string;
  new_password: string;
}

/**
 * Generic message response
 */
export interface MessageResponse {
  message: string;
}
```

---

## Validation Rules

### Password Change

| Field | Rule | Error Message |
|-------|------|---------------|
| `current_password` | required | "Current password is required" |
| `new_password` | min 8 chars | "Password must be at least 8 characters" |
| `confirm_password` | must match new_password | "Passwords do not match" |

### Settings Update

| Field | Rule | Error Message |
|-------|------|---------------|
| `theme` | enum: light/dark/system | "Invalid theme value" |
| `email_notifications` | boolean | N/A (toggle) |

### Zod Schemas (Frontend)

```typescript
// src/lib/validations/settings.ts
import { z } from 'zod';

export const passwordChangeSchema = z
  .object({
    currentPassword: z.string().min(1, 'Current password is required'),
    newPassword: z
      .string()
      .min(8, 'Password must be at least 8 characters'),
    confirmPassword: z.string().min(1, 'Please confirm your password'),
  })
  .refine((data) => data.newPassword === data.confirmPassword, {
    message: 'Passwords do not match',
    path: ['confirmPassword'],
  });

export type PasswordChangeFormData = z.infer<typeof passwordChangeSchema>;
```

---

## State Transitions

### Password Change Flow

```
┌─────────────┐    POST /auth/change-password    ┌─────────────┐
│    Idle     │ ──────────────────────────────► │  Verifying  │
│             │                                  │  Password   │
└─────────────┘                                  └─────────────┘
      ▲                                                │
      │         Success: 200 OK                        │
      │◄───────────────────────────────────────────────┤
      │                                                │
      │         Error: 401 (wrong password)            │
      │◄───────────────────────────────────────────────┘
```

### Account Deletion Flow

```
┌──────────────┐   Open Dialog   ┌──────────────┐   Type DELETE   ┌──────────────┐
│   Settings   │ ──────────────► │  Confirm     │ ───────────────► │   Enabled    │
│    Page      │                 │   Dialog     │                  │   Delete     │
└──────────────┘                 └──────────────┘                  └──────────────┘
                                       │                                 │
                                       │ Cancel                          │ DELETE /auth/me
                                       ▼                                 ▼
                                 ┌──────────────┐                 ┌──────────────┐
                                 │   Closed     │                 │   Redirect   │
                                 │              │                 │   to Home    │
                                 └──────────────┘                 └──────────────┘
```

---

## Database Migration

### Alembic Migration Script

```python
# alembic/versions/YYYYMMDD_add_user_settings_fields.py
"""Add theme and email_notifications to users table

Revision ID: 005
Revises: 004
Create Date: 2025-12-15
"""

from typing import Sequence, Union
import sqlalchemy as sa
from alembic import op

revision: str = "005"
down_revision: Union[str, None] = "004"
branch_labels: Union[str, Sequence[str], None] = None
depends_on: Union[str, Sequence[str], None] = None


def upgrade() -> None:
    # Add theme column with default
    op.add_column(
        'users',
        sa.Column('theme', sa.String(length=20), nullable=False, server_default='system')
    )
    # Add email_notifications column with default
    op.add_column(
        'users',
        sa.Column('email_notifications', sa.Boolean(), nullable=False, server_default='true')
    )


def downgrade() -> None:
    op.drop_column('users', 'email_notifications')
    op.drop_column('users', 'theme')
```

---

## Cascade Delete

When a user is deleted, the following data is cascade deleted:

```
User (DELETE)
  └── Tasks (CASCADE DELETE via owner_id FK)
```

**Implementation**: SQLModel foreign key with `ON DELETE CASCADE` already configured in Task model.
