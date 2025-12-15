# Specification — User Profile Fields (firstName/lastName)

## Feature Overview

Add `first_name` and `last_name` fields to the User model, update the registration endpoint to accept these fields, include them in JWT payload, and expose a computed `display_name` property.

## Current State Analysis

### Existing User Model (`app/models/user.py`)
```python
class User(SQLModel, table=True):
    id: UUID
    email: str
    hashed_password: str
    is_active: bool
    created_at: datetime
```

### Existing Schemas (`app/schemas/auth.py`)
- `UserCreate`: email, password
- `UserResponse`: id, email, is_active, created_at
- `LoginRequest`: email, password
- `TokenResponse`: access_token, token_type

### Existing JWT Payload (`app/core/security.py`)
```python
{
    "sub": str(user_id),
    "email": email,
    "iat": now,
    "exp": expire,
}
```

---

## Functional Specifications

### 1. User Model Changes

**File:** `app/models/user.py`

Add two new fields:
| Field | Type | Constraints | Default |
|-------|------|-------------|---------|
| `first_name` | `str` | max_length=100, nullable | `None` |
| `last_name` | `str` | max_length=100, nullable | `None` |

Add computed property:
```python
@property
def display_name(self) -> str:
    """Returns full name if available, otherwise email username."""
    if self.first_name and self.last_name:
        return f"{self.first_name} {self.last_name}"
    elif self.first_name:
        return self.first_name
    elif self.last_name:
        return self.last_name
    return self.email.split("@")[0]
```

**Validation:**
- `first_name`: Strip whitespace, allow empty string (stored as None)
- `last_name`: Strip whitespace, allow empty string (stored as None)

### 2. Schema Changes

**File:** `app/schemas/auth.py`

#### UserCreate (Registration Request)
Add optional fields:
```python
first_name: str | None = Field(
    default=None,
    max_length=100,
    description="User's first name",
    json_schema_extra={"example": "John"},
)
last_name: str | None = Field(
    default=None,
    max_length=100,
    description="User's last name",
    json_schema_extra={"example": "Doe"},
)
```

#### UserResponse
Add fields:
```python
first_name: str | None = Field(None, description="User's first name")
last_name: str | None = Field(None, description="User's last name")
display_name: str = Field(..., description="Computed display name")
```

### 3. JWT Payload Update

**File:** `app/core/security.py`

Update `create_access_token()` signature:
```python
def create_access_token(
    user_id: UUID,
    email: str,
    display_name: str,  # NEW
    expires_delta: timedelta | None = None,
) -> str:
```

New payload structure:
```python
{
    "sub": str(user_id),
    "email": email,
    "name": display_name,  # NEW - use "name" for frontend compatibility
    "iat": now,
    "exp": expire,
}
```

### 4. Auth Service Update

**File:** `app/services/auth_service.py`

Update `register_user()` to accept and store new fields:
```python
async def register_user(
    session: AsyncSession,
    email: str,
    password: str,
    first_name: str | None = None,  # NEW
    last_name: str | None = None,   # NEW
) -> User | None:
```

### 5. Auth Routes Update

**File:** `app/api/routes/auth.py`

Update registration endpoint to:
1. Accept `first_name` and `last_name` from `UserCreate`
2. Pass them to `register_user()`

Update login endpoint to:
1. Pass `user.display_name` to `create_access_token()`

Update `/auth/me` endpoint response includes new fields automatically via `UserResponse`.

### 6. Database Migration

**File:** `alembic/versions/20251215_000000_add_user_profile_fields.py`

```python
revision: str = "003"
down_revision: str = "002"

def upgrade() -> None:
    op.add_column("users", sa.Column("first_name", sa.String(100), nullable=True))
    op.add_column("users", sa.Column("last_name", sa.String(100), nullable=True))

def downgrade() -> None:
    op.drop_column("users", "last_name")
    op.drop_column("users", "first_name")
```

---

## API Contract Changes

### POST /auth/register

**Request (Updated):**
```json
{
  "email": "user@example.com",
  "password": "securepassword123",
  "first_name": "John",
  "last_name": "Doe"
}
```

**Response (Updated):**
```json
{
  "id": "uuid-here",
  "email": "user@example.com",
  "first_name": "John",
  "last_name": "Doe",
  "display_name": "John Doe",
  "is_active": true,
  "created_at": "2025-12-15T00:00:00Z"
}
```

### POST /auth/login

**Response (Unchanged structure):**
```json
{
  "access_token": "eyJ...",
  "token_type": "bearer"
}
```

**JWT Payload (Updated):**
```json
{
  "sub": "user-uuid",
  "email": "user@example.com",
  "name": "John Doe",
  "iat": 1734220800,
  "exp": 1734222600
}
```

### GET /auth/me

**Response (Updated):**
```json
{
  "id": "uuid-here",
  "email": "user@example.com",
  "first_name": "John",
  "last_name": "Doe",
  "display_name": "John Doe",
  "is_active": true,
  "created_at": "2025-12-15T00:00:00Z"
}
```

---

## Constraints

1. **Backward Compatibility:**
   - `first_name` and `last_name` are optional (nullable)
   - Existing users without names use email username as `display_name`
   - Registration works with or without name fields

2. **Database:**
   - Fields are nullable to support existing users
   - No index needed on name fields (not used for lookups)
   - Migration is non-breaking (adds columns only)

3. **Validation:**
   - Max length: 100 characters each
   - Whitespace-only values stored as NULL
   - No special character restrictions

---

## Acceptance Criteria

- [ ] User model has `first_name` and `last_name` fields
- [ ] User model has `display_name` computed property
- [ ] Registration accepts optional `first_name`/`last_name`
- [ ] UserResponse includes all three name fields
- [ ] JWT payload includes `name` claim with display_name value
- [ ] Alembic migration adds columns without data loss
- [ ] Existing users show email username as display_name
- [ ] All existing tests pass
- [ ] New tests cover name field scenarios

---

## Files to Modify

| File | Changes |
|------|---------|
| `app/models/user.py` | Add fields + property |
| `app/schemas/auth.py` | Update UserCreate, UserResponse |
| `app/core/security.py` | Update create_access_token signature/payload |
| `app/services/auth_service.py` | Update register_user signature |
| `app/api/routes/auth.py` | Pass new fields through |
| `alembic/versions/003_*.py` | New migration file |

---

## Out of Scope

- Profile update endpoint (separate feature)
- Avatar/profile picture
- Additional profile fields (phone, bio, etc.)
- Name uniqueness constraints
- Frontend changes (separate task)
