# Specification — Phase 2 Part 2: Authentication

> **Status:** Draft
> **Created:** 2025-12-14
> **Feature:** User Authentication & Task Ownership

---

## 1. Requirements

### 1.1 Functional Requirements

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-01 | Users can register with email and password | Must |
| FR-02 | Users can login and receive JWT access token | Must |
| FR-03 | Users can retrieve their profile via protected endpoint | Must |
| FR-04 | Tasks are scoped to the authenticated user | Must |
| FR-05 | Users cannot access other users' tasks | Must |
| FR-06 | Rate limiting on auth endpoints prevents brute-force | Should |
| FR-07 | Inactive users cannot authenticate | Should |

### 1.2 Non-Functional Requirements

| ID | Requirement | Target |
|----|-------------|--------|
| NFR-01 | Password hashing uses Argon2id | Security |
| NFR-02 | JWT tokens expire in 30 minutes | Security |
| NFR-03 | Auth endpoints respond < 500ms p95 | Performance |
| NFR-04 | Rate limit: 5 req/min register, 10 req/min login | Security |

### 1.3 Out of Scope (Deferred to Part 3)

- Email verification
- Password reset flow
- Refresh tokens
- User profile fields (username, avatar)
- OAuth/social login

---

## 2. Architecture Decisions

### 2.1 Authentication Strategy: JWT

**Decision:** Stateless JWT authentication

**Rationale:**
- Stateless: No server-side session storage required
- Scalable: Works across multiple API instances
- Standard: Well-supported in FastAPI ecosystem
- API-first: Ideal for REST API consumption

**Trade-offs:**
- Cannot revoke tokens (mitigated by short expiry)
- Token size larger than session ID
- Requires secure client-side storage

### 2.2 Password Hashing: Argon2id

**Decision:** Use Argon2id via `argon2-cffi`

**Rationale:**
- Winner of Password Hashing Competition (2015)
- Resistant to GPU/ASIC attacks
- Memory-hard algorithm
- Recommended by OWASP

**Configuration:**
```python
# Default argon2-cffi parameters (secure defaults)
time_cost=3
memory_cost=65536  # 64 MB
parallelism=4
hash_len=32
salt_len=16
```

### 2.3 Task Ownership: Service-Layer Filtering

**Decision:** Filter tasks by user_id at service layer, not middleware

**Rationale:**
- More explicit and testable
- Allows flexible authorization logic
- Keeps routes clean via dependency injection
- Easier to audit and reason about

---

## 3. Implementation Details

### 3.1 Database Models (SQLModel)

```python
from datetime import datetime
from uuid import UUID, uuid4
from sqlmodel import SQLModel, Field

class User(SQLModel, table=True):
    """User account for authentication."""
    __tablename__ = "users"

    id: UUID = Field(default_factory=uuid4, primary_key=True)
    email: str = Field(unique=True, index=True, max_length=255)
    hashed_password: str = Field(max_length=255)
    is_active: bool = Field(default=True)
    created_at: datetime = Field(default_factory=datetime.utcnow)


class Task(SQLModel, table=True):
    """Task with user ownership."""
    __tablename__ = "tasks"

    id: UUID = Field(default_factory=uuid4, primary_key=True)
    title: str = Field(max_length=255)
    description: str | None = Field(default=None, max_length=1000)
    status: str = Field(default="pending")  # pending | completed
    user_id: UUID = Field(foreign_key="users.id", index=True)
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)
```

### 3.2 Pydantic Schemas

```python
from datetime import datetime
from uuid import UUID
from pydantic import BaseModel, EmailStr, Field

# === Auth Requests ===

class UserCreate(BaseModel):
    """Registration request."""
    email: EmailStr
    password: str = Field(min_length=8, max_length=128)


class LoginRequest(BaseModel):
    """Login request."""
    email: EmailStr
    password: str


# === Auth Responses ===

class TokenResponse(BaseModel):
    """JWT token response."""
    access_token: str
    token_type: str = "bearer"


class UserResponse(BaseModel):
    """User profile response."""
    id: UUID
    email: str
    is_active: bool
    created_at: datetime

    class Config:
        from_attributes = True


# === Error Responses ===

class ErrorResponse(BaseModel):
    """Standard error response."""
    detail: str


class ValidationErrorResponse(BaseModel):
    """Validation error response."""
    detail: list[dict]
```

### 3.3 API Endpoints

| Method | Endpoint | Auth | Request Body | Response | Status Codes |
|--------|----------|------|--------------|----------|--------------|
| POST | `/auth/register` | No | `UserCreate` | `UserResponse` | 201, 400, 409, 429 |
| POST | `/auth/login` | No | `LoginRequest` | `TokenResponse` | 200, 401, 429 |
| GET | `/auth/me` | Yes | - | `UserResponse` | 200, 401 |

**Error Codes:**
- `400` - Validation error (invalid email format, weak password)
- `401` - Invalid credentials or missing/invalid token
- `409` - Email already registered
- `429` - Rate limit exceeded

### 3.4 JWT Configuration

```python
from datetime import timedelta

JWT_CONFIG = {
    "SECRET_KEY": "env:JWT_SECRET_KEY",  # Load from environment
    "ALGORITHM": "HS256",
    "ACCESS_TOKEN_EXPIRE_MINUTES": 30,
}

# Token payload structure
{
    "sub": "user-uuid-here",      # Subject (user ID)
    "email": "user@example.com",  # User email
    "exp": 1734200000,            # Expiration timestamp
    "iat": 1734198200,            # Issued at timestamp
}
```

### 3.5 Rate Limiting Configuration

```python
from slowapi import Limiter
from slowapi.util import get_remote_address

limiter = Limiter(key_func=get_remote_address)

RATE_LIMITS = {
    "register": "5/minute",
    "login": "10/minute",
}
```

### 3.6 Project Structure

```
phase2/
├── src/
│   ├── api/
│   │   ├── __init__.py
│   │   ├── deps.py           # Dependency injection (get_current_user)
│   │   └── routes/
│   │       ├── __init__.py
│   │       ├── auth.py       # Auth endpoints
│   │       └── tasks.py      # Task endpoints (updated)
│   ├── core/
│   │   ├── __init__.py
│   │   ├── config.py         # Settings and JWT config
│   │   └── security.py       # Password hashing, JWT utils
│   ├── models/
│   │   ├── __init__.py
│   │   ├── user.py           # User SQLModel
│   │   └── task.py           # Task SQLModel (updated)
│   ├── schemas/
│   │   ├── __init__.py
│   │   ├── auth.py           # Auth Pydantic schemas
│   │   └── task.py           # Task Pydantic schemas
│   ├── services/
│   │   ├── __init__.py
│   │   ├── auth_service.py   # Auth business logic
│   │   └── task_service.py   # Task service (updated)
│   └── db/
│       ├── __init__.py
│       └── session.py        # Database session
├── alembic/
│   ├── versions/
│   │   └── 002_add_users_table.py
│   └── env.py
├── tests/
│   ├── unit/
│   │   ├── test_auth_service.py
│   │   ├── test_security.py
│   │   └── test_user_model.py
│   └── integration/
│       ├── test_auth_endpoints.py
│       └── test_task_ownership.py
└── requirements.txt
```

### 3.7 Alembic Migration Plan

**Migration: `002_add_users_table.py`**

```python
"""Add users table and task.user_id foreign key

Revision ID: 002
Revises: 001
Create Date: 2025-12-14
"""
from alembic import op
import sqlalchemy as sa
from sqlalchemy.dialects.postgresql import UUID

revision = '002'
down_revision = '001'
branch_labels = None
depends_on = None


def upgrade() -> None:
    # Create users table
    op.create_table(
        'users',
        sa.Column('id', UUID(as_uuid=True), primary_key=True),
        sa.Column('email', sa.String(255), unique=True, nullable=False),
        sa.Column('hashed_password', sa.String(255), nullable=False),
        sa.Column('is_active', sa.Boolean(), default=True, nullable=False),
        sa.Column('created_at', sa.DateTime(), nullable=False),
    )
    op.create_index('ix_users_email', 'users', ['email'])

    # Add user_id to tasks table
    op.add_column(
        'tasks',
        sa.Column('user_id', UUID(as_uuid=True), nullable=True)
    )
    op.create_index('ix_tasks_user_id', 'tasks', ['user_id'])
    op.create_foreign_key(
        'fk_tasks_user_id',
        'tasks', 'users',
        ['user_id'], ['id'],
        ondelete='CASCADE'
    )


def downgrade() -> None:
    op.drop_constraint('fk_tasks_user_id', 'tasks', type_='foreignkey')
    op.drop_index('ix_tasks_user_id', 'tasks')
    op.drop_column('tasks', 'user_id')
    op.drop_index('ix_users_email', 'users')
    op.drop_table('users')
```

**Migration Strategy:**
1. Deploy migration with `user_id` as nullable
2. Run data migration to assign existing tasks (if any) to a default user or delete
3. Run second migration to make `user_id` NOT NULL

### 3.8 Dependencies

```
# requirements.txt additions
python-jose[cryptography]==3.3.0
argon2-cffi==23.1.0
slowapi==0.1.9
```

---

## 4. Success Criteria

### 4.1 Test Cases Matrix

| ID | Category | Test Case | Expected Result | DB |
|----|----------|-----------|-----------------|-----|
| TC-01 | Register | Valid email and password | 201 + UserResponse | Both |
| TC-02 | Register | Duplicate email | 409 Conflict | Both |
| TC-03 | Register | Invalid email format | 400 Validation Error | Unit |
| TC-04 | Register | Password < 8 chars | 400 Validation Error | Unit |
| TC-05 | Register | Password > 128 chars | 400 Validation Error | Unit |
| TC-06 | Login | Correct credentials | 200 + TokenResponse | Both |
| TC-07 | Login | Wrong password | 401 Unauthorized | Both |
| TC-08 | Login | Non-existent email | 401 Unauthorized | Both |
| TC-09 | Login | Inactive user | 401 Unauthorized | Integration |
| TC-10 | Me | Valid token | 200 + UserResponse | Both |
| TC-11 | Me | No token | 401 Unauthorized | Unit |
| TC-12 | Me | Expired token | 401 Unauthorized | Unit |
| TC-13 | Me | Invalid token | 401 Unauthorized | Unit |
| TC-14 | Tasks | User A creates task | Task has user_id = A | Both |
| TC-15 | Tasks | User A lists tasks | Only A's tasks returned | Both |
| TC-16 | Tasks | User A gets B's task | 404 Not Found | Integration |
| TC-17 | Tasks | User A updates B's task | 404 Not Found | Integration |
| TC-18 | Tasks | User A deletes B's task | 404 Not Found | Integration |
| TC-19 | Rate Limit | 6th register in 1 min | 429 Too Many Requests | Integration |
| TC-20 | Rate Limit | 11th login in 1 min | 429 Too Many Requests | Integration |

### 4.2 Acceptance Checklist

- [ ] User can register with valid email/password
- [ ] Duplicate email registration returns 409
- [ ] User can login and receive JWT token
- [ ] Invalid credentials return 401
- [ ] `/auth/me` returns current user with valid token
- [ ] `/auth/me` returns 401 without token
- [ ] Tasks are filtered by authenticated user
- [ ] User cannot access another user's tasks
- [ ] Rate limiting prevents brute-force attacks
- [ ] Passwords are hashed with Argon2id
- [ ] JWT tokens expire after 30 minutes
- [ ] All unit tests pass with SQLite
- [ ] All integration tests pass with Neon PostgreSQL
- [ ] Alembic migration runs successfully

### 4.3 Definition of Done

1. **Code Complete:** All endpoints implemented per spec
2. **Tests Pass:** 100% of test cases green
3. **Security Review:** Password hashing and JWT verified
4. **Migration Verified:** Alembic up/down works on Neon
5. **Documentation:** API docs auto-generated via FastAPI
6. **PHR Created:** Prompt history recorded

---

## Clarifications

### Session 2025-12-14

- Q: JWT Secret - generate new or use existing from Phase 1? → A: New random secret, minimum 32 characters, loaded from `JWT_SECRET_KEY` environment variable
- Q: Rate Limiting Backend - Redis or in-memory? → A: In-memory (slowapi default) for dev/MVP; Redis deferred until horizontal scaling required
- Q: Task.user_id FK - add constraint now or allow NULL? → A: Nullable initially in migration, then NOT NULL in follow-up migration (safe migration strategy)
- Q: Email Validation - EmailStr or custom regex? → A: Pydantic EmailStr sufficient (RFC 5322 compliant); no domain blocking for MVP
- Q: Password Policy - min 8 chars or stronger? → A: Minimum 8 characters only; stronger rules (uppercase, numbers, special chars) deferred to Part 3
- Q: Refresh Tokens - include in Phase 2? → A: Defer to Part 3; access token only with 30-minute expiry for MVP
- Q: Alembic Migration Order? → A: Users table first → FK constraint on tasks.user_id → indexes (standard pattern)

---

## References

- [FastAPI Security](https://fastapi.tiangolo.com/tutorial/security/)
- [python-jose](https://python-jose.readthedocs.io/)
- [argon2-cffi](https://argon2-cffi.readthedocs.io/)
- [OWASP Password Storage](https://cheatsheetseries.owasp.org/cheatsheets/Password_Storage_Cheat_Sheet.html)
