# Quickstart: Phase 2 Part 2 - Authentication

## Prerequisites

- Python 3.13+
- UV package manager
- Neon PostgreSQL database (from Part 1)
- Git

## Setup Steps

### 1. Navigate to Backend

```bash
cd phase2/backend
```

### 2. Install New Dependencies

```bash
uv add python-jose[cryptography] argon2-cffi slowapi aiosqlite
```

### 3. Update Environment Variables

Add to `.env`:

```env
# Existing from Part 1
DATABASE_URL=postgresql+asyncpg://user:pass@host/db?sslmode=require
TEST_DATABASE_URL=postgresql+asyncpg://user:pass@host/test_db?sslmode=require

# New for Part 2
JWT_SECRET_KEY=your-super-secret-key-min-32-chars-long
JWT_ALGORITHM=HS256
ACCESS_TOKEN_EXPIRE_MINUTES=30
```

**Generate secure JWT secret**:
```bash
python -c "import secrets; print(secrets.token_urlsafe(32))"
```

### 4. Run Migration

```bash
# Generate migration (if not using provided)
alembic revision --autogenerate -m "Add users table"

# Apply migration
alembic upgrade head
```

### 5. Verify Database

```bash
# Connect to Neon and check tables
psql $DATABASE_URL -c "\dt"

# Expected output:
#  Schema |  Name  | Type
# --------+--------+-------
#  public | tasks  | table
#  public | users  | table
```

### 6. Run Tests

```bash
# Unit tests (SQLite)
pytest tests/unit -v

# Integration tests (Neon)
pytest tests/integration -v --tb=short

# All tests with coverage
pytest --cov=app --cov-report=term-missing
```

### 7. Start Development Server

```bash
uvicorn app.main:app --reload --port 8000
```

## Quick Verification

### Register User

```bash
curl -X POST http://localhost:8000/auth/register \
  -H "Content-Type: application/json" \
  -d '{"email": "test@example.com", "password": "password123"}'
```

Expected response (201):
```json
{
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "email": "test@example.com",
  "is_active": true,
  "created_at": "2025-12-14T10:00:00Z"
}
```

### Login

```bash
curl -X POST http://localhost:8000/auth/login \
  -H "Content-Type: application/json" \
  -d '{"email": "test@example.com", "password": "password123"}'
```

Expected response (200):
```json
{
  "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "bearer"
}
```

### Get Profile

```bash
TOKEN="eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9..."

curl http://localhost:8000/auth/me \
  -H "Authorization: Bearer $TOKEN"
```

Expected response (200):
```json
{
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "email": "test@example.com",
  "is_active": true,
  "created_at": "2025-12-14T10:00:00Z"
}
```

### Create Task (Protected)

```bash
curl -X POST http://localhost:8000/tasks \
  -H "Authorization: Bearer $TOKEN" \
  -H "Content-Type: application/json" \
  -d '{"title": "My first task", "description": "Test task"}'
```

Expected response (201):
```json
{
  "id": "...",
  "user_id": "550e8400-e29b-41d4-a716-446655440000",
  "title": "My first task",
  "description": "Test task",
  "status": "pending",
  "created_at": "...",
  "updated_at": "..."
}
```

## API Documentation

After starting the server:
- Swagger UI: http://localhost:8000/docs
- ReDoc: http://localhost:8000/redoc
- OpenAPI JSON: http://localhost:8000/openapi.json

## Troubleshooting

### "JWT_SECRET_KEY not set"

Ensure `.env` file exists and contains `JWT_SECRET_KEY`:
```bash
echo "JWT_SECRET_KEY=$(python -c 'import secrets; print(secrets.token_urlsafe(32))')" >> .env
```

### "Email already registered" on first register

Database may have stale data. Reset:
```bash
alembic downgrade base
alembic upgrade head
```

### Rate limit hit during testing

Wait 60 seconds or restart the server (clears in-memory rate limit state).

### SQLite tests failing with async errors

Ensure `aiosqlite` is installed:
```bash
uv add aiosqlite
```

## File Checklist

After implementation, verify these files exist:

```
phase2/backend/
├── app/
│   ├── main.py                    ✓
│   ├── core/
│   │   ├── config.py              ✓ (extended)
│   │   ├── database.py            ✓ (existing)
│   │   └── security.py            ✓ (new)
│   ├── models/
│   │   ├── task.py                ✓ (existing)
│   │   └── user.py                ✓ (new)
│   ├── schemas/
│   │   ├── auth.py                ✓ (new)
│   │   └── task.py                ✓ (new)
│   ├── services/
│   │   ├── auth_service.py        ✓ (new)
│   │   └── task_service.py        ✓ (new)
│   └── api/
│       ├── deps.py                ✓ (new)
│       └── routes/
│           ├── auth.py            ✓ (new)
│           └── tasks.py           ✓ (new)
├── alembic/versions/
│   └── 002_add_users_table.py     ✓ (new)
└── tests/
    ├── conftest.py                ✓ (new)
    ├── unit/
    │   ├── test_security.py       ✓ (new)
    │   └── test_auth_service.py   ✓ (new)
    └── integration/
        ├── test_auth_endpoints.py ✓ (new)
        └── test_task_ownership.py ✓ (new)
```
