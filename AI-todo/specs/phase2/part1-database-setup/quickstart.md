# Quickstart: Phase 2 Part 1 - Database Setup

**Feature**: Database Foundation for Multi-User Todo Application
**Date**: 2025-12-13

---

## Prerequisites

1. **Python 3.13+** installed
2. **UV package manager** installed (`pip install uv`)
3. **Neon account** with:
   - Production database created
   - Test database created (separate from production)
4. **Connection strings** for both databases

---

## 1. Environment Setup

### Create Environment File

```bash
cd phase2/backend
cp .env.example .env
```

### Configure `.env`

```env
# Production database
DATABASE_URL=postgresql+asyncpg://user:pass@ep-xxx.neon.tech/neondb?sslmode=require

# Test database (separate Neon project/branch)
TEST_DATABASE_URL=postgresql+asyncpg://user:pass@ep-yyy.neon.tech/testdb?sslmode=require
```

### Install Dependencies

```bash
uv sync
```

---

## 2. Verify Installation

### Check SQLModel Installation

```bash
uv run python -c "import sqlmodel; print(sqlmodel.__version__)"
```

Expected: `0.0.14` or higher

### Check Database Connection

```bash
uv run python -c "
import asyncio
from app.core.database import get_async_engine

async def test():
    engine = get_async_engine()
    async with engine.connect() as conn:
        result = await conn.execute(text('SELECT 1'))
        print('Connection OK:', result.scalar())

asyncio.run(test())
"
```

Expected: `Connection OK: 1`

---

## 3. Run Migrations

### Apply All Migrations

```bash
cd phase2/backend
uv run alembic upgrade head
```

Expected output:
```
INFO  [alembic.runtime.migration] Running upgrade  -> xxxx, create tasks table
```

### Verify Migration

```bash
uv run alembic current
```

Expected: Shows current revision hash

### Rollback (if needed)

```bash
uv run alembic downgrade -1
```

---

## 4. Run Tests

### Unit Tests (SQLite)

```bash
cd phase2/backend
uv run pytest tests/unit/ -v
```

Expected: All tests pass

### Integration Tests (Neon)

```bash
cd phase2/backend
uv run pytest tests/integration/ -v
```

Expected: All tests pass

### Full Test Suite with Coverage

```bash
cd phase2/backend
uv run pytest --cov=app --cov-report=term-missing
```

Expected: >= 80% coverage on `app/models/`

---

## 5. Verify Model

### Interactive Check

```bash
uv run python
```

```python
from uuid import uuid4
from app.models.task import Task, TaskStatus

# Valid task
task = Task(
    user_id=uuid4(),
    title="My first task",
    description="This is a test",
    status=TaskStatus.PENDING
)
print(task)
print(f"ID: {task.id}")
print(f"Created: {task.created_at}")

# Invalid task (should raise ValidationError)
try:
    invalid = Task(user_id=uuid4(), title="")
except Exception as e:
    print(f"Validation works: {e}")
```

---

## 6. Directory Structure After Setup

```
phase2/
└── backend/
    ├── app/
    │   ├── __init__.py
    │   ├── core/
    │   │   ├── __init__.py
    │   │   ├── config.py
    │   │   └── database.py
    │   └── models/
    │       ├── __init__.py
    │       └── task.py
    ├── alembic/
    │   ├── versions/
    │   │   └── xxxx_create_tasks_table.py
    │   ├── env.py
    │   └── script.py.mako
    ├── alembic.ini
    ├── tests/
    │   ├── __init__.py
    │   ├── conftest.py
    │   ├── unit/
    │   │   └── test_task_model.py
    │   └── integration/
    │       └── test_task_db.py
    ├── pyproject.toml
    ├── .env
    └── .env.example
```

---

## Common Issues

### SSL Connection Error

**Error**: `SSL SYSCALL error: Connection reset by peer`

**Solution**: Ensure `sslmode=require` is in DATABASE_URL

---

### Import Error: No module named 'app'

**Solution**: Run commands from `phase2/backend/` directory, or:
```bash
export PYTHONPATH="${PYTHONPATH}:$(pwd)/phase2/backend"
```

---

### Alembic: Target database is not up to date

**Solution**:
```bash
uv run alembic stamp head  # Mark current state
uv run alembic upgrade head
```

---

### Test Database Isolation

**Symptom**: Tests affecting each other

**Solution**: Ensure `conftest.py` uses transaction rollback:
```python
@pytest.fixture
async def async_session():
    async with async_session_factory() as session:
        async with session.begin():
            yield session
            await session.rollback()
```

---

## Next Steps

After Part 1 is complete:

1. **Part 2**: Add User model and FK constraint
2. **Part 3**: Create FastAPI endpoints using this database layer
3. **Part 4+**: Add frontend components

---

## Useful Commands

| Command | Purpose |
|---------|---------|
| `uv run alembic revision --autogenerate -m "description"` | Create new migration |
| `uv run alembic upgrade head` | Apply all migrations |
| `uv run alembic downgrade -1` | Rollback one migration |
| `uv run alembic history` | Show migration history |
| `uv run pytest -v` | Run all tests |
| `uv run pytest --cov=app` | Run tests with coverage |
| `uv run mypy app/` | Type check |
