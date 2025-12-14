# Database Contracts: Phase 2 Part 1

**Feature**: Database Foundation for Multi-User Todo Application
**Date**: 2025-12-13

---

## Contract: Database Session Factory

### Function: `get_async_engine()`

**Purpose**: Create and return the async database engine singleton.

**Signature**:
```python
def get_async_engine() -> AsyncEngine
```

**Returns**: SQLAlchemy `AsyncEngine` configured for Neon Postgres

**Environment Variables**:
| Variable | Required | Description |
|----------|----------|-------------|
| `DATABASE_URL` | Yes | Neon Postgres connection string |

**Guarantees**:
- SSL mode is `require`
- Engine is reusable (singleton pattern)
- Connection errors raise `SQLAlchemyError`

---

### Function: `get_async_session()`

**Purpose**: Yield an async database session for dependency injection.

**Signature**:
```python
async def get_async_session() -> AsyncGenerator[AsyncSession, None]
```

**Yields**: SQLModel `AsyncSession` with transaction context

**Behavior**:
- Session auto-commits on successful yield
- Session auto-rollbacks on exception
- `expire_on_commit=False` (prevents implicit I/O)

**Usage**:
```python
# FastAPI dependency (future Part 3)
async def endpoint(session: AsyncSession = Depends(get_async_session)):
    ...

# Direct usage (tests)
async with async_session_factory() as session:
    ...
```

---

## Contract: Task Model

### Class: `Task`

**Purpose**: SQLModel entity representing a todo item.

**Table**: `tasks`

**Fields**:
```python
class Task(SQLModel, table=True):
    id: UUID                    # Primary key
    user_id: UUID               # Owner (indexed)
    title: str                  # Required, max 200 chars
    description: str | None     # Optional, max 1000 chars
    status: TaskStatus          # Enum: pending | completed
    created_at: datetime        # UTC, auto-set
    updated_at: datetime        # UTC, auto-updated
```

**Validation Contract**:

| Input | Expected Behavior |
|-------|-------------------|
| `Task(title="")` | Raises `ValidationError` |
| `Task(title="   ")` | Raises `ValidationError` |
| `Task(title="x"*201)` | Raises `ValidationError` |
| `Task(description="x"*1001)` | Raises `ValidationError` |
| `Task(status="invalid")` | Raises `ValidationError` |
| `Task(title="Valid", user_id=uuid4())` | Creates valid instance |

**Timestamp Contract**:

| Operation | `created_at` | `updated_at` |
|-----------|--------------|--------------|
| INSERT | Set to UTC now | Set to UTC now |
| UPDATE | Unchanged | Set to UTC now |
| SELECT | Returns original | Returns latest |

---

## Contract: Enum TaskStatus

### Class: `TaskStatus`

**Purpose**: Define valid task statuses.

**Values**:
```python
class TaskStatus(str, Enum):
    PENDING = "pending"
    COMPLETED = "completed"
```

**Storage**: VARCHAR(50) (non-native enum)

**Extensibility Contract**:
- Adding new enum values in Python code does NOT require database migration
- New values are stored as VARCHAR strings
- Existing data remains valid

---

## Contract: Alembic Migrations

### Command: `alembic upgrade head`

**Preconditions**:
- `DATABASE_URL` environment variable set
- Database exists and is accessible
- SSL connection available

**Postconditions**:
- `tasks` table exists with all columns
- `ix_tasks_user_id` index exists
- `ck_tasks_title_not_empty` constraint exists
- `alembic_version` table updated

**Idempotency**: Safe to run multiple times (no-op if current)

---

### Command: `alembic downgrade -1`

**Preconditions**:
- At least one migration applied

**Postconditions**:
- Previous migration state restored
- For initial migration: `tasks` table dropped

**Reversibility**: All migrations must support downgrade

---

## Contract: Test Fixtures

### Fixture: `async_session`

**Purpose**: Provide isolated database session for tests.

**Scope**: Function (each test gets fresh session)

**Behavior**:
```python
@pytest.fixture
async def async_session():
    async with async_session_factory() as session:
        async with session.begin():
            yield session
            await session.rollback()  # Isolation
```

**Guarantees**:
- Test data is rolled back after each test
- No test affects another test's data
- Works with both SQLite (unit) and Neon (integration)

---

### Fixture: `sample_task`

**Purpose**: Provide valid Task instance for tests.

**Returns**:
```python
Task(
    user_id=uuid4(),  # Random UUID per spec
    title="Test Task",
    description="Test Description",
    status=TaskStatus.PENDING,
)
```

**Guarantees**:
- Valid according to all validators
- Ready for database insertion
- Does not persist (test must save if needed)
