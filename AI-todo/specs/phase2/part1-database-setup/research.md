# Research: Phase 2 Part 1 - Database Setup

**Feature**: Database Foundation for Multi-User Todo Application
**Date**: 2025-12-13
**Status**: Complete

---

## 1. SQLModel with Async Support

### Decision
Use `create_async_engine` + `async_sessionmaker` with SQLModel's `AsyncSession` wrapper.

### Rationale
- SQLModel 0.0.14+ requires SQLAlchemy's async constructs directly
- `expire_on_commit=False` is CRITICAL to prevent implicit I/O during async operations
- Pydantic v2 validators use `@field_validator` with `@classmethod` decorator

### Alternatives Considered
| Alternative | Rejected Because |
|-------------|------------------|
| Sync SQLModel only | Neon recommends async for serverless connections |
| Direct asyncpg without SQLModel | Loses ORM benefits, more boilerplate |

### Code Pattern
```python
from sqlmodel.ext.asyncio.session import AsyncSession
from sqlalchemy.ext.asyncio import create_async_engine, async_sessionmaker

engine = create_async_engine(DATABASE_URL, echo=False)
async_session_factory = async_sessionmaker(
    engine, class_=AsyncSession, expire_on_commit=False
)

async def get_async_session() -> AsyncSession:
    async with async_session_factory() as session:
        yield session
```

---

## 2. Neon Postgres Connection

### Decision
Use `postgresql+asyncpg://` scheme with `sslmode=require` query parameter.

### Rationale
- Neon REQUIRES SSL (serverless security mandate)
- asyncpg is the recommended async driver for PostgreSQL
- Query parameter approach is simpler than explicit SSL context

### Connection String Format
```
postgresql+asyncpg://[user]:[password]@[endpoint-id].neon.tech/[dbname]?sslmode=require
```

### Alternatives Considered
| Alternative | Rejected Because |
|-------------|------------------|
| psycopg2 (sync) | Async is preferred for serverless |
| Explicit SSL context | Unnecessary complexity for Neon's defaults |

### Gotcha
asyncpg doesn't support all query params that psycopg2 does (e.g., `channel_binding`). Parse and strip if needed.

---

## 3. Alembic Async Migrations

### Decision
Initialize Alembic with `-t async` template and import `sqlmodel.sql.sqltypes` in mako.

### Rationale
- Async template provides `async_engine_from_config` setup
- SQLModel types need explicit import for proper migration generation
- Naming convention prevents auto-generated constraint name conflicts

### Configuration Pattern
```python
# alembic/env.py
from sqlmodel import SQLModel
from app.models.task import Task  # Import to register metadata

target_metadata = SQLModel.metadata
```

### Alternatives Considered
| Alternative | Rejected Because |
|-------------|------------------|
| Sync Alembic with sync engine | Mismatch with async app code |
| Manual migration files | Error-prone, loses autogenerate benefits |

---

## 4. Non-Native Enum Pattern

### Decision
Use `native_enum=False` with explicit `length=50` for TaskStatus.

### Rationale
- PostgreSQL native ENUMs require `ALTER TYPE` for new values (migration needed)
- VARCHAR storage allows adding statuses without schema migration
- `length=50` provides headroom for future status names

### Code Pattern
```python
from sqlalchemy import Column, Enum as SAEnum

class Task(SQLModel, table=True):
    status: TaskStatus = Field(
        default=TaskStatus.PENDING,
        sa_column=Column(
            SAEnum(TaskStatus, native_enum=False, length=50),
            nullable=False,
        )
    )
```

### Alternatives Considered
| Alternative | Rejected Because |
|-------------|------------------|
| Native PostgreSQL ENUM | Requires migration for each new status |
| Plain VARCHAR without enum | Loses Python type safety |

---

## 5. CHECK Constraints for Title Validation

### Decision
Use `length(trim(title)) > 0` for SQLite compatibility.

### Rationale
- PostgreSQL supports regex (`!~`) but SQLite does not
- `length(trim(title)) > 0` works in both databases
- Python validation is primary; DB constraint is safety net

### Code Pattern
```python
from sqlalchemy import CheckConstraint

class Task(SQLModel, table=True):
    __table_args__ = (
        CheckConstraint(
            "length(trim(title)) > 0",
            name="ck_tasks_title_not_empty"
        ),
    )
```

### Alternatives Considered
| Alternative | Rejected Because |
|-------------|------------------|
| PostgreSQL-only regex | Breaks SQLite unit tests |
| No CHECK constraint | Loses DB-level protection |
| Conditional constraints | Adds complexity, different behavior in tests |

---

## 6. SQLite vs PostgreSQL Compatibility

### Decision
Accept SQLite limitations for unit tests; use Neon for full integration tests.

### Key Differences Impacting Tests

| Feature | PostgreSQL | SQLite | Mitigation |
|---------|-----------|--------|------------|
| TIMESTAMPTZ | Native | TEXT | Python handles conversion |
| UUID | Native | TEXT | SQLModel auto-serializes |
| Regex CHECK | Supported | Not supported | Use length(trim()) |
| Type enforcement | Strict | Loose | Integration tests catch issues |

### Rationale
- Unit tests focus on Python logic, not DB behavior
- Integration tests against Neon catch Postgres-specific issues
- Dual-layer validation (Python + DB) provides defense in depth

### Alternatives Considered
| Alternative | Rejected Because |
|-------------|------------------|
| PGLite for unit tests | Additional dependency, still experimental |
| Postgres Docker for unit tests | Out of scope per spec |
| SQLite only | Misses Postgres-specific behavior |

---

## Summary of Technical Decisions

| Area | Decision | Key Benefit |
|------|----------|-------------|
| Async Engine | SQLAlchemy async with SQLModel | Proper serverless support |
| Connection | asyncpg + sslmode=require | Neon compliance |
| Migrations | Alembic async template | Consistent with app code |
| Enum Storage | native_enum=False | Extensible without migration |
| CHECK Constraint | length(trim()) pattern | SQLite compatible |
| Test Strategy | SQLite unit + Neon integration | Speed + production parity |

---

**References**:
- [FastAPI with Async SQLAlchemy, SQLModel, and Alembic | TestDriven.io](https://testdriven.io/blog/fastapi-sqlmodel/)
- [Connect a Python application to Neon Postgres](https://neon.com/docs/guides/python)
- [The ultimate async setup: FastAPI, SQLModel, Alembic, Pytest](https://medium.com/@estretyakov/the-ultimate-async-setup-fastapi-sqlmodel-alembic-pytest-ae5cdcfed3d4)
