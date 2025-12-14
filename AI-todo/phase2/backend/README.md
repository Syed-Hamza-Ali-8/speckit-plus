# Todo Backend

Multi-user Todo application backend with SQLModel and Neon Postgres.

## Phase 2 Part 1: Database Setup

This module provides the database foundation for the multi-user Todo web application.

### Features

- Task model with UUID primary key and user ownership
- Dual-layer validation (Python + Database constraints)
- Extensible status enum without schema migrations
- Async database connections with SSL support
- Alembic migrations for schema versioning

### Setup

```bash
# Install dependencies
uv sync --all-extras

# Configure environment
cp .env.example .env
# Edit .env with your Neon database credentials

# Run migrations
alembic upgrade head

# Run tests
pytest tests/ -v
```

### Tech Stack

- Python 3.13+
- SQLModel 0.0.14+
- SQLAlchemy 2.0+ (async)
- Neon Postgres (serverless)
- Alembic (migrations)
- pytest + pytest-asyncio (testing)
