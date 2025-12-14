# Quickstart — Phase 2 Part 3: Task CRUD API

> **Created:** 2025-12-14
> **Prerequisites:** Phase 2 Part 2 (Authentication) completed

---

## Overview

This guide covers implementing the Task CRUD API with:
- Query parameters (filtering, sorting, pagination)
- Rate limiting on write endpoints
- Updated response schemas (hiding `user_id`)

## Prerequisites Check

```bash
# Verify Part 2 is working
cd phase2/backend
source .venv/Scripts/activate  # Windows
# source .venv/bin/activate     # Linux/Mac

# Run existing tests
pytest tests/ -v

# Start the server
uvicorn app.main:app --reload
```

Test authentication:
```bash
# Register a user
curl -X POST http://localhost:8000/auth/register \
  -H "Content-Type: application/json" \
  -d '{"email": "test@example.com", "password": "password123"}'

# Login and get token
curl -X POST http://localhost:8000/auth/login \
  -H "Content-Type: application/json" \
  -d '{"email": "test@example.com", "password": "password123"}'

# Save the access_token for use below
export TOKEN="<your-access-token>"
```

---

## Implementation Steps

### Step 1: Update Schemas

**File:** `app/schemas/task.py`

Add new schemas:

```python
# Add to existing imports
from datetime import date

# Add TaskQueryParams for query validation
class TaskQueryParams(BaseModel):
    """Query parameters for listing tasks."""
    status: TaskStatus | None = None
    created_after: date | None = None
    created_before: date | None = None
    sort: str = "created_at:desc"
    limit: int = Field(default=20, ge=1, le=100)
    offset: int = Field(default=0, ge=0)

# Add TaskRead (without user_id)
class TaskRead(BaseModel):
    """Response schema for task (user_id hidden)."""
    id: UUID
    title: str
    description: str | None
    status: TaskStatus
    created_at: datetime
    updated_at: datetime

    model_config = {"from_attributes": True}

# Add PaginatedTaskResponse
class PaginatedTaskResponse(BaseModel):
    """Paginated response for task listing."""
    items: list[TaskRead]
    total: int
    limit: int
    offset: int
```

### Step 2: Extend Task Service

**File:** `app/services/task_service.py`

Update `get_tasks()` to support filtering, sorting, and pagination:

```python
from datetime import date
from sqlalchemy import func

async def get_tasks(
    db: AsyncSession,
    user_id: UUID,
    status: TaskStatus | None = None,
    created_after: date | None = None,
    created_before: date | None = None,
    sort_field: str = "created_at",
    sort_direction: str = "desc",
    limit: int = 20,
    offset: int = 0,
) -> tuple[list[Task], int]:
    """Get paginated tasks with filtering and sorting.

    Returns:
        Tuple of (tasks list, total count)
    """
    # Base query with user filter
    statement = select(Task).where(Task.user_id == user_id)

    # Apply filters
    if status is not None:
        statement = statement.where(Task.status == status)
    if created_after is not None:
        statement = statement.where(Task.created_at >= created_after)
    if created_before is not None:
        statement = statement.where(Task.created_at <= created_before)

    # Get total count (before pagination)
    count_statement = select(func.count()).select_from(statement.subquery())
    total = await db.scalar(count_statement) or 0

    # Apply sorting
    sort_column = getattr(Task, sort_field)
    if sort_direction == "desc":
        statement = statement.order_by(sort_column.desc())
    else:
        statement = statement.order_by(sort_column.asc())

    # Apply pagination
    statement = statement.offset(offset).limit(limit)

    result = await db.exec(statement)
    return list(result.all()), total
```

### Step 3: Update Task Router

**File:** `app/api/routes/tasks.py`

```python
from datetime import date
from fastapi import Query

from app.schemas.task import (
    TaskCreate, TaskRead, TaskUpdate, PaginatedTaskResponse
)
from app.middleware.rate_limit import limiter

# Allowed sort fields for validation
ALLOWED_SORT_FIELDS = {"created_at", "updated_at", "title", "status"}

def parse_sort(sort: str) -> tuple[str, str]:
    """Parse and validate sort parameter."""
    parts = sort.split(":")
    if len(parts) != 2:
        raise HTTPException(400, "Sort must be 'field:direction'")
    field, direction = parts
    if field not in ALLOWED_SORT_FIELDS:
        raise HTTPException(400, f"Invalid sort field: {field}")
    if direction not in {"asc", "desc"}:
        raise HTTPException(400, f"Invalid sort direction: {direction}")
    return field, direction


@router.get("", response_model=PaginatedTaskResponse)
@limiter.limit("100/hour")
async def list_tasks(
    request: Request,
    db: DbSession,
    current_user: CurrentUser,
    status: TaskStatus | None = Query(default=None),
    created_after: date | None = Query(default=None),
    created_before: date | None = Query(default=None),
    sort: str = Query(default="created_at:desc"),
    limit: int = Query(default=20, ge=1, le=100),
    offset: int = Query(default=0, ge=0),
) -> PaginatedTaskResponse:
    """List tasks with filtering, sorting, and pagination."""
    sort_field, sort_direction = parse_sort(sort)

    tasks, total = await task_service.get_tasks(
        db, current_user.id,
        status=status,
        created_after=created_after,
        created_before=created_before,
        sort_field=sort_field,
        sort_direction=sort_direction,
        limit=limit,
        offset=offset,
    )

    return PaginatedTaskResponse(
        items=[TaskRead.model_validate(t) for t in tasks],
        total=total,
        limit=limit,
        offset=offset,
    )


@router.post("", response_model=TaskRead, status_code=201)
@limiter.limit("30/hour")
async def create_task(
    request: Request,
    data: TaskCreate,
    db: DbSession,
    current_user: CurrentUser,
) -> TaskRead:
    """Create a new task."""
    task = await task_service.create_task(db, current_user.id, data)
    return TaskRead.model_validate(task)


# Update other endpoints similarly...
```

### Step 4: Configure Rate Limiting

**File:** `app/middleware/rate_limit.py`

```python
from slowapi import Limiter
from slowapi.util import get_remote_address
from starlette.requests import Request

def get_user_key(request: Request) -> str:
    """Get rate limit key from user ID or IP."""
    if hasattr(request.state, "user") and request.state.user:
        return f"user:{request.state.user.id}"
    return get_remote_address(request)

limiter = Limiter(key_func=get_user_key)
```

---

## Testing

### Manual Testing

```bash
# List tasks (empty)
curl -X GET "http://localhost:8000/tasks" \
  -H "Authorization: Bearer $TOKEN"

# Create tasks
curl -X POST "http://localhost:8000/tasks" \
  -H "Authorization: Bearer $TOKEN" \
  -H "Content-Type: application/json" \
  -d '{"title": "Task 1", "description": "First task"}'

curl -X POST "http://localhost:8000/tasks" \
  -H "Authorization: Bearer $TOKEN" \
  -H "Content-Type: application/json" \
  -d '{"title": "Task 2", "description": "Second task"}'

# List with pagination
curl -X GET "http://localhost:8000/tasks?limit=1&offset=0" \
  -H "Authorization: Bearer $TOKEN"

# List with filtering
curl -X GET "http://localhost:8000/tasks?status=pending" \
  -H "Authorization: Bearer $TOKEN"

# List with sorting
curl -X GET "http://localhost:8000/tasks?sort=title:asc" \
  -H "Authorization: Bearer $TOKEN"

# Update task status
curl -X PATCH "http://localhost:8000/tasks/<task-id>" \
  -H "Authorization: Bearer $TOKEN" \
  -H "Content-Type: application/json" \
  -d '{"status": "completed"}'

# Delete task
curl -X DELETE "http://localhost:8000/tasks/<task-id>" \
  -H "Authorization: Bearer $TOKEN"
```

### Automated Tests

```bash
# Run all tests
pytest tests/ -v

# Run only task tests
pytest tests/integration/test_tasks_crud.py -v

# Run with coverage
pytest tests/ --cov=app --cov-report=html
```

---

## Verification Checklist

- [ ] `GET /tasks` returns `PaginatedTaskResponse` (not list)
- [ ] `GET /tasks?status=pending` filters correctly
- [ ] `GET /tasks?created_after=2025-01-01` filters correctly
- [ ] `GET /tasks?sort=title:asc` sorts correctly
- [ ] `GET /tasks?limit=5&offset=5` paginates correctly
- [ ] Response does NOT include `user_id`
- [ ] Rate limits work on POST/PATCH/DELETE
- [ ] 404 returned for other user's tasks (not 403)
- [ ] All test cases pass

---

## Troubleshooting

### Rate Limit Not Working
1. Check `limiter` is added to FastAPI app state
2. Verify middleware is registered in `main.py`
3. Check custom key function is extracting user ID

### Pagination Returns Wrong Count
1. Verify `total` is calculated before applying LIMIT/OFFSET
2. Check subquery is used for count

### Sort Not Working
1. Verify field name matches model attribute exactly
2. Check `getattr(Task, field)` is valid

---

## Next Steps

After completing Part 3:
1. Run full test suite
2. Update OpenAPI documentation
3. Proceed to Part 4 (Bulk Operations - if planned)
