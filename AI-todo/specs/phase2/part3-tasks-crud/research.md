# Research — Phase 2 Part 3: Task CRUD API

> **Created:** 2025-12-14
> **Status:** Complete

---

## 1. Pagination Strategy

### Decision: Offset-based pagination with `limit/offset`

### Rationale
- **Simplicity**: Easy to implement and understand
- **Random access**: Users can jump to any page
- **Sufficient for scale**: Adequate for ~100-10k tasks per user
- **SQLAlchemy native**: Direct mapping to `.limit().offset()` clauses

### Alternatives Considered
| Strategy | Pros | Cons | Verdict |
|----------|------|------|---------|
| Offset-based | Simple, random access | Performance degrades at high offsets | **Selected** |
| Cursor-based | Consistent for real-time | Complex, no random access | Overkill for MVP |
| Keyset pagination | Best performance | Requires stable sort key | Future consideration |

### Implementation
```python
# SQLAlchemy query pattern
statement = (
    select(Task)
    .where(Task.user_id == user_id)
    .order_by(Task.created_at.desc())
    .offset(offset)
    .limit(limit)
)
```

---

## 2. Query Parameter Handling

### Decision: Use FastAPI Query parameters with Pydantic validation

### Rationale
- **Type safety**: Automatic validation and coercion
- **Documentation**: Auto-generated OpenAPI docs
- **Defaults**: Clean handling of optional parameters
- **Errors**: Standard 422 validation errors

### Filtering Implementation
```python
from datetime import date
from fastapi import Query

async def list_tasks(
    status: TaskStatus | None = Query(default=None),
    created_after: date | None = Query(default=None),
    created_before: date | None = Query(default=None),
    sort: str = Query(default="created_at:desc"),
    limit: int = Query(default=20, ge=1, le=100),
    offset: int = Query(default=0, ge=0),
):
    ...
```

### Sorting Implementation
```python
ALLOWED_SORT_FIELDS = {"created_at", "updated_at", "title", "status"}
ALLOWED_SORT_DIRECTIONS = {"asc", "desc"}

def parse_sort(sort: str) -> tuple[str, str]:
    """Parse 'field:direction' into validated tuple."""
    parts = sort.split(":")
    if len(parts) != 2:
        raise ValueError("Sort must be 'field:direction'")
    field, direction = parts
    if field not in ALLOWED_SORT_FIELDS:
        raise ValueError(f"Invalid sort field: {field}")
    if direction not in ALLOWED_SORT_DIRECTIONS:
        raise ValueError(f"Invalid sort direction: {direction}")
    return field, direction
```

---

## 3. Response Schema Design

### Decision: Hide `user_id` from `TaskRead` response

### Rationale
- **Security**: No information leakage about user-task mapping
- **Implicit**: User context is already known from JWT
- **Cleaner API**: Reduces response payload size
- **Spec compliance**: Matches Part 3 spec requirement

### Existing vs. New Schema
```python
# Current TaskResponse (includes user_id)
class TaskResponse(BaseModel):
    id: UUID
    user_id: UUID  # Will be removed
    title: str
    ...

# New TaskRead (excludes user_id)
class TaskRead(BaseModel):
    id: UUID
    title: str
    description: str | None
    status: TaskStatus
    created_at: datetime
    updated_at: datetime
    # user_id excluded

# New PaginatedTaskResponse
class PaginatedTaskResponse(BaseModel):
    items: list[TaskRead]
    total: int
    limit: int
    offset: int
```

### Migration Strategy
- Create new `TaskRead` schema alongside existing `TaskResponse`
- Update list endpoint to use `PaginatedTaskResponse`
- Consider deprecating `TaskResponse` or keeping for backward compatibility

---

## 4. Rate Limiting Strategy

### Decision: Per-user rate limiting using SlowAPI with custom key function

### Rationale
- **User-specific**: Rate limits per authenticated user (not IP)
- **Granular**: Different limits for different endpoints
- **Existing integration**: SlowAPI already configured in Part 2
- **Stateless**: In-memory storage sufficient for MVP

### Configuration
```python
from slowapi import Limiter
from slowapi.util import get_remote_address

# Custom key function for user-based limiting
def get_user_key(request: Request) -> str:
    """Extract user ID from JWT for rate limiting."""
    # Fallback to IP if no user context
    if hasattr(request.state, "user"):
        return f"user:{request.state.user.id}"
    return get_remote_address(request)

limiter = Limiter(key_func=get_user_key)

# Endpoint-specific limits
@router.post("")
@limiter.limit("30/hour")
async def create_task(...): ...

@router.patch("/{task_id}")
@limiter.limit("60/hour")
async def update_task(...): ...

@router.delete("/{task_id}")
@limiter.limit("30/hour")
async def delete_task(...): ...
```

### Trade-offs
- **Pro**: Simple, no Redis dependency
- **Con**: Limits reset on server restart
- **Mitigation**: Acceptable for MVP; Redis upgrade path for production

---

## 5. Authorization Pattern

### Decision: Return 404 (not 403) for unauthorized access

### Rationale
- **Security**: Prevents task ID enumeration attacks
- **Consistency**: Same response for "not found" and "not owned"
- **Spec alignment**: Matches Part 2 authentication pattern
- **Simplicity**: Single code path for access denial

### Implementation Pattern
```python
async def get_task_or_404(
    task_id: UUID,
    current_user: User,
    db: AsyncSession
) -> Task:
    """Returns task or raises 404 (never 403)."""
    task = await task_service.get_task(db, task_id, current_user.id)
    if task is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Task not found"
        )
    return task
```

---

## 6. Partial Update Strategy

### Decision: Use `exclude_unset=True` for PATCH operations

### Rationale
- **True partial updates**: Only modify explicitly provided fields
- **Null distinction**: Can set field to `null` vs. not providing field
- **Pydantic native**: Built-in support via `model_dump(exclude_unset=True)`
- **Already implemented**: Pattern exists in current `task_service.py`

### Current Implementation (line 92-94)
```python
# app/services/task_service.py
update_data = data.model_dump(exclude_unset=True)
for field, value in update_data.items():
    setattr(task, field, value)
```

### Validation
- `title` if provided: Must be non-empty (min_length=1)
- `description` if provided: Can be empty or null
- `status` if provided: Must be valid enum
- `user_id`: Never allowed (not in schema)

---

## 7. Existing Code Assessment

### Current State (Phase 2 Part 2)
| Component | Status | Gap |
|-----------|--------|-----|
| `TaskCreate` schema | Exists | None |
| `TaskUpdate` schema | Exists | None |
| `TaskResponse` schema | Exists | Has `user_id` (needs `TaskRead`) |
| `PaginatedTaskResponse` | Missing | Need to create |
| `task_service.get_tasks()` | Exists | No filtering/sorting/pagination |
| `task_service.get_task()` | Exists | None |
| `task_service.create_task()` | Exists | None |
| `task_service.update_task()` | Exists | None |
| `task_service.delete_task()` | Exists | None |
| Task router | Exists | No query params, no rate limits |

### Files to Modify
1. `app/schemas/task.py` - Add `TaskRead`, `PaginatedTaskResponse`, `TaskQueryParams`
2. `app/services/task_service.py` - Extend `get_tasks()` with filtering/sorting/pagination
3. `app/api/routes/tasks.py` - Add query params, rate limiting, use new schemas

### Files to Create
- None (all extensions to existing files)

---

## 8. Test Strategy

### Unit Tests (SQLite)
- Schema validation (empty title, max length)
- Sort parsing (valid/invalid formats)
- Query param defaults

### Integration Tests (Neon PostgreSQL)
- Full CRUD with pagination
- Filtering by status and date
- Sorting ascending/descending
- Cross-user access denial (404)
- Rate limit enforcement

### Test Data Setup
```python
# Create multiple tasks for pagination testing
async def create_test_tasks(db, user_id, count=25):
    for i in range(count):
        await task_service.create_task(
            db, user_id,
            TaskCreate(title=f"Task {i}", description=f"Desc {i}")
        )
```

---

## Summary

| Question | Decision |
|----------|----------|
| Pagination strategy | Offset-based (`limit/offset`) |
| Query handling | FastAPI Query with Pydantic |
| Response schema | Hide `user_id` via new `TaskRead` |
| Rate limiting | Per-user via SlowAPI custom key |
| Authorization denial | 404 (not 403) for security |
| Partial updates | `exclude_unset=True` (existing) |
| New files needed | None (all extensions) |
