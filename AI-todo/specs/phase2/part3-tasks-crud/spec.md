# Specification — Phase 2 Part 3: Task CRUD API

> **Status:** Draft
> **Created:** 2025-12-14
> **Feature:** Task CRUD Endpoints with Authentication
> **Depends On:** Part 2 (Authentication)

---

## 1. Requirements

### 1.1 Functional Requirements

| ID | Requirement | Priority |
|----|-------------|----------|
| FR-01 | Users can list their own tasks with pagination | Must |
| FR-02 | Users can create new tasks | Must |
| FR-03 | Users can retrieve a specific task by ID | Must |
| FR-04 | Users can partially update a task (title, description, status) | Must |
| FR-05 | Users can delete their own tasks | Must |
| FR-06 | Users cannot modify `user_id` field | Must |
| FR-07 | Users cannot access other users' tasks | Must |
| FR-08 | Task listing supports filtering by status and date | Should |
| FR-09 | Task listing supports sorting by multiple fields | Should |
| FR-10 | Rate limiting prevents abuse on write endpoints | Should |

### 1.2 Non-Functional Requirements

| ID | Requirement | Target |
|----|-------------|--------|
| NFR-01 | All endpoints require valid JWT | Security |
| NFR-02 | Ownership violation returns 404 (not 403) | Security |
| NFR-03 | List endpoint responds < 200ms p95 | Performance |
| NFR-04 | Rate limit: 30 POST/DELETE, 60 PATCH per hour per user | Security |

### 1.3 API Endpoints

| Method | Endpoint | Auth | Description | Status Codes |
|--------|----------|------|-------------|--------------|
| `GET` | `/tasks` | Yes | List current user's tasks (paginated) | 200, 401 |
| `POST` | `/tasks` | Yes | Create a new task | 201, 400, 401, 429 |
| `GET` | `/tasks/{id}` | Yes | Get a specific task by ID | 200, 401, 404 |
| `PATCH` | `/tasks/{id}` | Yes | Partially update a task | 200, 400, 401, 404, 429 |
| `DELETE` | `/tasks/{id}` | Yes | Delete a task | 204, 401, 404, 429 |

### 1.4 Query Parameters

#### Filtering

| Parameter | Type | Example | Description |
|-----------|------|---------|-------------|
| `status` | enum | `?status=pending` | Filter by status (pending/completed) |
| `created_after` | ISO date | `?created_after=2025-01-01` | Tasks created after date |
| `created_before` | ISO date | `?created_before=2025-12-31` | Tasks created before date |

#### Sorting

| Parameter | Format | Default | Description |
|-----------|--------|---------|-------------|
| `sort` | `field:direction` | `created_at:desc` | Sort by field |

**Allowed sort fields:** `created_at`, `updated_at`, `title`, `status`
**Allowed directions:** `asc`, `desc`

#### Pagination

| Parameter | Type | Default | Max | Description |
|-----------|------|---------|-----|-------------|
| `limit` | int | 20 | 100 | Number of items per page |
| `offset` | int | 0 | - | Number of items to skip |

**Example Request:**
```
GET /tasks?status=pending&sort=created_at:desc&limit=20&offset=0
```

### 1.5 Out of Scope (Deferred to Part 4)

- Bulk create/update/delete operations
- Task search by title/description
- Task tagging/categories
- Task due dates and reminders

---

## 2. Schemas

### 2.1 Request Schemas

```python
from datetime import datetime
from uuid import UUID
from pydantic import BaseModel, Field
from enum import Enum


class TaskStatus(str, Enum):
    """Task status enumeration."""
    PENDING = "pending"
    COMPLETED = "completed"


class TaskCreate(BaseModel):
    """Request schema for creating a task."""
    title: str = Field(min_length=1, max_length=255)
    description: str | None = Field(default=None, max_length=1000)
    status: TaskStatus = Field(default=TaskStatus.PENDING)


class TaskUpdate(BaseModel):
    """Request schema for updating a task (partial update)."""
    title: str | None = Field(default=None, min_length=1, max_length=255)
    description: str | None = Field(default=None, max_length=1000)
    status: TaskStatus | None = Field(default=None)
    # NOTE: user_id is NOT allowed - immutable field
```

### 2.2 Response Schemas

```python
class TaskRead(BaseModel):
    """Response schema for a single task."""
    id: UUID
    title: str
    description: str | None
    status: TaskStatus
    created_at: datetime
    updated_at: datetime
    # NOTE: user_id excluded - implied by authentication

    class Config:
        from_attributes = True


class PaginatedTaskResponse(BaseModel):
    """Response schema for paginated task list."""
    items: list[TaskRead]
    total: int
    limit: int
    offset: int


class ErrorResponse(BaseModel):
    """Standard error response."""
    detail: str


class ValidationErrorResponse(BaseModel):
    """Validation error response."""
    detail: list[dict]
```

### 2.3 Query Parameter Schema

```python
from datetime import date
from typing import Literal


class TaskQueryParams(BaseModel):
    """Query parameters for listing tasks."""
    status: TaskStatus | None = None
    created_after: date | None = None
    created_before: date | None = None
    sort: str = "created_at:desc"  # field:direction
    limit: int = Field(default=20, ge=1, le=100)
    offset: int = Field(default=0, ge=0)
```

---

## 3. Authorization

### 3.1 Ownership Pattern

**Security Decision:** Return `404 Not Found` (not `403 Forbidden`) when a user attempts to access a task they don't own. This prevents task ID enumeration attacks.

### 3.2 Authorization Helper

```python
from uuid import UUID
from fastapi import HTTPException, status
from sqlalchemy.ext.asyncio import AsyncSession

from src.models.task import Task
from src.models.user import User


async def get_task_or_404(
    task_id: UUID,
    current_user: User,
    db: AsyncSession
) -> Task:
    """
    Retrieve a task by ID, ensuring ownership.

    Returns 404 for both non-existent tasks AND tasks owned by other users.
    This prevents enumeration attacks.
    """
    task = await db.get(Task, task_id)

    # Return 404 for both cases to prevent enumeration
    if not task or task.user_id != current_user.id:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Task not found"
        )

    return task
```

### 3.3 Endpoint Authorization Flow

```
Request → JWT Validation → User Extraction → Ownership Check → Response
            ↓                   ↓                  ↓
         401 if              401 if            404 if
         invalid           user not found     not owner
```

### 3.4 Rate Limiting Configuration

**Rate Limit Key Strategy:** Per-user (from JWT `current_user.id`) with IP fallback for unauthenticated requests.

```python
from slowapi import Limiter
from slowapi.util import get_remote_address
from starlette.requests import Request


def get_user_rate_limit_key(request: Request) -> str:
    """
    Extract rate limit key: user ID from JWT if authenticated, else IP.

    This prevents bypass via multiple accounts while still protecting
    unauthenticated endpoints from abuse.
    """
    if hasattr(request.state, "user") and request.state.user:
        return f"user:{request.state.user.id}"
    return get_remote_address(request)


limiter = Limiter(key_func=get_user_rate_limit_key)

# Per-user rate limits
RATE_LIMITS = {
    "create_task": "30/hour",
    "update_task": "60/hour",
    "delete_task": "30/hour",
    "list_tasks": "100/hour",
    "get_task": "200/hour",
}
```

---

## 4. Success Criteria

### 4.1 Test Cases Matrix

| ID | Category | Test Case | Expected Result | DB |
|----|----------|-----------|-----------------|-----|
| TC-01 | List | List own tasks (empty) | 200 + empty items | Both |
| TC-02 | List | List own tasks (with data) | 200 + TaskRead[] | Both |
| TC-03 | List | Filter by status=pending | Only pending tasks | Both |
| TC-04 | List | Filter by created_after | Filtered results | Both |
| TC-05 | List | Sort by created_at:asc | Ascending order | Both |
| TC-06 | List | Pagination limit=5 offset=5 | Correct slice | Both |
| TC-07 | List | No auth token | 401 Unauthorized | Unit |
| TC-08 | Create | Valid task | 201 + TaskRead | Both |
| TC-09 | Create | Empty title | 400 Validation Error | Unit |
| TC-10 | Create | Title > 255 chars | 400 Validation Error | Unit |
| TC-11 | Create | No auth token | 401 Unauthorized | Unit |
| TC-12 | Get | Own task by ID | 200 + TaskRead | Both |
| TC-13 | Get | Non-existent ID | 404 Not Found | Both |
| TC-14 | Get | Other user's task | 404 Not Found | Integration |
| TC-15 | Get | Invalid UUID format | 422 Validation Error | Unit |
| TC-16 | Update | Update title only | 200 + updated TaskRead | Both |
| TC-17 | Update | Update status only | 200 + updated TaskRead | Both |
| TC-18 | Update | Update multiple fields | 200 + updated TaskRead | Both |
| TC-19 | Update | Empty title | 400 Validation Error | Unit |
| TC-20 | Update | Other user's task | 404 Not Found | Integration |
| TC-21 | Update | Attempt to change user_id | 400/422 (field ignored) | Unit |
| TC-22 | Delete | Delete own task | 204 No Content | Both |
| TC-23 | Delete | Delete non-existent | 404 Not Found | Both |
| TC-24 | Delete | Delete other user's task | 404 Not Found | Integration |
| TC-25 | Rate | 31st POST in 1 hour | 429 Too Many Requests | Integration |

### 4.2 curl Examples

#### List Tasks
```bash
# List all tasks (default pagination)
curl -X GET "http://localhost:8000/tasks" \
  -H "Authorization: Bearer <token>"

# List with filters and pagination
curl -X GET "http://localhost:8000/tasks?status=pending&sort=created_at:desc&limit=10&offset=0" \
  -H "Authorization: Bearer <token>"
```

**Expected Response (200):**
```json
{
  "items": [
    {
      "id": "550e8400-e29b-41d4-a716-446655440000",
      "title": "Complete project",
      "description": "Finish the API implementation",
      "status": "pending",
      "created_at": "2025-12-14T10:00:00Z",
      "updated_at": "2025-12-14T10:00:00Z"
    }
  ],
  "total": 1,
  "limit": 20,
  "offset": 0
}
```

#### Create Task
```bash
curl -X POST "http://localhost:8000/tasks" \
  -H "Authorization: Bearer <token>" \
  -H "Content-Type: application/json" \
  -d '{"title": "New task", "description": "Task description"}'
```

**Expected Response (201):**
```json
{
  "id": "550e8400-e29b-41d4-a716-446655440001",
  "title": "New task",
  "description": "Task description",
  "status": "pending",
  "created_at": "2025-12-14T10:30:00Z",
  "updated_at": "2025-12-14T10:30:00Z"
}
```

#### Get Task
```bash
curl -X GET "http://localhost:8000/tasks/550e8400-e29b-41d4-a716-446655440000" \
  -H "Authorization: Bearer <token>"
```

#### Update Task
```bash
curl -X PATCH "http://localhost:8000/tasks/550e8400-e29b-41d4-a716-446655440000" \
  -H "Authorization: Bearer <token>" \
  -H "Content-Type: application/json" \
  -d '{"status": "completed"}'
```

**Expected Response (200):**
```json
{
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "title": "Complete project",
  "description": "Finish the API implementation",
  "status": "completed",
  "created_at": "2025-12-14T10:00:00Z",
  "updated_at": "2025-12-14T11:00:00Z"
}
```

#### Delete Task
```bash
curl -X DELETE "http://localhost:8000/tasks/550e8400-e29b-41d4-a716-446655440000" \
  -H "Authorization: Bearer <token>"
```

**Expected Response:** `204 No Content` (empty body)

### 4.3 Error Response Examples

#### 401 Unauthorized (No Token)
```json
{
  "detail": "Not authenticated"
}
```

#### 404 Not Found (Task doesn't exist or wrong owner)
```json
{
  "detail": "Task not found"
}
```

#### 400 Validation Error (Empty Title)
```json
{
  "detail": [
    {
      "loc": ["body", "title"],
      "msg": "String should have at least 1 character",
      "type": "string_too_short"
    }
  ]
}
```

#### 429 Rate Limited
```json
{
  "detail": "Rate limit exceeded. Try again in 1800 seconds."
}
```

### 4.4 Acceptance Checklist

- [ ] `GET /tasks` returns paginated list of user's tasks only
- [ ] `GET /tasks` supports status, date filtering, and sorting
- [ ] `POST /tasks` creates task with correct user_id
- [ ] `POST /tasks` validates title (non-empty, max 255 chars)
- [ ] `GET /tasks/{id}` returns task if owned by user
- [ ] `GET /tasks/{id}` returns 404 for other user's tasks
- [ ] `PATCH /tasks/{id}` allows partial updates (title, description, status)
- [ ] `PATCH /tasks/{id}` ignores/rejects user_id changes
- [ ] `DELETE /tasks/{id}` removes task if owned by user
- [ ] `DELETE /tasks/{id}` returns 404 for other user's tasks
- [ ] All endpoints return 401 without valid JWT
- [ ] Rate limiting enforced on write endpoints
- [ ] All unit tests pass with SQLite
- [ ] All integration tests pass with Neon PostgreSQL

### 4.5 Definition of Done

1. **Code Complete:** All 5 endpoints implemented per spec
2. **Tests Pass:** 100% of test cases green
3. **Security Verified:** Ownership checks prevent cross-user access
4. **Rate Limits Active:** Write endpoints protected
5. **Documentation:** OpenAPI spec auto-generated via FastAPI
6. **PHR Created:** Prompt history recorded

---

## Clarifications

### Session 2025-12-14

- Q: Rate Limiting Key: Per-user (current_user.id) OR per-IP? → A: Per-user with IP fallback for unauthenticated requests

---

## References

- [FastAPI Query Parameters](https://fastapi.tiangolo.com/tutorial/query-params/)
- [FastAPI Path Parameters](https://fastapi.tiangolo.com/tutorial/path-params/)
- [Pydantic Field Validators](https://docs.pydantic.dev/latest/concepts/fields/)
- [SlowAPI Rate Limiting](https://slowapi.readthedocs.io/)
