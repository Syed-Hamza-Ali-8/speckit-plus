# Implementation Plan: Phase 2 Part 3 - Task CRUD API

**Branch**: `main` | **Date**: 2025-12-14 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/phase2/part3-tasks-crud/spec.md`

## Summary

Extend the existing Task CRUD API with query parameters (filtering, sorting, pagination), rate limiting on write endpoints, and updated response schemas that hide `user_id`. This builds on Phase 2 Part 2 (Authentication) which established basic CRUD operations with user ownership.

## Technical Context

**Language/Version**: Python 3.13, UV package manager
**Primary Dependencies**: FastAPI, SQLModel, slowapi (existing)
**Storage**: Neon PostgreSQL (existing), Alembic migrations
**Testing**: pytest, pytest-asyncio, httpx (SQLite unit + Neon integration)
**Target Platform**: Linux server (API), Windows dev
**Project Type**: Web application (backend focus)
**Performance Goals**: < 200ms p95 for list endpoint
**Constraints**: Rate limits (30 POST/DELETE, 60 PATCH, 100 GET per hour per user)
**Scale/Scope**: Single-tenant MVP, ~100-10k tasks per user

## Constitution Check

*GATE: Must pass before implementation. Verified against `.specify/memory/constitution.md`*

| Gate | Status | Notes |
|------|--------|-------|
| Phase II Technology Stack | PASS | FastAPI, SQLModel, Neon DB - all permitted |
| Spec-Driven Development | PASS | spec.md exists at `/specs/phase2/part3-tasks-crud/spec.md` |
| Clean Architecture | PASS | Layered: schemas, services, routes |
| Type Hints | PASS | All functions include type hints |
| Testing Coverage | PASS | 25 test cases defined, targeting 80%+ coverage |
| API Endpoint Verification | PASS | Matches spec.md Section 1.3 |

**Violation Justifications:** None - all gates pass.

## Project Structure

### Documentation (this feature)

```text
specs/phase2/part3-tasks-crud/
├── spec.md              # Feature specification
├── plan.md              # This file
├── research.md          # Technology decisions
├── data-model.md        # Entity definitions
├── quickstart.md        # Setup instructions
├── contracts/           # OpenAPI spec
│   └── tasks-api.yaml
└── tasks.md             # Implementation tasks (created by /sp.tasks)
```

### Source Code (files to modify)

```text
phase2/backend/
├── app/
│   ├── schemas/
│   │   └── task.py              # MODIFY: Add TaskRead, PaginatedTaskResponse, TaskQueryParams
│   ├── services/
│   │   └── task_service.py      # MODIFY: Extend get_tasks() with filtering/sorting/pagination
│   ├── api/
│   │   └── routes/
│   │       └── tasks.py         # MODIFY: Add query params, rate limiting, new response schemas
│   └── middleware/
│       └── rate_limit.py        # MODIFY: Add user-based rate limit key function
├── tests/
│   ├── unit/
│   │   └── test_task_schemas.py     # NEW: Schema validation tests
│   └── integration/
│       └── test_tasks_crud.py       # NEW: Full CRUD with pagination tests
└── requirements.txt                 # No changes (deps already installed)
```

**Structure Decision**: Extend existing files only - no new modules needed.

## Implementation Phases

### Phase 1: Schemas + Pydantic Models

**Files:**
- `app/schemas/task.py` - Add `TaskRead`, `PaginatedTaskResponse`, `TaskQueryParams`

**Dependencies:** None

**Deliverables:**
- [ ] `TaskRead` schema (excludes `user_id`)
- [ ] `PaginatedTaskResponse` schema (items, total, limit, offset)
- [ ] `TaskQueryParams` schema (status, created_after, created_before, sort, limit, offset)
- [ ] Keep existing `TaskCreate`, `TaskUpdate`, `TaskResponse` for backward compatibility

**Code Changes:**
```python
# Add to app/schemas/task.py
class TaskRead(BaseModel):
    """Response schema without user_id."""
    id: UUID
    title: str
    description: str | None
    status: TaskStatus
    created_at: datetime
    updated_at: datetime
    model_config = {"from_attributes": True}

class PaginatedTaskResponse(BaseModel):
    """Paginated list response."""
    items: list[TaskRead]
    total: int
    limit: int
    offset: int
```

### Phase 2: TaskService CRUD Methods

**Files:**
- `app/services/task_service.py` - Extend `get_tasks()` with filtering/sorting/pagination

**Dependencies:** Phase 1 (uses `TaskQueryParams` for type hints)

**Deliverables:**
- [ ] `get_tasks()` accepts filter parameters (status, created_after, created_before)
- [ ] `get_tasks()` accepts sort parameters (field, direction)
- [ ] `get_tasks()` accepts pagination (limit, offset)
- [ ] `get_tasks()` returns tuple of (tasks, total_count)
- [ ] Helper function `parse_sort()` validates sort string

**Code Changes:**
```python
# Modify get_tasks signature
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
    ...
```

### Phase 3: Task Router + Endpoints

**Files:**
- `app/api/routes/tasks.py` - Update endpoints with query params and new schemas

**Dependencies:** Phase 1 (schemas), Phase 2 (service)

**Deliverables:**
- [ ] `GET /tasks` accepts query parameters
- [ ] `GET /tasks` returns `PaginatedTaskResponse`
- [ ] `POST /tasks` returns `TaskRead` (not `TaskResponse`)
- [ ] `GET /tasks/{id}` returns `TaskRead`
- [ ] `PATCH /tasks/{id}` returns `TaskRead`
- [ ] Sort parameter validation with helpful errors

**Code Changes:**
```python
@router.get("", response_model=PaginatedTaskResponse)
async def list_tasks(
    db: DbSession,
    current_user: CurrentUser,
    status: TaskStatus | None = Query(default=None),
    created_after: date | None = Query(default=None),
    created_before: date | None = Query(default=None),
    sort: str = Query(default="created_at:desc"),
    limit: int = Query(default=20, ge=1, le=100),
    offset: int = Query(default=0, ge=0),
) -> PaginatedTaskResponse:
    ...
```

### Phase 4: Query Params + Pagination

**Files:**
- Covered in Phase 3 (combined for efficiency)

**Dependencies:** Phase 3

**Deliverables:**
- [ ] Filtering by status works correctly
- [ ] Filtering by date range works correctly
- [ ] Sorting by all allowed fields works
- [ ] Pagination returns correct slices
- [ ] Total count is accurate (before pagination)

### Phase 5: Tests + Rate Limiting

**Files:**
- `app/middleware/rate_limit.py` - Add user-based key function
- `app/api/routes/tasks.py` - Add rate limit decorators
- `tests/unit/test_task_schemas.py` - Schema validation tests
- `tests/integration/test_tasks_crud.py` - Full CRUD tests

**Dependencies:** Phases 1-4 complete

**Deliverables:**
- [ ] User-based rate limit key function
- [ ] Rate limit decorators on POST (30/hr), PATCH (60/hr), DELETE (30/hr)
- [ ] Unit tests: schema validation (10 tests)
- [ ] Integration tests: CRUD + pagination (15 tests)
- [ ] 80%+ code coverage

**Code Changes:**
```python
# app/middleware/rate_limit.py
def get_user_key(request: Request) -> str:
    if hasattr(request.state, "user") and request.state.user:
        return f"user:{request.state.user.id}"
    return get_remote_address(request)

# app/api/routes/tasks.py
@router.post("", ...)
@limiter.limit("30/hour")
async def create_task(...): ...
```

## Dependency Graph

```text
Phase 1: Schemas + Pydantic Models
    │
    ▼
Phase 2: TaskService CRUD Methods
    │
    ▼
Phase 3: Task Router + Endpoints
    │
    ▼
Phase 4: Query Params + Pagination (integrated with Phase 3)
    │
    ▼
Phase 5: Tests + Rate Limiting
```

## Success Criteria

| Criterion | Validation Method |
|-----------|-------------------|
| `GET /tasks?limit=20&status=pending` → PaginatedTaskResponse | Integration test TC-03 |
| `POST /tasks` → 201 TaskRead (no user_id) | Integration test TC-08 |
| `PATCH /tasks/{id}` → 200 updated task | Integration test TC-16 |
| `DELETE /tasks/{id}` → 204 | Integration test TC-22 |
| Wrong user → 404 (not 403) | Integration test TC-14, TC-20, TC-24 |
| Rate limits per user (30 POST/hr) | Integration test TC-25 |
| Response excludes `user_id` | All response tests |
| 25 test cases passing | `pytest --cov` shows 25 pass |

## Risks & Mitigations

| Risk | Impact | Mitigation |
|------|--------|------------|
| Breaking existing clients expecting `user_id` | Medium | Keep `TaskResponse` for backward compatibility |
| Rate limit bypass via multiple accounts | Low | Acceptable for MVP; consider IP+user combo later |
| Slow queries at scale with offset pagination | Medium | Add composite index if needed; keyset pagination for v2 |
| Count query performance | Low | Use approximate count if scale requires |

## File Change Summary

| File | Action | Changes |
|------|--------|---------|
| `app/schemas/task.py` | Modify | +3 schemas (~40 lines) |
| `app/services/task_service.py` | Modify | Extend `get_tasks()` (~30 lines) |
| `app/api/routes/tasks.py` | Modify | Query params, rate limits (~50 lines) |
| `app/middleware/rate_limit.py` | Modify | User key function (~10 lines) |
| `tests/unit/test_task_schemas.py` | Create | ~100 lines |
| `tests/integration/test_tasks_crud.py` | Create | ~250 lines |

**Total estimated changes:** ~480 lines (excluding test fixtures)

## References

- Spec: `specs/phase2/part3-tasks-crud/spec.md`
- Research: `specs/phase2/part3-tasks-crud/research.md`
- Data Model: `specs/phase2/part3-tasks-crud/data-model.md`
- API Contract: `specs/phase2/part3-tasks-crud/contracts/tasks-api.yaml`
- Constitution: `.specify/memory/constitution.md`
- Part 2 Plan: `specs/phase2/part2-authentication/plan.md`
