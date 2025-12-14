---
id: 002
title: Task CRUD API Plan Creation
stage: plan
date: 2025-12-14
surface: agent
model: claude-sonnet-4-5-20250929
feature: phase2-tasks-crud
branch: main
user: developer
command: /sp.plan
labels: ["api", "crud", "tasks", "planning", "architecture"]
links:
  spec: specs/phase2/part3-tasks-crud/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/phase2/part3-tasks-crud/research.md
  - specs/phase2/part3-tasks-crud/data-model.md
  - specs/phase2/part3-tasks-crud/contracts/tasks-api.yaml
  - specs/phase2/part3-tasks-crud/quickstart.md
  - specs/phase2/part3-tasks-crud/plan.md
tests:
  - null
---

## Prompt

Run Plan phase for Phase 2 Part 3: TASK CRUD API spec.

Generate specs/phase2/part3-tasks-crud/plan.md with:

## 1. Main Implementation Phases (5 phases)
Phase 1: Schemas + Pydantic models (1hr)
Phase 2: TaskService CRUD methods (1.5hr)
Phase 3: Task Router + Endpoints (2hr)
Phase 4: Query params + Pagination (1hr)
Phase 5: Tests + Rate limiting (2.5hr)

## 2. Key Dependencies
Schemas → TaskService → Router → Tests

## 3. File Structure
phase2/backend/app/schemas/task.py
phase2/backend/app/services/task_service.py (extend)
phase2/backend/app/routers/tasks.py
tests/integration/test_tasks_crud.py

## 4. Success Criteria
- GET /tasks?limit=20&status=pending → PaginatedTaskResponse
- POST /tasks → 201 TaskRead
- PATCH /tasks/{id} → 200 updated task
- DELETE /tasks/{id} → 204
- Wrong user → 404 (not 403)
- Rate limits per user (30 POST/hr)

## Response snapshot

Created complete planning artifacts for Phase 2 Part 3: Task CRUD API:

**Phase 0 - Research (research.md):**
- Pagination strategy: Offset-based with limit/offset
- Query handling: FastAPI Query with Pydantic validation
- Response schema: Hide user_id via new TaskRead schema
- Rate limiting: Per-user via SlowAPI custom key function
- Authorization: Return 404 (not 403) for security
- Partial updates: Use exclude_unset=True (existing pattern)
- Existing code assessment: Identified gaps in current implementation

**Phase 1 - Design Artifacts:**
- data-model.md: Task entity specs, request/response schemas, validation rules
- contracts/tasks-api.yaml: OpenAPI 3.1 spec with all endpoints, schemas, error responses
- quickstart.md: Step-by-step implementation guide with curl examples

**Implementation Plan (plan.md):**
- 5 phases: Schemas → Service → Router → Pagination → Tests+RateLimits
- Dependency graph showing sequential flow
- File change summary (~480 lines total)
- Success criteria mapped to 25 test cases
- Risks and mitigations identified

**Key Decisions:**
- Extend existing files only (no new modules)
- Keep TaskResponse for backward compatibility
- User-based rate limiting (not IP-based)
- Composite index recommended if scale requires

## Outcome

- Impact: Created comprehensive planning documentation for Task CRUD API implementation
- Tests: N/A (planning phase - test cases defined but not implemented)
- Files: 5 files created (research.md, data-model.md, tasks-api.yaml, quickstart.md, plan.md)
- Next prompts: /sp.tasks to generate tasks.md, then /sp.implement to execute
- Reflection: Plan builds on existing Part 2 infrastructure; identified minimal code changes needed (~480 lines)

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): Standard plan generation with user-provided phases
- Next experiment: Verify Phase 4 can be integrated with Phase 3 for efficiency
