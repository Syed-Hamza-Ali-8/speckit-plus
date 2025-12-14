---
id: 004
title: Task CRUD API Spec Clarifications
stage: spec
date: 2025-12-14
surface: agent
model: claude-sonnet-4-5-20250929
feature: phase2-tasks-crud
branch: main
user: developer
command: /sp.clarify
labels: ["api", "crud", "clarification", "rate-limiting"]
links:
  spec: specs/phase2/part3-tasks-crud/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/phase2/part3-tasks-crud/spec.md
tests:
  - null
---

## Prompt

1. Rate Limiting Key: Per-user (current_user.id) OR per-IP? (main.py limiter config)

2. Pagination Defaults: limit=20, offset=0 OR limit=10, offset=0? Max limit=100 confirmed?

3. Sort Format: ?sort=created_at:desc OR ?order_by=-created_at? Multiple sorts allowed?

4. Date Filtering: created_after/before as ISO string OR timestamp? Timezone handling (UTC only)?

5. Error Responses: Wrong user → 404 "Task not found" OR 403 "Forbidden"? (spec says 404)

6. TaskCreate Validation: description optional (None/empty) OR required non-empty string?

7. DELETE Response: 204 No Content OR 200 {"message": "Task deleted"}?

## Response snapshot

Analyzed 7 clarification questions against the spec:

**Already Clear (6 of 7):**
- Pagination: limit=20, offset=0, max=100 (lines 69-70)
- Sort Format: `?sort=field:direction`, single sort (line 60)
- Date Filtering: ISO date string YYYY-MM-DD (lines 53-54)
- Error Responses: 404 for wrong owner (line 176)
- TaskCreate description: Optional (line 106)
- DELETE Response: 204 No Content (line 44, 357)

**Needed Clarification (1 of 7):**
- Rate Limiting Key: Spec had inconsistency (code showed `get_remote_address` but comment said "per-user")

**Resolution:**
- Q: Rate Limiting Key → A: Per-user (current_user.id from JWT) with IP fallback for unauthenticated

**Spec Updated:**
- Section 3.4 Rate Limiting Configuration: Added explicit key strategy and updated code to use `get_user_rate_limit_key()` function
- Added Clarifications section with Session 2025-12-14

## Outcome

- Impact: Resolved rate limiting key strategy inconsistency in spec
- Tests: N/A (clarification phase)
- Files: 1 file updated (spec.md)
- Next prompts: /sp.implement to execute tasks
- Reflection: 6 of 7 questions were already answered in spec; only rate limiting key had genuine inconsistency

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): User-provided clarification questions
- Next experiment: Consider pre-validation step to filter already-answered questions
