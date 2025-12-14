---
id: 002
title: Database Setup Spec Refinement with Dual-Layer Validation
stage: spec
date: 2025-12-13
surface: agent
model: claude-sonnet-4-5-20250929
feature: phase2-database-setup
branch: main
user: hamza
command: /sp.specify
labels: ["database", "sqlmodel", "validation", "enum-extensibility", "refinement"]
links:
  spec: specs/phase2/part1-database-setup/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/phase2/part1-database-setup/spec.md
tests:
  - null
---

## Prompt

I'm using Spec-Driven Development in @phase2 context. Start the Specify phase for **Phase 2 Part 1: DATABASE SETUP** using sqlmodel-schema-generator agent.

Following the book pattern (pre-spec conversation → formal 4-section spec):

**FIRST: Short pre-spec conversation to clarify:**
- Intent: What exactly the Task model + Neon Postgres schema must provide for multi-user Todo app
- Constraints: Phase 2 stack only (FastAPI + SQLModel + Neon Postgres + Alembic), code under phase2/backend/app/models, reference specs/database/task-model.md
- SMART Success Evals: What must be true when database layer is "done" (exact fields, constraints, indexes, tests, migration status)
- Non-Goals: No API endpoints, no auth logic, no frontend, no business logic (pure data layer only)

**Clarifying Questions Asked:**

1. **Q1: User ID Strategy (Pre-Auth)** - How to handle user_id in tests before User table exists?
2. **Q2: Status Enum Extensibility** - Should enum be designed for future statuses?
3. **Q3: Timestamp Precision & Timezone** - UTC storage and precision requirements?
4. **Q4: Test Database Strategy** - Support local Postgres or Neon-only?
5. **Q5: Validation Location** - Python-only, DB-only, or both?

**User Answers:**

Q1: Use random UUIDs for testing (generate with uuid.uuid4()). No placeholder UUIDs.

Q2: Design enum extensible (use SQLModel Field(sa_column=Column(Enum(TaskStatus, native_enum=False))) so we can add statuses later without schema breakage.

Q3: UTC always (TIMESTAMPTZ in Neon). Millisecond precision is fine (default DateTime behavior).

Q4: Neon-only for integration tests. Local Postgres/Docker is out of scope for Part 1.

Q5: Both layers - Python/SQLModel validators + Database CHECK constraints. DB is ultimate source of truth, but frontend/API validation prevents unnecessary DB roundtrips.

## Response snapshot

Finalized spec at `specs/phase2/part1-database-setup/spec.md` with key refinements:

1. **Enum Extensibility**: Added constraint that TaskStatus MUST use `native_enum=False` in SQLAlchemy column definition, allowing future statuses without schema migration.

2. **Dual-Layer Validation**: New section 3.2 with 5 SMART criteria (V1-V5):
   - Python validates empty title before DB call
   - Python validates title length before DB call
   - DB CHECK rejects empty title via constraint
   - DB CHECK rejects whitespace-only title
   - Python validates description length

3. **Testing Constraints**:
   - Random UUIDs via `uuid.uuid4()` for user_id (no placeholders)
   - Neon-only for integration (no local Postgres/Docker)
   - New criterion T8: User ID uses random UUIDs

4. **Timestamp Constraints**: Explicitly documented UTC storage with millisecond precision, timezone conversion at API/frontend layer.

5. **Model Reference Updated**: Appendix A now includes `@validator('title')` for Pydantic-level validation and `CheckConstraint` for database-level enforcement.

Total Success Evals increased from 25 to 30 criteria across 6 categories (added Validation section).

## Outcome

- ✅ Impact: Spec now bulletproof for dual-layer validation and future enum extensibility
- 🧪 Tests: No tests run (specification refinement phase)
- 📁 Files: 1 file modified (spec.md)
- 🔁 Next prompts: `/sp.plan` for architectural decisions, `/sp.tasks` for implementation breakdown
- 🧠 Reflection: Dual-layer validation prevents bad data at earliest possible point while ensuring DB integrity. Non-native enum choice future-proofs status field changes.

## Evaluation notes (flywheel)

- Failure modes observed: None - user provided clear answers to all 5 questions
- Graders run and results (PASS/FAIL): N/A (spec phase)
- Prompt variant (if applicable): Pre-spec clarifying questions pattern
- Next experiment (smallest change to try): Verify CHECK constraint syntax works with SQLModel/Alembic migration generation
