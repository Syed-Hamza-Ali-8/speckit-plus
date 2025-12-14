---
id: 001
title: Phase 2 Part 1 Database Setup Specification
stage: spec
date: 2025-12-13
surface: agent
model: claude-sonnet-4-5-20250929
feature: phase2-database-setup
branch: main
user: hamza
command: /sp.specify
labels: ["database", "sqlmodel", "neon-postgres", "phase2", "specification"]
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

**User Answers to Clarifying Questions:**

1. Multi-User Ownership Model
- No separate User model in Part 1 (deferred to Part 2 Authentication).
- Task model should have non-nullable user_id: UUID foreign key now. Enforce ownership from day 1.

2. Task Field Evolution
- Add timestamps: created_at, updated_at (automatic server-side).
- Keep minimal for Part 1: id (UUID), user_id (UUID), title (required), description (optional), status (pending/completed enum), created_at, updated_at.
- No priority/due_date/soft delete (future parts).

3. UUID vs Integer Primary Keys
- UUID primary keys for both id and user_id. Better for distributed systems + no enumeration attacks.

4. Migration Strategy Requirements
- Reversible Alembic migrations required.
- No seed data in Part 1 (add test data in integration tests).
- Standard Neon Postgres connection (asyncpg + sslmode=require).

5. Test Database Strategy
- SQLite in-memory for unit tests (speed).
- Separate Neon test database for integration tests (production parity).

## Response snapshot

Created formal 4-section specification at `specs/phase2/part1-database-setup/spec.md` with:

1. **Intent**: Establish production-ready database foundation for multi-user Todo app using SQLModel + Neon Postgres. Task model with UUID keys, user ownership enforced from day 1.

2. **Constraints**:
   - Technology: SQLModel, Neon Postgres, asyncpg, Alembic, Python 3.13+
   - Schema: 7 fields (id, user_id, title, description, status enum, created_at, updated_at)
   - Paths: phase2/backend/app/models/, phase2/backend/app/core/database.py
   - Migrations: Reversible, no seed data

3. **Success Evals**: 25 SMART criteria across 5 categories:
   - Model Definition (M1-M6): Field existence, types, constraints, auto-timestamps
   - Database Connectivity (C1-C5): Async engine, SSL, env config
   - Migration Infrastructure (A1-A5): Alembic setup, reversibility
   - Test Infrastructure (T1-T7): SQLite unit tests, Neon integration tests
   - Code Quality (Q1-Q3): Type hints, no hardcoded creds, 80% coverage

4. **Non-Goals**: No User model, no FK constraint, no API/auth/frontend, no priority/due_date/soft delete

## Outcome

- ✅ Impact: Established clear database foundation spec that enables Part 2+ to build on solid ground
- 🧪 Tests: No tests run (specification phase only)
- 📁 Files: 1 file created (spec.md)
- 🔁 Next prompts: `/sp.plan` for architectural decisions, `/sp.tasks` for implementation breakdown
- 🧠 Reflection: Pre-spec clarification prevented scope creep (User model, priority fields deferred). UUID decision locks in distributed-system readiness.

## Evaluation notes (flywheel)

- Failure modes observed: None - clear answers to all clarifying questions
- Graders run and results (PASS/FAIL): N/A (spec phase)
- Prompt variant (if applicable): Standard 4-section spec format
- Next experiment (smallest change to try): Validate spec structure against implementation friction in Plan phase
