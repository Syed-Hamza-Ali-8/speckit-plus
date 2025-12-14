---
id: 008
title: Implement Auth Setup Dependencies
stage: green
date: 2025-12-14
surface: agent
model: claude-sonnet-4-5-20250929
feature: phase2-authentication
branch: main
user: developer
command: /sp.implement
labels: ["implementation", "setup", "dependencies", "jwt", "phase2"]
links:
  spec: specs/phase2/part2-authentication/spec.md
  ticket: null
  adr: history/adr/ADR-001-authentication-security-stack.md
  pr: null
files:
  - phase2/backend/pyproject.toml
  - phase2/backend/.env.example
  - phase2/backend/.env
  - phase2/backend/app/core/config.py
  - specs/phase2/part2-authentication/tasks.md
tests:
  - null
---

## Prompt

/sp.implement Task 3

## Response snapshot

Implemented Phase 1: Setup tasks T001-T003 from tasks.md:

**T001: Install auth dependencies**
- Added to pyproject.toml dependencies:
  - argon2-cffi>=23.1.0 (password hashing)
  - python-jose[cryptography]>=3.3.0 (JWT tokens)
  - python-multipart>=0.0.6 (form data parsing)
  - slowapi>=0.1.9 (rate limiting)
  - fastapi>=0.109.0 (web framework)
  - uvicorn[standard]>=0.27.0 (ASGI server)

**T002: Add JWT_SECRET_KEY to .env.example**
- Created phase2/backend/.env.example with:
  - DATABASE_URL template
  - TEST_DATABASE_URL template
  - JWT_SECRET_KEY (with generation instructions)
  - JWT_ALGORITHM (default: HS256)
  - JWT_ACCESS_TOKEN_EXPIRE_MINUTES (default: 30)
- Updated .env with development JWT secret

**T003: Extend Settings with JWT config**
- Added to Settings class in config.py:
  - jwt_secret_key: str (required)
  - jwt_algorithm: str = "HS256"
  - jwt_access_token_expire_minutes: int = 30

Updated tasks.md to mark T001, T002, T003 as [x] complete.

## Outcome

- ✅ Impact: Phase 1 Setup complete - auth dependencies and JWT config ready
- 🧪 Tests: Pending (run `uv sync` to install dependencies)
- 📁 Files: 5 files created/modified
- 🔁 Next prompts: T008-T012 (Phase 3: Registration) or T013-T017 (Phase 4: Login)
- 🧠 Reflection: Added FastAPI and uvicorn as they're needed for endpoints but weren't in original dependencies

## Evaluation notes (flywheel)

- Failure modes observed: uv/pip commands not available, resolved by editing pyproject.toml directly
- Graders run and results (PASS/FAIL): Pending - needs `uv sync` to verify
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Run `uv sync` to verify dependencies install correctly
