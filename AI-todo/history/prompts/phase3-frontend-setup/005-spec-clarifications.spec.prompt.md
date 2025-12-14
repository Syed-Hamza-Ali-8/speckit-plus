---
id: 005
title: Frontend Setup Spec Clarifications
stage: spec
date: 2025-12-14
surface: agent
model: claude-sonnet-4-5-20250929
feature: phase3-frontend-setup
branch: main
user: developer
command: /sp.clarify
labels: ["frontend", "clarification", "dark-mode", "auth", "protected-route"]
links:
  spec: specs/phase3/part1-frontend-setup/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/phase3/part1-frontend-setup/spec.md
tests:
  - null
---

## Prompt

1. Backend API URL: http://localhost:8000 confirmed running? (uv run uvicorn main:app --reload)
2. Vite Port: Default 5173 OK OR custom port (3000 conflict with other apps)?
3. Router Library: React Router v6 (recommended) OR TanStack Router? (shadcn/ui examples use React Router)
4. shadcn/ui Components Priority: button/card/input/label FIRST (Task 2) OR all at once (Task 7)?
5. Dark Mode Default: 'system' preference OR 'light' initially? Toggle in Header?
6. ProtectedRoute Behavior: Check localStorage JWT OR call /auth/me first (validate token)?
7. RTK Query Tags: Single api OR separate authApi + taskApi (injectEndpoints pattern)?

## Response snapshot

Analyzed 7 clarification questions against spec. Found 5 already resolved:

**Already Resolved in Spec:**
- Q1: Backend API URL → `http://localhost:8000` (spec line 127, 352, 388)
- Q2: Vite Port → `5173` (spec line 349, 461)
- Q3: Router Library → `React Router 6.28.x` (spec line 49)
- Q4: shadcn components → Install incrementally per tasks.md
- Q7: RTK Query → `injectEndpoints` pattern (spec lines 231, 260)

**Clarified via Q&A (2 questions):**

1. **Dark Mode Default**
   - Options: A) 'system' B) 'light' C) 'dark'
   - Recommended: A - 'system' (respects OS preference, modern UX standard)
   - User Answer: A (yes)
   - Resolution: Default theme is 'system' (already in Zustand store code)

2. **ProtectedRoute Behavior**
   - Options: A) localStorage only B) /auth/me call C) Hybrid
   - Recommended: A - localStorage check (fast, 401 handler catches invalid tokens)
   - User Answer: A (yes)
   - Resolution: Check localStorage JWT only, no API validation on route change

**Spec Updated:**
- Added `## Clarifications` section with `### Session 2025-12-14`
- 2 Q&A bullets recorded

## Outcome

- Impact: 2 clarifications resolved, spec updated with decisions
- Tests: N/A (clarification phase)
- Files: 1 file updated (spec.md - added Clarifications section)
- Next prompts: /sp.implement Task 1 to begin frontend scaffolding
- Reflection: Most questions were already answered in spec; only UX/behavior decisions needed user input

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A (clarification)
- Prompt variant (if applicable): 7 questions provided, 2 needed clarification
- Next experiment: Begin implementation with /sp.implement
