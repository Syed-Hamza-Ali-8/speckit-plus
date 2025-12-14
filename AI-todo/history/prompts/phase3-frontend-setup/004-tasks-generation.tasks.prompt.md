---
id: 004
title: Frontend Setup Tasks Generation
stage: tasks
date: 2025-12-14
surface: agent
model: claude-sonnet-4-5-20250929
feature: phase3-frontend-setup
branch: main
user: developer
command: /sp.tasks
labels: ["frontend", "tasks", "scaffolding", "vite", "react"]
links:
  spec: specs/phase3/part1-frontend-setup/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/phase3/part1-frontend-setup/tasks.md
tests:
  - null
---

## Prompt

## Task 1: Vite + React + TypeScript Setup (45m)
**Files:** phase3/frontend/ (package.json, vite.config.ts, tsconfig.json)
**Commands:** npm create vite@latest phase3/frontend -- react-ts
**Description:** Basic Vite React TS project + .env.local (VITE_API_URL)
**Checkpoint:** cd phase3/frontend && npm install && npm run dev → localhost:5173

## Task 2: Tailwind CSS + shadcn/ui Init (30m)
**Files:** tailwind.config.ts, globals.css
**Commands:** npx shadcn-ui@latest init
**Description:** Tailwind + shadcn/ui setup + darkMode: 'class'
**Checkpoint:** Install button component → renders correctly

## Task 3: RTK Query API Store (60m)
**Files:** src/lib/api.ts, src/store.ts
**Description:** createApi baseQuery + auth/task endpoints + Provider
**Checkpoint:** Login mutation compiles (no runtime test needed yet)

## Task 4: Zustand UI Store + Theme (30m)
**Files:** src/stores/uiStore.ts
**Description:** theme: 'light'|'dark'|'system' + setTheme
**Checkpoint:** Zustand store compiles + basic usage in App.tsx

## Task 5: Vite Proxy + Backend Connection (20m)
**Files:** vite.config.ts
**Description:** proxy /api → localhost:8000 + test curl localhost:5173/api/auth/me
**Checkpoint:** curl localhost:5173/api/auth/login → backend response (401 ok)

## Task 6: React Router + ProtectedRoute (45m)
**Files:** src/routes/index.tsx, ProtectedRoute.tsx
**Description:** Router setup + ProtectedRoute (localStorage JWT check)
**Checkpoint:** Navigate / → /login (unauth) → /dashboard (mock auth)

## Task 7: Basic Layout + Auth Pages (60m)
**Files:** src/components/layout/Layout.tsx, LoginPage.tsx, RegisterPage.tsx
**Description:** shadcn/ui Header + Sidebar + basic auth forms
**Checkpoint:** localhost:5173/login → styled form renders

## Task 8: Types + Utils + Final Polish (45m)
**Files:** src/types/task.ts, auth.ts, src/lib/utils.ts
**Description:** TypeScript types + cn() utility + dark mode toggle
**Checkpoint:** npm run build → 0 errors + tsc --noEmit passes

## Response snapshot

Generated comprehensive tasks.md with 57 granular tasks across 8 phases:

**Phase 1: Project Initialization (T001-T005)**
- Vite project creation, env files, TypeScript config
- Checkpoint: dev server runs at localhost:5173

**Phase 2: Tailwind + shadcn/ui (T006-T016)**
- Tailwind CSS setup with dark mode
- shadcn/ui init + base components (button, card, input, label)
- Lucide React icons
- Checkpoint: Button renders with styling

**Phase 3: RTK Query API Store (T017-T026)**
- TypeScript types (auth, task)
- Base API with auth header injection + 401 handler
- Auth and task API endpoints
- Redux store with middleware
- Checkpoint: TypeScript compiles

**Phase 4: Zustand UI Store + Theme (T027-T032)**
- Zustand store with theme state
- Persist middleware for localStorage
- useTheme hook for dark mode
- Checkpoint: Zustand accessible in App

**Phase 5: Vite Proxy (T033-T036)**
- Proxy config: /api → localhost:8000
- Path alias resolution
- Checkpoint: curl test passes

**Phase 6: Router + ProtectedRoute (T037-T045)**
- React Router v6 setup
- ProtectedRoute with JWT check
- Placeholder pages
- Checkpoint: redirect works

**Phase 7: Layout + Auth Pages (T046-T052)**
- Layout, Header, ThemeToggle components
- Styled auth forms
- Checkpoint: login page styled

**Phase 8: Polish + Build (T053-T057)**
- TypeScript verification
- Production build
- Final acceptance tests
- Checkpoint: build succeeds

**Parallel Opportunities Identified:**
- T012-T014: shadcn components
- T018-T019: Type definitions
- T022-T023: API slices
- T041-T043: Placeholder pages
- T049-T050: Styled pages

## Outcome

- Impact: 57 executable tasks ready for implementation
- Tests: No tests (scaffolding phase - manual checkpoints)
- Files: 1 file created (tasks.md ~350 lines)
- Next prompts: /sp.implement Task 1 to begin scaffolding
- Reflection: Tasks broken down to atomic operations; each checkpoint verifiable before proceeding

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A (task generation)
- Prompt variant (if applicable): User-provided 8 high-level tasks expanded to 57 granular tasks
- Next experiment: Execute tasks sequentially with /sp.implement
