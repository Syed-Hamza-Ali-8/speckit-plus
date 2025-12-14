---
id: 008
title: Implement RTK Query API Store
stage: green
date: 2025-12-14
surface: agent
model: claude-sonnet-4-5-20250929
feature: phase3-frontend-setup
branch: main
user: user
command: /sp.implement Task 3: RTK Query API Store
labels: ["rtk-query", "redux", "api", "typescript", "frontend"]
links:
  spec: specs/phase3/part1-frontend-setup/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - phase2/frontend/src/types/auth.ts
  - phase2/frontend/src/types/task.ts
  - phase2/frontend/src/services/api.ts
  - phase2/frontend/src/services/authApi.ts
  - phase2/frontend/src/services/taskApi.ts
  - phase2/frontend/src/lib/store.ts
  - phase2/frontend/src/main.tsx
tests:
  - npx tsc --noEmit (PASSED)
  - npm run build (PASSED)
---

## Prompt

Task 3: RTK Query API Store

## Response snapshot

Implemented Phase 3 (RTK Query API Store) of the frontend setup with 10 tasks (T017-T026):

1. **T017**: Installed RTK dependencies (`@reduxjs/toolkit`, `react-redux`)
2. **T018**: Created auth TypeScript types (`LoginRequest`, `RegisterRequest`, `AuthResponse`, `JWTPayload`)
3. **T019**: Created task TypeScript types (`Task`, `TaskCreate`, `TaskUpdate`, `PaginatedResponse`, `TaskQueryParams`)
4. **T020**: Created base API with auth header injection using `fetchBaseQuery`
5. **T021**: Added 401 response handler (`baseQueryWithReauth`) that clears token and redirects to login
6. **T022**: Created auth API endpoints (`login`, `register` mutations) in `authApi.ts`
7. **T023**: Created task API endpoints (`getTasks`, `getTask`, `createTask`, `updateTask`, `deleteTask`) with cache tag invalidation in `taskApi.ts`
8. **T024**: Created Redux store configuration with RTK Query middleware in `store.ts`
9. **T025**: Wrapped App with Redux Provider in `main.tsx`
10. **T026**: Verified TypeScript compilation passes with `npx tsc --noEmit`

Key implementation details:
- Base API uses `VITE_API_URL` environment variable with `/api` fallback
- Auth header injection reads JWT from localStorage
- 401 handler auto-clears token and redirects to `/login`
- Task API uses cache tags (`Task`, `LIST`) for smart invalidation
- Fixed `verbatimModuleSyntax` TypeScript error by using type-only imports

## Outcome

- Impact: RTK Query API layer fully configured - auth and task endpoints ready for UI integration
- Tests: TypeScript compilation (PASSED), Vite build (PASSED)
- Files: 7 files created/modified
- Next prompts: Phase 4 (Zustand + Theme), Phase 5 (Vite Proxy), Phase 6 (React Router)
- Reflection: The `verbatimModuleSyntax` setting required separating type imports - good to catch early

## Evaluation notes (flywheel)

- Failure modes observed: Initial build failed due to `verbatimModuleSyntax` requiring type-only imports
- Graders run and results (PASS/FAIL): TypeScript: PASS, Build: PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
