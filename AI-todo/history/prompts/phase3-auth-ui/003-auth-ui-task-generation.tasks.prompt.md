---
id: 003
title: Auth UI Task Generation
stage: tasks
date: 2025-12-15
surface: agent
model: claude-sonnet-4-5-20250929
feature: phase3-auth-ui
branch: main
user: user
command: /sp.tasks Phase 3 Part 2: Authentication UI
labels: ["tasks", "task-breakdown", "react-hook-form", "zod", "sonner", "authentication"]
links:
  spec: specs/phase3/part2-auth-ui/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/phase3/part2-auth-ui/tasks.md
tests:
  - null
---

## Prompt

User provided 8-task outline for authentication UI implementation:
- Task 1: Install Auth Dependencies
- Task 2: shadcn/ui Form Components
- Task 3: Zod Validation Schemas
- Task 4: LoginForm Component
- Task 5: RegisterForm Component
- Task 6: Auth Pages Layout
- Task 7: Return URL Redirect Logic
- Task 8: Auth UI Tests

## Response snapshot

Generated detailed task breakdown at `specs/phase3/part2-auth-ui/tasks.md` with 40 tasks across 9 phases:

**Phase 1: Dependencies + Setup (T001-T003)**
- Install react-hook-form, @hookform/resolvers, zod, sonner
- Add Sonner Toaster to App.tsx
- Verify TypeScript compilation

**Phase 2: Zod Validation Schemas (T004-T006)**
- Create loginSchema (email, password)
- Create registerSchema with confirmPassword refinement
- Export TypeScript types

**Phase 3: Loading Button Component (T007-T009)**
- Create LoadingButton extending shadcn Button
- Add Loader2 spinner icon
- Support loading/loadingText props

**Phase 4: LoginForm Component (T010-T016)**
- React Hook Form + Zod resolver setup
- Email/password fields with FormField pattern
- RTK Query useLoginMutation integration
- Sonner toast notifications
- LoadingButton with isLoading state
- Inline validation errors

**Phase 5: RegisterForm Component (T017-T023)**
- React Hook Form + Zod resolver setup
- Email/password/confirmPassword fields
- Cross-field validation (password match)
- RTK Query useRegisterMutation
- Toast notifications
- LoadingButton with isLoading

**Phase 6: Return URL Hook (T024-T027)**
- Create useReturnUrl hook
- Extract returnTo from URL search params
- Validate internal path (prevent open redirect)
- Default fallback to /tasks

**Phase 7: Updated Auth Pages (T028-T032)**
- Update LoginPage with LoginForm + useReturnUrl
- Update RegisterPage with RegisterForm
- Handle redirects after success

**Phase 8: Protected Route Enhancement (T033-T035)**
- Capture current path in ProtectedRoute
- Add returnTo param to login redirect
- End-to-end redirect flow verification

**Phase 9: Final Polish + Build (T036-T040)**
- TypeScript verification
- Production build
- Manual testing of all flows

**Summary:**
- Total: 40 tasks
- New files: 6
- Modified files: 4
- Parallel opportunities: T031-T032 (different pages)

## Outcome

- Impact: Complete task breakdown ready for implementation
- Tests: N/A (tasks phase)
- Files: 1 file created (tasks.md)
- Next prompts: /sp.implement to execute tasks
- Reflection: Granular task breakdown enables incremental implementation with clear checkpoints

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
