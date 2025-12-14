---
id: 002
title: Auth UI Implementation Plan
stage: plan
date: 2025-12-15
surface: agent
model: claude-sonnet-4-5-20250929
feature: phase3-auth-ui
branch: main
user: user
command: /sp.plan Phase 3 Part 2: Authentication UI
labels: ["planning", "implementation", "react-hook-form", "zod", "sonner", "authentication"]
links:
  spec: specs/phase3/part2-auth-ui/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/phase3/part2-auth-ui/plan.md
tests:
  - null
---

## Prompt

Run Plan phase for Phase 3 Part 2: AUTHENTICATION UI spec.

Generate specs/phase3/part2-auth-ui/plan.md with:

## 1. Main Implementation Phases (5 phases)
Phase 1: Dependencies + shadcn/ui Forms
Phase 2: React Hook Form + Zod Schemas
Phase 3: Sonner Toasts + Loading States
Phase 4: Login/Register Pages
Phase 5: Tests + Return URL Redirect

## 2. Key Dependencies
Dependencies → RHF+Zod → Forms → Toasts → Pages → Tests

## 3. File Structure
phase2/frontend/ with components/auth/, lib/validations/, hooks/useReturnUrl.ts

## 4. Success Criteria
15 acceptance tests covering forms, validation, toasts, redirects

## Response snapshot

Created implementation plan at `specs/phase3/part2-auth-ui/plan.md` with 5 sequential phases:

**Phase 1: Dependencies + Zod Schemas**
- Install react-hook-form, @hookform/resolvers, zod, sonner
- Create Zod validation schemas (loginSchema, registerSchema)

**Phase 2: Sonner Toast Integration**
- Add Toaster component to App.tsx
- Create toast utility functions

**Phase 3: Loading Button Component**
- Create LoadingButton with spinner icon
- Support loading state and custom text

**Phase 4: Login/Register Forms**
- LoginForm with RHF + Zod + RTK mutation + toast
- RegisterForm with password confirmation validation
- Update LoginPage and RegisterPage

**Phase 5: Return URL + Protected Route**
- useReturnUrl hook for URL extraction
- Update ProtectedRoute with returnTo support
- Complete redirect flow verification

**File Structure**:
- `src/components/auth/LoginForm.tsx` - NEW
- `src/components/auth/RegisterForm.tsx` - NEW
- `src/components/ui/loading-button.tsx` - NEW
- `src/lib/validations/auth.ts` - NEW
- `src/lib/toast.ts` - NEW
- `src/hooks/useReturnUrl.ts` - NEW
- Updated: LoginPage, RegisterPage, ProtectedRoute, App.tsx

**Success Criteria**: 15 verification checkpoints

## Outcome

- Impact: Complete implementation plan ready for task generation
- Tests: N/A (plan phase)
- Files: 1 file created (plan.md)
- Next prompts: /sp.tasks to generate task breakdown
- Reflection: Sequential phase approach ensures dependencies are resolved before dependent tasks

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
